/**
 * @file    app_state_machine.c
 * @brief   구현부
 *
 * 연동 방법:
 *   1) main.c 에 #include "app_state_machine.h" 추가
 *   2) FrontendLink_Init() 이후 StateMachine_Init() 호출
 *   3) while(1) 메인 루프에서 StateMachine_Poll() 반복 호출
 *      (내부에서 프론트엔드의 새 프레임 도착 여부를 자체 감지하므로,
 *       메인 루프 주기가 50ms보다 빨라도 문제없음)
 *   4) 판정 결과가 필요한 곳(USB 프레임 조립, LED 등)에서
 *      StateMachine_GetState() 호출
 */

#include "app_state_machine.h"
#include "app_frontend_link.h"

/* ------------------------------------------------------------------------ */
/* 파라미터 (필요 시 튜닝)                                                    */
/* ------------------------------------------------------------------------ */
#define LEVEL_HIGH_PCT           70u   /* Power_raw 대비 %, 이상이면 High */
#define LEVEL_LOW_PCT            30u   /* Power_raw 대비 %, 이하면 Low */
#define TREND_PCT                 5u   /* Power_raw 대비 %, 300ms 변화 임계값 */
#define TREND_WINDOW_SAMPLES       6u  /* 300ms / 50ms = 6 샘플 */
#define POWER_ABSOLUTE_MIN_RAW   200u  /* ADC 원시값 절대 하한 (약 1.3V 라인 상당) */

typedef enum { LVL_LOW = 0, LVL_HIGH } level_t;
typedef enum { TR_STABLE = 0, TR_FALLING, TR_RISING } trend_t;

/* ------------------------------------------------------------------------ */
/* 내부 상태                                                                 */
/* ------------------------------------------------------------------------ */
static uint16_t s_tx_hist[TREND_WINDOW_SAMPLES];
static uint16_t s_rx_hist[TREND_WINDOW_SAMPLES];
static uint8_t  s_hist_idx = 0;
static uint8_t  s_hist_filled = 0;   /* TREND_WINDOW_SAMPLES개 채워지기 전에는 추세 판정 보류 */

static level_t  s_tx_level = LVL_LOW;
static level_t  s_rx_level = LVL_HIGH;  /* 대기 상태(Rx High)를 기본 가정으로 초기화 */
static uint8_t  s_level_initialized = 0;

static uint32_t s_last_processed_tick = 0; /* FrontendLink의 last_valid_tick_ms 중복처리 방지용 */

static ip_state_t s_state = IP_STATE_IDLE;

/* ------------------------------------------------------------------------ */
static level_t ClassifyLevel(uint16_t value, uint16_t power_raw, level_t prev)
{
    /* power_raw==0 등 이상치 방어 (1순위 전원단절에서 이미 걸러지지만 안전차원) */
    if (power_raw == 0)
    {
        return prev;
    }

    uint32_t high_th = ((uint32_t)power_raw * LEVEL_HIGH_PCT) / 100u;
    uint32_t low_th  = ((uint32_t)power_raw * LEVEL_LOW_PCT) / 100u;

    if (value >= high_th)
    {
        return LVL_HIGH;
    }
    if (value <= low_th)
    {
        return LVL_LOW;
    }
    return prev; /* 이력구간 - 직전 레벨 유지 */
}

static trend_t ClassifyTrend(uint16_t current, uint16_t past, uint16_t power_raw)
{
    if (power_raw == 0)
    {
        return TR_STABLE;
    }

    int32_t delta = (int32_t)current - (int32_t)past;
    int32_t th = (int32_t)(((uint32_t)power_raw * TREND_PCT) / 100u);

    if (delta <= -th)
    {
        return TR_FALLING;
    }
    if (delta >= th)
    {
        return TR_RISING;
    }
    return TR_STABLE;
}

void StateMachine_Init(void)
{
    for (uint8_t i = 0; i < TREND_WINDOW_SAMPLES; i++)
    {
        s_tx_hist[i] = 0;
        s_rx_hist[i] = 0;
    }
    s_hist_idx = 0;
    s_hist_filled = 0;
    s_level_initialized = 0;
    s_last_processed_tick = 0;
    s_state = IP_STATE_IDLE;
}

void StateMachine_Poll(void)
{
    frontend_data_t d;
    FrontendLink_GetSnapshot(&d);

    /* 아직 유효 프레임을 못 받았거나(부팅 직후), 새 프레임이 아니면 처리 안 함 */
    if (d.last_valid_tick_ms == 0 || d.last_valid_tick_ms == s_last_processed_tick)
    {
        return;
    }
    s_last_processed_tick = d.last_valid_tick_ms;

    /* --- 1순위: 전원단절 (절대 하한) --- */
    if (d.power_raw < POWER_ABSOLUTE_MIN_RAW)
    {
        s_state = IP_STATE_POWERFAIL;
        /* 전원단절 중에는 레벨/이력을 갱신하지 않고 다음 정상 프레임부터 재개 */
        return;
    }

    /* --- 레벨 갱신 (이력구간 시 직전값 유지) --- */
    if (!s_level_initialized)
    {
        /* 최초 1회는 이력구간 개념 없이 바로 분류 (직전값이 없으므로 임의 초기값 기준) */
        s_tx_level = (d.tx_raw >= ((uint32_t)d.power_raw * LEVEL_HIGH_PCT) / 100u) ? LVL_HIGH : LVL_LOW;
        s_rx_level = (d.rx_raw >= ((uint32_t)d.power_raw * LEVEL_HIGH_PCT) / 100u) ? LVL_HIGH : LVL_LOW;
        s_level_initialized = 1;
    }
    else
    {
        s_tx_level = ClassifyLevel(d.tx_raw, d.power_raw, s_tx_level);
        s_rx_level = ClassifyLevel(d.rx_raw, d.power_raw, s_rx_level);
    }

    /* --- 추세 판정 (히스토리 6개 찰 때까지는 STABLE로 간주) --- */
    trend_t tx_trend = TR_STABLE;
    trend_t rx_trend = TR_STABLE;
    if (s_hist_filled)
    {
        /* s_hist_idx 위치가 "가장 오래된(=300ms 전)" 값이 있는 자리 (원형버퍼, 다음에 덮어쓸 자리) */
        tx_trend = ClassifyTrend(d.tx_raw, s_tx_hist[s_hist_idx], d.power_raw);
        rx_trend = ClassifyTrend(d.rx_raw, s_rx_hist[s_hist_idx], d.power_raw);
    }

    /* --- 상태 판정 (2~4순위, 어느 것도 해당 없으면 직전 상태 유지) --- */
    uint8_t tx_low_or_falling = (s_tx_level == LVL_LOW) || (tx_trend == TR_FALLING);

    if (s_tx_level == LVL_HIGH && (rx_trend == TR_FALLING || s_rx_level == LVL_LOW))
    {
        s_state = IP_STATE_TALKING;
    }
    else if (tx_low_or_falling && s_rx_level == LVL_LOW)
    {
        s_state = IP_STATE_RINGING;
    }
    else if (tx_low_or_falling && s_rx_level == LVL_HIGH)
    {
        s_state = IP_STATE_IDLE;
    }
    /* else: 어느 규칙도 해당 없음 -> s_state 직전 값 그대로 유지 */

    /* --- 히스토리 버퍼 갱신 (원형버퍼, 항상 마지막에 수행) --- */
    s_tx_hist[s_hist_idx] = d.tx_raw;
    s_rx_hist[s_hist_idx] = d.rx_raw;
    s_hist_idx++;
    if (s_hist_idx >= TREND_WINDOW_SAMPLES)
    {
        s_hist_idx = 0;
        s_hist_filled = 1;
    }
}

ip_state_t StateMachine_GetState(void)
{
    return s_state;
}
