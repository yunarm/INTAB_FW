/**
 * @file    app_state_machine.c
 * @brief   구현부 (v3, 2026-10-16 - 상태 다이어그램 기반 판정, 통신_프로토콜_정리.md v1.5)
 *
 * 연동 방법:
 *   1) main.c 에 #include "app_state_machine.h" 추가
 *   2) FrontendLink_Init() 이후 StateMachine_Init() 호출
 *   3) while(1) 메인 루프에서 StateMachine_Poll() 반복 호출
 *      (내부에서 프론트엔드의 새 프레임 도착 여부를 자체 감지하므로,
 *       메인 루프 주기가 50ms보다 빨라도 문제없음)
 *   4) 판정 결과가 필요한 곳에서 호출
 *        - LED               : StateMachine_GetState()
 *        - USB 응답 프레임   : StateMachine_GetResult()  (상태+Tx‰+Rx‰+Power를 일관되게 복사)
 */

#include "app_state_machine.h"
#include "app_frontend_link.h"

/* ------------------------------------------------------------------------ */
/* 파라미터 (필요 시 튜닝)                                                    */
/* ------------------------------------------------------------------------ */
#define TX_HIGH_PM               750u  /* Tx  > Power의 75.0% 이면 TxH */
#define RX_LOW_PM                380u  /* Rx  < Power의 38.0% 이면 RxL */
#define RX_HIGH_PM               750u  /* Rx  > Power의 75.0% 이면 RxH (호출→대기 복귀 확인) */
#define DWELL_MS                 400u  /* 지정시간: 조건이 연속으로 참이어야 하는 시간 */
#define POWER_ABSOLUTE_MIN_RAW   200u  /* Power ADC 절대 하한 (약 1.3V 라인 상당) */

/* 전압 환산: Power_dv = Power_ADC × 3.3 × 7.98 × 10 / 4095 = Power_ADC × 26334 / 409500 */
#define POWER_DV_NUM             26334u
#define POWER_DV_DEN            409500u

#define NO_PENDING               0xFFu

/* ------------------------------------------------------------------------ */
/* 내부 상태                                                                 */
/* ------------------------------------------------------------------------ */
static ip_state_t s_state = IP_STATE_IDLE;

static uint8_t    s_pending = NO_PENDING;   /* 지정시간 대기 중인 천이 목표 상태 */
static uint32_t   s_pending_start_ms = 0;   /* 그 조건이 처음 참이 된 프레임의 tick */

static uint32_t   s_last_processed_tick = 0; /* FrontendLink의 last_valid_tick_ms 중복처리 방지용 */
static uint32_t   s_init_tick = 0;           /* 부팅 직후 유예 판단용 */

static volatile sm_result_t s_result;        /* 외부 공개용 (임계구역에서만 갱신) */

/* ------------------------------------------------------------------------ */
static uint16_t ToPerMille(uint16_t value, uint16_t power_raw)
{
    /* power_raw >= POWER_ABSOLUTE_MIN_RAW 인 경우에만 호출됨 (0 나눗셈 없음) */
    uint32_t pm = (((uint32_t)value * 1000u) + (power_raw / 2u)) / power_raw;
    return (pm > 1000u) ? 1000u : (uint16_t)pm;
}

static uint16_t ToDeciVolt(uint16_t power_raw)
{
    return (uint16_t)((((uint32_t)power_raw * POWER_DV_NUM) + (POWER_DV_DEN / 2u)) / POWER_DV_DEN);
}

/* 상태+값을 USB 인터럽트와 경합 없이 한 번에 갱신 */
static void Publish(ip_state_t st, uint16_t tx_pm, uint16_t rx_pm, uint16_t power_dv)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_result.state    = st;
    s_result.tx_pm    = tx_pm;
    s_result.rx_pm    = rx_pm;
    s_result.power_dv = power_dv;
    __set_PRIMASK(primask);
}

static void ClearPending(void)
{
    s_pending = NO_PENDING;
}

void StateMachine_Init(void)
{
    s_state = IP_STATE_IDLE;
    s_pending = NO_PENDING;
    s_pending_start_ms = 0;
    s_last_processed_tick = 0;
    s_init_tick = HAL_GetTick();
    Publish(IP_STATE_IDLE, 0, 0, 0);
}

void StateMachine_Poll(void)
{
    frontend_data_t d;
    FrontendLink_GetSnapshot(&d);

    /* --- 링크 이상: 프레임이 아예 안 오거나(부팅 유예 이후) 끊김 --- */
    if (d.last_valid_tick_ms == 0)
    {
        /* 부팅 이후 아직 한 번도 못 받음: 유예시간 동안은 대기, 지나면 링크 이상 */
        if ((HAL_GetTick() - s_init_tick) >= FL_LINK_TIMEOUT_MS)
        {
            s_state = IP_STATE_LINKFAIL;
            ClearPending();
            Publish(IP_STATE_LINKFAIL, 0, 0, 0);
        }
        return;
    }
    if (FrontendLink_IsStale())
    {
        s_state = IP_STATE_LINKFAIL;
        ClearPending();
        Publish(IP_STATE_LINKFAIL, 0, 0, 0);
        return;
    }

    /* 새 프레임이 아니면 처리 안 함 */
    if (d.last_valid_tick_ms == s_last_processed_tick)
    {
        return;
    }
    s_last_processed_tick = d.last_valid_tick_ms;
    const uint32_t now = d.last_valid_tick_ms;

    /* --- 전원 이상 (절대 하한, 최우선) --- */
    if (d.power_raw < POWER_ABSOLUTE_MIN_RAW)
    {
        s_state = IP_STATE_POWERFAIL;
        ClearPending();
        /* Power가 거의 0이라 비율이 의미 없음: Tx/Rx는 0, Power는 실측값 */
        Publish(IP_STATE_POWERFAIL, 0, 0, ToDeciVolt(d.power_raw));
        return;
    }

    /* 링크/전원 이상에서 복구되면 대기에서 재시작 */
    if (s_state == IP_STATE_POWERFAIL || s_state == IP_STATE_LINKFAIL)
    {
        s_state = IP_STATE_IDLE;
        ClearPending();
    }

    /* --- 입력 환산 --- */
    const uint16_t tx_pm = ToPerMille(d.tx_raw, d.power_raw);
    const uint16_t rx_pm = ToPerMille(d.rx_raw, d.power_raw);
    const uint16_t pw_dv = ToDeciVolt(d.power_raw);

    const uint8_t tx_h     = (tx_pm > TX_HIGH_PM) ? 1u : 0u;
    const uint8_t tx_l     = (uint8_t)(1u - tx_h);
    const uint8_t rx_l     = (rx_pm < RX_LOW_PM) ? 1u : 0u;
    const uint8_t rx_h     = (rx_pm > RX_HIGH_PM) ? 1u : 0u;
    const uint8_t tx_gt_rx = (tx_pm > rx_pm) ? 1u : 0u;
    const uint8_t rx_gt_tx = (rx_pm > tx_pm) ? 1u : 0u;

    /* --- 천이 판정 (현재 상태에서 나가는 조건만 평가) --- */
    ip_state_t immediate = s_state;     /* 지정시간 없는 천이 결과 */
    uint8_t    dwell_target = NO_PENDING; /* 지정시간이 필요한 천이 목표 */

    switch (s_state)
    {
        case IP_STATE_IDLE:
            if (tx_h && tx_gt_rx)      { dwell_target = IP_STATE_TALKING; }  /* ① */
            else if (tx_l && rx_l)     { dwell_target = IP_STATE_RINGING; }  /* ③ */
            break;

        case IP_STATE_RINGING:
            if (tx_h && tx_gt_rx)      { dwell_target = IP_STATE_TALKING; }  /* ④ (=①) */
            else if (rx_gt_tx && rx_h) { immediate = IP_STATE_IDLE; }        /* Rx 복귀 */
            break;

        case IP_STATE_TALKING:
            /* RxL(호출)이 "Rx>Tx(대기)"보다 우선 → 호출 조건을 먼저 검사 */
            if (tx_l && rx_l)          { dwell_target = IP_STATE_RINGING; }  /* 통화→호출 */
            else if (rx_gt_tx && tx_l) { immediate = IP_STATE_IDLE; }        /* ② ⑤ */
            break;

        default:
            s_state = IP_STATE_IDLE;
            break;
    }

    if (immediate != s_state)
    {
        s_state = immediate;
        ClearPending();
    }
    else if (dwell_target != NO_PENDING)
    {
        if (s_pending != dwell_target)
        {
            /* 조건이 새로 참이 됨(또는 목표가 바뀜): 타이머 시작 */
            s_pending = dwell_target;
            s_pending_start_ms = now;
        }
        if ((now - s_pending_start_ms) >= DWELL_MS)
        {
            s_state = (ip_state_t)dwell_target;
            ClearPending();
        }
    }
    else
    {
        ClearPending(); /* 조건이 끊기면 타이머 초기화 */
    }

    Publish(s_state, tx_pm, rx_pm, pw_dv);
}

ip_state_t StateMachine_GetState(void)
{
    return s_result.state;
}

void StateMachine_GetResult(sm_result_t *out)
{
    if (out == 0)
    {
        return;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->state    = s_result.state;
    out->tx_pm    = s_result.tx_pm;
    out->rx_pm    = s_result.rx_pm;
    out->power_dv = s_result.power_dv;
    __set_PRIMASK(primask);
}
