/**
 * @file    app_ver_led.c
 * @brief   부팅 직후 LED 버전 표시 시퀀서 (app_ver_led.h 참조)
 */

#include "app_ver_led.h"

typedef enum
{
    VP_LEAD = 0,   /* 2초 OFF */
    VP_MAJOR,      /* Major 번호 */
    VP_GAP1,       /* 1초 OFF */
    VP_MINOR,      /* Minor 번호 */
    VP_GAP2,       /* 1초 OFF */
    VP_DONE
} vp_t;

static vp_t     s_phase = VP_DONE;
static uint16_t s_tick_ms = 1u;
static uint16_t s_remain = 0u;        /* 현재 구간의 남은 틱 수 */
static uint16_t s_blinks_left = 0u;   /* 번호 표시 중 남은 세트 수 */
static uint8_t  s_level = VERLED_OFF; /* 현재 구간의 LED 레벨 */

static uint16_t Ticks(uint16_t ms)
{
    uint16_t t = (uint16_t)(ms / s_tick_ms);
    return (t == 0u) ? 1u : t;
}

static void EnterPhase(vp_t p);

static void EnterNumber(vp_t p, uint16_t n)
{
    s_phase = p;
    s_blinks_left = n;
    if (n == 0u)
    {
        EnterPhase((vp_t)(p + 1));   /* 번호 0: 깜빡임 없이 다음 구간으로 */
    }
    else
    {
        s_level = VERLED_ON;
        s_remain = Ticks(VERLED_BLINK_ON_MS);
    }
}

static void EnterPhase(vp_t p)
{
    switch (p)
    {
        case VP_LEAD:
            s_phase = p;
            s_level = VERLED_OFF;
            s_remain = Ticks(VERLED_LEAD_OFF_MS);
            break;
        case VP_MAJOR:
            EnterNumber(p, FW_VERSION_MAJOR);
            break;
        case VP_GAP1:
        case VP_GAP2:
            s_phase = p;
            s_level = VERLED_OFF;
            s_remain = Ticks(VERLED_GAP_OFF_MS);
            break;
        case VP_MINOR:
            EnterNumber(p, FW_VERSION_MINOR);
            break;
        default:
            s_phase = VP_DONE;
            break;
    }
}

/* 현재 구간이 끝났을 때 다음 구간을 준비한다 */
static void Advance(void)
{
    switch (s_phase)
    {
        case VP_LEAD:
        case VP_GAP1:
        case VP_GAP2:
            EnterPhase((vp_t)(s_phase + 1));
            break;

        case VP_MAJOR:
        case VP_MINOR:
            if (s_level == VERLED_ON)
            {
                /* ON 0.1초 끝 -> OFF 0.3초 */
                s_level = VERLED_OFF;
                s_remain = Ticks(VERLED_BLINK_OFF_MS);
            }
            else
            {
                /* OFF 0.3초 끝 -> 세트 1개 완료 */
                s_blinks_left--;
                if (s_blinks_left > 0u)
                {
                    s_level = VERLED_ON;
                    s_remain = Ticks(VERLED_BLINK_ON_MS);
                }
                else
                {
                    EnterPhase((vp_t)(s_phase + 1));
                }
            }
            break;

        default:
            s_phase = VP_DONE;
            break;
    }
}

void VerLed_Start(uint16_t tick_ms)
{
    s_tick_ms = (tick_ms == 0u) ? 1u : tick_ms;
    EnterPhase(VP_LEAD);
}

uint8_t VerLed_Tick(void)
{
    uint8_t out;

    if (s_phase == VP_DONE)
    {
        return VERLED_DONE;
    }

    out = s_level;
    s_remain--;
    if (s_remain == 0u)
    {
        Advance();
    }
    return out;
}
