/**
 * @file    app_monitor.c
 * @brief   엘리베이터 비상통화장치 상태감지 모듈 - 구현부
 *
 * 연동 방법 (STM32CubeMX HAL 프로젝트 기준):
 *   1) main.c 에 #include "app_monitor.h" 추가
 *   2) MX_USB_DEVICE_Init() 이후 AppMonitor_Init() 호출
 *   3) while(1) 메인 루프 안에서 AppMonitor_Poll() 반복 호출
 *   4) stm32f1xx_it.c 의 HAL_SYSTICK_Callback()에서 AppMonitor_Tick1ms() 호출
 *      (HAL_SYSTICK_Callback은 weak 함수이므로 재정의만 하면 됨)
 *   5) usbd_cdc_if.c 의 CDC_Receive_FS() 콜백 안에서
 *      AppMonitor_UsbRxHandler(Buf, *Len) 호출
 *   6) IWDG는 CubeMX에서 활성화하고, AppMonitor_Poll() 안에서 refresh 됨
 *      (hiwdg 핸들이 CubeMX에 의해 생성되어 있다고 가정)
 */

#include "app_monitor.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS() */
#include <string.h>

extern IWDG_HandleTypeDef hiwdg;

/* ========================================================================
 * 내부 상태 자료구조
 * ==================================================================== */

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       raw;          /* 이번 틱에서 읽은 순간값 (0/1) */
    uint8_t       confirmed;    /* 디바운스 확정값 (0/1) */
    uint16_t      stable_ms;    /* raw != confirmed 상태가 지속된 시간(ms) */
    uint16_t      debounce_ms;  /* 이 채널에 적용할 디바운스 시간 */
} channel_t;

static channel_t s_ch1; /* Tx_state */
static channel_t s_ch2; /* Rx_state */
static channel_t s_ch3; /* Power_present */

static volatile uint16_t s_rx_lockout_remain_ms = 0; /* 통화종료 후 Rx 마스킹 잔여시간 */
static volatile uint8_t  s_ch1_prev_confirmed   = 0;
static volatile uint8_t  s_ch1_initialized      = 0;

static volatile uint32_t s_last_request_tick_ms = 0;

/* USB 소프트 디스커넥트 진행 상태 */
static volatile uint8_t  s_disconnect_in_progress = 0;
static volatile uint32_t s_disconnect_start_ms     = 0;

/* 요청 프레임 파서 상태 */
typedef enum { WAIT_STX, WAIT_CMD, WAIT_ETX1, WAIT_ETX2 } parse_state_t;
static parse_state_t s_parse_state = WAIT_STX;
static uint8_t        s_parsed_cmd = 0;

/* 인터폰 표시 상태 (LED 표시용, 매 tick 갱신) */
static volatile interphone_state_t s_ip_state = IP_STATE_IDLE;

/* LED 상태머신 */
typedef enum { LED_PHASE_BOOT, LED_PHASE_NORMAL } led_phase_t;
static led_phase_t s_led_phase = LED_PHASE_BOOT;
static uint8_t     s_led_on = 0;
static uint16_t    s_led_remain_ms = 0;
static uint8_t     s_led_boot_blink_done = 0; /* 완료된 부팅 점멸 횟수 */

/* ------------------------------------------------------------------------ */
/* 내부 함수 원형                                                            */
/* ------------------------------------------------------------------------ */
static void GPIO_Config(void);
static void Channel_Init(channel_t *ch, GPIO_TypeDef *port, uint16_t pin, uint16_t debounce_ms);
static void Channel_Sample1ms(channel_t *ch);
static void SendStateFrame(void);
static void StartUsbSoftDisconnect(void);
static void ServiceUsbSoftDisconnect(void);
static void ParseIncomingByte(uint8_t byte);
static void UpdateInterphoneState(uint8_t ch1, uint8_t ch2_effective, uint8_t ch3);
static void Led_SetPhysical(uint8_t on);
static void Led_GetPatternForState(interphone_state_t st, uint16_t *on_ms, uint16_t *off_ms);
static void Led_Tick1ms(void);

/* ========================================================================
 * 공개 API
 * ==================================================================== */

void AppMonitor_Init(void)
{
    GPIO_Config();

    Channel_Init(&s_ch1, CH1_GPIO_PORT, CH1_GPIO_PIN, DEBOUNCE_TX_MS);
    Channel_Init(&s_ch2, CH2_GPIO_PORT, CH2_GPIO_PIN, DEBOUNCE_RX_MS);
    Channel_Init(&s_ch3, CH3_GPIO_PORT, CH3_GPIO_PIN, DEBOUNCE_POWER_MS);

    s_rx_lockout_remain_ms = 0;
    s_ch1_initialized      = 0;
    s_last_request_tick_ms = HAL_GetTick();
    s_disconnect_in_progress = 0;
    s_parse_state = WAIT_STX;

    /* USB 초기 상태: 연결 (PA8 Low) */
    HAL_GPIO_WritePin(USB_CTRL_GPIO_PORT, USB_CTRL_GPIO_PIN, USB_CONNECT_LEVEL);

    /* LED 초기 상태: 부팅 점멸 시작 (켜짐부터 시작) */
    s_led_phase = LED_PHASE_BOOT;
    s_led_boot_blink_done = 0;
    s_led_on = 1;
    s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
    Led_SetPhysical(1);

    UpdateInterphoneState(s_ch1.confirmed, s_ch2.confirmed, s_ch3.confirmed);
}

/* 1ms 주기 호출 - 디바운스 샘플링, 내부 타이머, LED 갱신 (가벼운 작업만) */
void AppMonitor_Tick1ms(void)
{
    uint8_t ch1_prev_confirmed = s_ch1.confirmed;

    Channel_Sample1ms(&s_ch1);
    Channel_Sample1ms(&s_ch2);
    Channel_Sample1ms(&s_ch3);

    /* CH1(Tx) 확정값이 최초로 정해졌을 때 기준점 셋업 (부팅 직후 오검출 방지) */
    if (!s_ch1_initialized)
    {
        s_ch1_prev_confirmed = s_ch1.confirmed;
        s_ch1_initialized = 1;
    }
    else if (ch1_prev_confirmed != s_ch1.confirmed)
    {
        if (ch1_prev_confirmed == 0 && s_ch1.confirmed == 1)
        {
            /* L->H : 통화 시작. 별도 락아웃 불필요 */
        }
        else if (ch1_prev_confirmed == 1 && s_ch1.confirmed == 0)
        {
            /* H->L : 통화 종료. Rx 뮤트 트랜지언트 구간 동안 CH2 보고값 마스킹 시작 */
            s_rx_lockout_remain_ms = RX_LOCKOUT_MS;
        }
        s_ch1_prev_confirmed = s_ch1.confirmed;
    }

    if (s_rx_lockout_remain_ms > 0)
    {
        s_rx_lockout_remain_ms--;
    }

    {
        uint8_t ch2_effective = s_ch2.confirmed;
        if (s_rx_lockout_remain_ms > 0)
        {
            ch2_effective = 1; /* 락아웃 중에는 대기 예상값(High)으로 간주 */
        }
        UpdateInterphoneState(s_ch1.confirmed, ch2_effective, s_ch3.confirmed);
    }

    Led_Tick1ms();
    ServiceUsbSoftDisconnect();
}

/* USB CDC 수신 콜백에서 호출 */
void AppMonitor_UsbRxHandler(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        ParseIncomingByte(buf[i]);
    }
}

/* main 루프에서 반복 호출 */
void AppMonitor_Poll(void)
{
#ifdef HAL_IWDG_MODULE_ENABLED
    HAL_IWDG_Refresh(&hiwdg);
#endif

    /* 마지막 요청(또는 마지막 재연결 시점)으로부터 10초 이상 요청이 없으면
     * 계속 반복해서 재연결 시뮬레이션을 시도한다 (한 번으로 끝나지 않음) */
    if (!s_disconnect_in_progress)
    {
        if ((HAL_GetTick() - s_last_request_tick_ms) >= REQUEST_TIMEOUT_MS)
        {
            StartUsbSoftDisconnect();
        }
    }
}

/* ========================================================================
 * 내부 구현 - GPIO / 채널 디바운스
 * ==================================================================== */

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CH1~CH3: 아이솔레이터 2차측 출력 입력 (push-pull 출력이므로 풀 저항 불필요) */
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;

    gi.Pin = CH1_GPIO_PIN;
    HAL_GPIO_Init(CH1_GPIO_PORT, &gi);

    gi.Pin = CH2_GPIO_PIN;
    HAL_GPIO_Init(CH2_GPIO_PORT, &gi);

    gi.Pin = CH3_GPIO_PIN;
    HAL_GPIO_Init(CH3_GPIO_PORT, &gi);

    /* USB 연결 제어 핀: 푸시풀 출력 (초기값은 AppMonitor_Init에서 설정) */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = USB_CTRL_GPIO_PIN;
    HAL_GPIO_Init(USB_CTRL_GPIO_PORT, &gi);

    /* LED 핀: 푸시풀 출력 */
    gi.Pin   = LED_GPIO_PIN;
    HAL_GPIO_Init(LED_GPIO_PORT, &gi);
}

static void Channel_Init(channel_t *ch, GPIO_TypeDef *port, uint16_t pin, uint16_t debounce_ms)
{
    memset(ch, 0, sizeof(*ch));
    ch->port = port;
    ch->pin = pin;
    ch->debounce_ms = debounce_ms;

    uint8_t v = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
    ch->raw = v;
    ch->confirmed = v;
    ch->stable_ms = 0;
}

static void Channel_Sample1ms(channel_t *ch)
{
    uint8_t v = (HAL_GPIO_ReadPin(ch->port, ch->pin) == GPIO_PIN_SET) ? 1u : 0u;

    if (v == ch->raw)
    {
        if (v != ch->confirmed)
        {
            ch->stable_ms++;
            if (ch->stable_ms >= ch->debounce_ms)
            {
                ch->confirmed = v;
                ch->stable_ms = 0;
            }
        }
    }
    else
    {
        ch->raw = v;
        ch->stable_ms = 0;
    }
}

/* ========================================================================
 * 내부 구현 - 인터폰 표시 상태 판정 (LED 용)
 * ==================================================================== */

static void UpdateInterphoneState(uint8_t ch1, uint8_t ch2_effective, uint8_t ch3)
{
    interphone_state_t st;

    if (ch3 == 0)
    {
        st = IP_STATE_POWERFAIL;          /* 1순위: 전원단절 */
    }
    else if (ch1 == 1)
    {
        st = IP_STATE_TALKING;            /* Tx High = 통화 중 */
    }
    else if (ch2_effective == 0)
    {
        st = IP_STATE_RINGING;            /* Tx Low, Rx Low = 호출 */
    }
    else
    {
        st = IP_STATE_IDLE;               /* Tx Low, Rx High = 대기 */
    }

    s_ip_state = st;
}

/* ========================================================================
 * 내부 구현 - LED 상태머신
 * ==================================================================== */

static void Led_SetPhysical(uint8_t on)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

static void Led_GetPatternForState(interphone_state_t st, uint16_t *on_ms, uint16_t *off_ms)
{
    switch (st)
    {
        case IP_STATE_IDLE:
            *on_ms = LED_IDLE_ON_MS;  *off_ms = LED_IDLE_OFF_MS;  break;
        case IP_STATE_RINGING:
            *on_ms = LED_RING_ON_MS;  *off_ms = LED_RING_OFF_MS;  break;
        case IP_STATE_TALKING:
            *on_ms = LED_TALK_ON_MS;  *off_ms = LED_TALK_OFF_MS;  break;
        case IP_STATE_POWERFAIL:
        default:
            *on_ms = LED_FAULT_ON_MS; *off_ms = LED_FAULT_OFF_MS; break;
    }
}

/* 매 1ms 호출. 부팅 시퀀스(0.7초 주기 3회 점멸) 완료 후 인터폰 상태별 패턴으로 전환 */
static void Led_Tick1ms(void)
{
    if (s_led_remain_ms > 0)
    {
        s_led_remain_ms--;
        return;
    }

    if (s_led_phase == LED_PHASE_BOOT)
    {
        if (s_led_on)
        {
            /* 방금 ON 구간이 끝남 -> OFF 구간 시작 */
            s_led_on = 0;
            Led_SetPhysical(0);
            s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
        }
        else
        {
            /* 방금 OFF 구간이 끝남 -> 한 번의 점멸(ON+OFF) 완료 */
            s_led_boot_blink_done++;
            if (s_led_boot_blink_done >= LED_BOOT_BLINK_COUNT)
            {
                /* 부팅 점멸 종료, 정상 표시 모드로 전환 */
                s_led_phase = LED_PHASE_NORMAL;
                s_led_on = 1;
                Led_SetPhysical(1);
                {
                    uint16_t on_ms, off_ms;
                    Led_GetPatternForState(s_ip_state, &on_ms, &off_ms);
                    s_led_remain_ms = on_ms;
                }
            }
            else
            {
                /* 다음 점멸 사이클 ON 시작 */
                s_led_on = 1;
                Led_SetPhysical(1);
                s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
            }
        }
    }
    else /* LED_PHASE_NORMAL */
    {
        uint16_t on_ms, off_ms;
        Led_GetPatternForState(s_ip_state, &on_ms, &off_ms);

        s_led_on = !s_led_on;
        Led_SetPhysical(s_led_on);
        s_led_remain_ms = s_led_on ? on_ms : off_ms;
    }
}

/* ========================================================================
 * 내부 구현 - 응답 프레임 생성/전송
 * ==================================================================== */

static void SendStateFrame(void)
{
    uint8_t frame[8];
    uint8_t ch1 = s_ch1.confirmed;
    uint8_t ch2 = s_ch2.confirmed;
    uint8_t ch3 = s_ch3.confirmed;
    uint8_t ch4 = 0; /* 하드웨어 GND 고정, 항상 Low */

    if (s_rx_lockout_remain_ms > 0)
    {
        ch2 = 1; /* 통화종료 직후 락아웃 구간: Rx 트랜지언트 숨기고 대기 예상값으로 보고 */
    }

    uint8_t sum_index = (ch1 ? 1 : 0) + (ch2 ? 2 : 0) + (ch3 ? 3 : 0) + (ch4 ? 4 : 0);

    frame[0] = FRAME_STX;
    frame[1] = ch1 ? STATE_HIGH_BYTE : STATE_LOW_BYTE;
    frame[2] = ch2 ? STATE_HIGH_BYTE : STATE_LOW_BYTE;
    frame[3] = ch3 ? STATE_HIGH_BYTE : STATE_LOW_BYTE;
    frame[4] = ch4 ? STATE_HIGH_BYTE : STATE_LOW_BYTE;
    frame[5] = (uint8_t)(0x30 + sum_index);
    frame[6] = FRAME_ETX1;
    frame[7] = FRAME_ETX2;

    CDC_Transmit_FS(frame, sizeof(frame));
}

/* ========================================================================
 * 내부 구현 - 요청 프레임 파서
 * ==================================================================== */

static void ParseIncomingByte(uint8_t byte)
{
    switch (s_parse_state)
    {
        case WAIT_STX:
            if (byte == FRAME_STX)
            {
                s_parse_state = WAIT_CMD;
            }
            break;

        case WAIT_CMD:
            s_parsed_cmd = byte;
            s_parse_state = WAIT_ETX1;
            break;

        case WAIT_ETX1:
            if (byte == FRAME_ETX1)
            {
                s_parse_state = WAIT_ETX2;
            }
            else
            {
                s_parse_state = (byte == FRAME_STX) ? WAIT_CMD : WAIT_STX;
            }
            break;

        case WAIT_ETX2:
            if (byte == FRAME_ETX2 && s_parsed_cmd == CMD_REQUEST_STATE)
            {
                s_last_request_tick_ms = HAL_GetTick();
                SendStateFrame();
            }
            s_parse_state = WAIT_STX;
            break;

        default:
            s_parse_state = WAIT_STX;
            break;
    }
}

/* ========================================================================
 * 내부 구현 - USB 소프트 디스커넥트/재연결 (10초 무응답 시, 3초간 끊고 재연결, 반복)
 *   PA8 High = USB 끊기, PA8 Low = USB 연결
 * ==================================================================== */

static void StartUsbSoftDisconnect(void)
{
    /* 순서 주의: 플래그(s_disconnect_in_progress)는 반드시 마지막에 세운다.
     * 이 함수는 메인 루프에서 호출되고, ServiceUsbSoftDisconnect()는 1ms SysTick
     * 인터럽트에서 호출되므로, 플래그를 먼저 세우면 타임스탬프/GPIO가 아직
     * 갱신되지 않은 상태에서 인터럽트가 끼어들어 오동작할 수 있다. */
    s_disconnect_start_ms = HAL_GetTick();
    HAL_GPIO_WritePin(USB_CTRL_GPIO_PORT, USB_CTRL_GPIO_PIN, USB_DISCONNECT_LEVEL);
    s_disconnect_in_progress = 1;
}

static void ServiceUsbSoftDisconnect(void)
{
    if (s_disconnect_in_progress)
    {
        if ((HAL_GetTick() - s_disconnect_start_ms) >= USB_DISCONNECT_HOLD_MS)
        {
            /* PA8 Low = 연결 복귀 */
            HAL_GPIO_WritePin(USB_CTRL_GPIO_PORT, USB_CTRL_GPIO_PIN, USB_CONNECT_LEVEL);
            s_disconnect_in_progress = 0;

            /* 재연결 시점부터 곧바로 다음 10초 감시 창을 시작한다.
             * 호스트가 여전히 응답하지 않으면 이 사이클이 계속 반복된다. */
            s_last_request_tick_ms = HAL_GetTick();
        }
    }
}
