/**
 * @file    app_monitor.c
 * @brief   구현부 (v2 - StateMachine 연동, PA9 USB 5V 감지 반영)
 *
 * 연동 방법:
 *   1) main.c 에 #include "app_monitor.h" 추가
 *   2) MX_USB_DEVICE_Init() 이후 AppMonitor_Init() 호출
 *      (FrontendLink_Init(), StateMachine_Init()도 함께 호출 - 순서 무관)
 *   3) while(1) 메인 루프에서 AppMonitor_Poll(), StateMachine_Poll() 반복 호출
 *   4) stm32f1xx_it.c 의 HAL_SYSTICK_Callback()에서 AppMonitor_Tick1ms() 호출
 *   5) usbd_cdc_if.c 의 CDC_Receive_FS() 콜백 안에서
 *      AppMonitor_UsbRxHandler(Buf, *Len) 호출
 */

#include "app_monitor.h"
#include "app_state_machine.h"
#include "usbd_cdc_if.h"   /* CDC_Transmit_FS() */

extern IWDG_HandleTypeDef hiwdg;

/* ========================================================================
 * 내부 상태
 * ==================================================================== */

static volatile uint32_t s_last_request_tick_ms = 0;

static volatile uint8_t  s_disconnect_in_progress = 0;
static volatile uint32_t s_disconnect_start_ms     = 0;

typedef enum { WAIT_STX, WAIT_CMD, WAIT_ETX1, WAIT_ETX2 } parse_state_t;
static parse_state_t s_parse_state = WAIT_STX;
static uint8_t        s_parsed_cmd = 0;

typedef enum { LED_PHASE_BOOT, LED_PHASE_NORMAL } led_phase_t;
static led_phase_t s_led_phase = LED_PHASE_BOOT;
static uint8_t     s_led_on = 0;
static uint16_t    s_led_remain_ms = 0;
static uint8_t     s_led_boot_blink_done = 0;

/* ------------------------------------------------------------------------ */
static void GPIO_Config(void);
static void SendStateFrame(void);
static void StartUsbSoftDisconnect(void);
static void ServiceUsbSoftDisconnect(void);
static void ParseIncomingByte(uint8_t byte);
static uint8_t IsUsb5vConnected(void);
static void Led_SetPhysical(uint8_t on);
static void Led_GetPatternForState(uint16_t *on_ms, uint16_t *off_ms);
static void Led_Tick1ms(void);

/* ========================================================================
 * 공개 API
 * ==================================================================== */

void AppMonitor_Init(void)
{
    GPIO_Config();

    s_last_request_tick_ms = HAL_GetTick();
    s_disconnect_in_progress = 0;
    s_parse_state = WAIT_STX;

    /* USB 연결제어 초기 상태: 연결 (PA8 Low) */
    HAL_GPIO_WritePin(USB_CTRL_GPIO_PORT, USB_CTRL_GPIO_PIN, USB_CONNECT_LEVEL);

    /* LED 초기 상태: 부팅 점멸 시작 */
    s_led_phase = LED_PHASE_BOOT;
    s_led_boot_blink_done = 0;
    s_led_on = 1;
    s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
    Led_SetPhysical(1);
}

/* 1ms 주기 호출 - LED, USB 소프트 디스커넥트 타이머만 처리 (가벼운 작업만) */
void AppMonitor_Tick1ms(void)
{
    Led_Tick1ms();
    ServiceUsbSoftDisconnect();
}

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

    if (!IsUsb5vConnected())
    {
        /* 케이블 자체가 없음 - 재연결 시도가 의미 없으므로 타이머만 계속
         * 리셋해둔다. 이렇게 해두면 케이블이 다시 꽂히는 순간부터 정확히
         * REQUEST_TIMEOUT_MS짜리 새 감시 구간이 시작된다(끊겨있던 동안
         * 쌓인 경과시간 때문에 꽂자마자 바로 재연결 펄스가 나가는 것 방지). */
        s_last_request_tick_ms = HAL_GetTick();
        return;
    }

    if (!s_disconnect_in_progress)
    {
        if ((HAL_GetTick() - s_last_request_tick_ms) >= REQUEST_TIMEOUT_MS)
        {
            StartUsbSoftDisconnect();
        }
    }
}

/* ========================================================================
 * 내부 구현 - GPIO
 * ==================================================================== */

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USB 연결 제어 핀: 푸시풀 출력 */
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin   = USB_CTRL_GPIO_PIN;
    HAL_GPIO_Init(USB_CTRL_GPIO_PORT, &gi);

    /* LED 핀: 푸시풀 출력 */
    gi.Pin   = LED_GPIO_PIN;
    HAL_GPIO_Init(LED_GPIO_PORT, &gi);

    /* USB 5V 감지 핀: 입력 (저항분압 외부회로이므로 내부 풀 불필요) */
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    gi.Pin  = USB5V_GPIO_PIN;
    HAL_GPIO_Init(USB5V_GPIO_PORT, &gi);
}

static uint8_t IsUsb5vConnected(void)
{
    return (HAL_GPIO_ReadPin(USB5V_GPIO_PORT, USB5V_GPIO_PIN) == USB5V_CONNECTED_LEVEL) ? 1u : 0u;
}

/* ========================================================================
 * 내부 구현 - LED 상태머신
 * ==================================================================== */

static void Led_SetPhysical(uint8_t on)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

/* USB 케이블 미연결이 최우선 - 그 외에는 StateMachine이 판정한 인터폰 상태를 따름 */
static void Led_GetPatternForState(uint16_t *on_ms, uint16_t *off_ms)
{
    if (!IsUsb5vConnected())
    {
        *on_ms = LED_USBDISC_ON_MS;
        *off_ms = LED_USBDISC_OFF_MS;
        return;
    }

    switch (StateMachine_GetState())
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
            s_led_on = 0;
            Led_SetPhysical(0);
            s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
        }
        else
        {
            s_led_boot_blink_done++;
            if (s_led_boot_blink_done >= LED_BOOT_BLINK_COUNT)
            {
                s_led_phase = LED_PHASE_NORMAL;
                s_led_on = 1;
                Led_SetPhysical(1);
                {
                    uint16_t on_ms, off_ms;
                    Led_GetPatternForState(&on_ms, &off_ms);
                    s_led_remain_ms = on_ms;
                }
            }
            else
            {
                s_led_on = 1;
                Led_SetPhysical(1);
                s_led_remain_ms = LED_BOOT_HALF_PERIOD_MS;
            }
        }
    }
    else /* LED_PHASE_NORMAL */
    {
        uint16_t on_ms, off_ms;
        Led_GetPatternForState(&on_ms, &off_ms);

        s_led_on = !s_led_on;
        Led_SetPhysical(s_led_on);
        s_led_remain_ms = s_led_on ? on_ms : off_ms;
    }
}

/* ========================================================================
 * 내부 구현 - USB 응답 프레임 생성 (StateMachine 판정 결과를 CH1~4로 역변환)
 *
 * 기존 Android/Windows 호스트 파싱 로직(전원단절>통화>호출>대기)과의
 * 호환을 위해, 판정된 ip_state_t를 그 로직이 정확히 같은 결과로
 * 복원되도록 CH1~3 조합으로 인코딩한다.
 * ==================================================================== */

static void SendStateFrame(void)
{
    uint8_t frame[8];
    uint8_t ch1, ch2, ch3;
    const uint8_t ch4 = 0; /* 하드웨어 고정 스페어, 항상 Low */

    switch (StateMachine_GetState())
    {
        case IP_STATE_POWERFAIL:
            ch3 = 0; ch1 = 0; ch2 = 0;
            break;
        case IP_STATE_TALKING:
            ch3 = 1; ch1 = 1; ch2 = 1; /* ch2는 호스트 디코드상 don't-care */
            break;
        case IP_STATE_RINGING:
            ch3 = 1; ch1 = 0; ch2 = 0;
            break;
        case IP_STATE_IDLE:
        default:
            ch3 = 1; ch1 = 0; ch2 = 1;
            break;
    }

    uint8_t sum_index = (uint8_t)((ch1 ? 1 : 0) + (ch2 ? 2 : 0) + (ch3 ? 3 : 0) + (ch4 ? 4 : 0));

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
 * 내부 구현 - USB 소프트 디스커넥트/재연결 (10초 무응답 시, 3초간 끊고
 * 재연결, USB5V 미연결 시에는 AppMonitor_Poll()에서 애초에 호출되지 않음)
 *   PA8 High = USB 끊기, PA8 Low = USB 연결
 * ==================================================================== */

static void StartUsbSoftDisconnect(void)
{
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
            HAL_GPIO_WritePin(USB_CTRL_GPIO_PORT, USB_CTRL_GPIO_PIN, USB_CONNECT_LEVEL);
            s_disconnect_in_progress = 0;
            s_last_request_tick_ms = HAL_GetTick();
        }
    }
}
