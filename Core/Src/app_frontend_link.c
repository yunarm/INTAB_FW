/**
 * @file    app_frontend_link.c
 * @brief   구현부
 *
 * 연동 방법:
 *   1) main.c 에 #include "app_frontend_link.h" 추가
 *   2) MX_USART1_UART_Init() 이후 FrontendLink_Init() 호출
 *   3) stm32f1xx_it.c 또는 별도 콜백 파일에 아래 추가:
 *        void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *        {
 *            FrontendLink_OnByteReceived(huart);
 *        }
 *      (다른 UART 콜백과 함께 쓰는 경우, huart->Instance 로 분기 필요)
 *   3-1) 같은 파일에 오류 복구 콜백도 추가 (v1.1, 2026-10-16):
 *        void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 *        {
 *            FrontendLink_OnError(huart);
 *        }
 *      오버런이 나면 HAL이 수신을 중단하고 스스로 재개하지 않기 때문에 필수.
 *   4) 상태 판정 로직(app_state_machine.c)에서는 FrontendLink_GetSnapshot() 또는
 *      FrontendLink_IsStale() 로 최신 데이터를 읽어가면 됨
 */

#include "app_frontend_link.h"
#include <string.h>

extern UART_HandleTypeDef huart1;

/* 파서 상태머신 */
typedef enum
{
    FL_WAIT_HEADER = 0,
    FL_WAIT_DATA,       /* 6개 데이터 바이트(인덱스 1~6) 수신 중 */
    FL_WAIT_CHECKSUM,
    FL_WAIT_TRAILER1,
    FL_WAIT_TRAILER2
} fl_parse_state_t;

static fl_parse_state_t s_state = FL_WAIT_HEADER;
static uint8_t  s_data_buf[6];     /* CH0~CH2 상/하위바이트 6개 */
static uint8_t  s_data_idx = 0;
static uint8_t  s_rx_byte;         /* HAL_UART_Receive_IT용 1바이트 수신 버퍼 */

/* 공개 데이터 - 인터럽트에서 갱신, GetSnapshot()으로만 외부에 노출 */
static volatile frontend_data_t s_data = {0};

static void ResetParser(void);
static void OnFrameValid(void);
static void OnFrameError(void);
static void RearmReceive(void);

void FrontendLink_Init(void)
{
    memset((void *)&s_data, 0, sizeof(s_data));
    ResetParser();
    RearmReceive();
}

/* USART1 1바이트 수신 완료 인터럽트에서 호출됨 */
void FrontendLink_OnByteReceived(UART_HandleTypeDef *huart)
{
    if (huart->Instance != huart1.Instance)
    {
        return;
    }

    uint8_t byte = s_rx_byte;

    switch (s_state)
    {
        case FL_WAIT_HEADER:
            if (byte == FL_FRAME_HEADER)
            {
                s_data_idx = 0;
                s_state = FL_WAIT_DATA;
            }
            /* 헤더가 아니면 계속 헤더를 찾으며 대기 (버림) */
            break;

        case FL_WAIT_DATA:
            s_data_buf[s_data_idx++] = byte;
            if (s_data_idx >= 6u)
            {
                s_state = FL_WAIT_CHECKSUM;
            }
            break;

        case FL_WAIT_CHECKSUM:
        {
            uint8_t chk = 0;
            for (uint8_t i = 0; i < 6u; i++)
            {
                chk ^= s_data_buf[i];
            }
            if (chk == byte)
            {
                s_state = FL_WAIT_TRAILER1;
            }
            else
            {
                /* 체크섬 불일치 - 이번 바이트가 새 헤더일 가능성도 있으니 재검사 */
                OnFrameError();
                if (byte == FL_FRAME_HEADER)
                {
                    s_data_idx = 0;
                    s_state = FL_WAIT_DATA;
                }
                else
                {
                    s_state = FL_WAIT_HEADER;
                }
            }
            break;
        }

        case FL_WAIT_TRAILER1:
            if (byte == FL_FRAME_TRAILER1)
            {
                s_state = FL_WAIT_TRAILER2;
            }
            else
            {
                OnFrameError();
                if (byte == FL_FRAME_HEADER)
                {
                    s_data_idx = 0;
                    s_state = FL_WAIT_DATA;
                }
                else
                {
                    s_state = FL_WAIT_HEADER;
                }
            }
            break;

        case FL_WAIT_TRAILER2:
            if (byte == FL_FRAME_TRAILER2)
            {
                OnFrameValid();
            }
            else
            {
                OnFrameError();
            }
            s_state = FL_WAIT_HEADER;
            break;

        default:
            s_state = FL_WAIT_HEADER;
            break;
    }

    RearmReceive();
}

/* USART1 오류(HAL_UART_ErrorCallback)에서 호출됨.
 *
 * HAL_UART_IRQHandler 동작 (stm32f1xx_hal_uart.c):
 *   - 오버런(ORE): "blocking error"로 보고 UART_EndRxTransfer()로 수신을 중단(RxState=READY,
 *     RXNE/PE/ERR 인터럽트 끔) 한 뒤 ErrorCallback만 호출한다 → 여기서 다시 걸어주지 않으면 영구 정지.
 *   - 패리티/노이즈/프레이밍: "non-blocking error"로 수신은 계속된다(RxState=BUSY_RX).
 *     이 경우 아래 HAL_UART_Receive_IT()는 HAL_BUSY를 반환할 뿐 아무 영향이 없다.
 * 어느 경우든 진행 중이던 프레임은 믿을 수 없으므로 파서를 헤더 탐색 상태로 되돌린다. */
void FrontendLink_OnError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != huart1.Instance)
    {
        return;
    }

    OnFrameError();                        /* 진단용 오류 카운트 */
    if ((huart->ErrorCode & HAL_UART_ERROR_ORE) != 0u)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);   /* SR→DR 읽기로 ORE 플래그 해소 (재무장 직후 오류 인터럽트 재진입 방지) */
    }
    ResetParser();
    RearmReceive();                        /* 수신 중단 상태였다면 재개, 진행 중이면 HAL_BUSY로 무시됨 */
}

void FrontendLink_GetSnapshot(frontend_data_t *out)
{
    if (out == NULL)
    {
        return;
    }

    /* 짧은 구간만 인터럽트를 막아 일관된 스냅샷을 보장 (F103, 인터럽트 몇 개
     * 필드 복사하는 짧은 시간이라 다른 인터럽트 지연에 미치는 영향은 무시 가능) */
    __disable_irq();
    out->tx_raw             = s_data.tx_raw;
    out->rx_raw              = s_data.rx_raw;
    out->power_raw            = s_data.power_raw;
    out->last_valid_tick_ms  = s_data.last_valid_tick_ms;
    out->valid_frame_count   = s_data.valid_frame_count;
    out->error_count          = s_data.error_count;
    __enable_irq();
}

uint8_t FrontendLink_IsStale(void)
{
    uint32_t last;
    __disable_irq();
    last = s_data.last_valid_tick_ms;
    __enable_irq();

    if (last == 0)
    {
        /* 부팅 이후 아직 한 번도 유효한 프레임을 못 받은 상태 */
        return 1;
    }
    return ((HAL_GetTick() - last) >= FL_LINK_TIMEOUT_MS) ? 1u : 0u;
}

/* ------------------------------------------------------------------------ */
static void ResetParser(void)
{
    s_state = FL_WAIT_HEADER;
    s_data_idx = 0;
}

static void OnFrameValid(void)
{
    s_data.tx_raw    = (uint16_t)(((uint16_t)s_data_buf[0] << 8) | s_data_buf[1]);
    s_data.rx_raw    = (uint16_t)(((uint16_t)s_data_buf[2] << 8) | s_data_buf[3]);
    s_data.power_raw = (uint16_t)(((uint16_t)s_data_buf[4] << 8) | s_data_buf[5]);
    s_data.last_valid_tick_ms = HAL_GetTick();
    s_data.valid_frame_count++;
}

static void OnFrameError(void)
{
    s_data.error_count++;
}

static void RearmReceive(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}
