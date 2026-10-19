/**
 * @file    app_frontend_link.h
 * @brief   프론트엔드 MCU(STM32G030C6T6)로부터 UART로 수신되는
 *          ADC 원시값 프레임을 파싱하여 최신값을 제공하는 모듈
 *
 * 하드웨어 배정 (메인 MCU측):
 *   USART1, PA9(TX, 미사용) / PA10(RX) — 절연 배리어(ISO7740FDB) 1채널 통과
 *   Baud Rate: 38400bps (프론트엔드와 동일)
 *
 * 프레임 포맷 (프론트엔드_MCU_개발사양서.md 6.2절과 동일, 10바이트):
 *   [0]     0xFF                  헤더
 *   [1..2]  CH0(Tx) 상위/하위바이트 (12비트 ADC 원시값)
 *   [3..4]  CH1(Rx) 상위/하위바이트
 *   [5..6]  CH2(Power) 상위/하위바이트
 *   [7]     체크섬 (바이트 1~6 XOR)
 *   [8..9]  0x0D 0x0A             트레일러
 */

#ifndef APP_FRONTEND_LINK_H
#define APP_FRONTEND_LINK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define FL_FRAME_HEADER    0xFFu
#define FL_FRAME_TRAILER1  0x0Du
#define FL_FRAME_TRAILER2  0x0Au
#define FL_FRAME_LEN       10u

/* 프론트엔드가 50ms마다 보내므로, 이보다 훨씬 오래 프레임이 없으면
 * 링크 이상(프론트엔드 고장/케이블 이탈 등)으로 간주하는 기준 시간 */
#define FL_LINK_TIMEOUT_MS 500u

typedef struct
{
    uint16_t tx_raw;              /* CH0(Tx) ADC 원시값, 0~4095 */
    uint16_t rx_raw;              /* CH1(Rx) ADC 원시값, 0~4095 */
    uint16_t power_raw;           /* CH2(Power) ADC 원시값, 0~4095 */
    uint32_t last_valid_tick_ms;  /* 마지막으로 유효한 프레임을 받은 HAL_GetTick() 시점 */
    uint32_t valid_frame_count;   /* 누적 유효 프레임 수 (디버그/진단용) */
    uint32_t error_count;         /* 누적 오류(체크섬/트레일러 불일치 등) 프레임 수 */
} frontend_data_t;

/* 공개 API */
void FrontendLink_Init(void);                 /* main()에서 1회 호출 - UART 수신 시작 */

/* USART1의 HAL_UART_RxCpltCallback에서 호출 (stm32f1xx_it.c 또는 별도 콜백 파일에서 연결) */
void FrontendLink_OnByteReceived(UART_HandleTypeDef *huart);

/* USART1의 HAL_UART_ErrorCallback에서 호출 (stm32f1xx_it.c 등에서 연결).
 * 오버런(ORE) 등으로 HAL이 수신을 중단(abort)하면 스스로 재개하지 않으므로,
 * 여기서 오류 플래그를 지우고 파서를 초기화한 뒤 1바이트 수신을 다시 건다.
 * (이 호출이 없으면 한 번의 오버런으로 프론트엔드 수신이 영구히 멈춘다) */
void FrontendLink_OnError(UART_HandleTypeDef *huart);

/* 최신 데이터의 스냅샷을 out에 복사 (인터럽트와의 경합 없이 안전하게 읽기 위함) */
void FrontendLink_GetSnapshot(frontend_data_t *out);

/* 마지막 유효 프레임 이후 FL_LINK_TIMEOUT_MS 이상 경과했으면 1 반환 (링크 이상) */
uint8_t FrontendLink_IsStale(void);

#endif /* APP_FRONTEND_LINK_H */
