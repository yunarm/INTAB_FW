/**
 * @file    app_monitor.h
 * @brief   엘리베이터 비상통화장치 상태감지 모듈 - 채널 감시/USB-CDC 프로토콜/LED 표시
 *
 * 전제 조건 (STM32CubeMX 생성 프로젝트에 통합):
 *   - USB_DEVICE 미들웨어 (CDC_FS 클래스) 이미 활성화되어 있음
 *   - HAL SysTick 1ms 틱 사용 (기본값, Timebase Source = SysTick)
 *   - IWDG(독립 워치독) 활성화 권장 (본 모듈은 주기적으로 IWDG_Refresh 호출)
 *
 * 하드웨어 배정 (v1.1):
 *   CH1 (Tx_state)       : PB12  (ISO7740FDB 출력)
 *   CH2 (Rx_state)       : PB13  (ISO7740FDB 출력)
 *   CH3 (Power_present)  : PB14  (ISO7740FDB 출력)
 *   CH4 (Spare)          : 미사용 - 하드웨어 GND 고정, 항상 0x30 전송 (핀 리딩 안 함)
 *   USB 연결 제어         : PA8   (Output PP) - High = USB 끊기, Low = USB 연결
 *   상태표시 LED          : PA3   (Output PP) - High = LED 켜짐
 */

#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------------ */
/* GPIO 핀 배정                                                              */
/* ------------------------------------------------------------------------ */
#define CH1_GPIO_PORT           GPIOB
#define CH1_GPIO_PIN            GPIO_PIN_12   /* Tx_state  (통화 시작/종료 트리거) */

#define CH2_GPIO_PORT           GPIOB
#define CH2_GPIO_PIN            GPIO_PIN_13   /* Rx_state  (호출 감지) */

#define CH3_GPIO_PORT           GPIOB
#define CH3_GPIO_PIN            GPIO_PIN_14   /* Power_present (전원 감시, 최우선) */

/* CH4는 하드웨어 GND 고정이므로 GPIO 입력을 두지 않는다 (항상 0x30 전송) */

#define USB_CTRL_GPIO_PORT      GPIOA
#define USB_CTRL_GPIO_PIN       GPIO_PIN_8    /* USB 연결 제어 */
#define USB_CONNECT_LEVEL       GPIO_PIN_RESET /* PA8 Low  = USB 연결 */
#define USB_DISCONNECT_LEVEL    GPIO_PIN_SET   /* PA8 High = USB 끊기 */

#define LED_GPIO_PORT            GPIOA
#define LED_GPIO_PIN             GPIO_PIN_3    /* 상태표시 LED */
#define LED_ON_LEVEL              GPIO_PIN_SET
#define LED_OFF_LEVEL             GPIO_PIN_RESET

/* ------------------------------------------------------------------------ */
/* 타이밍 파라미터 (ms 단위)                                                 */
/* ------------------------------------------------------------------------ */
#define DEBOUNCE_TX_MS           300u   /* Tx(CH1) 상태 전이 디바운스 */
#define DEBOUNCE_RX_MS           300u   /* Rx(CH2) 상태 전이 디바운스 */
#define DEBOUNCE_POWER_MS        250u   /* Power(CH3) 디바운스 (200~300ms 범위 중간값) */
#define RX_LOCKOUT_MS           1500u   /* 통화종료 후 Rx 무시 구간 (뮤트 트랜지언트 대응) */
#define REQUEST_TIMEOUT_MS      10000u  /* 무응답 시 USB 재연결 시뮬레이션 타임아웃 */
#define USB_DISCONNECT_HOLD_MS   3000u  /* 소프트 디스커넥트 유지 시간 */

/* LED - 부팅 표시 (0.7초 주기 3회 점멸 = 0.35초 ON / 0.35초 OFF x 3) */
#define LED_BOOT_HALF_PERIOD_MS   350u
#define LED_BOOT_BLINK_COUNT        3u

/* LED - 인터폰 상태별 점멸 패턴 */
#define LED_IDLE_ON_MS      1000u
#define LED_IDLE_OFF_MS     1000u
#define LED_RING_ON_MS       300u
#define LED_RING_OFF_MS      700u
#define LED_TALK_ON_MS       500u
#define LED_TALK_OFF_MS      500u
/* 전원단절(Power_present=Low) - 패턴 미확정, 임시로 빠른 점멸 지정 (변경 시 이 두 값만 수정) */
#define LED_FAULT_ON_MS       150u
#define LED_FAULT_OFF_MS      150u

/* ------------------------------------------------------------------------ */
/* 프로토콜 상수                                                            */
/* ------------------------------------------------------------------------ */
#define FRAME_STX                0x0D
#define FRAME_ETX1                0x0D
#define FRAME_ETX2                0x0A
#define CMD_REQUEST_STATE         0x52  /* 'R' */

#define STATE_HIGH_BYTE           0x31
#define STATE_LOW_BYTE            0x30

/* 인터폰 표시 상태 (LED 및 내부 판정용) */
typedef enum
{
    IP_STATE_IDLE = 0,   /* 대기 */
    IP_STATE_RINGING,    /* 호출 */
    IP_STATE_TALKING,    /* 통화 */
    IP_STATE_POWERFAIL   /* 전원단절 (최우선) */
} interphone_state_t;

/* 공개 API */
void AppMonitor_Init(void);
void AppMonitor_Tick1ms(void);                 /* HAL_SYSTICK_Callback 등 1ms 주기에서 호출 */
void AppMonitor_UsbRxHandler(uint8_t *buf, uint32_t len); /* CDC_Receive_FS 콜백에서 호출 */
void AppMonitor_Poll(void);                    /* main 루프에서 반복 호출 (타임아웃 검사 등) */

#endif /* APP_MONITOR_H */
