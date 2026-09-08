/**
 * @file    app_monitor.h
 * @brief   메인 MCU(STM32F103C8T6) - USB-CDC 프로토콜 처리 / LED 표시 / USB 연결감지
 *
 * v2: 아키텍처 변경 반영
 *   - CH1~3 GPIO 직접폴링(비교기 방식) 제거 -> app_state_machine.c(StateMachine_GetState())가
 *     프론트엔드 UART 데이터를 분석해 넘겨주는 상태를 그대로 사용
 *   - PA9(USB 5V 감지, 저항분압) 추가: 케이블 물리적 연결 여부를 최우선으로 반영
 *
 * 하드웨어 배정:
 *   USB 연결 제어(D+ 풀업 개폐, 재연결 시뮬레이션) : PA8  - High=끊기, Low=연결
 *   USB 5V 감지(케이블 연결 여부, 저항분압)         : PA9  - High=연결됨, Low=연결안됨
 *   상태표시 LED                                    : PA3  - High=켜짐
 */

#ifndef APP_MONITOR_H
#define APP_MONITOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ------------------------------------------------------------------------ */
/* GPIO 핀 배정                                                              */
/* ------------------------------------------------------------------------ */
#define USB_CTRL_GPIO_PORT      GPIOA
#define USB_CTRL_GPIO_PIN       GPIO_PIN_8
#define USB_CONNECT_LEVEL       GPIO_PIN_RESET  /* PA8 Low  = USB 연결 */
#define USB_DISCONNECT_LEVEL    GPIO_PIN_SET    /* PA8 High = USB 끊기 */

#define USB5V_GPIO_PORT          GPIOA
#define USB5V_GPIO_PIN           GPIO_PIN_9
#define USB5V_CONNECTED_LEVEL    GPIO_PIN_SET    /* PA9 High = 케이블 연결됨 */
#define USB5V_DISCONNECTED_LEVEL GPIO_PIN_RESET  /* PA9 Low  = 케이블 연결안됨 */

#define LED_GPIO_PORT            GPIOA
#define LED_GPIO_PIN             GPIO_PIN_3
#define LED_ON_LEVEL              GPIO_PIN_SET
#define LED_OFF_LEVEL             GPIO_PIN_RESET

/* ------------------------------------------------------------------------ */
/* 타이밍 파라미터 (ms 단위)                                                 */
/* ------------------------------------------------------------------------ */
#define REQUEST_TIMEOUT_MS      10000u  /* 무응답 시 USB 재연결 시뮬레이션 타임아웃 */
#define USB_DISCONNECT_HOLD_MS   3000u  /* 소프트 디스커넥트 유지 시간 */

/* LED - 부팅 표시 (0.7초 주기 3회 점멸) */
#define LED_BOOT_HALF_PERIOD_MS   350u
#define LED_BOOT_BLINK_COUNT        3u

/* LED - 인터폰 상태별 점멸 패턴 */
#define LED_IDLE_ON_MS      1000u
#define LED_IDLE_OFF_MS     1000u
#define LED_RING_ON_MS       300u
#define LED_RING_OFF_MS      700u
#define LED_TALK_ON_MS       500u
#define LED_TALK_OFF_MS      500u
#define LED_FAULT_ON_MS       150u   /* 전원단절 */
#define LED_FAULT_OFF_MS      150u

/* LED - USB 케이블 미연결 (최우선 표시) */
#define LED_USBDISC_ON_MS     100u
#define LED_USBDISC_OFF_MS    900u

/* ------------------------------------------------------------------------ */
/* 프로토콜 상수                                                            */
/* ------------------------------------------------------------------------ */
#define FRAME_STX                0x0D
#define FRAME_ETX1                0x0D
#define FRAME_ETX2                0x0A
#define CMD_REQUEST_STATE         0x52  /* 'R' */

#define STATE_HIGH_BYTE           0x31
#define STATE_LOW_BYTE            0x30

/* 공개 API */
void AppMonitor_Init(void);
void AppMonitor_Tick1ms(void);                 /* HAL_SYSTICK_Callback 등 1ms 주기에서 호출 */
void AppMonitor_UsbRxHandler(uint8_t *buf, uint32_t len); /* CDC_Receive_FS 콜백에서 호출 */
void AppMonitor_Poll(void);                    /* main 루프에서 반복 호출 */

#endif /* APP_MONITOR_H */
