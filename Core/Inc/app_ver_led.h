/**
 * @file    app_ver_led.h
 * @brief   부팅 직후 LED로 펌웨어 버전을 표시하는 시퀀서 (HAL 비의존, Main/Front 공용)
 *
 * 표시 순서:
 *   1) 2초 OFF
 *   2) Major 번호 표시
 *   3) 1초 OFF
 *   4) Minor 번호 표시
 *   5) 1초 OFF
 *   6) 완료 -> 호출 측이 평상시(RUN) 표시로 전환
 *
 * 번호 표시 = (0.1초 ON + 0.3초 OFF)를 1세트로 번호 수만큼 반복. 번호가 0이면 세트 없음.
 * 버전 1.1의 총 길이는 4.8초이다 (2.0 + 0.4 + 1.0 + 0.4 + 1.0).
 * 마지막 세트의 0.3초 OFF 뒤에 구간 OFF(1초)가 이어지므로 번호 뒤 OFF는 총 1.3초이다.
 *
 * 사용법: VerLed_Start(주기_ms) 후, 그 주기마다 VerLed_Tick()을 호출하고 반환값대로 LED를 구동한다.
 *   Main : 1ms 틱에서 VerLed_Start(1)
 *   Front: 50ms ADC 콜백에서 VerLed_Start(50)
 * 모든 구간 길이는 호출 주기의 정수배여야 한다 (50ms 이하 주기면 충족).
 */
#ifndef APP_VER_LED_H
#define APP_VER_LED_H

#include <stdint.h>
#include "fw_version.h"

#define VERLED_LEAD_OFF_MS    2000u   /* 시작 전 OFF */
#define VERLED_GAP_OFF_MS     1000u   /* 번호 사이/뒤 OFF */
#define VERLED_BLINK_ON_MS     100u
#define VERLED_BLINK_OFF_MS    300u

/* VerLed_Tick() 반환값 */
#define VERLED_OFF    0u   /* 이번 틱은 LED 꺼짐 */
#define VERLED_ON     1u   /* 이번 틱은 LED 켜짐 */
#define VERLED_DONE   2u   /* 표시 종료 (이후 계속 DONE) */

void    VerLed_Start(uint16_t tick_ms);   /* 시퀀스 처음부터 시작 */
uint8_t VerLed_Tick(void);                /* 주기마다 1회 호출 */

#endif /* APP_VER_LED_H */
