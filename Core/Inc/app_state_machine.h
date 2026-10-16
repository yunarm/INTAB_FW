/**
 * @file    app_state_machine.h
 * @brief   프론트엔드로부터 받은 Tx/Rx/Power ADC 원시값으로
 *          인터폰 상태(대기/호출/통화/전원 이상/링크 이상)를 판정하고,
 *          USB 응답 프레임에 실을 값(Tx‰, Rx‰, Power×10V)을 산출한다.
 *
 * v3 (2026-10-16): 통신_프로토콜_정리.md v1.5 반영
 *   - 판정 기준을 "상태 다이어그램"(1.4절) 방식으로 교체 (기존 70%/30%+추세 방식 폐기)
 *   - Tx/Rx는 Power ADC 대비 0.1%(‰) 단위, Power는 전압×10 (0.1V 단위)
 *   - 상태에 '링크 이상'(프론트엔드 무응답) 추가
 *
 * 입력 환산 (Power ADC 기준):
 *   tx_pm = round(Tx_ADC × 1000 / Power_ADC)   0~1000 (1000 초과는 1000으로 제한)
 *   rx_pm = round(Rx_ADC × 1000 / Power_ADC)   0~1000
 *   power_dv = round(Power_ADC × 3.3V × 7.98 × 10 / 4095)   0~263 (0.1V 단위)
 *
 * 판정 기준 (‰):  TxH: tx_pm > 750 / TxL: 그 외   RxL: rx_pm < 380   RxH: rx_pm > 750
 *
 *   전원 이상 : Power_ADC < 200 이면 상태와 무관하게 최우선 (Tx/Rx ‰는 0으로 보고)
 *   링크 이상 : 프론트엔드 유효 프레임이 FL_LINK_TIMEOUT_MS(500ms) 이상 없으면 (값은 모두 0으로 보고)
 *   부팅 직후 : 대기. 부팅 후 FL_LINK_TIMEOUT_MS 안에 첫 유효 프레임이 오지 않으면 링크 이상
 *
 *   현재   → 다음  조건                            지정시간
 *   대기   → 통화  TxH AND Tx>Rx                   400ms
 *   대기   → 호출  TxL AND RxL                     400ms
 *   호출   → 통화  TxH AND Tx>Rx                   400ms
 *   호출   → 대기  Rx>Tx AND RxH                   없음
 *   통화   → 대기  Rx>Tx AND TxL (RxL이면 제외)    없음
 *   통화   → 호출  TxL AND RxL                     400ms
 *   (그 외 : 현재 상태 유지.  RxL(호출)이 "Rx>Tx(대기)"보다 우선)
 *
 * 프론트엔드 프레임 주기(50ms)에 맞춰 한 번씩만 갱신되며,
 * StateMachine_Poll()을 더 자주 호출해도 무방하다(내부에서 새 프레임
 * 도착 여부를 자체적으로 감지함).
 */

#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include <stdint.h>

/* 값은 USB 응답 프레임의 상태 문자('0'+값)와 1:1 대응한다. 순서를 바꾸지 말 것. */
typedef enum
{
    IP_STATE_IDLE = 0,    /* '0' 대기 */
    IP_STATE_RINGING = 1, /* '1' 호출 */
    IP_STATE_TALKING = 2, /* '2' 통화 */
    IP_STATE_POWERFAIL = 3, /* '3' 전원 이상 (Power ADC < 200) */
    IP_STATE_LINKFAIL = 4   /* '4' 프론트엔드 통신 이상 */
} ip_state_t;

/* USB 응답 프레임용 판정 결과 (한 번에 일관되게 읽기 위한 묶음) */
typedef struct
{
    ip_state_t state;
    uint16_t   tx_pm;     /* Tx / Power, 0.1% 단위 (0~1000) */
    uint16_t   rx_pm;     /* Rx / Power, 0.1% 단위 (0~1000) */
    uint16_t   power_dv;  /* Power 전압 × 10 (0.1V 단위, 0~263) */
} sm_result_t;

/* 공개 API */
void        StateMachine_Init(void);
void        StateMachine_Poll(void);                 /* 메인 루프에서 반복 호출 */
ip_state_t  StateMachine_GetState(void);             /* 현재 판정 상태 조회 (LED 등) */
void        StateMachine_GetResult(sm_result_t *out); /* 상태+값을 일관되게 복사 (USB 인터럽트에서 호출 가능) */

#endif /* APP_STATE_MACHINE_H */
