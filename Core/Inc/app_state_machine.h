/**
 * @file    app_state_machine.h
 * @brief   프론트엔드로부터 받은 Tx/Rx/Power ADC 원시값으로
 *          인터폰 상태(대기/호출/통화/전원단절)를 판정
 *
 * 판정 규칙 (Power_raw 대비 상대 비율 기준 - 12V/24V 현장 겸용):
 *   레벨: High = Power_raw*70% 이상, Low = Power_raw*30% 이하, 사이는 이력구간(직전 유지)
 *   추세: 300ms(6샘플) 전 대비 Power_raw*5% 이상 변화 시 Falling/Rising
 *
 *   1순위: Power_raw < 절대최소값(200)              -> 전원단절
 *   2순위: Tx==High AND (Rx==Falling OR Rx==Low)     -> 통화중
 *   3순위: (Tx==Low OR Tx==Falling) AND Rx==Low      -> 호출중
 *   4순위: (Tx==Low OR Tx==Falling) AND Rx==High     -> 대기중
 *   어느 규칙에도 해당 없음                           -> 직전 상태 유지
 *
 * 프론트엔드 프레임 주기(50ms)에 맞춰 한 번씩만 갱신되며,
 * StateMachine_Poll()을 더 자주 호출해도 무방하다(내부에서 새 프레임
 * 도착 여부를 자체적으로 감지함).
 */

#ifndef APP_STATE_MACHINE_H
#define APP_STATE_MACHINE_H

#include <stdint.h>

typedef enum
{
    IP_STATE_IDLE = 0,   /* 대기 */
    IP_STATE_RINGING,    /* 호출 */
    IP_STATE_TALKING,    /* 통화 */
    IP_STATE_POWERFAIL   /* 전원단절 (최우선) */
} ip_state_t;

/* 공개 API */
void        StateMachine_Init(void);
void        StateMachine_Poll(void);      /* 메인 루프에서 반복 호출 */
ip_state_t  StateMachine_GetState(void);  /* 현재 판정 상태 조회 */

#endif /* APP_STATE_MACHINE_H */
