/**
 * @file    fw_version.h
 * @brief   펌웨어 버전 (Main MCU / Front-End MCU 공통 규칙)
 *
 * 부팅 직후 LED로 이 번호를 표시한다 (app_ver_led.h 참조).
 *   Major 번호 = 깜빡임 횟수, Minor 번호 = 깜빡임 횟수. 0이면 깜빡임 없음.
 * 버전을 올릴 때는 이 파일의 두 값만 수정한다.
 */
#ifndef FW_VERSION_H
#define FW_VERSION_H

#define FW_VERSION_MAJOR   1u   /* 정수부 */
#define FW_VERSION_MINOR   1u   /* 소수 이하 (정수로 표기: 1.1 -> 1, 1.10 -> 10) */

#endif /* FW_VERSION_H */
