#!/usr/bin/env python3
"""
ArduPilot / ArduCopter 공식 에러코드 전수조사 레퍼런스
=====================================================
소스: ArduPilot GitHub (AP_Logger.h, LogStructure.h, defines.h, ekf_check.cpp)
      ardupilot.org 공식 문서
기준: ArduCopter 4.5+ / master branch (2025~2026)

사용법:
    from ardupilot_error_codes import *
    print(ERR_SUBSYSTEM[16])  # 'EKFCHECK'
    print(ERR_CODES['FAILSAFE_BATT'])  # {0: {...}, 1: {...}}
"""

# =============================================================================
# 1. ERR 메시지 — Subsys(서브시스템) 전체 목록
#    소스: AP_Logger.h → enum class LogErrorSubsystem
# =============================================================================
ERR_SUBSYSTEM = {
    1:  "MAIN",
    2:  "RADIO",
    3:  "COMPASS",
    4:  "OPTFLOW",
    5:  "FAILSAFE_RADIO",
    6:  "FAILSAFE_BATT",
    7:  "FAILSAFE_GPS",           # 구버전 호환, 현재는 미사용
    8:  "FAILSAFE_GCS",
    9:  "FAILSAFE_FENCE",
    10: "FLIGHT_MODE",
    11: "GPS",
    12: "CRASH_CHECK",
    13: "FLIP",
    14: "AUTOTUNE",
    15: "PARACHUTES",
    16: "EKFCHECK",
    17: "FAILSAFE_EKFINAV",
    18: "BARO",
    19: "CPU",
    20: "FAILSAFE_ADSB",
    21: "TERRAIN",
    22: "NAVIGATION",
    23: "FAILSAFE_TERRAIN",
    24: "EKF_PRIMARY",
    25: "THRUST_LOSS_CHECK",
    26: "FAILSAFE_SENSORS",
    27: "FAILSAFE_LEAK",          # ArduSub 전용
    28: "PILOT_INPUT",
    29: "FAILSAFE_VIBE",
    30: "INTERNAL_ERROR",
    31: "FAILSAFE_DEADRECKON",
}

# =============================================================================
# 2. 각 Subsys별 ECode(에러코드) 전체 목록
#    소스: AP_Logger.h → enum class LogErrorCode
# =============================================================================

# --- 공통 에러코드 (대부분의 Failsafe 서브시스템에서 사용) ---
COMMON_ERROR_CODES = {
    0: "ERROR_RESOLVED / FAILSAFE_RESOLVED — 에러/페일세이프 해제됨",
    1: "FAILSAFE_OCCURRED — 페일세이프 발생",
}

ERR_CODES = {
    # --- MAIN (1) ---
    "MAIN": {
        1: {"name": "INS_DELAY",
            "desc": "IMU 샘플 지연 감지",
            "cause": "CPU 과부하 또는 IMU 하드웨어 문제",
            "fix": "로그 CPU 부하 확인, 불필요한 기능 비활성화"},
    },

    # --- RADIO (2) ---
    "RADIO": {
        0: {"name": "ERROR_RESOLVED", "desc": "라디오 에러 해제", "cause": "-", "fix": "-"},
        2: {"name": "LATE_FRAME", "desc": "RC 입력 프레임 지연", "cause": "수신기 신호 불안정", "fix": "수신기/안테나 점검"},
    },

    # --- COMPASS (3) ---
    "COMPASS": {
        0: {"name": "ERROR_RESOLVED", "desc": "컴파스 에러 해제", "cause": "-", "fix": "-"},
        1: {"name": "COMPASS_FAILED_TO_READ", "desc": "컴파스 읽기 실패", "cause": "I2C/SPI 통신 오류, 하드웨어 불량", "fix": "배선 점검, 컴파스 교체"},
        2: {"name": "COMPASS_NOT_HEALTHY", "desc": "컴파스 비정상", "cause": "센서 데이터 이상", "fix": "캘리브레이션 재실행"},
        3: {"name": "COMPASS_NOT_CALIBRATED", "desc": "컴파스 미교정", "cause": "캘리브레이션 미완료", "fix": "컴파스 캘리브레이션 수행"},
        4: {"name": "COMPASS_INCONSISTENT", "desc": "컴파스 간 불일치 (>45도)", "cause": "외부 자기장 간섭, COMPASS_ORIENT 오류", "fix": "COMPASS_ORIENT 확인, 자기 간섭원 제거"},
    },

    # --- OPTFLOW (4) ---
    "OPTFLOW": {
        1: {"name": "OPTFLOW_NOT_HEALTHY", "desc": "옵티컬 플로우 비정상", "cause": "센서 불량, 연결 끊김", "fix": "센서 연결/배선 점검"},
    },

    # --- FAILSAFE_RADIO (5) ---
    "FAILSAFE_RADIO": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "RC 페일세이프 해제", "cause": "RC 신호 복구", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED", "desc": "RC 페일세이프 발생",
            "cause": "RC 신호 손실 (THR < FS_THR_VALUE 또는 No Pulse)",
            "fix": "수신기/송신기 점검, FS_THR_VALUE 확인, 안테나 배치 점검"},
        2: {"name": "FAILSAFE_OCCURRED_NO_SIGNAL", "desc": "RC 신호 완전 손실",
            "cause": "수신기 전원 끊김 또는 완전 탈조",
            "fix": "수신기 전원/배선, 바인딩 확인"},
    },

    # --- FAILSAFE_BATT (6) ---
    "FAILSAFE_BATT": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "배터리 페일세이프 해제", "cause": "전압 회복", "fix": "-"},
        1: {"name": "FAILSAFE_BATT_LOW",
            "desc": "배터리 저전압 (Low)",
            "cause": "BATT_LOW_VOLT 이하로 10초 이상 유지",
            "fix": "배터리 충전/교체, BATT_LOW_VOLT 파라미터 확인"},
        2: {"name": "FAILSAFE_BATT_CRITICAL",
            "desc": "배터리 위험 전압 (Critical)",
            "cause": "BATT_CRT_VOLT 이하 (2단계 페일세이프)",
            "fix": "즉시 착륙, 배터리 상태 점검"},
    },

    # --- FAILSAFE_GPS (7) --- (레거시, 현재 대부분 미사용)
    "FAILSAFE_GPS": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "GPS 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED", "desc": "GPS 페일세이프 발생", "cause": "GPS Fix 상실", "fix": "GPS 안테나, 하늘 시야 확인"},
    },

    # --- FAILSAFE_GCS (8) ---
    "FAILSAFE_GCS": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "GCS 페일세이프 해제", "cause": "GCS 하트비트 복구", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "GCS 페일세이프 발생",
            "cause": "GCS 하트비트 FS_GCS_TIMEOUT초 이상 수신 없음 (기본 5초)",
            "fix": "텔레메트리 링크 점검, FS_GCS_ENABLE 확인"},
    },

    # --- FAILSAFE_FENCE (9) ---
    "FAILSAFE_FENCE": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "펜스 위반 해제", "cause": "펜스 안으로 복귀", "fix": "-"},
        1: {"name": "FAILSAFE_FENCE_ALT", "desc": "고도 펜스 위반", "cause": "최대 고도 초과", "fix": "FENCE_ALT_MAX 확인"},
        2: {"name": "FAILSAFE_FENCE_CIRCLE", "desc": "원형 펜스 위반", "cause": "반경 초과", "fix": "FENCE_RADIUS 확인"},
        3: {"name": "FAILSAFE_FENCE_ALT_AND_CIRCLE", "desc": "고도+원형 펜스 동시 위반",
            "cause": "고도와 반경 동시 초과", "fix": "펜스 파라미터 확인"},
        4: {"name": "FAILSAFE_FENCE_POLYGON", "desc": "폴리곤 펜스 위반", "cause": "다각형 영역 이탈", "fix": "펜스 경계 확인"},
        5: {"name": "FAILSAFE_FENCE_POLYGON_AND_ALT", "desc": "폴리곤+고도 펜스 위반",
            "cause": "다각형 이탈 + 고도 초과", "fix": "펜스 설정 전체 확인"},
        6: {"name": "FAILSAFE_FENCE_FLOOR", "desc": "최저 고도 펜스 위반", "cause": "최소 고도 이하", "fix": "FENCE_ALT_MIN 확인"},
    },

    # --- FLIGHT_MODE (10) ---
    "FLIGHT_MODE": {
        0: {"name": "ERROR_RESOLVED", "desc": "비행 모드 에러 해제", "cause": "-", "fix": "-"},
        2: {"name": "FLIGHT_MODE_CHANGE_FAILED",
            "desc": "비행 모드 전환 실패",
            "cause": "전환 조건 미충족 (GPS 미확보 상태에서 Loiter 등)",
            "fix": "모드 전환 전 필요 조건 확인 (GPS Fix, EKF 상태 등)"},
    },

    # --- GPS (11) ---
    "GPS": {
        0: {"name": "ERROR_RESOLVED", "desc": "GPS 에러 해제", "cause": "-", "fix": "-"},
        1: {"name": "GPS_GLITCH",
            "desc": "GPS 글리치 감지",
            "cause": "GPS 위치 급변, 멀티패스, 방해전파",
            "fix": "GPS 안테나 위치/하늘시야 확인, 전자기 차폐"},
        2: {"name": "GPS_GLITCH_CLEARED", "desc": "GPS 글리치 해제", "cause": "GPS 정상 복귀", "fix": "-"},
    },

    # --- CRASH_CHECK (12) ---
    "CRASH_CHECK": {
        1: {"name": "CRASH_CHECK_CRASH",
            "desc": "충돌 감지 — 모터 정지",
            "cause": "기체 전복 또는 급격한 자세 변화",
            "fix": "기체 상태 점검, 프롭/모터 확인"},
        2: {"name": "CRASH_CHECK_LOSS_OF_CONTROL",
            "desc": "제어 불능 감지",
            "cause": "자세 에러 지속 (>30도, 1초 이상)",
            "fix": "PID 튜닝, 모터/ESC 점검, 무게중심 확인"},
    },

    # --- FLIP (13) ---
    "FLIP": {
        1: {"name": "FLIP_ABANDONED",
            "desc": "플립 중단",
            "cause": "플립 동작 중 제한 시간 초과 또는 안전 조건 위반",
            "fix": "배터리 전압/모터 출력 확인"},
    },

    # --- AUTOTUNE (14) ---
    "AUTOTUNE": {
        2: {"name": "AUTOTUNE_FAILED",
            "desc": "오토튠 실패",
            "cause": "안정적인 호버링 불가, 바람 과다, 기계적 문제",
            "fix": "무풍 환경에서 재시도, 기본 PID 확인"},
    },

    # --- PARACHUTES (15) ---
    "PARACHUTES": {
        1: {"name": "PARACHUTE_TOO_LOW",
            "desc": "낙하산 전개 고도 부족",
            "cause": "현재 고도 < 최소 안전 고도",
            "fix": "CHUTE_ALT_MIN 파라미터 확인"},
        2: {"name": "PARACHUTE_RELEASED",
            "desc": "낙하산 전개됨",
            "cause": "낙하산 전개 조건 충족 (수동 또는 자동)",
            "fix": "낙하산 재장착 후 리셋"},
    },

    # --- EKFCHECK (16) ---
    "EKFCHECK": {
        0: {"name": "EKFCHECK_RESOLVED", "desc": "EKF 체크 정상 복귀", "cause": "EKF variance 정상 범위 진입", "fix": "-"},
        2: {"name": "EKFCHECK_BAD_VARIANCE",
            "desc": "EKF variance 이상 — 페일세이프 트리거",
            "cause": "compass/position/velocity 중 2개 이상 FS_EKF_THRESH 초과 1초 이상",
            "fix": "진동 저감, 컴파스 간섭 제거, GPS 시야 확보"},
    },

    # --- FAILSAFE_EKFINAV (17) ---
    "FAILSAFE_EKFINAV": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "EKF 항법 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "EKF 항법 페일세이프 발생",
            "cause": "EKF 위치 추정 신뢰도 급락",
            "fix": "FS_EKF_ACTION 확인, 진동/컴파스/GPS 점검"},
    },

    # --- BARO (18) ---
    "BARO": {
        1: {"name": "BARO_NOT_HEALTHY",
            "desc": "기압계 비정상",
            "cause": "기압계 하드웨어 오류, 통신 끊김",
            "fix": "기압계 연결 점검, 방수/방풍 처리 확인"},
        2: {"name": "BARO_GLITCH",
            "desc": "기압계 글리치",
            "cause": "급격한 기압 변화 (바람, 프롭워시)",
            "fix": "기압계 폼 커버 적용, 위치 재조정"},
    },

    # --- CPU (19) ---
    "CPU": {
        1: {"name": "CPU_OVERLOAD",
            "desc": "CPU 과부하",
            "cause": "메인 루프 실행 시간 초과, 과다 로깅",
            "fix": "불필요한 기능 비활성화, LOG_BITMASK 축소"},
    },

    # --- FAILSAFE_ADSB (20) ---
    "FAILSAFE_ADSB": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "ADS-B 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "ADS-B 회피 페일세이프 발생",
            "cause": "근접 항공기 감지",
            "fix": "ADS-B 수신기 점검, 회피 파라미터 확인"},
    },

    # --- TERRAIN (21) ---
    "TERRAIN": {
        2: {"name": "TERRAIN_MISSING_DATA",
            "desc": "지형 데이터 누락",
            "cause": "지형 DB 미로드 또는 해당 좌표 데이터 없음",
            "fix": "지형 데이터 사전 다운로드, GCS 연결 확인"},
    },

    # --- NAVIGATION (22) ---
    "NAVIGATION": {
        2: {"name": "NAVIGATION_FAILED",
            "desc": "네비게이션 실패 — 목적지 도달 불가",
            "cause": "웨이포인트 도달 시간 초과, 위치 추정 불안정",
            "fix": "미션 경로 확인, 풍속 확인, WPNAV 파라미터 점검"},
    },

    # --- FAILSAFE_TERRAIN (23) ---
    "FAILSAFE_TERRAIN": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "지형 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "지형 페일세이프 발생",
            "cause": "지형 데이터 수신 불가 상태 지속",
            "fix": "GCS 텔레메트리 확인, TERRAIN_ENABLE 파라미터 점검"},
    },

    # --- EKF_PRIMARY (24) ---
    "EKF_PRIMARY": {
        0: {"name": "EKF_PRIMARY_CHANGED",
            "desc": "EKF 기본 레인 변경 (0번으로)",
            "cause": "다른 EKF 레인의 건강도가 더 좋음",
            "fix": "정보성 메시지 — 보통 정상 동작"},
        1: {"name": "EKF_PRIMARY_CHANGED_1",
            "desc": "EKF 기본 레인 → 1번으로 변경", "cause": "레인 0 건강도 저하", "fix": "로그 분석"},
        2: {"name": "EKF_PRIMARY_CHANGED_2",
            "desc": "EKF 기본 레인 → 2번으로 변경", "cause": "다른 레인 건강도 저하", "fix": "로그 분석"},
    },

    # --- THRUST_LOSS_CHECK (25) ---
    "THRUST_LOSS_CHECK": {
        1: {"name": "THRUST_LOSS_DETECTED",
            "desc": "추력 손실 감지",
            "cause": "모터 고장, 프롭 탈락, ESC 불량",
            "fix": "모터/프롭/ESC 즉시 점검, 비행 중이면 즉시 착륙"},
    },

    # --- FAILSAFE_SENSORS (26) ---
    "FAILSAFE_SENSORS": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "센서 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "센서 페일세이프 발생",
            "cause": "주요 센서 다수 동시 실패",
            "fix": "IMU, 기압계, 컴파스 하드웨어 점검"},
    },

    # --- FAILSAFE_LEAK (27) — ArduSub 전용 ---
    "FAILSAFE_LEAK": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "누수 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "누수 감지 페일세이프",
            "cause": "수중 로봇 누수 센서 트리거",
            "fix": "즉시 회수, 실링 점검"},
    },

    # --- PILOT_INPUT (28) ---
    "PILOT_INPUT": {
        2: {"name": "PILOT_INPUT_LOST",
            "desc": "조종사 입력 손실",
            "cause": "조이스틱/RC 입력 없음",
            "fix": "RC 송신기 전원/연결 확인"},
    },

    # --- FAILSAFE_VIBE (29) ---
    "FAILSAFE_VIBE": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "진동 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "진동 페일세이프 발생",
            "cause": "IMU 가속도 클리핑 빈도 초과",
            "fix": "방진 마운트 개선, 프롭 밸런싱, 모터 점검"},
    },

    # --- INTERNAL_ERROR (30) ---
    "INTERNAL_ERROR": {
        1: {"name": "INTERNAL_ERROR_OCCURRED",
            "desc": "내부 에러 발생",
            "cause": "펌웨어 버그, 메모리 오버플로우, 스택 오버플로우 등",
            "fix": "펌웨어 최신 버전 업데이트, ArduPilot 이슈 리포트"},
    },

    # --- FAILSAFE_DEADRECKON (31) ---
    "FAILSAFE_DEADRECKON": {
        0: {"name": "FAILSAFE_RESOLVED", "desc": "데드레코닝 페일세이프 해제", "cause": "-", "fix": "-"},
        1: {"name": "FAILSAFE_OCCURRED",
            "desc": "데드레코닝 페일세이프 발생",
            "cause": "GPS 상실 후 추정 항법만으로 비행 중, 신뢰도 저하",
            "fix": "GPS 복구 대기, FS_DR_ENABLE 파라미터 확인"},
    },
}


# =============================================================================
# 3. Pre-Arm 체크 실패 메시지 전체 목록
#    소스: ardupilot.org/copter/docs/common-prearm-safety-checks.html
#    + AP_Arming.cpp, ArduCopter/arming_checks.cpp
# =============================================================================
PREARM_MESSAGES = {
    # --- 센서 관련 ---
    "Baro not healthy": {
        "category": "바로미터",
        "cause": "기압계 센서 하드웨어 오류",
        "fix": "기압계 하드웨어 점검, FC 교체 고려"
    },
    "Alt disparity": {
        "category": "바로미터",
        "cause": "기압계 고도와 관성항법 고도 차이 >1m",
        "fix": "기체 안정화 후 재시도, 가속도계 캘리브레이션"
    },
    "Gyro cal failed": {
        "category": "자이로",
        "cause": "자이로 캘리브레이션 실패 — 기체 움직임",
        "fix": "기체를 완전히 고정한 상태에서 재부팅"
    },
    "Gyros not healthy": {
        "category": "자이로",
        "cause": "자이로 하드웨어 이상",
        "fix": "FC 하드웨어 점검"
    },
    "Gyros inconsistent": {
        "category": "자이로",
        "cause": "자이로 간 회전율 차이 >20deg/s",
        "fix": "IMU 캘리브레이션, 하드웨어 점검"
    },
    "INS not calibrated": {
        "category": "가속도계",
        "cause": "가속도계 미교정 (오프셋 = 0)",
        "fix": "가속도계 캘리브레이션 수행 (6면 캘리브레이션)"
    },
    "Accels not healthy": {
        "category": "가속도계",
        "cause": "가속도계 하드웨어 이상",
        "fix": "FC 재부팅, 하드웨어 점검"
    },
    "Accels inconsistent": {
        "category": "가속도계",
        "cause": "가속도계 간 차이 >1m/s²",
        "fix": "가속도계 재교정"
    },
    "Compass not healthy": {
        "category": "컴파스",
        "cause": "컴파스 센서 이상",
        "fix": "I2C/SPI 배선 점검"
    },
    "Compass not calibrated": {
        "category": "컴파스",
        "cause": "컴파스 미교정 (COMPASS_OFS = 0)",
        "fix": "컴파스 캘리브레이션 수행"
    },
    "Compass offsets too high": {
        "category": "컴파스",
        "cause": "컴파스 오프셋 벡터 길이 >500",
        "fix": "금속 간섭 제거 후 재교정"
    },
    "Compasses inconsistent": {
        "category": "컴파스",
        "cause": "내/외부 컴파스 방향 차이 >45도",
        "fix": "COMPASS_ORIENT 확인, 간섭원 제거"
    },
    "Check mag field": {
        "category": "컴파스",
        "cause": "자기장 강도 예상 범위 밖 (185~874 범위 밖) 또는 WMM DB 기반 예측값 불일치",
        "fix": "자기 간섭원 제거, 다른 위치에서 재교정"
    },

    # --- GPS 관련 ---
    "GPS Glitch": {
        "category": "GPS",
        "cause": "GPS 위치 급변 감지",
        "fix": "GPS 안테나 위치 확인, 하늘시야 확보"
    },
    "Need 3D Fix": {
        "category": "GPS",
        "cause": "GPS 3D Fix 미확보 (GPS 필요 모드에서)",
        "fix": "개활지에서 GPS 위성 수신 대기"
    },
    "Bad Velocity": {
        "category": "GPS/INS",
        "cause": "속도 >50cm/s (기체 정지 상태여야 함)",
        "fix": "기체 완전 정지 확인, 가속도계 재교정"
    },
    "High GPS HDOP": {
        "category": "GPS",
        "cause": "GPS HDOP >2.0",
        "fix": "위성 수신 환경 개선, GPS_HDOP_GOOD 파라미터 확인"
    },
    "GPS not healthy": {
        "category": "GPS",
        "cause": "GPS 센서 비정상 상태 리포트",
        "fix": "GPS 모듈 배선/전원 점검"
    },

    # --- 전원 관련 ---
    "Check Board Voltage": {
        "category": "전원",
        "cause": "보드 전압 <4.3V 또는 >5.8V",
        "fix": "전원 공급 안정화, BEC/파워모듈 점검"
    },
    "Check Battery": {
        "category": "전원",
        "cause": "배터리 전압 이상 감지",
        "fix": "배터리 충전 상태/연결 확인"
    },

    # --- RC 관련 ---
    "RC not calibrated": {
        "category": "RC",
        "cause": "RC 미교정 (채널 min/max 미설정)",
        "fix": "RC 캘리브레이션 수행"
    },
    "Throttle below Failsafe": {
        "category": "RC",
        "cause": "스로틀 채널 PWM < FS_THR_VALUE",
        "fix": "RC 트림/캘리브레이션 확인, FS_THR_VALUE 조정"
    },
    "Channel X below minimum": {
        "category": "RC",
        "cause": "RC 채널 X의 PWM이 미교정 범위 밖",
        "fix": "RC 캘리브레이션 재수행"
    },

    # --- 시스템 관련 ---
    "Hardware safety switch": {
        "category": "시스템",
        "cause": "하드웨어 안전 스위치 미해제",
        "fix": "안전 스위치 버튼 길게 누르기, BRD_SAFETY_DEFLT=0 으로 비활성화 가능"
    },
    "No SD Card": {
        "category": "시스템",
        "cause": "SD 카드 미장착 (로깅 활성 시)",
        "fix": "SD 카드 삽입, FAT32 포맷 확인"
    },
    "Logging failed": {
        "category": "시스템",
        "cause": "로깅 시작 실패",
        "fix": "SD 카드 상태 점검, 포맷"
    },
    "AHRS not healthy": {
        "category": "시스템",
        "cause": "AHRS 초기화 미완료",
        "fix": "기체 정지 상태로 대기, EKF 초기화 완료 기다리기"
    },
    "EKF not healthy": {
        "category": "EKF",
        "cause": "EKF 상태 비정상",
        "fix": "GPS Fix 대기, 진동 저감, 컴파스 점검"
    },

    # --- 미션 관련 ---
    "No mission library present": {
        "category": "미션",
        "cause": "미션 체크 활성이나 미션 미로드",
        "fix": "미션 업로드 또는 미션 체크 비활성화"
    },
    "Missing mission item: xxxx": {
        "category": "미션",
        "cause": "필수 미션 항목 누락",
        "fix": "미션 재작성/업로드"
    },

    # --- 모터/ESC 관련 ---
    "Motors: Check frame class and type": {
        "category": "모터",
        "cause": "프레임 클래스/타입 미설정",
        "fix": "FRAME_CLASS, FRAME_TYPE 파라미터 설정"
    },
    "Motors: unable to setup motor outputs": {
        "category": "모터",
        "cause": "모터 출력 채널 설정 실패",
        "fix": "SERVOx_FUNCTION 파라미터 확인"
    },
}


# =============================================================================
# 4. EKF 상태 코드와 플래그
#    소스: AP_NavEKF3/LogStructure.h, ekf_check.cpp
# =============================================================================
EKF_STATUS = {
    "XKF4_FIELDS": {
        "SV": {
            "name": "Velocity Innovation Test Ratio",
            "desc": "속도 이노베이션 테스트 비율 (제곱근)",
            "threshold": "<1.0 정상 (측정값 수용), >1.0 이상 (측정값 거부)",
            "good": "<0.3 (비행 중 양호)",
        },
        "SP": {
            "name": "Position Innovation Test Ratio",
            "desc": "위치 이노베이션 테스트 비율 (제곱근)",
            "threshold": "<1.0 정상, >1.0 이상",
            "good": "<0.3",
        },
        "SH": {
            "name": "Height Innovation Test Ratio",
            "desc": "고도 이노베이션 테스트 비율 (제곱근)",
            "threshold": "<1.0 정상, >1.0 이상",
            "good": "<0.3",
        },
        "SM": {
            "name": "Magnetometer Innovation Test Ratio",
            "desc": "자력계 이노베이션 테스트 비율 (제곱근)",
            "threshold": "<1.0 정상, >1.0 이상",
            "good": "<0.3",
        },
        "SVT": {
            "name": "Velocity Innovation Test Ratio (vertical)",
            "desc": "수직 속도 이노베이션 테스트 비율",
            "threshold": "<1.0 정상",
            "good": "<0.3",
        },
        "errRP": {
            "name": "Roll/Pitch Error",
            "desc": "롤/피치 추정 오차 (rad)",
            "threshold": "낮을수록 좋음",
            "good": "<0.01 rad",
        },
        "errYaw": {
            "name": "Yaw Error",
            "desc": "요 추정 오차 (rad)",
            "threshold": "낮을수록 좋음",
            "good": "<0.05 rad",
        },
    },

    "XKF4_TS_BITS": {
        "desc": "TS 필드 — 타임아웃 상태 비트마스크",
        0: "위치(Position) 측정 타임아웃",
        1: "속도(Velocity) 측정 타임아웃",
        2: "고도(Height) 측정 타임아웃",
        3: "자력계(Magnetometer) 측정 타임아웃",
        4: "에어스피드(Airspeed) 측정 타임아웃",
        5: "드래그(Drag) 측정 타임아웃",
    },

    "XKF4_SS_BITS": {
        "desc": "SS 필드 — 솔루션 상태 비트마스크",
        0: "attitude 추정 사용 가능",
        1: "horizontal velocity 추정 사용 가능",
        2: "vertical velocity 추정 사용 가능",
        3: "horizontal position (relative) 추정 사용 가능",
        4: "horizontal position (absolute) 추정 사용 가능",
        5: "vertical position (absolute) 추정 사용 가능",
        6: "vertical position (above terrain) 추정 사용 가능",
        7: "constant position mode (no GPS)",
        8: "position estimate good for autopilot",
        9: "const pos mode(non-GPS, estimate position)",
        10: "EKF is in ground effect mitigation mode",
        11: "takeoff detected",
        12: "touchdown detected",
    },

    "EKF_FAILSAFE_PARAMS": {
        "FS_EKF_ACTION": {
            "values": {
                0: "보고만 (Report Only) — 모드 변경 없음",
                1: "Land 모드로 전환 (기본값)",
                2: "AltHold 모드로 전환",
                3: "Land 모드 (Stabilize에서도 강제 전환)",
            },
            "desc": "EKF 페일세이프 발생 시 동작"
        },
        "FS_EKF_THRESH": {
            "default": 0.8,
            "range": "0.6 ~ 1.0",
            "desc": "EKF variance 임계치 — compass/position/velocity 중 2개 이상이 이 값 초과 시 1초 후 페일세이프",
        },
    },
}


# =============================================================================
# 5. Failsafe 종류별 상세 동작
#    소스: ardupilot.org 공식 문서 (Copter)
# =============================================================================
FAILSAFE_TYPES = {
    "RC_RADIO": {
        "trigger": "RC 신호 손실 (FS_THR_VALUE 이하 또는 No Pulse)",
        "params": {
            "FS_THR_ENABLE": {
                0: "비활성",
                1: "RTL (GPS 없으면 Land)",
                2: "Continue (Auto 미션 중)",
                3: "Land 즉시",
                4: "SmartRTL → RTL → Land",
                5: "SmartRTL → Land",
                6: "Auto DO_LAND_START → RTL",
            },
            "FS_THR_VALUE": "PWM 값 (기본 975) — 이 값 이하면 페일세이프 트리거",
        },
        "behavior": "비행 중: 설정된 모드로 전환 / 이미 착륙 중이면 착륙 계속",
    },

    "BATTERY": {
        "trigger": "배터리 전압 BATT_LOW_VOLT 이하 10초 이상 또는 잔량 BATT_LOW_MAH 이하",
        "params": {
            "BATT_FS_LOW_ACT": {
                0: "없음 (경고만)",
                1: "Land",
                2: "RTL",
                3: "SmartRTL → RTL → Land",
                4: "SmartRTL → Land",
                5: "Terminate (모터 정지!)",
            },
            "BATT_FS_CRT_ACT": {
                "desc": "2단계 (Critical) 동작 — 동일 옵션 + 더 낮은 전압/잔량",
            },
            "BATT_LOW_VOLT": "저전압 임계치 (V) 기본 0=비활성",
            "BATT_LOW_MAH": "저잔량 임계치 (mAh) 기본 0=비활성",
            "BATT_CRT_VOLT": "위험 전압 임계치 (V)",
            "BATT_CRT_MAH": "위험 잔량 임계치 (mAh)",
            "BATT_LOW_TIMER": "저전압 유지 시간 (초) 기본 10",
            "BATT_FS_VOLTSRC": "0=원시전압, 1=새그 보정 전압",
        },
        "behavior": "트리거 후 리부팅 전까지 리셋 불가. 모드 스위치로 수동 제어 가능.",
    },

    "GCS": {
        "trigger": "GCS 하트비트 FS_GCS_TIMEOUT초 이상 수신 없음",
        "params": {
            "FS_GCS_ENABLE": {
                0: "비활성",
                1: "RTL",
                2: "Continue (Auto)",
                3: "Land",
                4: "SmartRTL → RTL → Land",
                5: "SmartRTL → Land",
            },
            "FS_GCS_TIMEOUT": "타임아웃 (초) 기본 5",
        },
        "behavior": "Disarmed 상태에서는 동작 안 함",
    },

    "EKF": {
        "trigger": "EKF variance 중 compass/position/velocity 2개 이상 FS_EKF_THRESH 초과 1초 이상",
        "params": {
            "FS_EKF_ACTION": {0: "보고만", 1: "Land (기본)", 2: "AltHold", 3: "Land(강제)"},
            "FS_EKF_THRESH": "기본 0.8 (범위 0.6~1.0)",
        },
        "behavior": "GPS 필요 모드(Loiter, PosHold, RTL, Guided, Auto)에서만 동작",
    },

    "TERRAIN": {
        "trigger": "지형 데이터 수신 불가 상태 지속",
        "params": {
            "FS_TERRAIN_ENAB": {0: "비활성", 1: "RTL", 2: "Land"},
            "TERRAIN_ENABLE": "지형 팔로우 활성화",
        },
        "behavior": "Terrain Following 사용 중일 때만 의미 있음",
    },

    "LEAK": {
        "trigger": "누수 센서 트리거 (ArduSub 전용)",
        "params": {
            "FS_LEAK_ENABLE": {0: "비활성", 1: "Surface"},
        },
        "behavior": "즉시 수면으로 상승",
    },

    "VIBE": {
        "trigger": "IMU 가속도 클리핑 빈도 초과 (AcClip 필드)",
        "params": {},
        "behavior": "FS_VIBE_ENABLE로 제어, Land 모드 전환",
    },

    "DEAD_RECKONING": {
        "trigger": "GPS 상실 후 Dead Reckoning 모드 진입, 위치 신뢰도 저하",
        "params": {
            "FS_DR_ENABLE": {0: "비활성", 1: "RTL", 2: "Land"},
            "FS_DR_TIMEOUT": "DR 유지 최대 시간 (초)",
        },
        "behavior": "ArduCopter 4.4+ 지원",
    },

    "FS_OPTIONS": {
        "desc": "비트마스크 — 여러 페일세이프 동작 수정",
        "bits": {
            0: "Continue if in auto mode on RC failsafe (RC FS 시 Auto 계속)",
            1: "Continue if in auto mode on GCS failsafe (GCS FS 시 Auto 계속)",
            2: "Continue if in guided mode on RC failsafe",
            3: "Continue if landing on any failsafe",
            4: "Continue if in pilot controlled modes on GCS failsafe",
            5: "Release gripper on any failsafe",
        },
    },
}


# =============================================================================
# 6. 이벤트(EV) 메시지 ID 전체 목록
#    소스: AP_Logger.h → enum class LogEvent
# =============================================================================
LOG_EVENT = {
    10: "ARMED — 시동 걸림",
    11: "DISARMED — 시동 꺼짐",
    15: "AUTO_ARMED — 자동 시동",
    17: "LAND_COMPLETE_MAYBE — 착륙 감지 (추정)",
    18: "LAND_COMPLETE — 착륙 확인",
    19: "LOST_GPS — GPS 신호 상실",
    21: "FLIP_START — 플립 시작",
    22: "FLIP_END — 플립 종료",
    25: "SET_HOME — 홈 위치 설정",
    26: "SET_SIMPLE_ON — Simple 모드 활성",
    27: "SET_SIMPLE_OFF — Simple 모드 비활성",
    28: "NOT_LANDED — 이륙 감지 (비착륙 상태)",
    29: "SET_SUPERSIMPLE_ON — SuperSimple 모드 활성",
    30: "AUTOTUNE_INITIALISED — 오토튠 초기화",
    31: "AUTOTUNE_OFF — 오토튠 비활성",
    32: "AUTOTUNE_RESTART — 오토튠 재시작",
    33: "AUTOTUNE_SUCCESS — 오토튠 성공",
    34: "AUTOTUNE_FAILED — 오토튠 실패",
    35: "AUTOTUNE_REACHED_LIMIT — 오토튠 한계 도달",
    36: "AUTOTUNE_PILOT_TESTING — 오토튠 파일럿 테스트 중",
    37: "AUTOTUNE_SAVEDGAINS — 오토튠 게인 저장",
    38: "SAVE_TRIM — 트림 저장",
    39: "SAVEWP_ADD_WP — 웨이포인트 저장/추가",
    41: "FENCE_ENABLE — 펜스 활성",
    42: "FENCE_DISABLE — 펜스 비활성",
    43: "ACRO_TRAINER_OFF — 아크로 트레이너 끔",
    44: "ACRO_TRAINER_LEVELING — 아크로 트레이너 레벨링",
    45: "ACRO_TRAINER_LIMITED — 아크로 트레이너 제한",
    46: "GRIPPER_GRAB — 그리퍼 잡기",
    47: "GRIPPER_RELEASE — 그리퍼 놓기",
    49: "PARACHUTE_DISABLED — 낙하산 비활성",
    50: "PARACHUTE_ENABLED — 낙하산 활성",
    51: "PARACHUTE_RELEASED — 낙하산 전개",
    52: "LANDING_GEAR_DEPLOYED — 랜딩기어 내림",
    53: "LANDING_GEAR_RETRACTED — 랜딩기어 올림",
    54: "MOTORS_EMERGENCY_STOPPED — 모터 비상 정지",
    55: "MOTORS_EMERGENCY_STOP_CLEARED — 모터 비상 정지 해제",
    56: "MOTORS_INTERLOCK_DISABLED — 모터 인터록 비활성",
    57: "MOTORS_INTERLOCK_ENABLED — 모터 인터록 활성",
    58: "ROTOR_RUNUP_COMPLETE — 로터 런업 완료 (헬리)",
    59: "ROTOR_SPEED_BELOW_CRITICAL — 로터 속도 위험 이하 (헬리)",
    60: "EKF_ALT_RESET — EKF 고도 리셋",
    61: "LAND_CANCELLED_BY_PILOT — 착륙 취소 (파일럿)",
    62: "EKF_YAW_RESET — EKF 요 리셋",
    63: "AVOIDANCE_ADSB — ADS-B 회피 동작",
    64: "AVOIDANCE_PROXIMITY — 근접 센서 회피 동작",
    65: "GPS_PRIMARY_CHANGED — 주 GPS 변경",
    66: "WINCH_RELAXED — 윈치 이완",
    67: "WINCH_LENGTH_CONTROL — 윈치 길이 제어",
    68: "WINCH_RATE_CONTROL — 윈치 속도 제어",
    69: "ZIGZAG_STORE_A — 지그재그 A점 저장",
    70: "ZIGZAG_STORE_B — 지그재그 B점 저장",
    71: "LAND_REPO_ACTIVE — 착륙 리포지션 활성",
    72: "STANDBY_ENABLE — 대기 모드 활성",
    73: "STANDBY_DISABLE — 대기 모드 비활성",
    74: "FENCE_FLOOR_ENABLE — 바닥 펜스 활성",
    75: "FENCE_FLOOR_DISABLE — 바닥 펜스 비활성",
    80: "EKF_LANE_SWITCH — EKF 레인 전환",
    # 81+: 차량 타입별 추가 이벤트 (Plane, Rover, Sub 등)
    163: "SURFACED — 수면 도달 (Sub)",
    164: "NOT_SURFACED — 수면 이탈 (Sub)",
    165: "BOTTOMED — 바닥 도달 (Sub)",
    166: "NOT_BOTTOMED — 바닥 이탈 (Sub)",
}


# =============================================================================
# 7. 모터/ESC 관련 에러
#    소스: ardupilot.org/copter/docs/common-esc-telemetry.html
# =============================================================================
MOTOR_ESC_ERRORS = {
    "THRUST_LOSS": {
        "subsys_id": 25,
        "desc": "추력 손실 감지 — 모터 1개 이상 출력 부족",
        "cause": "프롭 탈락, 모터 소손, ESC 불량, 케이블 단선",
        "log_msg": "ERR: THRUST_LOSS_CHECK",
        "detection": "모터 출력 PWM이 최대인데 기체 기울어짐 감지",
    },
    "ESC_TELEMETRY_ERRORS": {
        "desc": "ESC 텔레메트리 기반 에러 감지",
        "fields": {
            "ESCn.Err": "ESC 에러 카운트",
            "ESCn.RPM": "모터 RPM (0이면 모터 정지/센서 이상)",
            "ESCn.Curr": "ESC 전류 (비정상 고전류 = 단락 의심)",
            "ESCn.Temp": "ESC 온도 (과열 감지)",
            "ESCn.Volt": "ESC 전압",
        },
        "protocols": ["BLHeli32 텔레메트리", "DShot 텔레메트리", "Bi-directional DShot", "DroneCAN ESC"],
    },
    "MOTOR_IMBALANCE": {
        "desc": "모터 불균형 감지 방법",
        "method": "RCOUT 채널 간 PWM 차이 분석",
        "threshold": "정상: 모터 간 PWM 차이 <100μs / 주의: 100~200μs / 위험: >200μs",
        "cause": "프롭 밸런싱 불량, 모터 베어링 마모, ESC 캘리브레이션 불량, 무게중심 편차",
    },
}


# =============================================================================
# 8. GPS 관련 상태코드
#    소스: MAVLink GPS_FIX_TYPE enum, ardupilot.org
# =============================================================================
GPS_FIX_TYPE = {
    0: {"name": "NO_GPS",      "desc": "GPS 미연결",           "usable": False},
    1: {"name": "NO_FIX",      "desc": "GPS 연결, 위치 없음",   "usable": False},
    2: {"name": "2D_FIX",      "desc": "2D 위치 (고도 없음)",    "usable": False},
    3: {"name": "3D_FIX",      "desc": "3D 위치 확보",          "usable": True},
    4: {"name": "DGPS",        "desc": "DGPS/SBAS 보정 3D 위치", "usable": True},
    5: {"name": "RTK_FLOAT",   "desc": "RTK Float (cm급 정밀도)", "usable": True},
    6: {"name": "RTK_FIXED",   "desc": "RTK Fixed (mm급 정밀도)", "usable": True},
}

GPS_HDOP_QUALITY = {
    "excellent":  {"range": "< 1.0",   "desc": "매우 우수 — 정밀 위치"},
    "good":       {"range": "1.0~1.5", "desc": "양호 — 일반 비행 가능"},
    "moderate":   {"range": "1.5~2.0", "desc": "보통 — 주의 필요"},
    "poor":       {"range": "2.0~5.0", "desc": "불량 — 위치 부정확, 비행 비권장"},
    "very_poor":  {"range": "> 5.0",   "desc": "매우 불량 — 위치 신뢰 불가"},
}

GPS_PARAMS = {
    "GPS_HDOP_GOOD": {"default": 1.4, "desc": "Pre-Arm 통과 HDOP 임계치 (이하여야 함)"},
    "GPS_TYPE": {"desc": "GPS 타입 (0=None, 1=Auto, 2=uBlox, ...)"},
    "GPS_GNSS_MODE": {"desc": "사용할 GNSS 시스템 비트마스크 (GPS, GLONASS, BeiDou, Galileo)"},
}


# =============================================================================
# 9. 진동(VIBE) 판정 기준
#    소스: ardupilot.org/copter/docs/common-measuring-vibration.html
# =============================================================================
VIBE_THRESHOLDS = {
    "accel_vibration": {
        "unit": "m/s² (VibeX, VibeY, VibeZ 필드)",
        "good":      {"range": "< 15",   "desc": "양호 — 정상 비행"},
        "caution":   {"range": "15~30",  "desc": "주의 — 방진 개선 권장"},
        "bad":       {"range": "30~60",  "desc": "위험 — 위치/고도 유지 문제 가능"},
        "critical":  {"range": "> 60",   "desc": "비행불가 — 거의 확실히 문제 발생"},
    },
    "clipping": {
        "fields": "Clip0, Clip1, Clip2 (각 IMU별)",
        "desc": "가속도가 센서 최대 범위(±16G) 도달 횟수",
        "good":      {"value": "0",    "desc": "양호 — 클리핑 없음"},
        "acceptable": {"value": "<100", "desc": "허용 — 하드 랜딩 등 순간적이면 OK"},
        "bad":       {"value": "지속 증가", "desc": "위험 — 심각한 진동 문제, 즉시 수정 필요"},
    },
    "axis_diagnosis": {
        "X_AND_Y_high": "모터 베어링 또는 프롭 밸런스 문제",
        "X_OR_Y_high":  "FC 마운팅 문제 (특정 방향 진동 전달)",
        "Z_high":       "프로펠러 트랙 문제 (블레이드 굽힘) 또는 모터 수직 유격",
    },
    "vibration_failsafe": {
        "param": "FS_VIBE_ENABLE",
        "desc": "진동 페일세이프 — IMU 클리핑 빈도 기반",
        "action": "Land 모드 전환",
    },
}


# =============================================================================
# 10. RCOUT 채널별 의미와 정상 범위
#     소스: ardupilot.org/copter/docs/connect-escs-and-motors.html
#           ardupilot.org/copter/docs/common-rcoutput-mapping.html
# =============================================================================
RCOUT_CHANNELS = {
    "quad_x_motor_order": {
        "desc": "QuadX 프레임 기준 모터 순서 (SERVOx_FUNCTION 기준)",
        1: {"position": "우측 전방 (Front Right)",  "direction": "CCW"},
        2: {"position": "좌측 후방 (Rear Left)",    "direction": "CCW"},
        3: {"position": "좌측 전방 (Front Left)",   "direction": "CW"},
        4: {"position": "우측 후방 (Rear Right)",   "direction": "CW"},
    },

    "servo_functions": {
        "desc": "SERVOx_FUNCTION 주요 값",
        0:   "Disabled",
        33:  "Motor1",
        34:  "Motor2",
        35:  "Motor3",
        36:  "Motor4",
        37:  "Motor5",
        38:  "Motor6",
        39:  "Motor7",
        40:  "Motor8",
        51:  "RCIN1 (RC 패스스루)",
        52:  "RCIN2",
        53:  "RCIN3",
        54:  "RCIN4",
        # 73~78: Throttle Left/Right 등 (Rover)
        # 94: Script 1
    },

    "pwm_ranges": {
        "desc": "PWM 정상 범위 (ESC 캘리브레이션 기준)",
        "min":      1000,  # μs — 모터 정지
        "idle":     1100,  # μs — 최소 회전 (MOT_SPIN_ARM 기준)
        "hover":    "1300~1600",  # μs — 일반적 호버 범위
        "max":      2000,  # μs — 최대 출력
        "deadzone": "1000~MOT_SPIN_ARM — 모터 회전 안 함",
        "normal_hover_pct": "50~70% 스로틀이 정상 (MOT_THST_HOVER ≈ 0.35~0.50)",
    },

    "analysis_tips": {
        "모터 불균형": "RCOUT 채널 간 PWM 차이 >100μs → 무게중심 편차 또는 모터/프롭 문제",
        "최대 출력 도달": "RCOUT가 지속적으로 PWM_MAX 근처 → 추력 부족, 과적재",
        "비대칭 패턴": "대각선 모터 쌍의 PWM이 비슷해야 함 (Quad: 1+2 ≈ 3+4)",
        "호버 스로틀": "MOT_THST_HOVER > 0.60 → 과적재 경고",
    },
}


# =============================================================================
# Internal Error 비트 (보너스)
# 소스: AP_InternalError.h
# =============================================================================
INTERNAL_ERROR_BITS = {
    0:  "mapfailure",
    1:  "main_loop_stuck",
    2:  "mem_guard",
    3:  "params_restored",
    4:  "watchdog_reset",
    5:  "iomcu_reset",
    6:  "iomcu_fail",
    7:  "spi_fail",
    8:  "panic",
    9:  "stack_overflow",
    10: "isr_missed",
    11: "flow_of_control",
    12: "sfs_recursion",
    13: "bad_rotation",
    14: "ftp_failure",
    15: "ftp_too_many_replies",
    16: "log_too_long",
}


# =============================================================================
# ArduCopter 비행 모드 번호 (참고용)
# =============================================================================
COPTER_FLIGHT_MODES = {
    0:  "STABILIZE",
    1:  "ACRO",
    2:  "ALT_HOLD",
    3:  "AUTO",
    4:  "GUIDED",
    5:  "LOITER",
    6:  "RTL",
    7:  "CIRCLE",
    9:  "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",        # 헬리
    27: "AUTO_RTL",
    28: "TURTLE",
}


# =============================================================================
# 유틸리티 함수
# =============================================================================
def lookup_err(subsys_id: int, ecode: int) -> dict:
    """ERR 메시지의 Subsys ID와 ECode로 상세 정보 조회

    사용법:
        info = lookup_err(6, 1)
        # → {'subsystem': 'FAILSAFE_BATT', 'error': {'name': 'FAILSAFE_BATT_LOW', ...}}
    """
    subsys_name = ERR_SUBSYSTEM.get(subsys_id, f"UNKNOWN_{subsys_id}")
    codes = ERR_CODES.get(subsys_name, {})
    error_info = codes.get(ecode, {"name": "UNKNOWN", "desc": f"ECode {ecode} 미등록"})
    return {
        "subsystem_id": subsys_id,
        "subsystem": subsys_name,
        "ecode": ecode,
        "error": error_info,
    }


def lookup_event(ev_id: int) -> str:
    """이벤트 ID로 설명 조회"""
    return LOG_EVENT.get(ev_id, f"UNKNOWN_EVENT_{ev_id}")


def lookup_gps_fix(fix_type: int) -> dict:
    """GPS Fix 타입 조회"""
    return GPS_FIX_TYPE.get(fix_type, {"name": "UNKNOWN", "desc": f"Fix type {fix_type} 미등록"})


def check_vibe_level(vibe_value: float) -> str:
    """진동 수준 판정"""
    if vibe_value < 15:
        return "양호 (GOOD)"
    elif vibe_value < 30:
        return "주의 (CAUTION)"
    elif vibe_value < 60:
        return "위험 (BAD) — 위치/고도 유지 문제 가능"
    else:
        return "비행불가 (CRITICAL) — 즉시 착륙 필요"


def check_hdop(hdop: float) -> str:
    """HDOP 수준 판정"""
    if hdop < 1.0:
        return "매우 우수 (EXCELLENT)"
    elif hdop < 1.5:
        return "양호 (GOOD)"
    elif hdop < 2.0:
        return "보통 (MODERATE) — 주의"
    elif hdop < 5.0:
        return "불량 (POOR) — 비행 비권장"
    else:
        return "매우 불량 (VERY POOR) — 비행 금지"


# =============================================================================
# 사용 예시
# =============================================================================
if __name__ == "__main__":
    print("=== ArduPilot 에러코드 조회 테스트 ===\n")

    # ERR 메시지 조회
    result = lookup_err(6, 1)
    print(f"ERR Subsys=6, ECode=1 → {result['subsystem']}: {result['error']['name']}")
    print(f"  원인: {result['error']['cause']}")
    print(f"  해결: {result['error']['fix']}")
    print()

    # 이벤트 조회
    print(f"EV 10 → {lookup_event(10)}")
    print(f"EV 18 → {lookup_event(18)}")
    print()

    # GPS 조회
    gps = lookup_gps_fix(5)
    print(f"GPS Fix 5 → {gps['name']}: {gps['desc']}")
    print()

    # 진동 판정
    print(f"VIBE 12 → {check_vibe_level(12)}")
    print(f"VIBE 35 → {check_vibe_level(35)}")
    print(f"VIBE 65 → {check_vibe_level(65)}")
    print()

    # HDOP 판정
    print(f"HDOP 0.8 → {check_hdop(0.8)}")
    print(f"HDOP 2.5 → {check_hdop(2.5)}")
