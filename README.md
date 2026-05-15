# Flight Log Analyzer — ArduPilot & PX4

ArduPilot (.bin/.tlog) 및 PX4 (.ulg) 비행 로그를 업로드하면 **종합 분석 보고서**를 자동 생성합니다.
단순 수치 나열이 아닌, **근본 원인 분석**과 **비행 복기**까지 제공합니다.

## 주요 기능

### 분석 엔진 (20+ 자동 체크)
- **기본 분석** — 에러/이벤트, 진동, 배터리, EKF, GPS, 컴퍼스, 모터 균형, PID 튜닝, 바람 추정
- **착륙 분석** — 하강 속도, 바운스 감지, 착륙 정확도 (이륙점 대비)
- **크래시 자동 감지** — ATT 급변 + 고도 급락 패턴 매칭
- **파라미터 체크리스트** — 19개 핵심 파라미터 권장값 비교 (페일세이프/배터리/필터)
- **호버 안정성** — LOITER/POSHOLD 구간 GPS 위치 분산 (CEP50/CEP95)
- **ESC 텔레메트리** — 인스턴스별 온도, RPM, 전류 + RPM 불균형 감지
- **비행 구간 분류** — 스로틀+고도+속도 기반 7단계 자동 분류

### 종합 분석 엔진
- **이벤트 타임라인** — ERR, 이벤트, 모드변경, MSG, 센서 이상을 시간순 통합
- **근본 원인 분석** — 에러 발생 ±3초 내 모터/진동/배터리/EKF 교차 분석, 확신도 + 조치 순서 제시
- **비행 복기** — 전체 비행을 자연어로 서술 ("몇 분 몇 초에 무슨 일이 있었는지")

### 차트 (14종)
GPS 경로, 고도, 진동, 배터리, 자세, 모터 출력, EKF, 바람, PID 추적, 착륙 분석, ESC RPM/온도, 호버 산점도, 비행 구간 타임라인, 모드 타임라인

## 사용법

### CLI (PDF 보고서)
```bash
python3 analyze.py input/my_flight.bin
python3 analyze.py input/my_flight.bin -o output/custom_name.pdf
```

### 웹 서비스
```bash
cd web
pip install fastapi uvicorn httpx
uvicorn app:app --host 0.0.0.0 --port 8040
```
브라우저에서 `http://localhost:8040` 접속 → .bin 파일 업로드 → 즉시 분석

### API
```bash
curl -X POST http://localhost:8040/api/analyze -F "file=@flight.bin"
```

## 설치

```bash
pip install pymavlink matplotlib reportlab numpy
# 웹 서비스 추가
pip install fastapi uvicorn httpx
```

## 구조

```
├── analyze.py              # 분석 엔진 + 차트 + PDF
├── ardupilot_error_codes.py # ArduPilot 에러코드 레퍼런스
├── web/
│   ├── app.py              # FastAPI 웹 서비스
│   └── templates/
│       └── index.html      # 웹 UI
├── input/                  # .bin 파일 (gitignore)
└── output/                 # PDF 보고서 (gitignore)
```

## 라이선스

MIT License

Made by Kim.
