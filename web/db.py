"""
로그 분석 DB — 사용자별 로그 추적 + 기체별 정비/비행술 관리
SQLite 기반, 파일 하나로 운영
"""
import sqlite3
import json
from pathlib import Path

DB_PATH = Path(__file__).resolve().parent / "logdb.sqlite"


def get_db():
    conn = sqlite3.connect(str(DB_PATH))
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode=WAL")
    return conn


def init_db():
    conn = get_db()
    conn.executescript("""
    CREATE TABLE IF NOT EXISTS logs (
        id          INTEGER PRIMARY KEY AUTOINCREMENT,
        job_id      TEXT NOT NULL UNIQUE,
        user_id     TEXT NOT NULL,
        vehicle_id  INTEGER DEFAULT 0,       -- SYSID_THISMAV
        filename    TEXT NOT NULL,
        filesize_mb REAL,
        uploaded_at TEXT NOT NULL DEFAULT (datetime('now','localtime')),

        -- 비행 요약
        duration    TEXT,
        dur_sec     REAL DEFAULT 0,
        max_alt     REAL,
        max_spd     REAL,
        dist_m      REAL,
        v_min       REAL,
        v_max       REAL,

        -- 판정
        fail_count  INTEGER DEFAULT 0,
        warn_count  INTEGER DEFAULT 0,
        ok_count    INTEGER DEFAULT 0,
        overall     TEXT,
        score       INTEGER,

        -- 핵심 진단
        root_cause  TEXT,
        story       TEXT,

        -- 기체 건강 지표 (정비 소요용)
        vibe_avg    REAL,           -- 평균 진동
        motor_diff  REAL,           -- 모터 불균형 (PWM 차이)
        motor_sat_pct REAL,         -- 모터 포화율 (%)
        compass_r   REAL,           -- 컴퍼스-모터 간섭 상관계수
        baro_drift  REAL,           -- 기압계 드리프트 (m)
        alt_rms     REAL,           -- 고도 추적 RMS
        fs_count    INTEGER DEFAULT 0, -- 페일세이프 횟수
        mah_used    REAL DEFAULT 0, -- 소비 mAh

        -- 비행술 지표
        hard_landing_count INTEGER DEFAULT 0,  -- 하드랜딩 횟수
        bounce_count INTEGER DEFAULT 0,        -- 바운스 횟수
        rapid_maneuver_count INTEGER DEFAULT 0, -- 급기동 횟수
        hover_cep95 REAL,                      -- 호버 CEP95
        landing_speed REAL,                    -- 착륙 하강속도
        batt_end_pct REAL                      -- 비행 종료 시 배터리 잔량
    );

    CREATE INDEX IF NOT EXISTS idx_user_id ON logs(user_id);
    CREATE INDEX IF NOT EXISTS idx_uploaded ON logs(uploaded_at DESC);
    """)
    # 기존 테이블에 없는 컬럼 추가 (마이그레이션)
    existing = [r[1] for r in conn.execute("PRAGMA table_info(logs)").fetchall()]
    new_cols = {
        'vehicle_id': 'INTEGER DEFAULT 0',
        'dur_sec': 'REAL DEFAULT 0',
        'vibe_avg': 'REAL', 'motor_diff': 'REAL', 'motor_sat_pct': 'REAL',
        'compass_r': 'REAL', 'baro_drift': 'REAL', 'alt_rms': 'REAL',
        'fs_count': 'INTEGER DEFAULT 0', 'mah_used': 'REAL DEFAULT 0',
        'hard_landing_count': 'INTEGER DEFAULT 0', 'bounce_count': 'INTEGER DEFAULT 0',
        'rapid_maneuver_count': 'INTEGER DEFAULT 0', 'hover_cep95': 'REAL',
        'landing_speed': 'REAL', 'batt_end_pct': 'REAL',
    }
    for col, dtype in new_cols.items():
        if col not in existing:
            try:
                conn.execute(f"ALTER TABLE logs ADD COLUMN {col} {dtype}")
            except Exception:
                pass
    # 마이그레이션 후 인덱스 생성 (vehicle_id 컬럼 보장)
    for idx_sql in [
        "CREATE INDEX IF NOT EXISTS idx_vehicle_id ON logs(vehicle_id)",
        "CREATE INDEX IF NOT EXISTS idx_user_vehicle ON logs(user_id, vehicle_id)",
    ]:
        try:
            conn.execute(idx_sql)
        except Exception:
            pass
    conn.commit()
    conn.close()


def save_log(job_id: str, user_id: str, filename: str, filesize_mb: float,
             summary: dict, counts: dict, scores: dict,
             root_cause_text: str = '', story_text: str = '',
             vehicle_id: int = 0, health_data: dict = None,
             pilot_data: dict = None):
    """분석 결과를 DB에 저장"""
    conn = get_db()
    h = health_data or {}
    p = pilot_data or {}

    if counts['fail'] > 0:
        overall = 'FAIL'
    elif counts['warn'] > 0:
        overall = 'WARN'
    else:
        overall = 'OK'

    conn.execute("""
    INSERT OR REPLACE INTO logs
    (job_id, user_id, vehicle_id, filename, filesize_mb,
     duration, dur_sec, max_alt, max_spd, dist_m, v_min, v_max,
     fail_count, warn_count, ok_count, overall, score,
     root_cause, story,
     vibe_avg, motor_diff, motor_sat_pct, compass_r, baro_drift, alt_rms,
     fs_count, mah_used,
     hard_landing_count, bounce_count, rapid_maneuver_count,
     hover_cep95, landing_speed, batt_end_pct)
    VALUES (?,?,?,?,?, ?,?,?,?,?,?,?, ?,?,?,?,?, ?,?, ?,?,?,?,?,?, ?,?, ?,?,?,?,?,?)
    """, (
        job_id, user_id.strip().lower(), vehicle_id, filename, round(filesize_mb, 1),
        summary.get('duration', ''),
        summary.get('dur_sec', 0),
        summary.get('max_alt', 0),
        summary.get('max_spd', 0),
        summary.get('dist_m', 0),
        summary.get('v_min', 0),
        summary.get('v_max', 0),
        counts.get('fail', 0), counts.get('warn', 0), counts.get('ok', 0),
        overall, scores.get('overall', 0),
        root_cause_text[:500], story_text[:500],
        h.get('vibe_avg'), h.get('motor_diff'), h.get('motor_sat_pct'),
        h.get('compass_r'), h.get('baro_drift'), h.get('alt_rms'),
        h.get('fs_count', 0), h.get('mah_used', 0),
        p.get('hard_landing_count', 0), p.get('bounce_count', 0),
        p.get('rapid_maneuver_count', 0),
        p.get('hover_cep95'), p.get('landing_speed'), p.get('batt_end_pct'),
    ))
    conn.commit()
    conn.close()


def get_user_logs(user_id: str) -> list:
    conn = get_db()
    rows = conn.execute("""
    SELECT job_id, vehicle_id, filename, filesize_mb, uploaded_at,
           duration, max_alt, dist_m, v_min,
           fail_count, warn_count, ok_count, overall, score,
           root_cause, story
    FROM logs WHERE user_id = ?
    ORDER BY uploaded_at DESC LIMIT 50
    """, (user_id.strip().lower(),)).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_vehicle_report(user_id: str, vehicle_id: int) -> dict:
    """기체별 종합 리포트 — 정비 소요 + 비행술 지도"""
    conn = get_db()
    rows = conn.execute("""
    SELECT * FROM logs
    WHERE user_id = ? AND vehicle_id = ?
    ORDER BY uploaded_at ASC
    """, (user_id.strip().lower(), vehicle_id)).fetchall()
    conn.close()

    if not rows:
        return None

    logs = [dict(r) for r in rows]
    n = len(logs)

    # ── 누적 통계 ──
    total_flights = n
    total_dur_sec = sum(l.get('dur_sec', 0) or 0 for l in logs)
    total_hours = total_dur_sec / 3600
    total_dist = sum(l.get('dist_m', 0) or 0 for l in logs)
    total_mah = sum(l.get('mah_used', 0) or 0 for l in logs)
    total_fails = sum(l.get('fail_count', 0) or 0 for l in logs)
    total_warns = sum(l.get('warn_count', 0) or 0 for l in logs)

    # ── 정비 소요 분석 ──
    maintenance = []

    # 모터 수명 (50h 기준)
    motor_life_h = 50
    motor_pct = min(100, total_hours / motor_life_h * 100)
    if motor_pct > 80:
        maintenance.append({'part': '모터/베어링', 'urgency': 'HIGH',
                            'detail': f'누적 {total_hours:.1f}h / {motor_life_h}h ({motor_pct:.0f}%)',
                            'action': '모터 베어링 교체 또는 모터 교체 권장'})
    elif motor_pct > 50:
        maintenance.append({'part': '모터/베어링', 'urgency': 'MEDIUM',
                            'detail': f'누적 {total_hours:.1f}h / {motor_life_h}h ({motor_pct:.0f}%)',
                            'action': '다음 정기 점검 시 베어링 확인'})

    # 진동 추세
    vibes = [l['vibe_avg'] for l in logs if l.get('vibe_avg') is not None]
    if len(vibes) >= 3:
        recent = vibes[-3:]
        early = vibes[:3]
        avg_early = sum(early) / len(early)
        avg_recent = sum(recent) / len(recent)
        if avg_early > 0:
            vibe_change = (avg_recent - avg_early) / avg_early * 100
            if vibe_change > 30:
                maintenance.append({'part': '프롭/방진마운트', 'urgency': 'HIGH',
                                    'detail': f'진동 {vibe_change:+.0f}% 증가 ({avg_early:.1f}→{avg_recent:.1f})',
                                    'action': '프롭 교체, 밸런싱, FC 방진마운트 점검'})
            elif vibe_change > 15:
                maintenance.append({'part': '프롭/방진마운트', 'urgency': 'MEDIUM',
                                    'detail': f'진동 {vibe_change:+.0f}% 증가 추세',
                                    'action': '프롭 상태 확인'})

    # 모터 균형 추세
    motor_diffs = [l['motor_diff'] for l in logs if l.get('motor_diff') is not None]
    if len(motor_diffs) >= 3:
        recent_md = sum(motor_diffs[-3:]) / 3
        if recent_md > 200:
            maintenance.append({'part': 'ESC/모터', 'urgency': 'HIGH',
                                'detail': f'최근 모터 불균형 {recent_md:.0f}PWM',
                                'action': 'ESC 캘리브레이션, 모터 개별 테스트'})
        elif recent_md > 100:
            maintenance.append({'part': 'ESC/모터', 'urgency': 'MEDIUM',
                                'detail': f'모터 불균형 증가 추세 ({recent_md:.0f}PWM)',
                                'action': '프롭 밸런싱, CG 확인'})

    # 배터리 건강도
    v_mins = [l['v_min'] for l in logs if l.get('v_min') and l['v_min'] > 0]
    if len(v_mins) >= 3:
        recent_vmin = sum(v_mins[-3:]) / 3
        early_vmin = sum(v_mins[:3]) / 3
        if early_vmin > 0:
            v_drop = early_vmin - recent_vmin
            if v_drop > 1.0:
                maintenance.append({'part': '배터리', 'urgency': 'HIGH',
                                    'detail': f'최저전압 {v_drop:.1f}V 하락 ({early_vmin:.1f}→{recent_vmin:.1f}V)',
                                    'action': '배터리 내부저항 측정, 교체 검토'})
            elif v_drop > 0.5:
                maintenance.append({'part': '배터리', 'urgency': 'MEDIUM',
                                    'detail': f'최저전압 하락 추세 ({v_drop:.1f}V)',
                                    'action': '배터리 셀 밸런스 확인'})

    # 배터리 사이클
    batt_cycles = total_flights  # 1비행 ≈ 1사이클
    if batt_cycles > 250:
        maintenance.append({'part': '배터리(사이클)', 'urgency': 'HIGH',
                            'detail': f'{batt_cycles}사이클 (LiPo 수명 300~500)',
                            'action': '배터리 교체 시기'})

    # 컴퍼스 간섭 추세
    compass_rs = [l['compass_r'] for l in logs if l.get('compass_r') is not None]
    if len(compass_rs) >= 3:
        recent_cr = sum(compass_rs[-3:]) / 3
        if abs(recent_cr) > 0.4:
            maintenance.append({'part': '컴퍼스/배선', 'urgency': 'MEDIUM',
                                'detail': f'전자기 간섭 상관계수 {recent_cr:.2f}',
                                'action': '컴퍼스 위치 이격, 전원 케이블 트위스트'})

    # 페일세이프 누적
    total_fs = sum(l.get('fs_count', 0) or 0 for l in logs)
    if total_fs > 5:
        maintenance.append({'part': 'RC/텔레메트리', 'urgency': 'MEDIUM',
                            'detail': f'페일세이프 누적 {total_fs}회',
                            'action': '수신기/안테나 점검, FS 파라미터 확인'})

    # ── 비행술 지도 (파일럿 피드백) ──
    pilot_feedback = []

    # 착륙 품질 추세
    landing_spds = [l['landing_speed'] for l in logs if l.get('landing_speed') is not None]
    if len(landing_spds) >= 2:
        early_ls = landing_spds[0] if landing_spds else 0
        recent_ls = landing_spds[-1] if landing_spds else 0
        avg_ls = sum(landing_spds) / len(landing_spds)
        if len(landing_spds) >= 3 and recent_ls < early_ls:
            pilot_feedback.append({'category': '착륙 기술', 'grade': 'IMPROVING',
                                   'detail': f'하강속도 {early_ls:.1f}→{recent_ls:.1f}m/s (향상)',
                                   'advice': '좋은 추세입니다. 유지하세요.'})
        elif avg_ls > 2.0:
            pilot_feedback.append({'category': '착륙 기술', 'grade': 'NEEDS_WORK',
                                   'detail': f'평균 하강 {avg_ls:.1f}m/s (2.0 초과)',
                                   'advice': 'LAND_SPEED 파라미터 낮추기, 착지 직전 스로틀 부드럽게'})

    # 바운스 습관
    total_bounces = sum(l.get('bounce_count', 0) or 0 for l in logs)
    if total_bounces > 3:
        pilot_feedback.append({'category': '착륙 바운스', 'grade': 'NEEDS_WORK',
                               'detail': f'총 {total_bounces}회 바운스',
                               'advice': '착지 후 즉시 스로틀 컷 연습, 바람 고려'})

    # 급기동 습관
    rapid_counts = [l.get('rapid_maneuver_count', 0) or 0 for l in logs]
    avg_rapid = sum(rapid_counts) / n if n > 0 else 0
    if avg_rapid > 5:
        pilot_feedback.append({'category': '급기동', 'grade': 'CAUTION',
                               'detail': f'비행당 평균 {avg_rapid:.1f}회 급기동',
                               'advice': '기체 수명 단축 원인. 부드러운 스틱 조작 연습'})
    elif avg_rapid > 2:
        pilot_feedback.append({'category': '급기동', 'grade': 'MODERATE',
                               'detail': f'비행당 평균 {avg_rapid:.1f}회',
                               'advice': '보통 수준. 불필요한 급기동 줄이면 기체 수명 연장'})

    # 배터리 관리 습관
    batt_ends = [l['batt_end_pct'] for l in logs if l.get('batt_end_pct') is not None]
    if batt_ends:
        avg_batt_end = sum(batt_ends) / len(batt_ends)
        low_batt_flights = sum(1 for b in batt_ends if b < 15)
        if avg_batt_end < 15:
            pilot_feedback.append({'category': '배터리 관리', 'grade': 'NEEDS_WORK',
                                   'detail': f'평균 잔량 {avg_batt_end:.0f}%에서 착륙 ({low_batt_flights}회 15% 미만)',
                                   'advice': '20% 이상 잔량에서 착륙 권장. 과방전은 배터리 수명 급감'})
        elif avg_batt_end < 20:
            pilot_feedback.append({'category': '배터리 관리', 'grade': 'MODERATE',
                                   'detail': f'평균 잔량 {avg_batt_end:.0f}%',
                                   'advice': '조금 더 여유있게 착륙하면 배터리 수명 연장'})
        else:
            pilot_feedback.append({'category': '배터리 관리', 'grade': 'GOOD',
                                   'detail': f'평균 잔량 {avg_batt_end:.0f}% — 적절',
                                   'advice': '좋은 습관입니다.'})

    # 호버 안정성 추세
    ceps = [l['hover_cep95'] for l in logs if l.get('hover_cep95') is not None]
    if len(ceps) >= 2:
        avg_cep = sum(ceps) / len(ceps)
        if avg_cep < 2:
            pilot_feedback.append({'category': '호버 안정성', 'grade': 'GOOD',
                                   'detail': f'평균 CEP95 {avg_cep:.1f}m',
                                   'advice': '위치 유지 능력 양호'})
        else:
            pilot_feedback.append({'category': '호버 안정성', 'grade': 'MODERATE',
                                   'detail': f'평균 CEP95 {avg_cep:.1f}m',
                                   'advice': 'GPS 환경 개선 또는 PID 튜닝 권장'})

    # 비행 모드 활용도 (score 추세)
    scores_list = [l.get('score', 0) or 0 for l in logs]
    if len(scores_list) >= 3:
        early_score = sum(scores_list[:3]) / 3
        recent_score = sum(scores_list[-3:]) / 3
        if recent_score > early_score + 5:
            pilot_feedback.append({'category': '비행 품질', 'grade': 'IMPROVING',
                                   'detail': f'점수 {early_score:.0f}→{recent_score:.0f}점 (향상)',
                                   'advice': '전반적 비행 품질이 개선되고 있습니다.'})
        elif recent_score < early_score - 5:
            pilot_feedback.append({'category': '비행 품질', 'grade': 'DECLINING',
                                   'detail': f'점수 {early_score:.0f}→{recent_score:.0f}점 (하락)',
                                   'advice': '기체 상태 또는 비행 환경 점검 필요'})

    # ── 비행별 이력 (이상치 + 특이점 하이라이트) ──
    flight_history = []
    for l in logs:
        issues = []
        # 이상치 탐지
        if (l.get('vibe_avg') or 0) > 30:
            issues.append(f"진동 과다 ({l['vibe_avg']:.1f})")
        if (l.get('motor_diff') or 0) > 150:
            issues.append(f"모터 불균형 ({l['motor_diff']:.0f}PWM)")
        if (l.get('motor_sat_pct') or 0) > 20:
            issues.append(f"모터 포화 {l['motor_sat_pct']:.0f}%")
        if l.get('v_min') and l['v_min'] > 0:
            cells_est = max(round((l.get('v_max', 0) or 16.8) / 4.2), 1)
            vpc = l['v_min'] / cells_est
            if vpc < 3.3:
                issues.append(f"저전압 경고 ({l['v_min']:.1f}V, 셀당 {vpc:.2f}V)")
        if (l.get('fs_count') or 0) > 0:
            issues.append(f"페일세이프 {l['fs_count']}회")
        if (l.get('hard_landing_count') or 0) > 0:
            issues.append("하드랜딩 감지")
        if (l.get('bounce_count') or 0) > 0:
            issues.append(f"바운스 {l['bounce_count']}회")
        if (l.get('landing_speed') or 0) > 2.5:
            issues.append(f"급강하 착륙 ({l['landing_speed']:.1f}m/s)")
        if (l.get('rapid_maneuver_count') or 0) > 10:
            issues.append(f"급기동 과다 ({l['rapid_maneuver_count']}회)")
        if (l.get('batt_end_pct') or 100) < 10:
            issues.append(f"과방전 착륙 ({l['batt_end_pct']:.0f}%)")
        if l.get('fail_count') and l['fail_count'] > 0:
            issues.append(f"FAIL {l['fail_count']}건")

        flight_history.append({
            'date': l.get('uploaded_at', ''),
            'filename': l.get('filename', ''),
            'job_id': l.get('job_id', ''),
            'duration': l.get('duration', ''),
            'max_alt': l.get('max_alt', 0),
            'dist_m': l.get('dist_m', 0),
            'score': l.get('score', 0),
            'overall': l.get('overall', 'OK'),
            'vibe_avg': l.get('vibe_avg'),
            'motor_diff': l.get('motor_diff'),
            'landing_speed': l.get('landing_speed'),
            'batt_end_pct': l.get('batt_end_pct'),
            'issues': issues,
            'root_cause': l.get('root_cause', ''),
        })

    # ── 교차 분석 (기체 특성 ↔ 조종 습관 상관) ──
    cross_analysis = []

    # 1. 진동 증가 ↔ 급기동 연관
    if len(vibes) >= 3 and len(rapid_counts) >= 3:
        # 최근 3회 진동 높은 비행에서 급기동도 많은지
        vibe_rapid_pairs = [(v, r) for v, r in zip(vibes, rapid_counts) if v is not None]
        if len(vibe_rapid_pairs) >= 3:
            high_vibe = [p for p in vibe_rapid_pairs if p[0] > 20]
            low_vibe = [p for p in vibe_rapid_pairs if p[0] <= 20]
            if high_vibe and low_vibe:
                avg_rapid_high = sum(p[1] for p in high_vibe) / len(high_vibe)
                avg_rapid_low = sum(p[1] for p in low_vibe) / len(low_vibe)
                if avg_rapid_high > avg_rapid_low * 1.5:
                    cross_analysis.append({
                        'type': 'CORRELATION',
                        'title': '급기동 → 진동 악화',
                        'detail': f'급기동 많은 비행(평균 {avg_rapid_high:.0f}회)에서 진동이 높음. 부드러운 조작이 기체 수명에 직접 영향.',
                        'severity': 'WARN'
                    })

    # 2. 배터리 과방전 ↔ 배터리 성능 저하
    if batt_ends and v_mins:
        low_batt_flights = [(e, v) for e, v in zip(batt_ends, v_mins) if e is not None and v is not None and e < 15]
        if len(low_batt_flights) >= 2:
            cross_analysis.append({
                'type': 'CAUSAL',
                'title': '과방전 습관 → 배터리 성능 저하',
                'detail': f'{len(low_batt_flights)}회 과방전 비행(15% 미만 착륙). 이것이 최저전압 하락의 주 원인일 수 있음.',
                'severity': 'WARN'
            })

    # 3. 하드랜딩 누적 → 기체 점검
    total_hard = sum(l.get('hard_landing_count', 0) or 0 for l in logs)
    total_bounces = sum(l.get('bounce_count', 0) or 0 for l in logs)
    if total_hard >= 2 or total_bounces >= 3:
        cross_analysis.append({
            'type': 'IMPACT',
            'title': '착륙 충격 → FC/GPS 마운트 점검 필요',
            'detail': f'하드랜딩 {total_hard}회 + 바운스 {total_bounces}회. 누적 충격이 센서 정렬·방진마운트에 영향.',
            'severity': 'WARN' if total_hard >= 3 else 'INFO'
        })

    # 4. 점수 하락 + 진동 증가 동시 발생
    if len(scores_list) >= 5 and len(vibes) >= 5:
        recent_scores = scores_list[-3:]
        early_scores = scores_list[:3]
        recent_vibes_val = vibes[-3:] if len(vibes) >= 3 else vibes
        early_vibes_val = vibes[:3]
        avg_rs = sum(recent_scores) / len(recent_scores)
        avg_es = sum(early_scores) / len(early_scores)
        avg_rv = sum(v for v in recent_vibes_val if v) / max(1, len([v for v in recent_vibes_val if v]))
        avg_ev = sum(v for v in early_vibes_val if v) / max(1, len([v for v in early_vibes_val if v]))
        if avg_rs < avg_es - 5 and avg_rv > avg_ev * 1.2:
            cross_analysis.append({
                'type': 'TREND',
                'title': '비행 품질 하락 + 진동 증가 동시 진행',
                'detail': f'점수 {avg_es:.0f}→{avg_rs:.0f}, 진동 {avg_ev:.1f}→{avg_rv:.1f}. 기체 노화 또는 정비 필요 시점.',
                'severity': 'FAIL'
            })

    # 5. 모터 불균형 + 모터 포화 동시
    motor_sats = [l.get('motor_sat_pct') for l in logs if l.get('motor_sat_pct') is not None]
    if motor_diffs and motor_sats:
        recent_md_val = motor_diffs[-1] if motor_diffs else 0
        recent_ms_val = motor_sats[-1] if motor_sats else 0
        if recent_md_val > 100 and recent_ms_val > 10:
            cross_analysis.append({
                'type': 'CRITICAL',
                'title': '모터 불균형 + 포화 — 추락 위험',
                'detail': f'불균형 {recent_md_val:.0f}PWM + 포화 {recent_ms_val:.0f}%. 한쪽 모터가 한계에 근접. ESC/모터/프롭 즉시 점검.',
                'severity': 'FAIL'
            })

    # 비행 없는 경우에도 cross_analysis 제공
    if not cross_analysis:
        cross_analysis.append({
            'type': 'INFO',
            'title': '교차 분석 이상 없음',
            'detail': '기체 상태와 조종 습관 간 우려되는 상관관계가 발견되지 않았습니다.',
            'severity': 'OK'
        })

    # ── 종합 등급 ──
    fail_items = [c for c in cross_analysis if c['severity'] == 'FAIL']
    warn_items = [c for c in cross_analysis if c['severity'] == 'WARN']
    if fail_items:
        overall_grade = 'CRITICAL'
        overall_color = 'red'
        overall_msg = f'즉시 조치 필요 ({len(fail_items)}건 위험)'
    elif warn_items:
        overall_grade = 'WARNING'
        overall_color = 'orange'
        overall_msg = f'주의 필요 ({len(warn_items)}건 경고)'
    else:
        overall_grade = 'GOOD'
        overall_color = 'green'
        overall_msg = '기체 + 비행술 모두 양호'

    return {
        'vehicle_id': vehicle_id,
        'user_id': user_id,
        'total_flights': total_flights,
        'total_hours': round(total_hours, 1),
        'total_dist_km': round(total_dist / 1000, 1),
        'total_mah': round(total_mah, 0),
        'total_fails': total_fails,
        'total_warns': total_warns,
        'first_flight': logs[0].get('uploaded_at', ''),
        'last_flight': logs[-1].get('uploaded_at', ''),
        'maintenance': maintenance,
        'pilot_feedback': pilot_feedback,
        'cross_analysis': cross_analysis,
        'flight_history': flight_history,
        'overall_grade': overall_grade,
        'overall_color': overall_color,
        'overall_msg': overall_msg,
        'score_trend': scores_list,
        'vibe_trend': vibes,
        'motor_diff_trend': motor_diffs,
        'motor_life_pct': round(motor_pct, 1),
    }


def get_user_vehicles(user_id: str) -> list:
    """사용자의 기체 목록"""
    conn = get_db()
    rows = conn.execute("""
    SELECT vehicle_id, COUNT(*) as flight_count,
           SUM(dur_sec) as total_sec,
           MAX(uploaded_at) as last_flight,
           AVG(score) as avg_score
    FROM logs
    WHERE user_id = ? AND vehicle_id > 0
    GROUP BY vehicle_id
    ORDER BY last_flight DESC
    """, (user_id.strip().lower(),)).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_log_by_job(job_id: str) -> dict:
    conn = get_db()
    row = conn.execute("SELECT * FROM logs WHERE job_id = ?", (job_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


def get_all_users() -> list:
    conn = get_db()
    rows = conn.execute("""
    SELECT user_id, COUNT(*) as log_count,
           MAX(uploaded_at) as last_upload
    FROM logs GROUP BY user_id ORDER BY last_upload DESC
    """).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# 앱 시작 시 DB 초기화
init_db()
