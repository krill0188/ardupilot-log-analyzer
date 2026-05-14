#!/usr/bin/env python3
"""
ArduCopter Log Analyzer — Web Service
Port: 8040
"""
import os, sys, json, uuid, shutil, asyncio
from pathlib import Path

from fastapi import FastAPI, UploadFile, File
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

# 분석 엔진 import
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from analyze import LogParser, Analyzer, ChartGen
from ardupilot_error_codes import lookup_event

app = FastAPI(title="ArduCopter Log Analyzer")

import numpy as np

def _json_safe(obj):
    """numpy 타입 → Python 네이티브 타입 재귀 변환"""
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    elif isinstance(obj, (np.integer,)):
        return int(obj)
    elif isinstance(obj, (np.floating,)):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, np.bool_):
        return bool(obj)
    return obj

BASE = Path(__file__).resolve().parent
app.mount("/static", StaticFiles(directory=BASE / "static"), name="static")
app.mount("/uploads", StaticFiles(directory=BASE / "uploads"), name="uploads")

UPLOAD_DIR = BASE / "uploads"
UPLOAD_DIR.mkdir(exist_ok=True)

@app.get("/", response_class=HTMLResponse)
async def index():
    with open(BASE / "templates" / "index.html", "r") as f:
        return HTMLResponse(f.read())


@app.post("/api/analyze")
async def analyze_log(file: UploadFile = File(...)):
    """파일 업로드 → 분석 → JSON 결과 반환"""
    job_id = uuid.uuid4().hex[:10]
    job_dir = UPLOAD_DIR / job_id
    job_dir.mkdir(exist_ok=True)
    chart_dir = job_dir / "charts"
    chart_dir.mkdir(exist_ok=True)

    # 파일 저장
    filepath = job_dir / file.filename
    with open(filepath, "wb") as f:
        content = await file.read()
        f.write(content)

    # 분석 실행 (blocking — 큰 파일은 추후 비동기화)
    try:
        parser = LogParser(str(filepath))
        analyzer = Analyzer(parser)
        cg = ChartGen(parser, analyzer, str(chart_dir))
        charts = cg.generate_all()
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)

    # 1차: 로그 인벤토리 (어떤 데이터가 있는지)
    log_inventory = parser.get_log_summary()

    # 차트 경로를 웹 URL로 변환
    chart_urls = {}
    for key, path in charts.items():
        rel = os.path.relpath(path, str(BASE))
        chart_urls[key] = f"/{rel}"

    # 분석 결과를 JSON으로
    s = analyzer.summary
    findings = []
    for f in analyzer.findings:
        findings.append({
            "sev": f.sev,
            "title": f.title,
            "detail": f.detail,
            "fix": f.fix,
        })

    # 시계열 데이터 (Plotly용)
    t0 = parser.t0()
    timeseries = {}

    # 고도
    gps = parser.get('GPS')
    if gps:
        timeseries['altitude'] = {
            't': [round(g['_ts'] - t0, 2) for g in gps],
            'alt': [round(g.get('Alt', 0), 2) for g in gps],
            'spd': [round(g.get('Spd', 0), 2) for g in gps],
        }

    # GPS 경로
    lats = [g.get('Lat', 0) for g in gps if g.get('Lat', 0) != 0]
    lons = [g.get('Lng', 0) for g in gps if g.get('Lng', 0) != 0]
    if len(lats) > 2:
        timeseries['gps'] = {'lat': lats, 'lon': lons}

    # 진동
    vibes = parser.get('VIBE')
    if vibes:
        timeseries['vibe'] = {
            't': [round(v['_ts'] - t0, 2) for v in vibes],
            'x': [round(v.get('VibeX', 0), 2) for v in vibes],
            'y': [round(v.get('VibeY', 0), 2) for v in vibes],
            'z': [round(v.get('VibeZ', 0), 2) for v in vibes],
        }

    # 배터리
    batt = parser.get('BAT') or parser.get('BATT') or parser.get('CURR')
    if batt:
        timeseries['battery'] = {
            't': [round(b['_ts'] - t0, 2) for b in batt],
            'v': [round(b.get('Volt', b.get('VoltR', 0)), 2) for b in batt],
            'i': [round(b.get('Curr', b.get('CurrR', 0)), 2) for b in batt],
        }

    # 자세
    att = parser.get('ATT')
    if att:
        # 다운샘플 (너무 많으면 브라우저 느림)
        step = max(1, len(att) // 3000)
        sampled = att[::step]
        timeseries['attitude'] = {
            't': [round(a['_ts'] - t0, 2) for a in sampled],
            'roll': [round(a.get('Roll', 0), 2) for a in sampled],
            'pitch': [round(a.get('Pitch', 0), 2) for a in sampled],
            'desroll': [round(a.get('DesRoll', 0), 2) for a in sampled],
            'despitch': [round(a.get('DesPitch', 0), 2) for a in sampled],
        }

    # 모터 출력
    rcou = parser.get('RCOU')
    if rcou:
        step = max(1, len(rcou) // 3000)
        sampled = rcou[::step]
        timeseries['rcout'] = {
            't': [round(r['_ts'] - t0, 2) for r in sampled],
            'm1': [r.get('C1', 0) for r in sampled],
            'm2': [r.get('C2', 0) for r in sampled],
            'm3': [r.get('C3', 0) for r in sampled],
            'm4': [r.get('C4', 0) for r in sampled],
        }

    # EKF
    ekf = parser.get('NKF4') or parser.get('XKF4') or parser.get('EKF4')
    if ekf:
        step = max(1, len(ekf) // 2000)
        sampled = ekf[::step]
        timeseries['ekf'] = {
            't': [round(e['_ts'] - t0, 2) for e in sampled],
            'sv': [round(e.get('SV', 0), 3) for e in sampled],
            'sp': [round(e.get('SP', 0), 3) for e in sampled],
            'sh': [round(e.get('SH', 0), 3) for e in sampled],
            'sm': [round(e.get('SM', 0), 3) for e in sampled],
        }

    # FFT 진동 주파수 분석
    imu = parser.get('IMU')
    if imu and len(imu) > 100:
        import numpy as np_fft
        # 샘플링 레이트 추정
        dt_arr = [imu[i+1]['_ts'] - imu[i]['_ts'] for i in range(min(200, len(imu)-1))]
        dt_avg = sum(dt_arr) / len(dt_arr) if dt_arr else 0.01
        fs = 1.0 / dt_avg if dt_avg > 0 else 100
        for ax_key, ax_name in [('AccX', 'x'), ('AccY', 'y'), ('AccZ', 'z')]:
            vals = np_fft.array([d.get(ax_key, 0) for d in imu])
            if len(vals) < 128: continue
            # DC 제거 + 윈도우
            vals = vals - np_fft.mean(vals)
            window = np_fft.hanning(len(vals))
            fft_result = np_fft.fft.rfft(vals * window)
            freqs = np_fft.fft.rfftfreq(len(vals), d=1.0/fs)
            magnitude = np_fft.abs(fft_result) * 2.0 / len(vals)
            # 1Hz~Nyquist만
            mask = freqs >= 1.0
            if f'fft' not in timeseries:
                timeseries['fft'] = {'freq': [round(f, 2) for f in freqs[mask][:500]]}
            timeseries['fft'][ax_name] = [round(m, 4) for m in magnitude[mask][:500]]
        if 'fft' in timeseries:
            timeseries['fft']['fs'] = round(fs, 1)

    # 3D 비행 경로 데이터
    if gps and len(lats) > 5:
        alts_3d = [g.get('Alt', 0) for g in gps if g.get('Lat', 0) != 0]
        timeseries['gps3d'] = {
            'lat': lats,
            'lon': lons,
            'alt': alts_3d[:len(lats)],
        }

    # 바람 추정
    wnd = parser.get('NKF2') or parser.get('XKF2')
    if wnd:
        step = max(1, len(wnd) // 2000)
        sampled = wnd[::step]
        import math
        vwn_arr = [w.get('VWN', 0) for w in sampled]
        vwe_arr = [w.get('VWE', 0) for w in sampled]
        timeseries['wind'] = {
            't': [round(w['_ts'] - t0, 2) for w in sampled],
            'spd': [round(math.sqrt(n**2 + e**2), 2) for n, e in zip(vwn_arr, vwe_arr)],
            'dir': [round(math.degrees(math.atan2(-e, -n)) % 360, 1) for n, e in zip(vwn_arr, vwe_arr)],
            'vwn': [round(v, 2) for v in vwn_arr],
            'vwe': [round(v, 2) for v in vwe_arr],
        }

    # PID 추적
    for axis_key, axis_name in [('PIDR', 'roll'), ('PIDP', 'pitch')]:
        pid_data = parser.get(axis_key)
        if pid_data and len(pid_data) > 50:
            step = max(1, len(pid_data) // 2000)
            sampled = pid_data[::step]
            timeseries[f'pid_{axis_name}'] = {
                't': [round(d['_ts'] - t0, 2) for d in sampled],
                'tar': [round(d.get('Tar', d.get('Des', 0)), 2) for d in sampled],
                'act': [round(d.get('Act', d.get('P', 0)), 2) for d in sampled],
            }

    # 비행 모드
    modes = []
    for m in s['modes']:
        modes.append({'t': round(m['t'] - t0, 2), 'name': m['name']})

    # 에러 시점
    err_times = [round(e['t'] - t0, 2) for e in s['errors'] if e['ec'] != 0]

    # 이벤트
    events = []
    for ev in s['events']:
        events.append({
            't': round(ev['t'] - t0, 2),
            'desc': lookup_event(ev['id']),
        })

    # GPS 품질 시계열
    if gps:
        step_g = max(1, len(gps) // 2000)
        timeseries['gps_quality'] = {
            't': [round(g['_ts'] - t0, 2) for g in gps[::step_g]],
            'nsats': [g.get('NSats', g.get('nSats', 0)) for g in gps[::step_g]],
            'hdop': [round(g.get('HDop', 99), 2) for g in gps[::step_g]],
        }

    # 모터 건강도 상세 (대각선 쌍 비교)
    rcou_data = parser.get('RCOU')
    motor_health = None
    if rcou_data and len(rcou_data) > 50:
        import numpy as np_m
        step_m = max(1, len(rcou_data) // 2000)
        sampled_m = rcou_data[::step_m]
        m1 = np_m.array([r.get('C1', 0) for r in sampled_m])
        m2 = np_m.array([r.get('C2', 0) for r in sampled_m])
        m3 = np_m.array([r.get('C3', 0) for r in sampled_m])
        m4 = np_m.array([r.get('C4', 0) for r in sampled_m])
        # QuadX: M1+M2 (CW대각) vs M3+M4 (CCW대각)
        diag_a = (m1 + m2) / 2  # 대각선 A
        diag_b = (m3 + m4) / 2  # 대각선 B
        mask = (diag_a > 1000) & (diag_b > 1000)
        if mask.sum() > 10:
            diff = np_m.abs(diag_a[mask] - diag_b[mask])
            timeseries['motor_health'] = {
                't': [round(sampled_m[i]['_ts'] - t0, 2) for i, v in enumerate(mask) if v],
                'diag_a': [round(v, 0) for v in diag_a[mask].tolist()],
                'diag_b': [round(v, 0) for v in diag_b[mask].tolist()],
                'diff': [round(v, 0) for v in diff.tolist()],
            }
            motor_health = {'avg_diff': round(float(np_m.mean(diff)), 1), 'max_diff': round(float(np_m.max(diff)), 1)}

    # 착륙 분석 데이터
    landing_data = getattr(analyzer, 'landing_data', None)

    # 호버 안정성 데이터
    hover_data = getattr(analyzer, 'hover_data', None)

    # ESC 텔레메트리 시계열
    esc_raw = parser.get('ESC')
    if esc_raw and len(esc_raw) > 10:
        esc_by_inst = {}
        for e in esc_raw:
            inst = e.get('Instance', e.get('Inst', 0))
            if inst not in esc_by_inst:
                esc_by_inst[inst] = []
            esc_by_inst[inst].append(e)
        for inst, data in sorted(esc_by_inst.items()):
            step_e = max(1, len(data) // 2000)
            sampled_e = data[::step_e]
            ts_key = f'esc_{inst}'
            timeseries[ts_key] = {
                't': [round(e['_ts'] - t0, 2) for e in sampled_e],
                'rpm': [e.get('RPM', 0) for e in sampled_e],
                'temp': [round(e.get('Temp', 0), 1) for e in sampled_e],
                'curr': [round(e.get('Curr', 0), 2) for e in sampled_e],
            }

    # 비행 구간 분류
    flight_phases = getattr(analyzer, 'flight_phases', None)

    # 파라미터 체크리스트
    param_check = getattr(analyzer, 'param_check', None)

    # 배터리 내부저항 추정
    batt_ir = None
    if batt and len(batt) > 100:
        import numpy as np_b
        volts_arr = np_b.array([b.get('Volt', b.get('VoltR', 0)) for b in batt])
        currs_arr = np_b.array([b.get('Curr', b.get('CurrR', 0)) for b in batt])
        mask_b = (currs_arr > 1) & (volts_arr > 1)
        if mask_b.sum() > 20:
            # 간이 내부저항 = -dV/dI (전류 증가 시 전압 하락)
            v_filt = volts_arr[mask_b]
            i_filt = currs_arr[mask_b]
            if len(v_filt) > 50:
                # 선형 회귀 간이
                slope = float(np_b.polyfit(i_filt[:500], v_filt[:500], 1)[0])
                ir_mohm = abs(slope) * 1000
                cells = max(round(float(np_b.max(volts_arr)) / 4.2), 1)
                ir_per_cell = ir_mohm / cells
                batt_ir = {'total_mohm': round(ir_mohm, 1), 'per_cell_mohm': round(ir_per_cell, 1), 'cells': cells}

    # 비행 점수 (5항목 100점 만점)
    import numpy as np_s
    scores = {}
    # 1. 고도 유지 (AltHold/Loiter 구간에서 고도 변동)
    alt_arr = np_s.array([g.get('Alt', 0) for g in gps]) if gps else np_s.array([])
    if len(alt_arr) > 50:
        alt_std = float(np_s.std(alt_arr[len(alt_arr)//4:]))
        scores['altitude'] = max(0, min(100, int(100 - alt_std * 5)))
    else:
        scores['altitude'] = 50
    # 2. 진동
    vibes_data = parser.get('VIBE')
    if vibes_data:
        worst_vibe = max(max(v.get('VibeX',0) for v in vibes_data),
                        max(v.get('VibeY',0) for v in vibes_data),
                        max(v.get('VibeZ',0) for v in vibes_data))
        scores['vibration'] = max(0, min(100, int(100 - worst_vibe * 2)))
    else:
        scores['vibration'] = 50
    # 3. 배터리 관리
    if s['v_min'] > 0 and s['v_max'] > 0:
        cells_s = max(round(s['v_max'] / 4.2), 1)
        vpc = s['v_min'] / cells_s
        scores['battery'] = max(0, min(100, int((vpc - 3.0) / (4.2 - 3.0) * 100)))
    else:
        scores['battery'] = 50
    # 4. GPS 품질
    if s['min_sats'] > 0:
        sat_score = min(100, int(s['min_sats'] * 10))
        hdop_score = max(0, min(100, int((3.0 - s['max_hdop']) / 2.0 * 100)))
        scores['gps'] = (sat_score + hdop_score) // 2
    else:
        scores['gps'] = 50
    # 5. 에러/안전
    fail_count = sum(1 for f in analyzer.findings if f.sev == 'FAIL')
    warn_count = sum(1 for f in analyzer.findings if f.sev == 'WARN')
    scores['safety'] = max(0, min(100, 100 - fail_count * 15 - warn_count * 5))
    # 종합
    scores['overall'] = sum(scores.values()) // len(scores)

    return _json_safe({
        "job_id": job_id,
        "summary": {
            "filename": s['filename'],
            "filesize_mb": round(s['filesize_mb'], 1),
            "duration": s['dur_str'],
            "max_alt": round(s['max_alt'], 1),
            "max_spd": round(s['max_spd'], 1),
            "dist_m": round(s['dist_m'], 0),
            "v_min": round(s['v_min'], 2),
            "v_max": round(s['v_max'], 2),
            "i_max": round(s['i_max'], 1),
            "min_sats": s['min_sats'],
            "max_sats": s['max_sats'],
            "max_hdop": round(s['max_hdop'], 1),
        },
        "findings": findings,
        "charts": chart_urls,
        "timeseries": timeseries,
        "modes": modes,
        "err_times": err_times,
        "events": events,
        "counts": {
            "fail": sum(1 for f in findings if f['sev'] == 'FAIL'),
            "warn": sum(1 for f in findings if f['sev'] == 'WARN'),
            "ok": sum(1 for f in findings if f['sev'] == 'OK'),
        },
        "scores": scores,
        "log_inventory": log_inventory,
        "motor_health": motor_health,
        "batt_ir": batt_ir,
        "landing": landing_data,
        "hover": hover_data,
        "flight_phases": flight_phases,
        "param_check": param_check,
        "timeline": analyzer.timeline,
        "root_cause": analyzer.root_cause,
        "flight_story": analyzer.flight_story,
        "compass_interference": getattr(analyzer, 'compass_interference', None),
        "current_spikes": getattr(analyzer, 'current_spikes', None),
        "motor_saturation": getattr(analyzer, 'motor_saturation', None),
        "power_efficiency": getattr(analyzer, 'power_efficiency', None),
        "baro_drift": getattr(analyzer, 'baro_drift', None),
        "dual_compass": getattr(analyzer, 'dual_compass', None),
        "alt_tracking": getattr(analyzer, 'alt_tracking', None),
        "failsafe_events": getattr(analyzer, 'failsafe_events', None),
        "fft_peaks": getattr(analyzer, 'fft_peaks', None),
    })


@app.get("/api/csv/{job_id}")
async def download_csv(job_id: str):
    """분석 결과 CSV 다운로드"""
    import csv, io
    job_dir = UPLOAD_DIR / job_id
    if not job_dir.exists():
        return JSONResponse({"error": "job not found"}, status_code=404)
    bins = list(job_dir.glob("*.bin")) + list(job_dir.glob("*.BIN"))
    if not bins:
        return JSONResponse({"error": "no bin file"}, status_code=404)

    parser = LogParser(str(bins[0]))
    analyzer = Analyzer(parser)
    t0 = parser.t0()

    # 시계열 CSV: 시간, GPS고도, 진동X/Y/Z, 배터리V, 전류A, Roll, Pitch
    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow(['time_s', 'gps_alt', 'gps_spd', 'gps_sats', 'baro_alt',
                     'vibe_x', 'vibe_y', 'vibe_z', 'batt_v', 'batt_a',
                     'roll', 'pitch', 'yaw', 'm1', 'm2', 'm3', 'm4'])

    gps = parser.get('GPS')
    for g in gps:
        t = round(g['_ts'] - t0, 2)
        writer.writerow([t, g.get('Alt',0), g.get('Spd',0),
                         g.get('NSats', g.get('nSats',0)), '', '', '', '',
                         '', '', '', '', '', '', '', '', ''])

    csv_path = job_dir / "timeseries.csv"
    with open(csv_path, 'w') as f:
        f.write(output.getvalue())

    return FileResponse(str(csv_path), filename=f"{bins[0].stem}_timeseries.csv",
                        media_type="text/csv")


@app.get("/api/json/{job_id}")
async def download_json(job_id: str):
    """전체 분석 결과 JSON 다운로드"""
    job_dir = UPLOAD_DIR / job_id
    if not job_dir.exists():
        return JSONResponse({"error": "job not found"}, status_code=404)
    bins = list(job_dir.glob("*.bin")) + list(job_dir.glob("*.BIN"))
    if not bins:
        return JSONResponse({"error": "no bin file"}, status_code=404)

    parser = LogParser(str(bins[0]))
    analyzer = Analyzer(parser)
    s = analyzer.summary

    result = _json_safe({
        "filename": s['filename'],
        "summary": {k: s[k] for k in ['dur_str','max_alt','max_spd','dist_m','v_min','v_max','i_max']},
        "findings": [{"sev":f.sev,"title":f.title,"detail":f.detail,"fix":f.fix} for f in analyzer.findings],
        "timeline": analyzer.timeline,
        "root_cause": analyzer.root_cause,
        "flight_story": analyzer.flight_story,
        "power_efficiency": getattr(analyzer, 'power_efficiency', None),
        "fft_peaks": getattr(analyzer, 'fft_peaks', None),
    })

    json_path = job_dir / "analysis.json"
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    return FileResponse(str(json_path), filename=f"{bins[0].stem}_analysis.json",
                        media_type="application/json")


@app.get("/api/pdf/{job_id}")
async def download_pdf(job_id: str):
    """기존 PDF 생성 후 다운로드"""
    job_dir = UPLOAD_DIR / job_id
    if not job_dir.exists():
        return JSONResponse({"error": "job not found"}, status_code=404)

    bins = list(job_dir.glob("*.bin")) + list(job_dir.glob("*.BIN"))
    if not bins:
        return JSONResponse({"error": "no bin file"}, status_code=404)

    from analyze import ReportBuilder
    pdf_path = job_dir / "report.pdf"
    if not pdf_path.exists():
        parser = LogParser(str(bins[0]))
        analyzer = Analyzer(parser)
        chart_dir = job_dir / "charts"
        cg = ChartGen(parser, analyzer, str(chart_dir))
        charts = cg.generate_all()
        ReportBuilder(analyzer, charts, str(pdf_path)).build()

    return FileResponse(str(pdf_path), filename=f"{bins[0].stem}_report.pdf",
                        media_type="application/pdf")


@app.post("/api/compare")
async def compare_logs(file1: UploadFile = File(...), file2: UploadFile = File(...)):
    """2개 로그 비교 분석"""
    results = []
    for f in [file1, file2]:
        job_id = uuid.uuid4().hex[:10]
        job_dir = UPLOAD_DIR / job_id
        job_dir.mkdir(exist_ok=True)
        fp = job_dir / f.filename
        with open(fp, "wb") as out:
            out.write(await f.read())
        parser = LogParser(str(fp))
        analyzer = Analyzer(parser)
        s = analyzer.summary
        t0 = parser.t0()
        gps = parser.get('GPS')
        vibes = parser.get('VIBE')
        batt = parser.get('BAT') or parser.get('BATT') or parser.get('CURR')
        r = {
            "filename": s['filename'],
            "summary": {"duration": s['dur_str'], "max_alt": round(s['max_alt'],1),
                        "dist_m": round(s['dist_m'],0), "v_min": round(s['v_min'],2)},
            "counts": {"fail": sum(1 for x in analyzer.findings if x.sev=='FAIL'),
                       "warn": sum(1 for x in analyzer.findings if x.sev=='WARN'),
                       "ok": sum(1 for x in analyzer.findings if x.sev=='OK')},
        }
        if gps:
            step = max(1, len(gps)//2000)
            r['alt'] = {'t':[round(g['_ts']-t0,2) for g in gps[::step]],
                        'v':[round(g.get('Alt',0),2) for g in gps[::step]]}
        if vibes:
            step = max(1, len(vibes)//2000)
            vz = [v.get('VibeZ',0) for v in vibes[::step]]
            r['vibe_z'] = {'t':[round(v['_ts']-t0,2) for v in vibes[::step]], 'v':[round(v,2) for v in vz]}
        if batt:
            step = max(1, len(batt)//2000)
            r['batt'] = {'t':[round(b['_ts']-t0,2) for b in batt[::step]],
                         'v':[round(b.get('Volt',b.get('VoltR',0)),2) for b in batt[::step]]}
        results.append(r)
    return {"logs": results}


@app.post("/api/ai-diagnose")
async def ai_diagnose(request: Request):
    """AI 자연어 진단 — 분석 결과를 LLM에 보내서 자연어 진단"""
    body = await request.json()
    question = body.get("question", "이 비행의 주요 문제점과 원인을 분석해주세요")
    summary = body.get("summary", {})
    findings = body.get("findings", [])

    # 컨텍스트 구성
    ctx_lines = [f"파일: {summary.get('filename','?')}, 비행시간: {summary.get('duration','?')}",
                 f"최대고도: {summary.get('max_alt',0)}m, 거리: {summary.get('dist_m',0)}m",
                 f"배터리: {summary.get('v_min',0)}~{summary.get('v_max',0)}V, 최대전류: {summary.get('i_max',0)}A",
                 f"GPS: 위성 {summary.get('min_sats',0)}~{summary.get('max_sats',0)}, HDop {summary.get('max_hdop',99)}",
                 "--- 발견사항 ---"]
    for f in findings[:30]:
        ctx_lines.append(f"[{f['sev']}] {f['title']}: {f.get('detail','')[:100]}")

    prompt = (
        "당신은 ArduCopter 드론 비행 로그 분석 전문가입니다.\n"
        "아래 비행 로그 분석 결과를 바탕으로 사용자의 질문에 한글로 답변하세요.\n"
        "비전문가도 이해할 수 있게 쉽게 설명하되, 기술적 근거를 포함하세요.\n\n"
        f"=== 비행 데이터 ===\n" + "\n".join(ctx_lines) + "\n\n"
        f"=== 사용자 질문 ===\n{question}"
    )

    # OpenAI API (기존 환경변수 활용)
    import os
    api_key = os.environ.get('OPENAI_API_KEY', '')
    if not api_key:
        # .env에서 로드 시도
        env_path = Path(__file__).resolve().parent.parent.parent / 'claudeclaw' / '.env'
        if env_path.exists():
            for line in open(env_path):
                if line.startswith('OPENAI_API_KEY='):
                    api_key = line.split('=', 1)[1].strip().strip('"').strip("'")
                    break

    if not api_key:
        return JSONResponse({"answer": "OpenAI API 키가 설정되지 않아 AI 진단을 사용할 수 없습니다.\n대신 Findings 탭의 분석 결과를 참고하세요."})

    try:
        import httpx
        async with httpx.AsyncClient(timeout=30) as client:
            resp = await client.post("https://api.openai.com/v1/chat/completions",
                headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
                json={"model": "gpt-4o-mini", "messages": [{"role": "user", "content": prompt}],
                      "temperature": 0.3, "max_tokens": 1500})
            data = resp.json()
            answer = data.get("choices", [{}])[0].get("message", {}).get("content", "응답 없음")
    except Exception as e:
        answer = f"AI 진단 오류: {str(e)[:200]}\n\nFindings 탭의 분석 결과를 참고하세요."

    return {"answer": answer}


if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("PORT", 8040))
    uvicorn.run(app, host="0.0.0.0", port=port)
