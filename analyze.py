#!/usr/bin/env python3
"""
ArduCopter .bin Log Analyzer v3 — UAV Log Viewer 스타일 PDF 보고서
Made by Kim.

사용법:
  python3 analyze.py input/flight.bin
  python3 analyze.py input/flight.bin -o output/report.pdf
"""

import sys
import os
import argparse
from datetime import datetime, timedelta
from pathlib import Path

from pymavlink import mavutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import LineCollection
import matplotlib.ticker as mticker
import numpy as np

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.colors import HexColor, white, lightgrey, black
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    Image, PageBreak, KeepTogether,
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

from ardupilot_error_codes import (
    ERR_SUBSYSTEM, ERR_CODES, PREARM_MESSAGES, EKF_STATUS,
    FAILSAFE_TYPES, LOG_EVENT, MOTOR_ESC_ERRORS,
    GPS_FIX_TYPE, GPS_HDOP_QUALITY, VIBE_THRESHOLDS,
    RCOUT_CHANNELS, COPTER_FLIGHT_MODES, INTERNAL_ERROR_BITS,
    lookup_err, lookup_event, lookup_gps_fix, check_vibe_level, check_hdop,
)

# ─── 한글 폰트 ───
FONT_PATHS = [
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
    '/System/Library/Fonts/Supplemental/AppleGothic.ttf',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
]
FONT_NAME = 'Helvetica'
for _fp in FONT_PATHS:
    if os.path.exists(_fp):
        try:
            pdfmetrics.registerFont(TTFont('KR', _fp))
            FONT_NAME = 'KR'
            break
        except Exception:
            continue

plt.rcParams['font.family'] = 'AppleGothic' if sys.platform == 'darwin' else 'NanumGothic'
plt.rcParams['axes.unicode_minus'] = False

# ─── 색상 ───
C_BG_DARK   = '#1a1a2e'
C_BG_CARD   = '#16213e'
C_ACCENT    = '#0f3460'
C_BLUE      = '#4fc3f7'
C_GREEN     = '#66bb6a'
C_YELLOW    = '#fdd835'
C_ORANGE    = '#ff9800'
C_RED       = '#ef5350'
C_PURPLE    = '#ab47bc'
C_CYAN      = '#26c6da'
C_GRID      = '#2a2a4a'
C_TEXT      = '#e0e0e0'
C_TEXT_DIM  = '#9e9e9e'

# 비행 모드 색상 (UAV Log Viewer 스타일)
MODE_COLORS = {
    'STABILIZE': '#4fc3f7', 'ALT_HOLD': '#29b6f6', 'LOITER': '#66bb6a',
    'AUTO': '#ab47bc', 'GUIDED': '#7e57c2', 'RTL': '#ef5350',
    'LAND': '#ff7043', 'POSHOLD': '#26c6da', 'ACRO': '#ffa726',
    'CIRCLE': '#78909c', 'DRIFT': '#8d6e63', 'SPORT': '#ec407a',
    'FLIP': '#d4e157', 'AUTOTUNE': '#ffee58', 'BRAKE': '#bdbdbd',
    'THROW': '#ff8a65', 'SMART_RTL': '#f44336', 'FOLLOW': '#42a5f5',
    'ZIGZAG': '#5c6bc0', 'FLOWHOLD': '#26a69a', 'SYSTEMID': '#78909c',
    'TURTLE': '#a1887f', 'AUTO_RTL': '#e53935', 'AVOID_ADSB': '#9e9e9e',
    'GUIDED_NOGPS': '#7986cb', 'AUTOROTATE': '#ffab91',
}
MODE_NAMES = COPTER_FLIGHT_MODES

# ─── 심각도 색상 ───
SEV_COLORS = {'OK': C_GREEN, 'WARN': C_YELLOW, 'FAIL': C_RED}


# ═══════════════════════════════════════════
# 1. 로그 파싱
# ═══════════════════════════════════════════
class LogParser:
    def __init__(self, filepath: str):
        self.filepath = filepath
        self.mlog = mavutil.mavlink_connection(filepath)
        self.data: dict = {}
        self._parse()

    def _parse(self):
        types = {
            'GPS','ATT','VIBE','BAT','BATT','CURR',
            'EKF4','CTUN','RCIN','RCOU','MODE','MSG',
            'ERR','EV','PM','IMU','NKF4','XKF4',
            'PARM','CMD','POS','RATE',
            'ESC','MOTB','POWR','MAG','MAG2',
            'NKF2','XKF2','PIDR','PIDP','PIDY',
            'BARO','RSSI',
        }
        for t in types:
            self.data[t] = []
        while True:
            msg = self.mlog.recv_match(blocking=False)
            if msg is None:
                break
            mt = msg.get_type()
            if mt in types:
                d = msg.to_dict()
                d['_ts'] = getattr(msg, '_timestamp', 0)
                self.data[mt].append(d)

    def get(self, t: str) -> list:
        return self.data.get(t, [])

    def t0(self) -> float:
        """첫 타임스탬프"""
        for key in ('GPS','ATT','IMU'):
            d = self.get(key)
            if d:
                return d[0]['_ts']
        return 0

    def rel_ts(self, data: list) -> np.ndarray:
        """상대 시간(초) 배열"""
        t0 = self.t0()
        return np.array([d['_ts'] - t0 for d in data])


# ═══════════════════════════════════════════
# 2. 분석 엔진
# ═══════════════════════════════════════════
class Finding:
    __slots__ = ('sev','title','detail','fix')
    def __init__(self, sev, title, detail, fix=''):
        self.sev = sev; self.title = title; self.detail = detail; self.fix = fix

class Analyzer:
    def __init__(self, p: LogParser):
        self.p = p
        self.findings: list = []
        self.summary = self._build_summary()
        self._run_all()
        # 종합 분석 (개별 체크 완료 후)
        self.timeline = self._build_timeline()
        self.root_cause = self._root_cause_analysis()
        self.flight_story = self._build_flight_story()

    def _add(self, sev, title, detail, fix=''):
        self.findings.append(Finding(sev, title, detail, fix))

    def _build_summary(self) -> dict:
        s = {'filename': os.path.basename(self.p.filepath),
             'filesize_mb': os.path.getsize(self.p.filepath)/1024/1024}
        gps = self.p.get('GPS')
        if gps:
            s['dur_sec'] = gps[-1]['_ts'] - gps[0]['_ts']
            s['dur_str'] = str(timedelta(seconds=int(s['dur_sec'])))
            alts = [g.get('Alt',0) for g in gps]
            s['max_alt'] = max(alts) if alts else 0
            s['max_spd'] = max((g.get('Spd',0) for g in gps), default=0)
            lats = [g.get('Lat',0) for g in gps if g.get('Lat',0)!=0]
            lons = [g.get('Lng',0) for g in gps if g.get('Lng',0)!=0]
            dist = 0
            for i in range(1, len(lats)):
                dy = (lats[i]-lats[i-1])*111320
                dx = (lons[i]-lons[i-1])*111320*np.cos(np.radians(lats[i]))
                dist += np.sqrt(dy**2+dx**2)
            s['dist_m'] = dist
            s['gps_n'] = len(lats)
            nsats = [g.get('NSats', g.get('nSats',0)) for g in gps]
            s['min_sats'] = min(nsats) if nsats else 0
            s['max_sats'] = max(nsats) if nsats else 0
            hdops = [g.get('HDop',99) for g in gps]
            s['max_hdop'] = max(h for h in hdops if h<90) if any(h<90 for h in hdops) else 99
        else:
            s.update(dur_sec=0,dur_str='N/A',max_alt=0,max_spd=0,dist_m=0,gps_n=0,
                     min_sats=0,max_sats=0,max_hdop=99)
        # 배터리
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        if batt:
            volts = [b.get('Volt',b.get('VoltR',0)) for b in batt]
            currs = [b.get('Curr',b.get('CurrR',0)) for b in batt]
            pos_v = [v for v in volts if v>0]
            s['v_min'] = min(pos_v) if pos_v else 0
            s['v_max'] = max(volts)
            s['i_max'] = max(currs) if currs else 0
            used = [b.get('CurrTot', b.get('EnrgTot',0)) for b in batt]
            s['mah'] = max(used) if used else 0
        else:
            s.update(v_min=0,v_max=0,i_max=0,mah=0)
        # 모드
        modes = self.p.get('MODE')
        s['modes'] = []
        for m in modes:
            mn = m.get('Mode', m.get('ModeNum',-1))
            s['modes'].append({'t': m['_ts'], 'num': mn,
                               'name': MODE_NAMES.get(mn, f'Mode{mn}')})
        # 에러
        s['errors'] = [{'t':e['_ts'],'sub':e.get('Subsys',0),'ec':e.get('ECode',0)}
                       for e in self.p.get('ERR')]
        # 이벤트
        s['events'] = [{'t':e['_ts'],'id':e.get('Id',e.get('Event',0))}
                       for e in self.p.get('EV')]
        # MSG
        s['msgs'] = [{'t':m['_ts'],'text':m.get('Message','')} for m in self.p.get('MSG')]
        return s

    def _run_all(self):
        self._ck_errors()
        self._ck_events()
        self._ck_vibe()
        self._ck_batt()
        self._ck_ekf()
        self._ck_gps()
        self._ck_compass()
        self._ck_motors()
        self._ck_power()
        self._ck_rc()
        self._ck_wind()
        self._ck_pid()
        self._ck_msgs()
        self._ck_prearm()
        self._ck_landing()
        self._ck_crash()
        self._ck_params()
        self._ck_hover()
        self._ck_esc()
        self._ck_phases()
        self._ck_compass_motor_interference()
        self._ck_current_spikes()
        self._ck_motor_saturation()
        self._ck_power_efficiency()
        self._ck_rc_signal()
        self._ck_baro_drift()
        self._ck_dual_compass()
        self._ck_des_vs_act()
        self._ck_failsafe_reconstruct()
        self._ck_fft()
        if not any(f.sev!='OK' for f in self.findings):
            self._add('OK','전체 정상','심각한 문제 미발견')

    # --- 에러 ---
    def _ck_errors(self):
        errs = self.summary['errors']
        if not errs:
            self._add('OK','에러 없음','ERR 메시지 0건')
            return
        for e in errs:
            info = lookup_err(e['sub'], e['ec'])
            sn = info['subsystem']
            ei = info['error']
            if e['ec']==0:
                continue  # 해소 — 별도 표시 안함
            sev = 'FAIL'
            if e['sub'] in (24,) and e['ec'] in (0,1,2):
                sev = 'WARN'
            if 'cause' in ei:
                self._add(sev, f"{ei.get('name',sn)}",
                          f"{ei['desc']}\n원인: {ei['cause']}", ei['fix'])
            else:
                self._add('WARN', f'{sn} (ECode {e["ec"]})',
                          f'Subsys {e["sub"]} 미등록 에러코드',
                          'ardupilot.org/copter/docs/logmessages.html 참조')

    def _ck_events(self):
        crit = {54,59}; warn_set = {19,60,62,80}
        for ev in self.summary['events']:
            eid = ev['id']
            desc = lookup_event(eid)
            if eid in crit:
                self._add('FAIL', desc, f'이벤트 {eid}', '기체 즉시 점검')
            elif eid in warn_set:
                self._add('WARN', desc, f'이벤트 {eid}')

    def _ck_vibe(self):
        vibes = self.p.get('VIBE')
        if not vibes: return
        vx=[v.get('VibeX',0) for v in vibes]
        vy=[v.get('VibeY',0) for v in vibes]
        vz=[v.get('VibeZ',0) for v in vibes]
        cl = sum(max(v.get(f'Clip{i}',0) for v in vibes) for i in range(3))
        worst = max(max(vx),max(vy),max(vz))
        grade = check_vibe_level(worst)
        sev = 'FAIL' if worst>30 else ('WARN' if worst>15 else 'OK')
        ax_diag = VIBE_THRESHOLDS.get('axis_diagnosis',{})
        hint = ''
        if max(vx)>15 and max(vy)>15: hint = ax_diag.get('X_AND_Y_high','')
        elif max(vx)>15 or max(vy)>15: hint = ax_diag.get('X_OR_Y_high','')
        if max(vz)>15: hint += ' / ' + ax_diag.get('Z_high','')
        self._add(sev, f'진동: {grade}',
                  f'X avg {np.mean(vx):.1f} max {max(vx):.1f} | '
                  f'Y avg {np.mean(vy):.1f} max {max(vy):.1f} | '
                  f'Z avg {np.mean(vz):.1f} max {max(vz):.1f}\n'
                  f'클리핑 합계: {cl}회{(" | "+hint) if hint else ""}',
                  '프롭 밸런싱, 모터 베어링, FC 방진마운트 점검' if sev!='OK' else '')
        if cl > 100:
            self._add('FAIL', f'IMU 클리핑 {cl}회', '센서 범위 초과', '방진 즉시 개선')

    def _ck_batt(self):
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        if not batt: return
        mv = self.summary['v_min']
        if mv <= 0: return
        cells = max(round(self.summary['v_max']/4.2), 1)
        vpc = mv/cells
        if vpc < 3.3:
            self._add('FAIL', f'배터리 과방전 — 셀당 {vpc:.2f}V',
                      f'최저 {mv:.2f}V / {cells}S', '비행시간 단축, 배터리 교체')
        elif vpc < 3.5:
            self._add('WARN', f'배터리 저전압 주의 — 셀당 {vpc:.2f}V',
                      f'최저 {mv:.2f}V / {cells}S', '비행시간 단축')
        else:
            self._add('OK', f'배터리 양호 — 셀당 {vpc:.2f}V', f'{mv:.2f}V / {cells}S')

    def _ck_ekf(self):
        ekf = self.p.get('NKF4') or self.p.get('XKF4') or self.p.get('EKF4')
        if not ekf: return
        thresh = 0.8
        results = {}
        for k in ['SV','SP','SH','SM','SVT']:
            vals = [e.get(k,0) for e in ekf if k in e]
            if vals: results[k] = max(vals)
        over = {k:v for k,v in results.items() if v > thresh}
        if len(over) >= 2:
            self._add('FAIL', f'EKF 페일세이프 위험 ({len(over)}항목 초과)',
                      ' | '.join(f'{k}={v:.2f}' for k,v in over.items()),
                      'IMU 캘리브레이션, GPS 환경, 진동 저감')
        elif len(over) == 1:
            k,v = next(iter(over.items()))
            self._add('WARN', f'EKF {k} 주의 ({v:.2f})',
                      f'임계 {thresh} 초과, 2개 이상 시 FS 트리거', 'GPS/컴퍼스/진동 확인')
        else:
            self._add('OK', 'EKF 양호', ' | '.join(f'{k}={v:.2f}' for k,v in results.items()))

    def _ck_gps(self):
        s = self.summary
        if s['gps_n'] == 0: return
        if s['min_sats'] < 6:
            self._add('WARN', f'GPS 위성 수 부족 (최소 {s["min_sats"]}개)',
                      f'범위: {s["min_sats"]}~{s["max_sats"]}', '개활지 비행, 듀얼GPS')
        if s['max_hdop'] > 2.0:
            self._add('WARN', f'GPS HDop 불량 ({s["max_hdop"]:.1f})',
                      check_hdop(s['max_hdop']), '멀티패스 환경 회피')
        if s['min_sats']>=8 and s['max_hdop']<=1.5:
            self._add('OK', f'GPS 양호 (위성 {s["min_sats"]}~{s["max_sats"]}, HDop {s["max_hdop"]:.1f})', '')

    def _ck_compass(self):
        mag = self.p.get('MAG')
        if not mag: return
        fields = [np.sqrt(m.get('MagX',0)**2+m.get('MagY',0)**2+m.get('MagZ',0)**2)
                  for m in mag]
        fields = [f for f in fields if f > 0]
        if fields:
            avg = np.mean(fields)
            if avg < 185 or avg > 874:
                self._add('WARN', f'컴퍼스 자기장 이상 ({avg:.0f})',
                          f'정상범위 185~874', '자기 간섭원 제거, 외장 컴퍼스')

    def _ck_motors(self):
        rcou = self.p.get('RCOU')
        if not rcou: return
        mavg = {}
        for i in range(1,5):
            vals = [r.get(f'C{i}',0) for r in rcou if r.get(f'C{i}',0)>900]
            if len(vals)>10: mavg[i] = (np.mean(vals), max(vals))
        if len(mavg) < 4: return
        means = [v[0] for v in mavg.values()]
        maxs = [v[1] for v in mavg.values()]
        diff = max(means)-min(means)
        detail = ' | '.join(f'M{i}: {mavg[i][0]:.0f}avg/{mavg[i][1]:.0f}max' for i in range(1,5))
        if diff > 200:
            self._add('FAIL', f'모터 불균형 심각 (차이 {diff:.0f})', detail,
                      '프롭 밸런싱, CG 확인, ESC 캘리브')
        elif diff > 100:
            self._add('WARN', f'모터 불균형 주의 (차이 {diff:.0f})', detail,
                      '프롭 밸런싱, CG 확인')
        else:
            self._add('OK', f'모터 균형 양호 (차이 {diff:.0f})', detail)
        if any(m > 1950 for m in maxs):
            self._add('WARN', '모터 최대출력 도달', '추력 여유 부족', '과적재 확인')
        overall = np.mean(means)
        hover_pct = (overall-1000)/1000
        if hover_pct > 0.65:
            self._add('WARN', f'호버 스로틀 {hover_pct*100:.0f}%', '추력 여유 부족', '프롭/모터 업그레이드')

    def _ck_power(self):
        powr = self.p.get('POWR')
        if not powr: return
        vcc = [p.get('Vcc',0) for p in powr if p.get('Vcc',0)>0]
        if vcc:
            mv = min(vcc)
            if mv < 4.3:
                self._add('FAIL', f'보드 전압 저전압 ({mv:.2f}V)', '리셋 위험', 'BEC/파워모듈 점검')
            elif mv < 4.5:
                self._add('WARN', f'보드 전압 주의 ({mv:.2f}V)', '', '전원 안정성 확인')

    def _ck_rc(self):
        rcin = self.p.get('RCIN')
        if not rcin: return
        ch3 = [r.get('C3',r.get('Chan3',0)) for r in rcin]
        if ch3 and max(ch3)-min(ch3) < 100:
            self._add('WARN', 'RC 스로틀 범위 좁음', f'{min(ch3)}~{max(ch3)}', 'RC 캘리브레이션')

    def _ck_wind(self):
        """바람 추정 (EKF3 XKF2/NKF2 → VWN/VWE)"""
        wnd = self.p.get('NKF2') or self.p.get('XKF2')
        if not wnd: return
        vwn = [w.get('VWN', 0) for w in wnd]
        vwe = [w.get('VWE', 0) for w in wnd]
        if not any(abs(v) > 0.01 for v in vwn): return
        speeds = [np.sqrt(n**2 + e**2) for n, e in zip(vwn, vwe)]
        max_spd = max(speeds)
        avg_spd = np.mean(speeds)
        # 풍향 (기상학 관례: 바람이 불어오는 방향)
        avg_dir = np.degrees(np.arctan2(-np.mean(vwe), -np.mean(vwn))) % 360
        compass = ['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW']
        dir_name = compass[int((avg_dir + 11.25) / 22.5) % 16]
        if max_spd > 10:
            self._add('WARN', f'강풍 감지 — 최대 {max_spd:.1f}m/s ({dir_name})',
                      f'평균 {avg_spd:.1f}m/s, 방향 {avg_dir:.0f}도 ({dir_name})',
                      '강풍 시 비행 자제, ATT 로그에서 기체 기울기 확인')
        else:
            self._add('OK', f'바람 추정 — 평균 {avg_spd:.1f}m/s {dir_name}',
                      f'최대 {max_spd:.1f}m/s, 방향 {avg_dir:.0f}도')

    def _ck_pid(self):
        """PID 튜닝 품질 분석 (PIDR/PIDP 응답 속도, 오버슈트)"""
        for axis, label in [('PIDR', 'Roll'), ('PIDP', 'Pitch'), ('PIDY', 'Yaw')]:
            data = self.p.get(axis)
            if not data or len(data) < 50: continue
            tar = np.array([d.get('Tar', d.get('Des', 0)) for d in data])
            act = np.array([d.get('Act', d.get('P', 0)) for d in data])
            if not any(abs(t) > 0.1 for t in tar): continue
            # 추적 오차
            err = np.abs(tar - act)
            rms_err = np.sqrt(np.mean(err**2))
            max_err = np.max(err)
            # 오버슈트 감지 (목표 방향을 넘어서는 경우)
            overshoot_count = 0
            for i in range(1, len(tar)):
                if tar[i] != 0 and act[i] != 0:
                    if (tar[i] > 0 and act[i] > tar[i] * 1.2) or (tar[i] < 0 and act[i] < tar[i] * 1.2):
                        overshoot_count += 1
            os_pct = overshoot_count / len(tar) * 100

            if rms_err > 15 or os_pct > 20:
                self._add('WARN', f'PID {label} 튜닝 주의',
                          f'RMS 오차: {rms_err:.1f}deg/s, 최대: {max_err:.1f}, 오버슈트: {os_pct:.1f}%',
                          f'{axis}_P 게인 조정, AutoTune 재실행 권장')
            elif rms_err > 5:
                self._add('OK', f'PID {label} 보통 (RMS {rms_err:.1f})', f'오버슈트 {os_pct:.1f}%')

    def _ck_msgs(self):
        kw_fail = ['fail','error','crash','lost','critical','emergency']
        kw_warn = ['warning','caution','bad','low','check']
        for m in self.summary['msgs']:
            t = m['text'].lower()
            if 'prearm' in t: continue
            matched = None
            for kw in kw_fail:
                if kw in t: matched = 'FAIL'; break
            if not matched:
                for kw in kw_warn:
                    if kw in t: matched = 'WARN'; break
            if matched:
                self._add(matched, m['text'][:70], '', 'ArduPilot 문서 참조')

    def _ck_prearm(self):
        for m in self.summary['msgs']:
            t = m['text']
            if 'prearm' not in t.lower(): continue
            for key, entry in PREARM_MESSAGES.items():
                if key.lower() in t.lower():
                    self._add('WARN', f'PreArm: {key}',
                              f'{entry["category"]} — {entry["cause"]}', entry['fix'])
                    return
            self._add('WARN', f'PreArm: {t[:60]}', '', 'PreArm Safety Checks 문서')

    # ── Feature 1: 착륙 분석 ──
    def _ck_landing(self):
        """하강 속도, 바운스, 착륙 정확도 분석"""
        ctun = self.p.get('CTUN')
        gps = self.p.get('GPS')
        if not gps or len(gps) < 20:
            return

        # CTUN 기반 하강률 (더 정확), 없으면 GPS Alt 사용
        if ctun and len(ctun) > 20:
            ts = np.array([c['_ts'] for c in ctun])
            alts = np.array([c.get('Alt', c.get('BAlt', 0)) for c in ctun])
            dalt = np.array([c.get('DAlt', c.get('DSAlt', 0)) for c in ctun])
            crt = np.array([c.get('CRt', 0) for c in ctun])  # climb rate (m/s, 음수=하강)
        else:
            ts = np.array([g['_ts'] for g in gps])
            alts = np.array([g.get('Alt', 0) for g in gps])
            dt = np.diff(ts)
            dt[dt == 0] = 0.1
            crt = np.concatenate([[0], np.diff(alts) / dt])
            dalt = alts

        if not any(alts > 2):
            return

        # 착륙 구간 찾기: 마지막으로 고도 2m 이상 → 0.5m 이하
        above_2m = np.where(alts > 2)[0]
        below_05m = np.where(alts < 0.5)[0]
        if len(above_2m) == 0 or len(below_05m) == 0:
            return
        last_above = above_2m[-1]
        after_last = below_05m[below_05m > last_above]
        if len(after_last) == 0:
            return
        land_start = last_above
        land_end = after_last[0]

        # 착륙 구간 하강률
        land_crt = crt[land_start:land_end+1]
        if len(land_crt) == 0:
            return
        max_descent = abs(min(land_crt))  # 최대 하강 속도 (양수 m/s)
        avg_descent = abs(np.mean(land_crt[land_crt < 0])) if any(land_crt < 0) else 0
        land_duration = ts[land_end] - ts[land_start]

        # 바운스 감지: 착륙 후 고도 재상승
        post_land = alts[land_end:]
        bounce_count = 0
        bounce_max = 0
        if len(post_land) > 5:
            baseline = np.min(post_land[:5])
            rises = post_land - baseline
            in_bounce = False
            for r in rises:
                if r > 0.3 and not in_bounce:
                    bounce_count += 1
                    in_bounce = True
                    bounce_max = max(bounce_max, r)
                elif r <= 0.1:
                    in_bounce = False

        # 착륙 정확도: GPS 위치 기준 (이륙 지점 대비)
        lats = [g.get('Lat', 0) for g in gps if g.get('Lat', 0) != 0]
        lons = [g.get('Lng', 0) for g in gps if g.get('Lng', 0) != 0]
        land_accuracy = None
        if len(lats) > 10 and len(lons) > 10:
            # 이륙점 (처음 5개 평균) vs 착륙점 (마지막 5개 평균)
            lat0, lon0 = np.mean(lats[:5]), np.mean(lons[:5])
            lat1, lon1 = np.mean(lats[-5:]), np.mean(lons[-5:])
            dy = (lat1 - lat0) * 111320
            dx = (lon1 - lon0) * 111320 * np.cos(np.radians(lat0))
            land_accuracy = np.sqrt(dy**2 + dx**2)

        # 저장 (PDF/웹에서 접근)
        self.landing_data = {
            'max_descent': round(max_descent, 2),
            'avg_descent': round(avg_descent, 2),
            'duration': round(land_duration, 1),
            'bounce_count': bounce_count,
            'bounce_max_m': round(bounce_max, 2),
            'accuracy_m': round(land_accuracy, 2) if land_accuracy else None,
        }

        # 하강 속도 판정
        detail = (f'최대 하강: {max_descent:.2f}m/s, 평균: {avg_descent:.2f}m/s\n'
                  f'착륙 소요: {land_duration:.1f}초')
        if land_accuracy is not None:
            detail += f'\n이륙→착륙 거리: {land_accuracy:.1f}m'
        if max_descent > 3.0:
            self._add('FAIL', f'착륙 하강 과속 ({max_descent:.1f}m/s)',
                      detail, '착륙 속도 줄이기, LAND_SPEED 파라미터 확인')
        elif max_descent > 1.5:
            self._add('WARN', f'착륙 하강 빠름 ({max_descent:.1f}m/s)',
                      detail, 'LAND_SPEED 낮추기 권장')
        else:
            self._add('OK', f'착륙 양호 ({max_descent:.1f}m/s)', detail)

        # 바운스 판정
        if bounce_count > 0:
            self._add('WARN', f'착륙 바운스 {bounce_count}회 (최대 {bounce_max:.2f}m)',
                      '착지 후 기체 재상승 감지',
                      '착륙 지면 상태 확인, LAND_SPEED 감소, 바람 영향 점검')

    # ── Feature 2: 크래시 자동 감지 ──
    def _ck_crash(self):
        """ATT 급변 + 고도 급락 → 크래시 패턴 감지"""
        att = self.p.get('ATT')
        gps = self.p.get('GPS')
        if not att or len(att) < 50:
            return

        ts_att = np.array([a['_ts'] for a in att])
        roll = np.array([a.get('Roll', 0) for a in att])
        pitch = np.array([a.get('Pitch', 0) for a in att])

        # ATT 급변 감지: 0.5초 내 30도 이상 변화
        window = 10  # ~0.5초 (20Hz ATT 기준)
        crash_candidates = []
        for i in range(window, len(roll)):
            dr = abs(roll[i] - roll[i-window])
            dp = abs(pitch[i] - pitch[i-window])
            if dr > 30 or dp > 30:
                crash_candidates.append({
                    't': ts_att[i],
                    'roll_delta': dr,
                    'pitch_delta': dp,
                })

        if not crash_candidates:
            return

        # 고도 급락 확인
        alt_data = gps if gps else []
        crash_confirmed = []
        for cand in crash_candidates:
            ct = cand['t']
            # ±2초 내 고도 급락 확인
            nearby_gps = [g for g in alt_data if abs(g['_ts'] - ct) < 2]
            if len(nearby_gps) >= 2:
                alts_nearby = [g.get('Alt', 0) for g in nearby_gps]
                alt_drop = max(alts_nearby) - min(alts_nearby)
                if alt_drop > 3:  # 3m 이상 급락
                    cand['alt_drop'] = alt_drop
                    crash_confirmed.append(cand)

        # 연속된 크래시 후보 병합 (5초 이내)
        merged = []
        for c in crash_confirmed:
            if merged and c['t'] - merged[-1]['t'] < 5:
                if c.get('alt_drop', 0) > merged[-1].get('alt_drop', 0):
                    merged[-1] = c
            else:
                merged.append(c)

        t0 = self.p.t0()
        for crash in merged:
            rel_t = crash['t'] - t0
            self._add('FAIL', f'크래시 감지 — {rel_t:.0f}초 시점',
                      f'자세 급변: Roll Δ{crash["roll_delta"]:.0f}° / Pitch Δ{crash["pitch_delta"]:.0f}°\n'
                      f'고도 급락: {crash.get("alt_drop", 0):.1f}m',
                      '기체 전체 점검 필요: 프레임, 프롭, 모터, FC 마운트')

    # ── Feature 3: 파라미터 체크리스트 ──
    def _ck_params(self):
        """주요 파라미터 값 vs 권장값 비교"""
        parms = self.p.get('PARM')
        if not parms:
            return

        param_map = {}
        for p in parms:
            name = p.get('Name', '')
            val = p.get('Value', 0)
            if name:
                param_map[name] = val

        if not param_map:
            return

        # 파라미터 권장값 딕셔너리: (min, max, 설명, 조치)
        RECOMMENDED = {
            # 안전 — 페일세이프
            'FS_THR_ENABLE': (1, 2, '스로틀 페일세이프', 'RTL(1) 또는 LAND(2) 설정 권장'),
            'FS_GCS_ENABLE': (1, 2, 'GCS 연결 끊김 페일세이프', '활성화 권장'),
            'FS_EKF_THRESH': (0.6, 1.0, 'EKF 페일세이프 임계값', '0.8 기본, 0.6~1.0 범위'),
            'FS_CRASH_CHECK': (1, 1, '크래시 체크 활성화', '반드시 1로 설정'),
            # 배터리
            'BATT_LOW_VOLT': (0.1, 99, '배터리 저전압 경고', '셀당 3.5V 기준 설정 권장'),
            'BATT_CRT_VOLT': (0.1, 99, '배터리 위험 전압', '셀당 3.3V 기준 설정 권장'),
            'BATT_FS_LOW_ACT': (1, 4, '저전압 시 동작', 'RTL(2) 또는 LAND(1) 설정'),
            'BATT_FS_CRT_ACT': (1, 4, '위험전압 시 동작', 'LAND(1) 권장'),
            # 센서 필터
            'INS_ACCEL_FILTER': (10, 30, '가속도계 필터', '10~20Hz 권장, 진동 많으면 낮추기'),
            'INS_GYRO_FILTER': (20, 80, '자이로 필터', '기본 20Hz, 대형기 낮추기'),
            # 위치/네비게이션
            'WPNAV_SPEED': (100, 2000, '미션 비행 속도 (cm/s)', '기체 크기에 맞게'),
            'WPNAV_SPEED_DN': (50, 500, '하강 속도 (cm/s)', '150cm/s 기본'),
            'WPNAV_SPEED_UP': (100, 500, '상승 속도 (cm/s)', '250cm/s 기본'),
            'RTL_ALT': (500, 10000, 'RTL 고도 (cm)', '장애물 높이 + 여유'),
            # 모터/ESC
            'MOT_SPIN_ARM': (0.05, 0.15, '모터 시동 회전', '0.10 기본'),
            'MOT_SPIN_MIN': (0.10, 0.25, '모터 최소 회전', '0.15 기본'),
            'MOT_BAT_VOLT_MAX': (1, 99, '최대 배터리 전압', '셀×4.2V'),
            'MOT_BAT_VOLT_MIN': (1, 99, '최소 배터리 전압', '셀×3.3V'),
            # 착륙
            'LAND_SPEED': (20, 100, '최종 착륙 속도 (cm/s)', '50cm/s 기본'),
        }

        issues = []
        good = []
        not_set = []

        for param, (vmin, vmax, desc, fix) in RECOMMENDED.items():
            if param in param_map:
                val = param_map[param]
                if param in ('FS_THR_ENABLE', 'FS_GCS_ENABLE', 'FS_CRASH_CHECK',
                             'BATT_FS_LOW_ACT', 'BATT_FS_CRT_ACT'):
                    # 이산값: 0이면 비활성화 → 문제
                    if val == 0:
                        issues.append((param, val, desc, fix))
                    else:
                        good.append(param)
                elif param in ('BATT_LOW_VOLT', 'BATT_CRT_VOLT'):
                    if val == 0:
                        issues.append((param, val, desc, '미설정 — ' + fix))
                    else:
                        good.append(param)
                elif val < vmin or val > vmax:
                    issues.append((param, val, desc, f'권장 {vmin}~{vmax}, 현재 {val} — {fix}'))
                else:
                    good.append(param)
            else:
                # 안전 관련 파라미터 미존재 (로그에 기록 안 됨 = 기본값)
                if param in ('FS_THR_ENABLE', 'FS_GCS_ENABLE', 'FS_CRASH_CHECK'):
                    not_set.append(param)

        self.param_check = {'issues': issues, 'good': good, 'not_set': not_set,
                            'total': len(param_map)}

        if issues:
            detail_lines = [f'  {p}: {v} — {d}' for p, v, d, _ in issues[:8]]
            self._add('WARN', f'파라미터 점검 필요 ({len(issues)}건)',
                      '\n'.join(detail_lines),
                      issues[0][3] if issues else '')
        if not_set:
            self._add('WARN', f'안전 파라미터 확인 필요: {", ".join(not_set)}',
                      '로그에서 값 확인 불가 — 기본값 사용 중일 수 있음',
                      'Mission Planner에서 페일세이프 설정 확인')
        if not issues and not not_set:
            self._add('OK', f'파라미터 점검 양호 ({len(good)}개 확인)',
                      f'총 {len(param_map)}개 파라미터 기록')

    # ── Feature 4: 호버 안정성 ──
    def _ck_hover(self):
        """LOITER/POSHOLD 모드에서 위치 유지 정확도 (GPS 분산)"""
        gps = self.p.get('GPS')
        modes = self.summary['modes']
        if not gps or not modes:
            return

        # LOITER/POSHOLD 구간 추출
        hover_modes = {'LOITER', 'POSHOLD', 'ALT_HOLD'}
        hover_spans = []
        for i, m in enumerate(modes):
            if m['name'] in hover_modes:
                t_start = m['t']
                t_end = modes[i+1]['t'] if i+1 < len(modes) else gps[-1]['_ts']
                if t_end - t_start > 3:  # 3초 이상 구간만
                    hover_spans.append((t_start, t_end, m['name']))

        if not hover_spans:
            return

        # 각 호버 구간에서 GPS 분산 측정
        all_dists = []
        span_results = []
        for t_start, t_end, mode_name in hover_spans:
            pts = [(g.get('Lat', 0), g.get('Lng', 0))
                   for g in gps if t_start <= g['_ts'] <= t_end
                   and g.get('Lat', 0) != 0]
            if len(pts) < 10:
                continue
            lats_h = np.array([p[0] for p in pts])
            lons_h = np.array([p[1] for p in pts])
            # 중심점 기준 편차 (미터)
            clat, clon = np.mean(lats_h), np.mean(lons_h)
            dy = (lats_h - clat) * 111320
            dx = (lons_h - clon) * 111320 * np.cos(np.radians(clat))
            dists = np.sqrt(dy**2 + dx**2)
            all_dists.extend(dists.tolist())
            span_results.append({
                'mode': mode_name,
                'duration': round(t_end - t_start, 1),
                'mean_m': round(float(np.mean(dists)), 2),
                'max_m': round(float(np.max(dists)), 2),
                'std_m': round(float(np.std(dists)), 2),
                'cep50_m': round(float(np.percentile(dists, 50)), 2),
                'cep95_m': round(float(np.percentile(dists, 95)), 2),
            })

        if not span_results:
            return

        self.hover_data = span_results
        all_dists = np.array(all_dists)
        overall_cep50 = float(np.percentile(all_dists, 50))
        overall_cep95 = float(np.percentile(all_dists, 95))
        overall_max = float(np.max(all_dists))

        detail = (f'CEP50: {overall_cep50:.2f}m, CEP95: {overall_cep95:.2f}m, '
                  f'최대 편차: {overall_max:.2f}m\n')
        for sr in span_results[:3]:
            detail += f'  {sr["mode"]} {sr["duration"]}초: 평균 {sr["mean_m"]}m, 최대 {sr["max_m"]}m\n'

        if overall_cep95 > 5:
            self._add('FAIL', f'호버 위치 불안정 (CEP95 {overall_cep95:.1f}m)',
                      detail, 'GPS 환경 개선, 컴퍼스 캘리브, 진동 저감')
        elif overall_cep95 > 2:
            self._add('WARN', f'호버 위치 보통 (CEP95 {overall_cep95:.1f}m)',
                      detail, 'RTK GPS 적용 고려, 진동 점검')
        else:
            self._add('OK', f'호버 위치 양호 (CEP95 {overall_cep95:.1f}m)', detail)

    # ── Feature 5: ESC 온도/RPM ──
    def _ck_esc(self):
        """ESC 텔레메트리 — 온도, RPM, 전류 분석"""
        esc = self.p.get('ESC')
        if not esc or len(esc) < 10:
            return

        # ESC 데이터 그룹핑 (인스턴스별)
        esc_by_inst = {}
        for e in esc:
            inst = e.get('Instance', e.get('Inst', 0))
            if inst not in esc_by_inst:
                esc_by_inst[inst] = []
            esc_by_inst[inst].append(e)

        temps_all = []
        rpms_all = {}
        esc_detail = []

        for inst, data in sorted(esc_by_inst.items()):
            temps = [e.get('Temp', 0) for e in data if e.get('Temp', 0) > 0]
            rpms = [e.get('RPM', 0) for e in data if e.get('RPM', 0) > 0]
            currs = [e.get('Curr', 0) for e in data if e.get('Curr', 0) > 0]

            info = f'ESC{inst+1}:'
            if temps:
                temps_all.extend(temps)
                info += f' 온도 {np.mean(temps):.0f}°C (max {max(temps):.0f}°C)'
            if rpms:
                rpms_all[inst] = rpms
                info += f' RPM {np.mean(rpms):.0f} (max {max(rpms):.0f})'
            if currs:
                info += f' 전류 {np.mean(currs):.1f}A (max {max(currs):.1f}A)'
            esc_detail.append(info)

        self.esc_data = {'by_instance': esc_by_inst, 'detail': esc_detail}

        # 온도 판정
        if temps_all:
            max_temp = max(temps_all)
            if max_temp > 100:
                self._add('FAIL', f'ESC 과열 ({max_temp:.0f}°C)',
                          '\n'.join(esc_detail), 'ESC 냉각 개선, 과부하 확인')
            elif max_temp > 80:
                self._add('WARN', f'ESC 온도 높음 ({max_temp:.0f}°C)',
                          '\n'.join(esc_detail), 'ESC 방열 확인')
            else:
                self._add('OK', f'ESC 온도 양호 (최대 {max_temp:.0f}°C)',
                          '\n'.join(esc_detail))

        # RPM 불균형 체크
        if len(rpms_all) >= 4:
            rpm_means = {k: np.mean(v) for k, v in rpms_all.items()}
            vals = list(rpm_means.values())
            rpm_diff_pct = (max(vals) - min(vals)) / np.mean(vals) * 100
            if rpm_diff_pct > 15:
                detail_rpm = ' | '.join(f'M{k+1}: {v:.0f}rpm' for k, v in rpm_means.items())
                self._add('WARN', f'모터 RPM 불균형 ({rpm_diff_pct:.0f}%)',
                          detail_rpm, '프롭 밸런싱, 모터 베어링 점검')

    # ── Feature 6: 비행 구간 자동 분류 ──
    def _ck_phases(self):
        """스로틀+고도+속도 기반 비행 구간 자동 분류"""
        gps = self.p.get('GPS')
        rcou = self.p.get('RCOU')
        if not gps or len(gps) < 30:
            return

        t0 = self.p.t0()
        ts_gps = np.array([g['_ts'] for g in gps])
        alts = np.array([g.get('Alt', 0) for g in gps])
        spds = np.array([g.get('Spd', 0) for g in gps])

        # 고도 변화율 계산
        dt = np.diff(ts_gps)
        dt[dt == 0] = 0.1
        alt_rate = np.concatenate([[0], np.diff(alts) / dt])

        # 스로틀 평균 (RCOU → 4모터 평균)
        throttle = np.zeros(len(gps))
        if rcou and len(rcou) > 10:
            rcou_ts = np.array([r['_ts'] for r in rcou])
            rcou_avg = np.array([np.mean([r.get(f'C{i}', 0) for i in range(1, 5)]) for r in rcou])
            # GPS 타임스탬프에 맞춰 보간
            throttle = np.interp(ts_gps, rcou_ts, rcou_avg)

        # 구간 분류 로직
        phases = []
        for i in range(len(gps)):
            alt = alts[i]
            spd = spds[i]
            ar = alt_rate[i]
            thr = throttle[i]

            if alt < 0.5 and thr < 1100:
                phase = 'GROUND'
            elif ar > 1.0 and alt < 5:
                phase = 'TAKEOFF'
            elif ar > 0.5:
                phase = 'CLIMB'
            elif ar < -0.5 and alt < 5:
                phase = 'LANDING'
            elif ar < -0.3:
                phase = 'DESCENT'
            elif spd > 1.5:
                phase = 'CRUISE'
            elif alt > 1.0:
                phase = 'HOVER'
            else:
                phase = 'GROUND'
            phases.append(phase)

        # 구간 병합 (연속 동일 phase)
        merged = []
        if phases:
            cur_phase = phases[0]
            start_idx = 0
            for i in range(1, len(phases)):
                if phases[i] != cur_phase:
                    duration = ts_gps[i-1] - ts_gps[start_idx]
                    if duration > 1:  # 1초 미만 무시
                        merged.append({
                            'phase': cur_phase,
                            't_start': round(ts_gps[start_idx] - t0, 1),
                            't_end': round(ts_gps[i-1] - t0, 1),
                            'duration': round(duration, 1),
                        })
                    cur_phase = phases[i]
                    start_idx = i
            # 마지막 구간
            duration = ts_gps[-1] - ts_gps[start_idx]
            if duration > 1:
                merged.append({
                    'phase': cur_phase,
                    't_start': round(ts_gps[start_idx] - t0, 1),
                    't_end': round(ts_gps[-1] - t0, 1),
                    'duration': round(duration, 1),
                })

        self.flight_phases = merged

        # 요약
        phase_summary = {}
        for m in merged:
            p = m['phase']
            phase_summary[p] = phase_summary.get(p, 0) + m['duration']

        if phase_summary:
            total = sum(phase_summary.values())
            detail_parts = []
            phase_order = ['GROUND', 'TAKEOFF', 'CLIMB', 'HOVER', 'CRUISE', 'DESCENT', 'LANDING']
            phase_kr = {'GROUND': '지상', 'TAKEOFF': '이륙', 'CLIMB': '상승',
                        'HOVER': '호버링', 'CRUISE': '이동', 'DESCENT': '하강', 'LANDING': '착륙'}
            for p in phase_order:
                if p in phase_summary:
                    pct = phase_summary[p] / total * 100
                    detail_parts.append(f'{phase_kr.get(p, p)}: {phase_summary[p]:.0f}초 ({pct:.0f}%)')
            self._add('OK', f'비행 구간 분류 ({len(merged)}구간)',
                      ' | '.join(detail_parts))

    # ── Feature 7: 컴퍼스-스로틀 간섭 분석 ──
    def _ck_compass_motor_interference(self):
        """MAG 자기장과 스로틀(RCOU) 상관계수 분석 — 모터 전자기 간섭 감지"""
        mag = self.p.get('MAG')
        rcou = self.p.get('RCOU')
        if not mag or not rcou or len(mag) < 50 or len(rcou) < 50:
            return

        # MAG 자기장 크기
        mag_ts = np.array([m['_ts'] for m in mag])
        mag_field = np.array([np.sqrt(m.get('MagX',0)**2+m.get('MagY',0)**2+m.get('MagZ',0)**2)
                              for m in mag])

        # RCOU 평균 스로틀
        rcou_ts = np.array([r['_ts'] for r in rcou])
        rcou_avg = np.array([np.mean([r.get(f'C{i}',0) for i in range(1,5)]) for r in rcou])

        # 시간 정렬 — MAG 타임스탬프에 RCOU 보간
        thr_interp = np.interp(mag_ts, rcou_ts, rcou_avg)

        # 유효 구간만 (모터 돌아가는 구간)
        mask = thr_interp > 1100
        if mask.sum() < 30:
            return

        mag_active = mag_field[mask]
        thr_active = thr_interp[mask]

        # 피어슨 상관계수
        corr = np.corrcoef(mag_active, thr_active)[0, 1]
        self.compass_interference = round(float(corr), 3)

        detail = f'상관계수: {corr:.3f} (|r|>0.3이면 간섭 의심)'
        if abs(corr) > 0.5:
            self._add('FAIL', f'컴퍼스-모터 간섭 심각 (r={corr:.2f})',
                      detail, '컴퍼스 위치 이격, 전원 케이블 트위스트, 외장 컴퍼스 사용')
        elif abs(corr) > 0.3:
            self._add('WARN', f'컴퍼스-모터 간섭 의심 (r={corr:.2f})',
                      detail, '외장 컴퍼스 권장, MOT_COMP 파라미터 확인')
        else:
            self._add('OK', f'컴퍼스 간섭 양호 (r={corr:.2f})', detail)

    # ── Feature 8: 전류 스파이크 감지 ──
    def _ck_current_spikes(self):
        """배터리 전류 급증 — 비정상 전력 소모 감지"""
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        if not batt or len(batt) < 50:
            return

        currs = np.array([b.get('Curr', b.get('CurrR', 0)) for b in batt])
        ts = np.array([b['_ts'] for b in batt])
        if not any(currs > 0):
            return

        # 이동평균 대비 스파이크 감지
        window = min(20, len(currs) // 5)
        if window < 3:
            return
        moving_avg = np.convolve(currs, np.ones(window)/window, mode='same')
        spikes = []
        t0 = self.p.t0()
        for i in range(window, len(currs)-window):
            if moving_avg[i] > 0 and currs[i] > moving_avg[i] * 2.0 and currs[i] > 5:
                spikes.append({
                    'ts': round(ts[i] - t0, 1),
                    'curr': round(float(currs[i]), 1),
                    'avg': round(float(moving_avg[i]), 1),
                    'ratio': round(float(currs[i] / moving_avg[i]), 1),
                })

        # 근접 스파이크 병합 (5초)
        merged = []
        for s in spikes:
            if merged and s['ts'] - merged[-1]['ts'] < 5:
                if s['curr'] > merged[-1]['curr']:
                    merged[-1] = s
            else:
                merged.append(s)

        self.current_spikes = merged

        if len(merged) > 3:
            detail = '\n'.join(f'  {s["ts"]:.0f}초: {s["curr"]}A (평균 {s["avg"]}A의 {s["ratio"]}배)'
                              for s in merged[:5])
            self._add('WARN', f'전류 스파이크 {len(merged)}회 감지', detail,
                      'ESC/모터 점검, 배선 접촉불량 확인')
        elif len(merged) > 0:
            s = merged[0]
            self._add('OK', f'전류 스파이크 {len(merged)}회 (최대 {s["curr"]}A)',
                      f'{s["ts"]:.0f}초 시점')

    # ── Feature 9: 모터 포화 감지 ──
    def _ck_motor_saturation(self):
        """모터 PWM 상한(1950+) 지속 시간 — 추력 한계 도달 분석"""
        rcou = self.p.get('RCOU')
        if not rcou or len(rcou) < 50:
            return

        ts = np.array([r['_ts'] for r in rcou])
        total_time = ts[-1] - ts[0]
        if total_time <= 0:
            return

        sat_threshold = 1950
        saturated_samples = 0
        motor_sat = {i: 0 for i in range(1, 5)}

        for r in rcou:
            for i in range(1, 5):
                val = r.get(f'C{i}', 0)
                if val >= sat_threshold:
                    motor_sat[i] += 1
            if any(r.get(f'C{i}', 0) >= sat_threshold for i in range(1, 5)):
                saturated_samples += 1

        dt = total_time / len(rcou)  # 평균 샘플 간격
        sat_time = saturated_samples * dt
        sat_pct = saturated_samples / len(rcou) * 100

        detail_parts = [f'M{i}: {motor_sat[i]}회' for i in range(1, 5) if motor_sat[i] > 0]
        detail = f'포화 시간: {sat_time:.1f}초 ({sat_pct:.1f}%)\n' + ' | '.join(detail_parts) if detail_parts else f'포화 시간: {sat_time:.1f}초'

        self.motor_saturation = {'pct': round(sat_pct, 1), 'time': round(sat_time, 1),
                                  'by_motor': motor_sat}

        if sat_pct > 10:
            self._add('FAIL', f'모터 포화 심각 — {sat_pct:.1f}% ({sat_time:.0f}초)',
                      detail, '과적재 확인, 프롭/모터 업그레이드, 비행 기동 완화')
        elif sat_pct > 2:
            self._add('WARN', f'모터 포화 감지 — {sat_pct:.1f}% ({sat_time:.0f}초)',
                      detail, '추력 여유 확보 권장')
        elif sat_pct > 0:
            self._add('OK', f'모터 포화 미미 — {sat_pct:.1f}%', detail)

    # ── Feature 10: 전력 효율 ──
    def _ck_power_efficiency(self):
        """Wh/km, Wh/min, 잔여 비행시간 추정"""
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        gps = self.p.get('GPS')
        if not batt or not gps or len(batt) < 20:
            return

        s = self.summary
        dur_sec = s.get('dur_sec', 0)
        dist_m = s.get('dist_m', 0)
        if dur_sec <= 0:
            return

        # 소비 전력 계산
        volts = np.array([b.get('Volt', b.get('VoltR', 0)) for b in batt])
        currs = np.array([b.get('Curr', b.get('CurrR', 0)) for b in batt])
        ts = np.array([b['_ts'] for b in batt])

        # 유효 데이터만
        mask = (volts > 0) & (currs > 0)
        if mask.sum() < 10:
            return

        # 전력 적분 (W·s → Wh)
        power = volts[mask] * currs[mask]  # 와트
        dt = np.diff(ts[mask])
        dt = np.clip(dt, 0, 10)  # 이상치 제거
        energy_ws = np.sum(power[1:] * dt)  # W·s
        energy_wh = energy_ws / 3600

        # mAh (비교용)
        mah_used = s.get('mah', 0)
        if mah_used <= 0:
            # CurrTot에서 가져오기
            currtot = [b.get('CurrTot', 0) for b in batt]
            mah_used = max(currtot) if currtot else 0

        dur_min = dur_sec / 60
        efficiency = {}
        efficiency['wh_total'] = round(energy_wh, 1)
        efficiency['wh_per_min'] = round(energy_wh / dur_min, 2) if dur_min > 0 else 0
        efficiency['mah_used'] = round(mah_used, 0)
        if dist_m > 100:
            efficiency['wh_per_km'] = round(energy_wh / (dist_m / 1000), 1)
        efficiency['avg_power_w'] = round(float(np.mean(power)), 1)
        efficiency['max_power_w'] = round(float(np.max(power)), 1)

        # 잔여 비행시간 추정 (배터리 잔량 기반)
        rem_pct = [b.get('RemPct', 0) for b in batt if b.get('RemPct', 0) > 0]
        if rem_pct and efficiency['wh_per_min'] > 0:
            last_rem = rem_pct[-1]
            used_pct = 100 - last_rem
            if used_pct > 5:
                total_wh_capacity = energy_wh / (used_pct / 100)
                remaining_wh = total_wh_capacity * (last_rem / 100)
                remaining_min = remaining_wh / efficiency['wh_per_min']
                efficiency['remaining_min'] = round(remaining_min, 1)
                efficiency['battery_capacity_wh'] = round(total_wh_capacity, 1)

        self.power_efficiency = efficiency

        detail = f'소비: {energy_wh:.1f}Wh ({mah_used:.0f}mAh)\n'
        detail += f'평균: {efficiency["avg_power_w"]}W, 최대: {efficiency["max_power_w"]}W\n'
        detail += f'{efficiency["wh_per_min"]}Wh/min'
        if 'wh_per_km' in efficiency:
            detail += f', {efficiency["wh_per_km"]}Wh/km'
        if 'remaining_min' in efficiency:
            detail += f'\n잔여 비행시간 추정: {efficiency["remaining_min"]:.0f}분'

        self._add('OK', f'전력 효율 — {efficiency["wh_per_min"]}Wh/min', detail)

    # ── Feature 11: RC 신호 품질 ──
    def _ck_rc_signal(self):
        """RSSI 트렌드 + RC 신호 급락 감지"""
        rssi_data = self.p.get('RSSI')
        rcin = self.p.get('RCIN')
        t0 = self.p.t0()

        if rssi_data and len(rssi_data) > 10:
            rssi_vals = [r.get('RXRSSI', r.get('Rx', 0)) for r in rssi_data]
            rssi_vals = [v for v in rssi_vals if v > 0]
            if rssi_vals:
                min_rssi = min(rssi_vals)
                avg_rssi = np.mean(rssi_vals)
                if min_rssi < 50:
                    self._add('WARN', f'RC 신호 약함 (RSSI 최저 {min_rssi:.0f})',
                              f'평균 {avg_rssi:.0f}', '안테나 방향/위치 확인')
                else:
                    self._add('OK', f'RC 신호 양호 (RSSI 평균 {avg_rssi:.0f})',
                              f'최저 {min_rssi:.0f}')
                return

        # RSSI 메시지 없으면 RCIN 채널 유효성으로 대체
        if not rcin or len(rcin) < 30:
            return

        # RCIN 채널 값이 0으로 떨어지는 구간 = 신호 손실
        dropouts = 0
        for r in rcin:
            ch3 = r.get('C3', r.get('Chan3', 0))
            if ch3 == 0 or ch3 < 800:
                dropouts += 1

        dropout_pct = dropouts / len(rcin) * 100
        if dropout_pct > 5:
            self._add('WARN', f'RC 신호 손실 {dropout_pct:.1f}%',
                      f'{dropouts}회 드롭아웃 / {len(rcin)}샘플',
                      '수신기 점검, 안테나 위치 변경')
        elif dropout_pct > 0:
            self._add('OK', f'RC 신호 드롭아웃 {dropouts}회 ({dropout_pct:.1f}%)', '')

    # ── Feature 12: 기압계 드리프트 ──
    def _ck_baro_drift(self):
        """BARO 고도 vs GPS 고도 편차 추적"""
        baro = self.p.get('BARO')
        gps = self.p.get('GPS')
        if not baro or not gps or len(baro) < 30 or len(gps) < 30:
            return

        # GPS 고도 보간하여 BARO 타임스탬프에 맞춤
        gps_ts = np.array([g['_ts'] for g in gps])
        gps_alt = np.array([g.get('Alt', 0) for g in gps])
        baro_ts = np.array([b['_ts'] for b in baro])
        baro_alt = np.array([b.get('Alt', 0) for b in baro])

        if not any(gps_alt > 0) or not any(baro_alt != 0):
            return

        # GPS 고도를 바로 타임스탬프에 보간
        gps_interp = np.interp(baro_ts, gps_ts, gps_alt)

        # 초기 오프셋 제거 (첫 10개 평균)
        init_offset = np.mean(gps_interp[:10]) - np.mean(baro_alt[:10])
        diff = (gps_interp - init_offset) - baro_alt

        # 시간에 따른 드리프트 추세
        drift_start = np.mean(diff[:len(diff)//10]) if len(diff) > 10 else 0
        drift_end = np.mean(diff[-len(diff)//10:]) if len(diff) > 10 else 0
        drift_total = drift_end - drift_start
        max_diff = float(np.max(np.abs(diff)))

        self.baro_drift = {'total_m': round(drift_total, 2), 'max_diff_m': round(max_diff, 2)}

        if abs(drift_total) > 5 or max_diff > 10:
            self._add('WARN', f'기압계 드리프트 감지 ({drift_total:+.1f}m)',
                      f'BARO-GPS 최대 편차: {max_diff:.1f}m\n'
                      f'시작→종료 드리프트: {drift_total:+.1f}m',
                      '기압계 온도 보상 확인, EKF_ALT_SOURCE 점검')
        else:
            self._add('OK', f'기압계 안정 (드리프트 {drift_total:+.1f}m)',
                      f'최대 편차: {max_diff:.1f}m')

    # ── Feature 13: 듀얼 컴퍼스 일관성 ──
    def _ck_dual_compass(self):
        """MAG vs MAG2 자기장 편차 모니터링"""
        mag1 = self.p.get('MAG')
        mag2 = self.p.get('MAG2')
        if not mag1 or not mag2 or len(mag1) < 30 or len(mag2) < 30:
            return  # 듀얼 컴퍼스 아닌 기체 → 스킵

        # 자기장 크기 비교
        mag1_ts = np.array([m['_ts'] for m in mag1])
        mag1_f = np.array([np.sqrt(m.get('MagX',0)**2+m.get('MagY',0)**2+m.get('MagZ',0)**2) for m in mag1])
        mag2_ts = np.array([m['_ts'] for m in mag2])
        mag2_f = np.array([np.sqrt(m.get('MagX',0)**2+m.get('MagY',0)**2+m.get('MagZ',0)**2) for m in mag2])

        # 시간 정렬
        mag2_interp = np.interp(mag1_ts, mag2_ts, mag2_f)
        diff = np.abs(mag1_f - mag2_interp)
        avg_diff = float(np.mean(diff))
        max_diff = float(np.max(diff))

        # 비율로도 확인
        ratio = mag1_f / np.clip(mag2_interp, 1, None)
        inconsistency = float(np.std(ratio))

        self.dual_compass = {'avg_diff': round(avg_diff, 1), 'max_diff': round(max_diff, 1),
                              'consistency': round(1 - inconsistency, 3)}

        if avg_diff > 100 or max_diff > 200:
            self._add('WARN', f'듀얼 컴퍼스 불일치 (평균 {avg_diff:.0f}, 최대 {max_diff:.0f})',
                      f'MAG1 vs MAG2 자기장 크기 편차',
                      '컴퍼스 재캘리브, 간섭원 확인, COMPASS_USE 설정')
        else:
            self._add('OK', f'듀얼 컴퍼스 일관 (편차 평균 {avg_diff:.0f})',
                      f'최대 편차: {max_diff:.0f}')

    # ── Feature 14: Desired vs Actual 위치/고도 ──
    def _ck_des_vs_act(self):
        """명령 고도 vs 실제 고도 추적 오차"""
        ctun = self.p.get('CTUN')
        if not ctun or len(ctun) < 50:
            return

        # DAlt(Desired Altitude) vs Alt(Actual Altitude)
        dalt = np.array([c.get('DAlt', 0) for c in ctun])
        alt = np.array([c.get('Alt', 0) for c in ctun])

        # 유효 구간 (비행 중)
        mask = alt > 1
        if mask.sum() < 20:
            return

        err = np.abs(dalt[mask] - alt[mask])
        rms_err = float(np.sqrt(np.mean(err**2)))
        max_err = float(np.max(err))
        avg_err = float(np.mean(err))

        self.alt_tracking = {'rms': round(rms_err, 2), 'max': round(max_err, 2),
                              'avg': round(avg_err, 2)}

        detail = f'RMS 오차: {rms_err:.2f}m, 평균: {avg_err:.2f}m, 최대: {max_err:.2f}m'
        if rms_err > 3:
            self._add('WARN', f'고도 추적 부정확 (RMS {rms_err:.1f}m)',
                      detail, 'PSC_POSZ_P, PSC_VELZ_P 게인 조정')
        elif rms_err > 1:
            self._add('OK', f'고도 추적 보통 (RMS {rms_err:.1f}m)', detail)
        else:
            self._add('OK', f'고도 추적 양호 (RMS {rms_err:.2f}m)', detail)

    # ── Feature 15: 페일세이프 재구성 ──
    def _ck_failsafe_reconstruct(self):
        """에러/이벤트/MSG를 조합하여 페일세이프 트리거 원인 역추적"""
        errs = self.summary['errors']
        msgs = self.summary['msgs']
        evts = self.summary['events']
        t0 = self.p.t0()

        # 페일세이프 관련 에러코드
        fs_subsys = {5, 11, 12, 17, 22, 23, 24, 26}  # Radio, Fence, Battery, GCS, EKF 등
        fs_events = []

        for e in errs:
            if e['ec'] == 0:
                continue
            info = lookup_err(e['sub'], e['ec'])
            ei = info['error']
            if e['sub'] in fs_subsys or 'failsafe' in ei.get('name', '').lower() or 'failsafe' in ei.get('desc', '').lower():
                fs_events.append({
                    'ts': round(e['t'] - t0, 1),
                    'type': 'ERR',
                    'subsys': info['subsystem'],
                    'desc': ei.get('desc', f'코드 {e["ec"]}'),
                    'cause': ei.get('cause', ''),
                    'fix': ei.get('fix', ''),
                })

        # MSG에서 Failsafe 키워드 추출
        for m in msgs:
            txt = m['text']
            if 'failsafe' in txt.lower() or 'fail safe' in txt.lower():
                fs_events.append({
                    'ts': round(m['t'] - t0, 1),
                    'type': 'MSG',
                    'subsys': 'Message',
                    'desc': txt[:80],
                    'cause': '',
                    'fix': '',
                })

        if not fs_events:
            return

        # 시간순 정렬
        fs_events.sort(key=lambda x: x['ts'])

        # 근접 중복 제거
        merged_fs = []
        for fs in fs_events:
            if merged_fs and abs(fs['ts'] - merged_fs[-1]['ts']) < 2 and fs['subsys'] == merged_fs[-1]['subsys']:
                continue
            merged_fs.append(fs)

        self.failsafe_events = merged_fs

        detail = '\n'.join(f'  [{fs["ts"]:.0f}초] {fs["subsys"]}: {fs["desc"]}'
                          + (f'\n    원인: {fs["cause"]}' if fs["cause"] else '')
                          for fs in merged_fs[:6])

        self._add('WARN' if len(merged_fs) <= 2 else 'FAIL',
                  f'페일세이프 이력 {len(merged_fs)}건',
                  detail,
                  merged_fs[0]['fix'] if merged_fs[0]['fix'] else '페일세이프 파라미터 점검')

    # ── Feature 16: FFT 진동 주파수 분석 ──
    def _ck_fft(self):
        """IMU 가속도계 FFT — 공진 주파수 + 노치필터 제안"""
        imu = self.p.get('IMU')
        if not imu or len(imu) < 256:
            return

        # 샘플링 레이트 추정
        ts = np.array([d['_ts'] for d in imu[:500]])
        dt_arr = np.diff(ts)
        dt_arr = dt_arr[dt_arr > 0]
        if len(dt_arr) == 0:
            return
        dt_avg = float(np.median(dt_arr))
        fs = 1.0 / dt_avg if dt_avg > 0 else 100

        # 최대 4096 샘플 사용
        n_samples = min(len(imu), 4096)
        fft_results = {}

        for ax_key, ax_name in [('AccX', 'X'), ('AccY', 'Y'), ('AccZ', 'Z')]:
            vals = np.array([d.get(ax_key, 0) for d in imu[:n_samples]])
            if len(vals) < 128:
                continue

            # DC 제거 + 해닝 윈도우
            vals = vals - np.mean(vals)
            window = np.hanning(len(vals))
            fft_result = np.fft.rfft(vals * window)
            freqs = np.fft.rfftfreq(len(vals), d=1.0/fs)
            magnitude = np.abs(fft_result) * 2.0 / len(vals)

            # 1Hz ~ Nyquist
            mask = freqs >= 1.0
            fft_results[ax_name] = {
                'freqs': freqs[mask],
                'magnitude': magnitude[mask],
            }

        if not fft_results:
            return

        # 각 축의 피크 주파수 찾기
        peaks = {}
        for ax, data in fft_results.items():
            mag = data['magnitude']
            freq = data['freqs']
            if len(mag) < 5:
                continue
            # 상위 3개 피크
            peak_indices = np.argsort(mag)[-3:][::-1]
            peaks[ax] = [(round(float(freq[i]), 1), round(float(mag[i]), 4)) for i in peak_indices]

        self.fft_peaks = peaks
        self.fft_fs = round(fs, 1)

        # 모든 축의 1위 피크 주파수
        all_peaks = []
        for ax, pk_list in peaks.items():
            if pk_list:
                all_peaks.append((ax, pk_list[0][0], pk_list[0][1]))

        if not all_peaks:
            return

        dominant = max(all_peaks, key=lambda x: x[2])
        detail = f'샘플링: {fs:.0f}Hz, {n_samples}샘플\n'
        for ax, pk_list in peaks.items():
            detail += f'  {ax}축: ' + ', '.join(f'{f:.0f}Hz({m:.3f})' for f, m in pk_list) + '\n'

        # 프롭 회전 주파수 범위 (보통 50~200Hz)
        dom_freq = dominant[1]
        if dom_freq > 30 and dominant[2] > 0.5:
            self._add('WARN', f'진동 피크 {dom_freq:.0f}Hz ({dominant[0]}축)',
                      detail,
                      f'INS_HNTCH_FREQ={dom_freq:.0f} 노치필터 설정 권장, 프롭 밸런싱')
        else:
            self._add('OK', f'FFT 피크 {dom_freq:.0f}Hz ({dominant[0]}축)', detail)

    # ══════════════════════════════════════════════
    # 종합 분석 엔진 — 이벤트 타임라인 + 근본 원인 + 비행 복기
    # ══════════════════════════════════════════════

    def _build_timeline(self) -> list:
        """모든 이벤트(ERR, EV, MODE, MSG, 주요 센서 이상)를 시간순 통합"""
        t0 = self.p.t0()
        events = []

        # 모드 변경
        for m in self.summary['modes']:
            events.append({
                'ts': round(m['t'] - t0, 1),
                'type': 'MODE',
                'icon': '🔄',
                'title': f'모드 변경 → {m["name"]}',
                'detail': '',
                'sev': 'INFO',
            })

        # 에러
        for e in self.summary['errors']:
            if e['ec'] == 0:
                continue
            info = lookup_err(e['sub'], e['ec'])
            sn = info['subsystem']
            ei = info['error']
            desc = ei.get('desc', f'코드 {e["ec"]}')
            cause = ei.get('cause', '')
            fix = ei.get('fix', '')
            events.append({
                'ts': round(e['t'] - t0, 1),
                'type': 'ERROR',
                'icon': '🔴',
                'title': f'에러: {ei.get("name", sn)}',
                'detail': f'{desc}' + (f' | 원인: {cause}' if cause else ''),
                'fix': fix,
                'sev': 'FAIL',
            })

        # 이벤트
        crit_ev = {54, 59}
        warn_ev = {19, 60, 62, 80}
        for ev in self.summary['events']:
            eid = ev['id']
            desc = lookup_event(eid)
            if eid in crit_ev:
                sev, icon = 'FAIL', '🔴'
            elif eid in warn_ev:
                sev, icon = 'WARN', '🟡'
            else:
                sev, icon = 'INFO', '🔵'
            events.append({
                'ts': round(ev['t'] - t0, 1),
                'type': 'EVENT',
                'icon': icon,
                'title': desc,
                'detail': f'이벤트 ID: {eid}',
                'sev': sev,
            })

        # MSG 중 중요한 것
        kw_fail = ['fail', 'error', 'crash', 'lost', 'critical', 'emergency']
        kw_warn = ['warning', 'caution', 'bad', 'low', 'check', 'prearm']
        for m in self.summary['msgs']:
            txt = m['text']
            tl = txt.lower()
            sev, icon = None, None
            for kw in kw_fail:
                if kw in tl:
                    sev, icon = 'FAIL', '🔴'
                    break
            if not sev:
                for kw in kw_warn:
                    if kw in tl:
                        sev, icon = 'WARN', '🟡'
                        break
            if sev:
                events.append({
                    'ts': round(m['t'] - t0, 1),
                    'type': 'MSG',
                    'icon': icon,
                    'title': txt[:80],
                    'detail': '',
                    'sev': sev,
                })

        # 센서 임계 초과 이벤트 (진동, EKF, 배터리, GPS)
        # 진동 스파이크
        vibes = self.p.get('VIBE')
        if vibes:
            for v in vibes:
                worst = max(v.get('VibeX', 0), v.get('VibeY', 0), v.get('VibeZ', 0))
                if worst > 30:
                    events.append({
                        'ts': round(v['_ts'] - t0, 1),
                        'type': 'SENSOR',
                        'icon': '📳',
                        'title': f'진동 급증 {worst:.0f}m/s²',
                        'detail': f'X:{v.get("VibeX",0):.0f} Y:{v.get("VibeY",0):.0f} Z:{v.get("VibeZ",0):.0f}',
                        'sev': 'WARN',
                    })

        # GPS 위성 수 급감
        gps = self.p.get('GPS')
        if gps:
            prev_sats = 99
            for g in gps:
                sats = g.get('NSats', g.get('nSats', 0))
                if prev_sats >= 8 and sats < 6:
                    events.append({
                        'ts': round(g['_ts'] - t0, 1),
                        'type': 'SENSOR',
                        'icon': '📡',
                        'title': f'GPS 위성 급감 {prev_sats}→{sats}',
                        'detail': f'HDop: {g.get("HDop", 99):.1f}',
                        'sev': 'WARN',
                    })
                prev_sats = sats

        # 배터리 전압 급락
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        if batt and len(batt) > 10:
            cells = max(round(self.summary['v_max'] / 4.2), 1) if self.summary['v_max'] > 0 else 1
            for b in batt:
                volt = b.get('Volt', b.get('VoltR', 0))
                if volt > 0 and volt / cells < 3.3:
                    events.append({
                        'ts': round(b['_ts'] - t0, 1),
                        'type': 'SENSOR',
                        'icon': '🔋',
                        'title': f'배터리 위험 전압 {volt:.1f}V (셀당 {volt/cells:.2f}V)',
                        'detail': f'전류: {b.get("Curr", b.get("CurrR", 0)):.1f}A',
                        'sev': 'FAIL',
                    })
                    break  # 최초 1회만

        # 시간순 정렬
        events.sort(key=lambda e: e['ts'])

        # 중복 근접 이벤트 제거 (같은 type, 2초 이내)
        filtered = []
        for ev in events:
            if filtered and ev['type'] == filtered[-1]['type'] and ev['ts'] - filtered[-1]['ts'] < 2:
                continue
            filtered.append(ev)

        return filtered

    def _root_cause_analysis(self) -> dict:
        """여러 센서 데이터를 교차 분석하여 근본 원인 추론"""
        t0 = self.p.t0()
        gps = self.p.get('GPS')
        att = self.p.get('ATT')
        rcou = self.p.get('RCOU')
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        vibes = self.p.get('VIBE')
        ekf = self.p.get('NKF4') or self.p.get('XKF4') or self.p.get('EKF4')

        causes = []       # 추론된 원인 리스트
        evidence = []     # 근거
        correlations = [] # 상관관계

        # ── 1. 에러 시점 주변 교차 분석 ──
        err_times = [e['t'] for e in self.summary['errors'] if e['ec'] != 0]
        for et in err_times:
            window = 3  # ±3초

            # 에러 시점의 진동
            if vibes:
                v_near = [v for v in vibes if abs(v['_ts'] - et) < window]
                if v_near:
                    worst_v = max(max(v.get('VibeX', 0), v.get('VibeY', 0), v.get('VibeZ', 0)) for v in v_near)
                    if worst_v > 15:
                        correlations.append({
                            'ts': round(et - t0, 1),
                            'finding': f'에러 발생 시 진동 {worst_v:.0f}m/s² 동반',
                            'link': 'vibration→error',
                        })

            # 에러 시점의 모터 출력
            if rcou:
                r_near = [r for r in rcou if abs(r['_ts'] - et) < window]
                if r_near:
                    for r in r_near:
                        vals = [r.get(f'C{i}', 0) for i in range(1, 5)]
                        vals = [v for v in vals if v > 900]
                        if vals and (max(vals) > 1900 or (max(vals) - min(vals)) > 300):
                            correlations.append({
                                'ts': round(et - t0, 1),
                                'finding': f'에러 시점 모터 불균형 (차이 {max(vals)-min(vals):.0f}μs, 최대 {max(vals):.0f}μs)',
                                'link': 'motor→error',
                            })
                            break

            # 에러 시점의 배터리
            if batt:
                b_near = [b for b in batt if abs(b['_ts'] - et) < window]
                if b_near:
                    volt = min(b.get('Volt', b.get('VoltR', 99)) for b in b_near)
                    curr = max(b.get('Curr', b.get('CurrR', 0)) for b in b_near)
                    cells = max(round(self.summary['v_max'] / 4.2), 1) if self.summary['v_max'] > 0 else 1
                    if volt / cells < 3.5:
                        correlations.append({
                            'ts': round(et - t0, 1),
                            'finding': f'에러 시점 저전압 {volt:.1f}V (셀당 {volt/cells:.2f}V), 전류 {curr:.1f}A',
                            'link': 'battery→error',
                        })

            # 에러 시점의 EKF
            if ekf:
                e_near = [e for e in ekf if abs(e['_ts'] - et) < window]
                if e_near:
                    for ek in e_near:
                        over = [k for k in ('SV', 'SP', 'SH', 'SM') if ek.get(k, 0) > 0.8]
                        if over:
                            correlations.append({
                                'ts': round(et - t0, 1),
                                'finding': f'에러 시점 EKF 초과: {", ".join(over)}',
                                'link': 'ekf→error',
                            })
                            break

        # ── 2. 패턴 기반 근본 원인 추론 ──
        link_counts = {}
        for c in correlations:
            link = c['link']
            link_counts[link] = link_counts.get(link, 0) + 1

        # 모터 관련 원인
        motor_issues = link_counts.get('motor→error', 0)
        vibe_issues = link_counts.get('vibration→error', 0)
        batt_issues = link_counts.get('battery→error', 0)
        ekf_issues = link_counts.get('ekf→error', 0)

        # 모터/기계 문제 판정
        if motor_issues >= 2 or (motor_issues >= 1 and vibe_issues >= 1):
            causes.append({
                'category': '모터/기계',
                'confidence': 'HIGH' if motor_issues >= 2 else 'MEDIUM',
                'title': '모터 또는 기계적 문제',
                'detail': '에러 발생 시 모터 출력 불균형 + 진동 급증이 반복 감지됨',
                'action': [
                    '모터 개별 테스트 (Motor Test)',
                    '프롭 밸런싱 및 손상 확인',
                    'ESC 캘리브레이션',
                    'FC 방진마운트 점검',
                ],
            })
            evidence.append(f'모터 불균형 {motor_issues}회, 진동 상관 {vibe_issues}회')

        # 배터리 문제
        if batt_issues >= 1:
            cells = max(round(self.summary['v_max'] / 4.2), 1) if self.summary['v_max'] > 0 else 1
            causes.append({
                'category': '배터리/전원',
                'confidence': 'HIGH' if batt_issues >= 2 else 'MEDIUM',
                'title': '배터리 또는 전원 공급 문제',
                'detail': f'에러 발생 시점에 저전압 감지 ({cells}S 배터리)',
                'action': [
                    '배터리 내부저항 측정',
                    '셀 밸런스 확인',
                    '비행시간 단축 or 배터리 교체',
                    '파워모듈/BEC 점검',
                ],
            })
            evidence.append(f'저전압 상관 {batt_issues}회')

        # GPS/EKF 문제
        if ekf_issues >= 1:
            causes.append({
                'category': 'GPS/네비게이션',
                'confidence': 'HIGH' if ekf_issues >= 2 else 'MEDIUM',
                'title': 'GPS 또는 네비게이션 시스템 문제',
                'detail': 'EKF 상태 악화와 에러 발생이 동시에 관찰됨',
                'action': [
                    'GPS 안테나 위치 및 차폐 확인',
                    '컴퍼스 캘리브레이션',
                    '자기 간섭원 제거 (전선, 모터 근접)',
                    '듀얼 GPS 고려',
                ],
            })
            evidence.append(f'EKF 상관 {ekf_issues}회')

        # 진동만 단독
        if vibe_issues >= 2 and motor_issues == 0:
            causes.append({
                'category': '진동/구조',
                'confidence': 'MEDIUM',
                'title': '구조적 진동 문제',
                'detail': '모터 출력은 정상이나 진동이 에러와 상관 → 구조적 문제 의심',
                'action': [
                    '프레임 볼트 체결 상태',
                    '프롭 밸런싱',
                    'FC 방진마운트 교체',
                    '공진 주파수 확인 (FFT)',
                ],
            })

        # 복합 원인
        active_links = sum(1 for v in link_counts.values() if v > 0)
        if active_links >= 3:
            causes.insert(0, {
                'category': '복합 원인',
                'confidence': 'HIGH',
                'title': '다중 시스템 연쇄 장애',
                'detail': f'{active_links}개 영역에서 동시 이상 감지 — 하나의 근본 원인이 연쇄 효과를 일으킨 가능성',
                'action': [
                    '가장 먼저 발생한 이상 징후부터 역추적',
                    '아래 개별 원인 순서대로 점검',
                ],
            })

        # 문제 없는 경우
        if not causes and not correlations:
            causes.append({
                'category': '정상',
                'confidence': 'HIGH',
                'title': '주요 이상 없음',
                'detail': '에러 시점 교차 분석에서 센서 간 상관관계 미발견',
                'action': ['정기 점검 유지'],
            })

        return {
            'causes': causes,
            'evidence': evidence,
            'correlations': correlations,
        }

    def _build_flight_story(self) -> str:
        """비행 전체를 자연어로 복기 — '무슨 일이 있었는가'"""
        s = self.summary
        t0 = self.p.t0()
        lines = []

        # 1. 기본 정보
        lines.append(f'■ 비행 개요')
        lines.append(f'파일 {s["filename"]}, 비행시간 {s["dur_str"]}, '
                     f'최대 고도 {s["max_alt"]:.0f}m, 이동거리 {s["dist_m"]:.0f}m')
        if s['v_min'] > 0:
            cells = max(round(s['v_max'] / 4.2), 1)
            lines.append(f'배터리 {cells}S, {s["v_max"]:.1f}V→{s["v_min"]:.1f}V '
                         f'(셀당 {s["v_min"]/cells:.2f}V), 최대 전류 {s["i_max"]:.1f}A')

        # 2. 비행 흐름 서술
        lines.append(f'\n■ 비행 흐름')
        modes = s['modes']
        if modes:
            mode_t0 = modes[0]['t']
            for i, m in enumerate(modes):
                rel_t = m['t'] - mode_t0
                mins = int(rel_t // 60)
                secs = int(rel_t % 60)
                time_str = f'{mins}분 {secs}초' if mins > 0 else f'{secs}초'
                next_t = modes[i+1]['t'] - mode_t0 if i+1 < len(modes) else s.get('dur_sec', 0)
                dur = next_t - rel_t
                lines.append(f'  {time_str} — {m["name"]} ({dur:.0f}초간)')

        # 3. 주요 사건 (타임라인에서 FAIL/WARN만)
        critical_events = [e for e in self.timeline if e['sev'] in ('FAIL', 'WARN')]
        if critical_events:
            lines.append(f'\n■ 주요 사건 ({len(critical_events)}건)')
            for ev in critical_events[:15]:
                mins = int(ev['ts'] // 60)
                secs = int(ev['ts'] % 60)
                time_str = f'{mins}:{secs:02d}'
                lines.append(f'  [{time_str}] {ev["icon"]} {ev["title"]}')
                if ev.get('detail'):
                    lines.append(f'         → {ev["detail"][:100]}')

        # 4. 근본 원인 결론
        if self.root_cause['causes']:
            lines.append(f'\n■ 원인 분석 결론')
            for i, cause in enumerate(self.root_cause['causes'], 1):
                conf_kr = {'HIGH': '높음', 'MEDIUM': '보통', 'LOW': '낮음'}
                lines.append(f'  {i}. [{cause["category"]}] {cause["title"]} '
                             f'(확신도: {conf_kr.get(cause["confidence"], cause["confidence"])})')
                lines.append(f'     {cause["detail"]}')
                if cause.get('action'):
                    lines.append(f'     조치: {" → ".join(cause["action"][:3])}')

        # 5. 상관관계 근거
        if self.root_cause['correlations']:
            lines.append(f'\n■ 교차 분석 근거')
            seen = set()
            for c in self.root_cause['correlations'][:10]:
                key = c['finding'][:50]
                if key in seen:
                    continue
                seen.add(key)
                mins = int(c['ts'] // 60)
                secs = int(c['ts'] % 60)
                lines.append(f'  [{mins}:{secs:02d}] {c["finding"]}')

        # 6. 최종 권고
        fails = sum(1 for f in self.findings if f.sev == 'FAIL')
        warns = sum(1 for f in self.findings if f.sev == 'WARN')
        lines.append(f'\n■ 종합 판정: FAIL {fails}건 / WARN {warns}건')
        if fails > 0:
            lines.append('  ⛔ 재비행 전 반드시 원인 해결 필요')
        elif warns > 3:
            lines.append('  ⚠️ 다수 경고 — 점검 후 비행 권장')
        else:
            lines.append('  ✅ 특이사항 없음 — 정상 비행')

        return '\n'.join(lines)
class ChartGen:
    """다크 테마, 비행모드 배경 밴드, 이벤트/에러 마커"""

    def __init__(self, parser: LogParser, analyzer: Analyzer, outdir: str):
        self.p = parser
        self.a = analyzer
        self.outdir = outdir
        os.makedirs(outdir, exist_ok=True)
        self.t0 = parser.t0()
        # 비행 모드 구간
        self.mode_spans = self._build_mode_spans()
        # 에러/이벤트 시점
        self.err_times = [e['t'] - self.t0 for e in self.a.summary['errors'] if e['ec']!=0]
        self.ev_times_warn = [e['t']-self.t0 for e in self.a.summary['events'] if e['id'] in {19,60,62,80}]

    def _build_mode_spans(self) -> list:
        modes = self.a.summary['modes']
        if not modes: return []
        spans = []
        gps = self.p.get('GPS')
        end_t = (gps[-1]['_ts'] - self.t0) if gps else 600
        for i, m in enumerate(modes):
            t_start = m['t'] - self.t0
            t_end = (modes[i+1]['t'] - self.t0) if i+1 < len(modes) else end_t
            spans.append((t_start, t_end, m['name']))
        return spans

    def _apply_style(self, ax, title='', ylabel=''):
        """UAV Log Viewer 다크 테마"""
        ax.set_facecolor(C_BG_DARK)
        ax.figure.patch.set_facecolor(C_BG_DARK)
        ax.tick_params(colors=C_TEXT_DIM, labelsize=8)
        ax.xaxis.label.set_color(C_TEXT_DIM)
        ax.yaxis.label.set_color(C_TEXT_DIM)
        ax.spines['bottom'].set_color(C_GRID)
        ax.spines['left'].set_color(C_GRID)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.grid(True, color=C_GRID, alpha=0.4, linewidth=0.5)
        if title: ax.set_title(title, color=C_TEXT, fontsize=11, fontweight='bold', pad=8)
        if ylabel: ax.set_ylabel(ylabel, fontsize=9)

    def _add_mode_bands(self, ax):
        """비행 모드 배경 색상 밴드"""
        ymin, ymax = ax.get_ylim()
        for t0, t1, name in self.mode_spans:
            color = MODE_COLORS.get(name, '#37474f')
            ax.axvspan(t0, t1, alpha=0.12, color=color, zorder=0)
            mid = (t0+t1)/2
            ax.text(mid, ymax*0.95, name, fontsize=6, color=color,
                    ha='center', va='top', alpha=0.8, zorder=1)

    def _add_err_markers(self, ax):
        """에러/이벤트 수직선"""
        for t in self.err_times:
            ax.axvline(t, color=C_RED, alpha=0.5, linewidth=0.8, linestyle='--', zorder=2)
        for t in self.ev_times_warn:
            ax.axvline(t, color=C_YELLOW, alpha=0.4, linewidth=0.6, linestyle=':', zorder=2)

    def _save(self, fig, name):
        path = os.path.join(self.outdir, f'{name}.png')
        fig.savefig(path, dpi=160, bbox_inches='tight', facecolor=C_BG_DARK)
        plt.close(fig)
        return path

    def mode_timeline(self) :
        """비행 모드 타임라인 바"""
        if not self.mode_spans: return None
        fig, ax = plt.subplots(figsize=(12, 1.2))
        self._apply_style(ax)
        ax.set_yticks([])
        for t0, t1, name in self.mode_spans:
            color = MODE_COLORS.get(name, '#37474f')
            ax.barh(0, t1-t0, left=t0, height=0.8, color=color, edgecolor='none', alpha=0.85)
            if t1-t0 > 5:
                ax.text((t0+t1)/2, 0, name, ha='center', va='center',
                        fontsize=7, color='white', fontweight='bold')
        ax.set_xlabel('시간 (초)', fontsize=8)
        ax.set_title('비행 모드 타임라인', color=C_TEXT, fontsize=11, fontweight='bold')
        self._add_err_markers(ax)
        return self._save(fig, 'mode_timeline')

    def altitude(self) :
        gps = self.p.get('GPS')
        if not gps: return None
        ts = self.p.rel_ts(gps)
        alt = np.array([g.get('Alt',0) for g in gps])
        ralt = np.array([g.get('RAlt', g.get('RelAlt',0)) for g in gps])
        if not any(alt > 0): return None

        fig, ax = plt.subplots(figsize=(12, 3.5))
        self._apply_style(ax, '고도 (Altitude)', 'm')
        ax.fill_between(ts, alt, alpha=0.15, color=C_BLUE)
        ax.plot(ts, alt, color=C_BLUE, linewidth=1, label='GPS Alt')
        if any(ralt > 0):
            ax.plot(ts, ralt, color=C_CYAN, linewidth=0.8, alpha=0.7, label='Rel Alt')
        ax.legend(fontsize=7, loc='upper right', facecolor=C_BG_DARK, edgecolor=C_GRID,
                  labelcolor=C_TEXT)
        self._add_mode_bands(ax)
        self._add_err_markers(ax)
        ax.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'altitude')

    def gps_track(self) :
        gps = self.p.get('GPS')
        lats = [g.get('Lat',0) for g in gps if g.get('Lat',0)!=0]
        lons = [g.get('Lng',0) for g in gps if g.get('Lng',0)!=0]
        if len(lats) < 5: return None

        fig, ax = plt.subplots(figsize=(8, 7))
        self._apply_style(ax, 'GPS 비행 경로', '위도')
        sc = ax.scatter(lons, lats, c=range(len(lats)), cmap='plasma', s=3, alpha=0.8, zorder=3)
        ax.plot(lons[0], lats[0], 'o', color=C_GREEN, markersize=12, zorder=5, label='이륙')
        ax.plot(lons[-1], lats[-1], 's', color=C_RED, markersize=12, zorder=5, label='착륙')
        ax.set_xlabel('경도', fontsize=9)
        ax.legend(fontsize=8, facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        plt.colorbar(sc, ax=ax, label='시간순서', pad=0.02)
        return self._save(fig, 'gps_track')

    def vibration(self) :
        vibes = self.p.get('VIBE')
        if not vibes: return None
        ts = self.p.rel_ts(vibes)
        vx = [v.get('VibeX',0) for v in vibes]
        vy = [v.get('VibeY',0) for v in vibes]
        vz = [v.get('VibeZ',0) for v in vibes]

        fig, ax = plt.subplots(figsize=(12, 3.5))
        self._apply_style(ax, '진동 (Vibration)', 'm/s²')
        ax.plot(ts, vx, color=C_RED, linewidth=0.7, alpha=0.85, label='X')
        ax.plot(ts, vy, color=C_GREEN, linewidth=0.7, alpha=0.85, label='Y')
        ax.plot(ts, vz, color=C_BLUE, linewidth=0.7, alpha=0.85, label='Z')
        # 임계선
        ax.axhline(15, color=C_YELLOW, ls='--', lw=0.8, alpha=0.7, label='주의 (15)')
        ax.axhline(30, color=C_ORANGE, ls='--', lw=0.8, alpha=0.7, label='위험 (30)')
        ax.axhline(60, color=C_RED, ls='--', lw=0.8, alpha=0.7, label='비행불가 (60)')
        ax.legend(fontsize=7, ncol=3, loc='upper right', facecolor=C_BG_DARK,
                  edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax)
        self._add_err_markers(ax)
        ax.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'vibration')

    def battery(self) :
        batt = self.p.get('BAT') or self.p.get('BATT') or self.p.get('CURR')
        if not batt: return None
        ts = self.p.rel_ts(batt)
        volts = [b.get('Volt',b.get('VoltR',0)) for b in batt]
        currs = [b.get('Curr',b.get('CurrR',0)) for b in batt]

        fig, ax1 = plt.subplots(figsize=(12, 3.5))
        self._apply_style(ax1, '배터리 (Battery)', 'V')
        ax1.plot(ts, volts, color=C_BLUE, linewidth=1, label='전압')
        ax1.tick_params(axis='y', labelcolor=C_BLUE)

        ax2 = ax1.twinx()
        ax2.plot(ts, currs, color=C_ORANGE, linewidth=0.7, alpha=0.7, label='전류')
        ax2.set_ylabel('A', fontsize=9, color=C_ORANGE)
        ax2.tick_params(axis='y', labelcolor=C_ORANGE)
        ax2.spines['right'].set_color(C_GRID)
        ax2.spines['top'].set_visible(False)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1+lines2, labels1+labels2, fontsize=7, loc='upper right',
                   facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax1)
        self._add_err_markers(ax1)
        ax1.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'battery')

    def attitude(self) :
        att = self.p.get('ATT')
        if not att: return None
        ts = self.p.rel_ts(att)
        roll = [a.get('Roll',0) for a in att]
        pitch = [a.get('Pitch',0) for a in att]
        des_r = [a.get('DesRoll',0) for a in att]
        des_p = [a.get('DesPitch',0) for a in att]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 5), sharex=True)
        for ax in (ax1, ax2):
            self._apply_style(ax)
            self._add_mode_bands(ax)
            self._add_err_markers(ax)

        ax1.plot(ts, roll, color=C_BLUE, lw=0.8, label='Roll')
        ax1.plot(ts, des_r, color=C_CYAN, lw=0.6, alpha=0.5, label='DesRoll')
        ax1.set_ylabel('Roll (°)', fontsize=9)
        ax1.set_title('자세 추적 (Attitude)', color=C_TEXT, fontsize=11, fontweight='bold')
        ax1.legend(fontsize=7, facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)

        ax2.plot(ts, pitch, color=C_GREEN, lw=0.8, label='Pitch')
        ax2.plot(ts, des_p, color='#81c784', lw=0.6, alpha=0.5, label='DesPitch')
        ax2.set_ylabel('Pitch (°)', fontsize=9)
        ax2.set_xlabel('시간 (초)', fontsize=8)
        ax2.legend(fontsize=7, facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        fig.tight_layout(pad=1.0)
        return self._save(fig, 'attitude')

    def rcout(self) :
        rcou = self.p.get('RCOU')
        if not rcou: return None
        ts = self.p.rel_ts(rcou)

        fig, ax = plt.subplots(figsize=(12, 3.5))
        self._apply_style(ax, '모터 출력 (RCOUT)', 'PWM (μs)')
        colors = [C_RED, C_GREEN, C_BLUE, C_PURPLE]
        for i in range(1, 5):
            vals = [r.get(f'C{i}', 0) for r in rcou]
            ax.plot(ts, vals, color=colors[i-1], lw=0.7, alpha=0.85, label=f'M{i}')
        ax.legend(fontsize=7, ncol=4, loc='upper right', facecolor=C_BG_DARK,
                  edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax)
        self._add_err_markers(ax)
        ax.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'rcout')

    def ekf(self) :
        ekf = self.p.get('NKF4') or self.p.get('XKF4') or self.p.get('EKF4')
        if not ekf: return None
        ts = self.p.rel_ts(ekf)
        fields = {'SV': C_RED, 'SP': C_BLUE, 'SH': C_GREEN, 'SM': C_PURPLE}

        fig, ax = plt.subplots(figsize=(12, 3))
        self._apply_style(ax, 'EKF Innovation Test Ratios', 'ratio')
        for k, c in fields.items():
            vals = [e.get(k, 0) for e in ekf]
            if any(v > 0 for v in vals):
                ax.plot(ts, vals, color=c, lw=0.7, alpha=0.85, label=k)
        ax.axhline(0.8, color=C_ORANGE, ls='--', lw=1, alpha=0.7, label='FS Thresh (0.8)')
        ax.axhline(1.0, color=C_RED, ls='--', lw=0.8, alpha=0.5, label='Reject (1.0)')
        ax.legend(fontsize=7, ncol=3, loc='upper right', facecolor=C_BG_DARK,
                  edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax)
        ax.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'ekf')

    def wind(self):
        """바람 추정 — 풍속/풍향 시계열 + 극좌표"""
        wnd = self.p.get('NKF2') or self.p.get('XKF2')
        if not wnd: return None
        ts = self.p.rel_ts(wnd)
        vwn = np.array([w.get('VWN', 0) for w in wnd])
        vwe = np.array([w.get('VWE', 0) for w in wnd])
        if not any(abs(v) > 0.01 for v in vwn): return None
        spd = np.sqrt(vwn**2 + vwe**2)
        direction = np.degrees(np.arctan2(-vwe, -vwn)) % 360

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4),
                                        gridspec_kw={'width_ratios': [3, 1]})
        # 풍속 시계열
        self._apply_style(ax1, '바람 추정 (Wind Estimation)', 'm/s')
        ax1.plot(ts, spd, color=C_CYAN, lw=1, label='풍속')
        ax1.fill_between(ts, spd, alpha=0.15, color=C_CYAN)
        ax2_twin = ax1.twinx()
        ax2_twin.plot(ts, direction, color=C_ORANGE, lw=0.5, alpha=0.5, label='풍향(°)')
        ax2_twin.set_ylabel('풍향 (°)', fontsize=8, color=C_ORANGE)
        ax2_twin.tick_params(axis='y', labelcolor=C_ORANGE, labelsize=7)
        ax2_twin.spines['right'].set_color(C_GRID)
        ax2_twin.spines['top'].set_visible(False)
        ax1.legend(fontsize=7, loc='upper left', facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax1)
        ax1.set_xlabel('시간 (초)', fontsize=8)

        # 풍향 극좌표 (Wind Rose)
        ax2.remove()
        ax2 = fig.add_subplot(1, 2, 2, polar=True)
        ax2.set_facecolor(C_BG_DARK)
        theta = np.radians(direction)
        ax2.scatter(theta, spd, c=spd, cmap='YlOrRd', s=2, alpha=0.5)
        ax2.set_theta_zero_location('N')
        ax2.set_theta_direction(-1)
        ax2.tick_params(labelsize=7, colors=C_TEXT_DIM)
        ax2.set_title('Wind Rose', color=C_TEXT, fontsize=9, pad=10)
        fig.tight_layout(pad=1.5)
        return self._save(fig, 'wind')

    def pid_tracking(self):
        """PID 추적 품질 — Roll/Pitch 목표 vs 실제"""
        pidr = self.p.get('PIDR')
        pidp = self.p.get('PIDP')
        if (not pidr or len(pidr) < 50) and (not pidp or len(pidp) < 50):
            return None

        axes_data = []
        if pidr and len(pidr) >= 50: axes_data.append(('Roll', pidr))
        if pidp and len(pidp) >= 50: axes_data.append(('Pitch', pidp))
        if not axes_data: return None

        fig, axs = plt.subplots(len(axes_data), 1, figsize=(12, 3*len(axes_data)), sharex=True)
        if len(axes_data) == 1: axs = [axs]

        for ax, (label, data) in zip(axs, axes_data):
            self._apply_style(ax)
            step = max(1, len(data) // 3000)
            sampled = data[::step]
            ts = self.p.rel_ts(sampled)
            tar = [d.get('Tar', d.get('Des', 0)) for d in sampled]
            act = [d.get('Act', d.get('P', 0)) for d in sampled]
            ax.plot(ts, tar, color=C_CYAN, lw=0.6, alpha=0.6, label=f'{label} Target')
            ax.plot(ts, act, color=C_BLUE, lw=0.8, label=f'{label} Actual')
            ax.set_ylabel('deg/s', fontsize=8)
            ax.legend(fontsize=7, facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
            self._add_mode_bands(ax)
            self._add_err_markers(ax)

        axs[0].set_title('PID 추적 품질 (Target vs Actual)', color=C_TEXT, fontsize=11, fontweight='bold')
        axs[-1].set_xlabel('시간 (초)', fontsize=8)
        fig.tight_layout(pad=1.0)
        return self._save(fig, 'pid')

    def landing(self):
        """착륙 분석 차트 — 하강률 + 고도"""
        ctun = self.p.get('CTUN')
        gps = self.p.get('GPS')
        if not gps or len(gps) < 20:
            return None

        # 착륙 구간 식별 (마지막 60초)
        t0 = self.t0
        ts_gps = self.p.rel_ts(gps)
        alts = np.array([g.get('Alt', 0) for g in gps])
        if not any(alts > 2):
            return None
        # 마지막 비행 구간
        last_high = np.where(alts > 2)[0]
        if len(last_high) == 0:
            return None
        start_idx = max(0, last_high[-1] - 100)  # 착륙 전 여유

        if ctun and len(ctun) > 20:
            ts_c = self.p.rel_ts(ctun)
            crt = np.array([c.get('CRt', 0) for c in ctun])
            alt_c = np.array([c.get('Alt', c.get('BAlt', 0)) for c in ctun])
            # 동일 시간 범위 필터
            t_start = ts_gps[start_idx]
            mask = ts_c >= t_start
            ts_plot = ts_c[mask]
            alt_plot = alt_c[mask]
            crt_plot = crt[mask]
        else:
            ts_plot = ts_gps[start_idx:]
            alt_plot = alts[start_idx:]
            dt = np.diff(ts_plot)
            dt[dt == 0] = 0.1
            crt_plot = np.concatenate([[0], np.diff(alt_plot) / dt])

        if len(ts_plot) < 5:
            return None

        fig, ax1 = plt.subplots(figsize=(12, 4))
        self._apply_style(ax1, '착륙 분석 (Landing Analysis)', 'm')
        ax1.plot(ts_plot, alt_plot, color=C_BLUE, lw=1.2, label='고도')
        ax1.fill_between(ts_plot, alt_plot, alpha=0.1, color=C_BLUE)
        ax1.set_ylabel('고도 (m)', fontsize=9, color=C_BLUE)
        ax1.tick_params(axis='y', labelcolor=C_BLUE)

        ax2 = ax1.twinx()
        ax2.plot(ts_plot, crt_plot, color=C_ORANGE, lw=0.8, alpha=0.8, label='하강률')
        ax2.axhline(0, color=C_TEXT_DIM, lw=0.5, alpha=0.3)
        ax2.axhline(-1.5, color=C_YELLOW, ls='--', lw=0.7, alpha=0.6, label='주의 (-1.5m/s)')
        ax2.axhline(-3.0, color=C_RED, ls='--', lw=0.7, alpha=0.6, label='위험 (-3.0m/s)')
        ax2.set_ylabel('수직 속도 (m/s)', fontsize=9, color=C_ORANGE)
        ax2.tick_params(axis='y', labelcolor=C_ORANGE)
        ax2.spines['right'].set_color(C_GRID)
        ax2.spines['top'].set_visible(False)

        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1+lines2, labels1+labels2, fontsize=7, loc='upper right',
                   facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        self._add_mode_bands(ax1)
        ax1.set_xlabel('시간 (초)', fontsize=8)
        return self._save(fig, 'landing')

    def esc(self):
        """ESC 텔레메트리 차트 — 온도 + RPM"""
        esc_data = self.p.get('ESC')
        if not esc_data or len(esc_data) < 10:
            return None

        # 인스턴스별 분류
        by_inst = {}
        for e in esc_data:
            inst = e.get('Instance', e.get('Inst', 0))
            if inst not in by_inst:
                by_inst[inst] = []
            by_inst[inst].append(e)

        has_temp = any(e.get('Temp', 0) > 0 for e in esc_data)
        has_rpm = any(e.get('RPM', 0) > 0 for e in esc_data)
        if not has_temp and not has_rpm:
            return None

        nplots = sum([has_temp, has_rpm])
        fig, axs = plt.subplots(nplots, 1, figsize=(12, 3.5*nplots), sharex=True)
        if nplots == 1:
            axs = [axs]
        ax_idx = 0
        colors = [C_RED, C_GREEN, C_BLUE, C_PURPLE, C_ORANGE, C_CYAN, C_YELLOW, '#ff69b4']

        if has_rpm:
            ax = axs[ax_idx]
            self._apply_style(ax, 'ESC RPM', 'RPM')
            for inst, data in sorted(by_inst.items()):
                rpms = [e.get('RPM', 0) for e in data]
                if not any(r > 0 for r in rpms):
                    continue
                ts = self.p.rel_ts(data)
                c = colors[inst % len(colors)]
                ax.plot(ts, rpms, color=c, lw=0.7, alpha=0.85, label=f'M{inst+1}')
            ax.legend(fontsize=7, ncol=4, loc='upper right',
                      facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
            self._add_mode_bands(ax)
            self._add_err_markers(ax)
            ax_idx += 1

        if has_temp:
            ax = axs[ax_idx]
            self._apply_style(ax, 'ESC 온도', '°C')
            for inst, data in sorted(by_inst.items()):
                temps = [e.get('Temp', 0) for e in data]
                if not any(t > 0 for t in temps):
                    continue
                ts = self.p.rel_ts(data)
                c = colors[inst % len(colors)]
                ax.plot(ts, temps, color=c, lw=0.8, alpha=0.85, label=f'ESC{inst+1}')
            ax.axhline(80, color=C_YELLOW, ls='--', lw=0.7, alpha=0.6, label='주의 (80°C)')
            ax.axhline(100, color=C_RED, ls='--', lw=0.7, alpha=0.6, label='위험 (100°C)')
            ax.legend(fontsize=7, ncol=4, loc='upper right',
                      facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
            self._add_mode_bands(ax)
            ax_idx += 1

        axs[-1].set_xlabel('시간 (초)', fontsize=8)
        fig.tight_layout(pad=1.0)
        return self._save(fig, 'esc')

    def hover_scatter(self):
        """호버 안정성 산점도 — 위치 분산 시각화"""
        if not hasattr(self.a, 'hover_data') or not self.a.hover_data:
            return None

        gps = self.p.get('GPS')
        modes = self.a.summary['modes']
        if not gps or not modes:
            return None

        hover_modes = {'LOITER', 'POSHOLD', 'ALT_HOLD'}
        hover_spans = []
        for i, m in enumerate(modes):
            if m['name'] in hover_modes:
                t_start = m['t']
                t_end = modes[i+1]['t'] if i+1 < len(modes) else gps[-1]['_ts']
                if t_end - t_start > 3:
                    hover_spans.append((t_start, t_end, m['name']))

        if not hover_spans:
            return None

        fig, ax = plt.subplots(figsize=(8, 7))
        self._apply_style(ax, '호버 위치 안정성 (Hover Stability)', 'Y 편차 (m)')
        mode_colors = {'LOITER': C_GREEN, 'POSHOLD': C_CYAN, 'ALT_HOLD': C_BLUE}

        for t_start, t_end, mode_name in hover_spans:
            pts = [(g.get('Lat', 0), g.get('Lng', 0))
                   for g in gps if t_start <= g['_ts'] <= t_end
                   and g.get('Lat', 0) != 0]
            if len(pts) < 10:
                continue
            lats_h = np.array([p[0] for p in pts])
            lons_h = np.array([p[1] for p in pts])
            clat, clon = np.mean(lats_h), np.mean(lons_h)
            dy = (lats_h - clat) * 111320
            dx = (lons_h - clon) * 111320 * np.cos(np.radians(clat))
            c = mode_colors.get(mode_name, C_BLUE)
            ax.scatter(dx, dy, c=c, s=3, alpha=0.5, label=mode_name)

        # 동심원 (1m, 2m, 5m)
        for r in [1, 2, 5]:
            circle = plt.Circle((0, 0), r, fill=False, color=C_TEXT_DIM,
                                ls='--', lw=0.6, alpha=0.5)
            ax.add_patch(circle)
            ax.text(r*0.707, r*0.707, f'{r}m', fontsize=7, color=C_TEXT_DIM, alpha=0.7)
        ax.set_aspect('equal')
        ax.set_xlabel('X 편차 (m)', fontsize=9)
        ax.plot(0, 0, '+', color=C_RED, markersize=15, markeredgewidth=2, zorder=10)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), fontsize=8,
                  facecolor=C_BG_DARK, edgecolor=C_GRID, labelcolor=C_TEXT)
        return self._save(fig, 'hover')

    def flight_phases(self):
        """비행 구간 분류 타임라인"""
        if not hasattr(self.a, 'flight_phases') or not self.a.flight_phases:
            return None

        phase_colors = {
            'GROUND': '#616161', 'TAKEOFF': '#66bb6a', 'CLIMB': '#29b6f6',
            'HOVER': '#26c6da', 'CRUISE': '#ab47bc', 'DESCENT': '#ffa726',
            'LANDING': '#ff7043',
        }

        fig, ax = plt.subplots(figsize=(12, 1.5))
        self._apply_style(ax)
        ax.set_yticks([])

        for ph in self.a.flight_phases:
            t0 = ph['t_start']
            t1 = ph['t_end']
            color = phase_colors.get(ph['phase'], '#37474f')
            ax.barh(0, t1-t0, left=t0, height=0.8, color=color, edgecolor='none', alpha=0.85)
            if t1 - t0 > 5:
                phase_kr = {'GROUND': '지상', 'TAKEOFF': '이륙', 'CLIMB': '상승',
                            'HOVER': '호버', 'CRUISE': '이동', 'DESCENT': '하강', 'LANDING': '착륙'}
                label = phase_kr.get(ph['phase'], ph['phase'])
                ax.text((t0+t1)/2, 0, label, ha='center', va='center',
                        fontsize=7, color='white', fontweight='bold')

        ax.set_xlabel('시간 (초)', fontsize=8)
        ax.set_title('비행 구간 자동 분류', color=C_TEXT, fontsize=11, fontweight='bold')
        self._add_err_markers(ax)
        return self._save(fig, 'phases')

    def generate_all(self) -> dict:
        charts = {}
        for name, func in [('mode', self.mode_timeline), ('alt', self.altitude),
                            ('gps', self.gps_track), ('vibe', self.vibration),
                            ('batt', self.battery), ('att', self.attitude),
                            ('rcout', self.rcout), ('ekf', self.ekf),
                            ('wind', self.wind), ('pid', self.pid_tracking),
                            ('landing', self.landing), ('esc', self.esc),
                            ('hover', self.hover_scatter), ('phases', self.flight_phases)]:
            try:
                path = func()
                if path: charts[name] = path
            except Exception as e:
                print(f'  [WARN] {name}: {e}')
        return charts


# ═══════════════════════════════════════════
# 4. PDF 보고서
# ═══════════════════════════════════════════
class ReportBuilder:
    CHART_TITLES = {
        'mode': '비행 모드 타임라인',
        'alt': '고도 프로파일',
        'gps': 'GPS 비행 경로',
        'vibe': '진동 분석',
        'batt': '배터리 전압/전류',
        'att': '자세 추적 (Roll/Pitch)',
        'rcout': '모터 출력 (RCOUT 1~4)',
        'ekf': 'EKF Innovation Ratios',
        'wind': '바람 추정 (Wind Estimation)',
        'pid': 'PID 추적 품질',
        'landing': '착륙 분석',
        'esc': 'ESC 텔레메트리 (RPM/온도)',
        'hover': '호버 위치 안정성',
        'phases': '비행 구간 자동 분류',
    }

    def __init__(self, analyzer: Analyzer, charts: dict, out_path: str):
        self.a = analyzer
        self.charts = charts
        self.out = out_path
        self.styles = getSampleStyleSheet()
        self._mk_styles()

    def _mk_styles(self):
        s = self.styles
        s.add(ParagraphStyle('Title2', fontName=FONT_NAME, fontSize=20,
              textColor=HexColor(C_BLUE), alignment=TA_CENTER, spaceAfter=4*mm, leading=26))
        s.add(ParagraphStyle('Sect', fontName=FONT_NAME, fontSize=13,
              textColor=HexColor(C_BLUE), spaceBefore=6*mm, spaceAfter=3*mm, leading=17))
        s.add(ParagraphStyle('Body', fontName=FONT_NAME, fontSize=9,
              textColor=HexColor('#333333'), leading=14, spaceAfter=1.5*mm))
        s.add(ParagraphStyle('Small', fontName=FONT_NAME, fontSize=7.5,
              textColor=HexColor('#666666'), leading=11, spaceAfter=1*mm))

    def _hdr_ftr(self, canvas, doc):
        canvas.saveState()
        canvas.setStrokeColor(HexColor(C_ACCENT))
        canvas.setLineWidth(1.5)
        canvas.line(18*mm, A4[1]-14*mm, A4[0]-18*mm, A4[1]-14*mm)
        canvas.setFont(FONT_NAME, 7.5)
        canvas.setFillColor(HexColor(C_TEXT_DIM))
        canvas.drawString(18*mm, A4[1]-12*mm, 'ArduCopter Flight Log Analysis')
        canvas.drawRightString(A4[0]-18*mm, A4[1]-12*mm, datetime.now().strftime('%Y-%m-%d'))
        canvas.setFont(FONT_NAME, 7.5)
        canvas.drawCentredString(A4[0]/2, 9*mm, 'Made by Kim.')
        canvas.drawRightString(A4[0]-18*mm, 9*mm, f'p.{doc.page}')
        canvas.restoreState()

    def build(self):
        doc = SimpleDocTemplate(self.out, pagesize=A4,
                                leftMargin=18*mm, rightMargin=18*mm,
                                topMargin=20*mm, bottomMargin=16*mm)
        story = []
        s = self.a.summary

        # ══ 페이지 1: 대시보드 ══
        story.append(Spacer(1, 6*mm))
        story.append(Paragraph('ArduCopter 비행 로그 분석', self.styles['Title2']))

        # 종합 판정
        fails = sum(1 for f in self.a.findings if f.sev=='FAIL')
        warns = sum(1 for f in self.a.findings if f.sev=='WARN')
        oks = sum(1 for f in self.a.findings if f.sev=='OK')
        if fails: ov_color, ov_text = C_RED, f'FAIL {fails} / WARN {warns} / OK {oks}'
        elif warns: ov_color, ov_text = C_YELLOW, f'WARN {warns} / OK {oks}'
        else: ov_color, ov_text = C_GREEN, f'ALL OK ({oks})'

        badge = Table([[Paragraph(f'<font color="white" size="13"><b>{ov_text}</b></font>',
                                  self.styles['Body'])]],
                      colWidths=[174*mm])
        badge.setStyle(TableStyle([
            ('BACKGROUND',(0,0),(-1,-1), HexColor(ov_color)),
            ('ALIGN',(0,0),(-1,-1),'CENTER'),
            ('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),
            ('ROUNDEDCORNERS', [4,4,4,4]),
        ]))
        story.append(badge)
        story.append(Spacer(1, 5*mm))

        # KPI 카드
        kpi = [
            ['비행시간', s['dur_str'], '최대 고도', f'{s["max_alt"]:.1f}m'],
            ['이동거리', f'{s["dist_m"]:.0f}m', '최대 속도', f'{s["max_spd"]:.1f}m/s'],
            ['배터리 최저', f'{s["v_min"]:.2f}V', '최대 전류', f'{s["i_max"]:.1f}A'],
            ['GPS 위성', f'{s["min_sats"]}~{s["max_sats"]}', 'HDop', f'{s["max_hdop"]:.1f}'],
            ['파일', s['filename'], '크기', f'{s["filesize_mb"]:.1f}MB'],
        ]
        kpi_t = Table(kpi, colWidths=[30*mm, 57*mm, 30*mm, 57*mm])
        kpi_t.setStyle(TableStyle([
            ('FONTNAME',(0,0),(-1,-1), FONT_NAME),
            ('FONTSIZE',(0,0),(-1,-1), 9),
            ('TEXTCOLOR',(0,0),(0,-1), HexColor(C_ACCENT)),
            ('TEXTCOLOR',(2,0),(2,-1), HexColor(C_ACCENT)),
            ('FONTSIZE',(0,0),(0,-1), 8),
            ('FONTSIZE',(2,0),(2,-1), 8),
            ('GRID',(0,0),(-1,-1), 0.5, lightgrey),
            ('ROWBACKGROUNDS',(0,0),(-1,-1), [white, HexColor('#f8f9fa')]),
            ('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
        ]))
        story.append(kpi_t)
        story.append(Spacer(1, 4*mm))

        # 비행 모드 요약
        if s['modes']:
            t0m = s['modes'][0]['t']
            mode_str = ' → '.join(f'{m["name"]}(+{int(m["t"]-t0m)}s)' for m in s['modes'][:12])
            if len(s['modes']) > 12: mode_str += f' ... (+{len(s["modes"])-12})'
            story.append(Paragraph(f'<b>모드:</b> {mode_str}', self.styles['Small']))

        # 모드 타임라인 차트
        if 'mode' in self.charts:
            story.append(Image(self.charts['mode'], width=174*mm, height=22*mm))

        # ══ 페이지 2~: 차트 ══
        story.append(PageBreak())
        story.append(Paragraph('상세 그래프', self.styles['Sect']))

        chart_order = ['alt','vibe','batt','att','rcout','ekf','gps',
                       'landing','esc','hover','phases']
        for key in chart_order:
            if key not in self.charts: continue
            title = self.CHART_TITLES.get(key, key)
            story.append(Paragraph(title, self.styles['Sect']))
            h = 130*mm if key == 'gps' else (95*mm if key == 'att' else 65*mm)
            story.append(Image(self.charts[key], width=174*mm, height=h))
            story.append(Spacer(1, 2*mm))

        # ══ 비행 복기 (Flight Story) ══
        story.append(PageBreak())
        story.append(Paragraph('비행 복기', self.styles['Sect']))
        for line in self.a.flight_story.split('\n'):
            if line.startswith('■'):
                story.append(Spacer(1, 3*mm))
                story.append(Paragraph(f'<b>{line}</b>', self.styles['Body']))
            elif line.strip():
                story.append(Paragraph(line.replace('<', '&lt;').replace('>', '&gt;'),
                                       self.styles['Small']))

        # ══ 이벤트 타임라인 ══
        if self.a.timeline:
            story.append(PageBreak())
            story.append(Paragraph('이벤트 타임라인', self.styles['Sect']))
            tl_header = [
                Paragraph('<b>시간</b>', self.styles['Small']),
                Paragraph('<b>유형</b>', self.styles['Small']),
                Paragraph('<b>내용</b>', self.styles['Small']),
            ]
            tl_rows = [tl_header]
            sev_bg = {'FAIL': HexColor('#ffebee'), 'WARN': HexColor('#fff8e1'), 'INFO': white}
            for ev in self.a.timeline[:40]:  # 최대 40건
                mins = int(ev['ts'] // 60)
                secs = int(ev['ts'] % 60)
                time_str = f'{mins}:{secs:02d}'
                detail_text = ev['title']
                if ev.get('detail'):
                    detail_text += f'<br/><font size="6" color="#888">{ev["detail"][:120]}</font>'
                tl_rows.append([
                    Paragraph(f'<font size="8">{time_str}</font>', self.styles['Small']),
                    Paragraph(f'<font size="7">{ev["type"]}</font>', self.styles['Small']),
                    Paragraph(f'<font size="7">{detail_text}</font>', self.styles['Small']),
                ])

            tl_table = Table(tl_rows, colWidths=[18*mm, 18*mm, 138*mm])
            tl_style = [
                ('FONTNAME', (0,0), (-1,-1), FONT_NAME),
                ('GRID', (0,0), (-1,-1), 0.3, lightgrey),
                ('BACKGROUND', (0,0), (-1,0), HexColor(C_ACCENT)),
                ('TEXTCOLOR', (0,0), (-1,0), white),
                ('TOPPADDING', (0,0), (-1,-1), 2),
                ('BOTTOMPADDING', (0,0), (-1,-1), 2),
                ('VALIGN', (0,0), (-1,-1), 'TOP'),
            ]
            # 행별 배경색
            for i, ev in enumerate(self.a.timeline[:40], 1):
                bg = sev_bg.get(ev['sev'], white)
                tl_style.append(('BACKGROUND', (0,i), (-1,i), bg))
            tl_table.setStyle(TableStyle(tl_style))
            story.append(tl_table)

        # ══ 근본 원인 분석 ══
        if self.a.root_cause['causes']:
            story.append(PageBreak())
            story.append(Paragraph('근본 원인 분석', self.styles['Sect']))
            story.append(Paragraph(
                '에러 발생 시점의 센서 데이터를 교차 분석하여 추론한 결과입니다.',
                self.styles['Small']))
            story.append(Spacer(1, 3*mm))

            for cause in self.a.root_cause['causes']:
                conf_colors = {'HIGH': C_RED, 'MEDIUM': C_YELLOW, 'LOW': C_GREEN}
                conf_kr = {'HIGH': '높음', 'MEDIUM': '보통', 'LOW': '낮음'}
                cc = conf_colors.get(cause['confidence'], C_YELLOW)

                # 원인 카드
                cause_rows = [[
                    Paragraph(f'<font color="white" size="8"><b>{cause["category"]}</b></font>',
                              self.styles['Body']),
                    Paragraph(f'<b>{cause["title"]}</b>', self.styles['Body']),
                ]]
                cause_rows.append(['',
                    Paragraph(cause['detail'].replace('\n', '<br/>'), self.styles['Small'])])
                if cause.get('action'):
                    action_text = '<br/>'.join(f'{i+1}. {a}' for i, a in enumerate(cause['action']))
                    cause_rows.append(['',
                        Paragraph(f'<font color="{C_ACCENT}"><b>조치 순서:</b></font><br/>{action_text}',
                                  self.styles['Small'])])
                cause_rows.append(['',
                    Paragraph(f'확신도: <b>{conf_kr.get(cause["confidence"], "?")}</b>',
                              self.styles['Small'])])

                ct = Table(cause_rows, colWidths=[26*mm, 148*mm])
                ct.setStyle(TableStyle([
                    ('BACKGROUND', (0,0), (0,0), HexColor(cc)),
                    ('VALIGN', (0,0), (0,-1), 'TOP'),
                    ('TOPPADDING', (0,0), (-1,-1), 3),
                    ('BOTTOMPADDING', (0,0), (-1,-1), 3),
                    ('BOX', (0,0), (-1,-1), 0.8, HexColor(cc)),
                ]))
                story.append(KeepTogether([ct, Spacer(1, 3*mm)]))

            # 교차 분석 근거
            if self.a.root_cause['correlations']:
                story.append(Spacer(1, 3*mm))
                story.append(Paragraph('<b>교차 분석 근거</b>', self.styles['Body']))
                seen_findings = set()
                for c in self.a.root_cause['correlations'][:12]:
                    key = c['finding'][:60]
                    if key in seen_findings:
                        continue
                    seen_findings.add(key)
                    mins = int(c['ts'] // 60)
                    secs = int(c['ts'] % 60)
                    story.append(Paragraph(
                        f'• [{mins}:{secs:02d}] {c["finding"]}',
                        self.styles['Small']))

        # ══ 분석 결과 ══
        story.append(PageBreak())
        story.append(Paragraph('분석 결과 — 발견사항', self.styles['Sect']))

        # FAIL 먼저, WARN, OK 순
        sorted_f = sorted(self.a.findings, key=lambda f: {'FAIL':0,'WARN':1,'OK':2}[f.sev])
        for f in sorted_f:
            color = SEV_COLORS[f.sev]
            rows = [[
                Paragraph(f'<font color="white"><b>[{f.sev}]</b></font>', self.styles['Body']),
                Paragraph(f'<b>{f.title}</b>', self.styles['Body']),
            ]]
            if f.detail:
                rows.append(['', Paragraph(f.detail.replace('\n','<br/>'), self.styles['Small'])])
            if f.fix:
                rows.append(['', Paragraph(
                    f'<font color="{C_ACCENT}"><b>조치:</b></font> {f.fix.replace(chr(10),"<br/>")}',
                    self.styles['Small'])])
            ft = Table(rows, colWidths=[14*mm, 160*mm])
            ft.setStyle(TableStyle([
                ('BACKGROUND',(0,0),(0,0), HexColor(color)),
                ('VALIGN',(0,0),(0,-1),'TOP'),
                ('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
                ('BOX',(0,0),(-1,-1),0.5, HexColor(color)),
            ]))
            story.append(KeepTogether([ft, Spacer(1,2*mm)]))

        # ══ 범례 ══
        story.append(Spacer(1, 6*mm))
        story.append(Paragraph('범례', self.styles['Sect']))
        legend_items = [
            '그래프 배경 색상 = 비행 모드 구간 (모드별 색상)',
            '빨간 점선 = ERR 에러 발생 시점',
            '노란 점선 = 주의 이벤트 (GPS Lost, EKF Reset 등)',
            '진동 임계: <15 양호 / 15~30 주의 / 30~60 위험 / >60 비행불가',
            'EKF: FS_EKF_THRESH(0.8) 2개 이상 초과 시 페일세이프',
        ]
        for item in legend_items:
            story.append(Paragraph(f'• {item}', self.styles['Small']))

        doc.build(story, onFirstPage=self._hdr_ftr, onLaterPages=self._hdr_ftr)
        return self.out


# ═══════════════════════════════════════════
# 5. 메인
# ═══════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description='ArduCopter .bin → PDF')
    ap.add_argument('logfile', help='.bin 파일')
    ap.add_argument('-o', '--output', help='출력 PDF 경로')
    args = ap.parse_args()

    if not os.path.exists(args.logfile):
        print(f'[ERROR] {args.logfile} 없음'); sys.exit(1)

    stem = Path(args.logfile).stem
    outdir = os.path.join(os.path.dirname(__file__), 'output')
    chartdir = os.path.join(outdir, f'{stem}_charts')
    pdf = args.output or os.path.join(outdir, f'{stem}_report.pdf')
    os.makedirs(outdir, exist_ok=True)

    print(f'[1/4] 파싱: {args.logfile}')
    parser = LogParser(args.logfile)
    print('[2/4] 분석...')
    analyzer = Analyzer(parser)
    print('[3/4] 차트 생성...')
    cg = ChartGen(parser, analyzer, chartdir)
    charts = cg.generate_all()
    print(f'  → {len(charts)}개 차트')
    print(f'[4/4] PDF: {pdf}')
    ReportBuilder(analyzer, charts, pdf).build()

    fails = sum(1 for f in analyzer.findings if f.sev=='FAIL')
    warns = sum(1 for f in analyzer.findings if f.sev=='WARN')
    oks = sum(1 for f in analyzer.findings if f.sev=='OK')
    print(f'\n{"="*50}')
    print(f'  FAIL: {fails}  WARN: {warns}  OK: {oks}')
    print(f'  {pdf}')
    print(f'{"="*50}')


if __name__ == '__main__':
    main()
