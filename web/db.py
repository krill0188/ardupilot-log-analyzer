"""
로그 분석 DB — 사용자별 로그 추적
SQLite 기반, 파일 하나로 운영
"""
import sqlite3
import os
from pathlib import Path
from datetime import datetime

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
        filename    TEXT NOT NULL,
        filesize_mb REAL,
        uploaded_at TEXT NOT NULL DEFAULT (datetime('now','localtime')),

        -- 비행 요약
        duration    TEXT,
        max_alt     REAL,
        max_spd     REAL,
        dist_m      REAL,
        v_min       REAL,
        v_max       REAL,

        -- 판정
        fail_count  INTEGER DEFAULT 0,
        warn_count  INTEGER DEFAULT 0,
        ok_count    INTEGER DEFAULT 0,
        overall     TEXT,          -- 'FAIL' / 'WARN' / 'OK'
        score       INTEGER,       -- 종합 점수

        -- 핵심 진단
        root_cause  TEXT,          -- 근본 원인 요약
        story       TEXT           -- 비행 복기 요약 (첫 200자)
    );

    CREATE INDEX IF NOT EXISTS idx_user_id ON logs(user_id);
    CREATE INDEX IF NOT EXISTS idx_uploaded ON logs(uploaded_at DESC);
    """)
    conn.commit()
    conn.close()


def save_log(job_id: str, user_id: str, filename: str, filesize_mb: float,
             summary: dict, counts: dict, scores: dict,
             root_cause_text: str = '', story_text: str = ''):
    """분석 결과를 DB에 저장"""
    conn = get_db()

    if counts['fail'] > 0:
        overall = 'FAIL'
    elif counts['warn'] > 0:
        overall = 'WARN'
    else:
        overall = 'OK'

    conn.execute("""
    INSERT OR REPLACE INTO logs
    (job_id, user_id, filename, filesize_mb,
     duration, max_alt, max_spd, dist_m, v_min, v_max,
     fail_count, warn_count, ok_count, overall, score,
     root_cause, story)
    VALUES (?,?,?,?, ?,?,?,?,?,?, ?,?,?,?,?, ?,?)
    """, (
        job_id, user_id.strip().lower(), filename, round(filesize_mb, 1),
        summary.get('duration', ''),
        summary.get('max_alt', 0),
        summary.get('max_spd', 0),
        summary.get('dist_m', 0),
        summary.get('v_min', 0),
        summary.get('v_max', 0),
        counts.get('fail', 0),
        counts.get('warn', 0),
        counts.get('ok', 0),
        overall,
        scores.get('overall', 0),
        root_cause_text[:500],
        story_text[:500],
    ))
    conn.commit()
    conn.close()


def get_user_logs(user_id: str) -> list:
    """사용자 ID로 로그 이력 조회"""
    conn = get_db()
    rows = conn.execute("""
    SELECT job_id, filename, filesize_mb, uploaded_at,
           duration, max_alt, dist_m, v_min,
           fail_count, warn_count, ok_count, overall, score,
           root_cause, story
    FROM logs
    WHERE user_id = ?
    ORDER BY uploaded_at DESC
    LIMIT 50
    """, (user_id.strip().lower(),)).fetchall()
    conn.close()
    return [dict(r) for r in rows]


def get_log_by_job(job_id: str) -> dict:
    """job_id로 단건 조회"""
    conn = get_db()
    row = conn.execute("SELECT * FROM logs WHERE job_id = ?", (job_id,)).fetchone()
    conn.close()
    return dict(row) if row else None


def get_all_users() -> list:
    """등록된 사용자 목록"""
    conn = get_db()
    rows = conn.execute("""
    SELECT user_id, COUNT(*) as log_count,
           MAX(uploaded_at) as last_upload
    FROM logs
    GROUP BY user_id
    ORDER BY last_upload DESC
    """).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# 앱 시작 시 DB 초기화
init_db()
