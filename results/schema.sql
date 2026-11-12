-- ens491-nkf results database schema (W01 v0).
-- One row per run in runs; metrics are long-form; failed runs are stored, never deleted.

CREATE TABLE IF NOT EXISTS runs (
    run_id            TEXT PRIMARY KEY,
    parent_run_id     TEXT,
    status            TEXT NOT NULL CHECK (status IN ('running', 'completed', 'failed')),
    start_utc         TEXT NOT NULL,
    end_utc           TEXT,
    owner             TEXT,
    issue             TEXT,
    purpose           TEXT,
    protocol_version  TEXT,
    platform          TEXT NOT NULL CHECK (platform IN ('ugv', 'uav')),
    method            TEXT NOT NULL,
    seed              INTEGER NOT NULL,
    split_id          TEXT,
    config_hash       TEXT NOT NULL,
    code_sha          TEXT,
    git_dirty         INTEGER,
    source_hash       TEXT,
    bundle_uri        TEXT,
    failure_reason    TEXT
);

CREATE TABLE IF NOT EXISTS metrics (
    run_id          TEXT NOT NULL REFERENCES runs(run_id),
    sequence_id     TEXT NOT NULL,
    split           TEXT NOT NULL,
    outage_id       TEXT NOT NULL,
    metric          TEXT NOT NULL,
    statistic       TEXT NOT NULL,
    value           REAL,
    unit            TEXT NOT NULL,
    reference_dof   TEXT,
    n_samples       INTEGER,
    status          TEXT NOT NULL CHECK (status IN ('ok', 'failed', 'na')),
    failure_reason  TEXT,
    PRIMARY KEY (run_id, sequence_id, split, outage_id, metric, statistic)
);

CREATE TABLE IF NOT EXISTS artifacts (
    run_id   TEXT NOT NULL REFERENCES runs(run_id),
    path     TEXT NOT NULL,
    sha256   TEXT NOT NULL,
    PRIMARY KEY (run_id, path)
);
