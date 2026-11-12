"""infra.results_db: SQLite results index + CSV registry. The DB is an index; bundles archive."""

import csv
import os
import sqlite3
from pathlib import Path

SCHEMA_PATH = Path(__file__).resolve().parents[2] / "results" / "schema.sql"
DEFAULT_REGISTRY_FIELDS = ["run_id", "status", "artifact_uri", "sha256"]


def connect(db_path):
    con = sqlite3.connect(db_path)
    con.execute("PRAGMA foreign_keys = ON")
    con.executescript(SCHEMA_PATH.read_text())
    return con


def insert_run(con, run):
    cols = ", ".join(run.keys())
    marks = ", ".join("?" for _ in run)
    con.execute(f"INSERT INTO runs ({cols}) VALUES ({marks})", list(run.values()))


def update_run(con, run_id, **fields):
    sets = ", ".join(f"{k} = ?" for k in fields)
    con.execute(f"UPDATE runs SET {sets} WHERE run_id = ?", [*fields.values(), run_id])


def insert_metrics(con, rows):
    con.executemany(
        "INSERT INTO metrics (run_id, sequence_id, split, outage_id, metric, statistic, value,"
        " unit, reference_dof, n_samples, status, failure_reason)"
        " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
        rows,
    )


def insert_artifacts(con, run_id, files):
    con.executemany(
        "INSERT INTO artifacts (run_id, path, sha256) VALUES (?, ?, ?)",
        [(run_id, p, h) for p, h in files.items()],
    )


def append_registry(registry_path, row):
    """Append one row, keeping whatever header the registry file already has."""
    fields = DEFAULT_REGISTRY_FIELDS
    has_header = os.path.exists(registry_path) and os.path.getsize(registry_path) > 0
    if has_header:
        with open(registry_path, newline="") as f:
            fields = next(csv.reader(f))
    with open(registry_path, "a", newline="") as f:
        w = csv.DictWriter(
            f, fieldnames=fields, extrasaction="ignore", restval="", lineterminator="\n"
        )
        if not has_header:
            w.writeheader()
        w.writerow(row)
