"""Consistency of data/manifests/*.csv with the code and with docs/data_manifest.md.

The manifests hold metadata only (no raw data). These tests keep the field table in step with
the KITTI loader and the allowlists of the inertial input.
"""

import csv
import re
from pathlib import Path

import pytest

from nkf.data.kitti import OXTS_FIELDS
from nkf.data.sequence import KITTI_ADMITTED, KITTI_REFERENCE_ONLY, KITTI_REJECTED

pytestmark = pytest.mark.fast

ROOT = Path(__file__).resolve().parents[2] / "data" / "manifests"
KITTI = "kitti_raw_oxts_extract"
ROLES = {"inertial_input", "rejected", "reference_only", "not_used"}


def _rows(name):
    with open(ROOT / name, newline="") as f:
        return list(csv.DictReader(f))


@pytest.mark.parametrize(
    "name", ["sources.csv", "fields.csv", "timebase.csv", "reference_quality.csv"]
)
def test_no_empty_required_cell_and_no_ragged_rows(name):
    with open(ROOT / name, newline="") as f:
        raw = list(csv.reader(f))
    assert len(raw) > 1
    assert all(len(r) == len(raw[0]) for r in raw)
    header = raw[0]
    optional = {"note"}
    for row in raw[1:]:
        for col, cell in zip(header, row, strict=True):
            assert cell != "" or col in optional, f"{name}: empty cell in column {col}"


def test_sources_have_valid_sha256_and_positive_size():
    for row in _rows("sources.csv"):
        assert re.fullmatch(r"[0-9a-f]{64}", row["sha256"]), row["file"]
        assert int(row["bytes"]) > 0


def test_kitti_field_table_matches_the_loader_order():
    fields = [r for r in _rows("fields.csv") if r["dataset"] == KITTI]
    assert [r["field"] for r in fields] == list(OXTS_FIELDS)
    assert [int(r["column_index"]) for r in fields] == list(range(1, len(OXTS_FIELDS) + 1))


def test_kitti_roles_match_the_allowlists():
    by_role = {}
    for r in _rows("fields.csv"):
        if r["dataset"] == KITTI:
            assert r["role"] in ROLES
            by_role.setdefault(r["role"], []).append(r["field"])
    admitted = [n for names in KITTI_ADMITTED.values() for n in names]
    assert sorted(by_role["inertial_input"]) == sorted(admitted)
    assert sorted(by_role["rejected"]) == sorted(KITTI_REJECTED)
    assert sorted(by_role["reference_only"]) == sorted(KITTI_REFERENCE_ONLY)


def test_inertial_fields_carry_physical_units():
    units = {"rad/s", "m/s^2"}
    for r in _rows("fields.csv"):
        if r["role"] == "inertial_input":
            assert r["unit"] in units, r["field"]


def test_timebase_is_consistent_with_the_audit_in_the_manifest_document():
    doc = (ROOT.parents[1] / "docs" / "data_manifest.md").read_text()
    kitti = next(r for r in _rows("timebase.csv") if r["dataset"] == KITTI)
    assert kitti["samples"] == "2967" and "2967" in doc
    assert float(kitti["dt_min_s"]) < float(kitti["dt_mean_s"]) < float(kitti["dt_max_s"])
    assert kitti["non_increasing"] == "0" and kitti["gaps"] == "0"
