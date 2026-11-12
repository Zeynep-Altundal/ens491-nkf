"""infra.provenance: run identity, config validation and hashing.

run_id format (plan §8.10):
    UTCtimestamp_platform_method_seed_config8_code8_nonce6
"""

import hashlib
import json
import re
import secrets
import subprocess
from datetime import UTC, datetime

RUN_ID_RE = re.compile(
    r"^\d{8}T\d{6}Z_(ugv|uav)_[a-z0-9]+_s\d{2}_[0-9a-f]{8}_[0-9a-f]{8}_[0-9a-f]{6}$"
)

REQUIRED_CONFIG_KEYS = {
    "protocol_version": str,
    "platform": str,
    "method": str,
    "seed": int,
    "split_id": str,
    "data": dict,
    "results_dir": str,
    "db_path": str,
    "registry": str,
}


def validate_config(cfg):
    for key, typ in REQUIRED_CONFIG_KEYS.items():
        if key not in cfg:
            raise ValueError(f"config missing key: {key}")
        if not isinstance(cfg[key], typ):
            raise ValueError(f"config key {key} must be {typ.__name__}")
    if cfg["platform"] not in ("ugv", "uav"):
        raise ValueError(f"bad platform {cfg['platform']}")
    return True


def config_hash(cfg):
    canonical = json.dumps(cfg, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(canonical.encode()).hexdigest()


def git_info():
    """Full SHA and dirty flag. The run registry itself is excluded from the dirty check."""
    try:
        sha = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL
        ).strip()
        dirty = (
            subprocess.check_output(
                ["git", "status", "--porcelain", "--", ".", ":!results/registry.csv"],
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            != ""
        )
        return sha, dirty
    except Exception:
        return "0" * 40, True


def utc_now(compact=False):
    now = datetime.now(UTC)
    return now.strftime("%Y%m%dT%H%M%SZ") if compact else now.strftime("%Y-%m-%dT%H:%M:%SZ")


def make_run_id(platform, method, seed, cfg_hash, code_sha):
    method_tag = re.sub(r"[^a-z0-9]", "", method.lower())
    nonce = secrets.token_hex(3)
    parts = [utc_now(compact=True), platform, method_tag, f"s{seed:02d}"]
    run_id = "_".join([*parts, cfg_hash[:8], code_sha[:8], nonce])
    assert RUN_ID_RE.match(run_id), run_id
    return run_id


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()
