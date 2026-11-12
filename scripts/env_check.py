"""W01-M-1 environment check.

Usage (inside the nkf conda env, from the repo root):
    python scripts/env_check.py --machine melike

Writes docs/env_checks/<machine>.json and exits non-zero if a required import fails.
"""

import argparse
import importlib
import json
import os
import platform
import subprocess
import sys
import time
from datetime import datetime, timezone

REQUIRED = ["numpy", "scipy", "pandas", "matplotlib", "yaml", "hydra", "omegaconf", "pytest", "torch"]
OPTIONAL = ["wandb"]


def get_version(name):
    try:
        mod = importlib.import_module(name)
        return getattr(mod, "__version__", "unknown"), None
    except Exception as e:
        return None, repr(e)


def read_os_release():
    info = {}
    try:
        with open("/etc/os-release") as f:
            for line in f:
                if "=" in line:
                    k, v = line.strip().split("=", 1)
                    info[k] = v.strip('"')
    except OSError:
        pass
    return info.get("PRETTY_NAME", platform.platform())


def total_ram_gb():
    try:
        return round(os.sysconf("SC_PAGE_SIZE") * os.sysconf("SC_PHYS_PAGES") / 1e9, 1)
    except (ValueError, OSError, AttributeError):
        return None


def git_sha():
    try:
        sha = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, stderr=subprocess.DEVNULL).strip()
        dirty = subprocess.check_output(["git", "status", "--porcelain"], text=True).strip() != ""
        return sha, dirty
    except Exception:
        return None, None


def torch_check():
    import torch

    out = {"cuda_available": torch.cuda.is_available(), "torch_cuda_build": torch.version.cuda}
    device = "cuda" if out["cuda_available"] else "cpu"
    if out["cuda_available"]:
        props = torch.cuda.get_device_properties(0)
        out["gpu_name"] = props.name
        out["gpu_vram_gb"] = round(props.total_memory / 1e9, 1)

    t0 = time.perf_counter()
    a = torch.randn(512, 512, dtype=torch.float64, device=device)
    b = a @ a.T
    ok = bool(torch.isfinite(b).all().item())
    if device == "cuda":
        torch.cuda.synchronize()
    out["matmul_device"] = device
    out["matmul_finite"] = ok
    out["matmul_ms"] = round((time.perf_counter() - t0) * 1000, 2)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--machine", required=True, help="betul | melike | zeynep")
    parser.add_argument("--outdir", default="docs/env_checks")
    args = parser.parse_args()

    packages, failures = {}, {}
    for name in REQUIRED + OPTIONAL:
        version, err = get_version(name)
        packages[name] = version
        if err and name in REQUIRED:
            failures[name] = err

    sha, dirty = git_sha()
    record = {
        "machine": args.machine,
        "checked_utc": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "os": read_os_release(),
        "kernel": platform.release(),
        "python": sys.version.split()[0],
        "ram_gb": total_ram_gb(),
        "conda_env": os.environ.get("CONDA_DEFAULT_ENV"),
        "git_sha": sha,
        "git_dirty": dirty,
        "packages": packages,
        "import_failures": failures,
    }
    if "torch" not in failures:
        record["torch"] = torch_check()

    record["status"] = "pass" if not failures else "fail"

    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir, f"{args.machine}.json")
    with open(path, "w") as f:
        json.dump(record, f, indent=2)

    print(json.dumps(record, indent=2))
    print(f"\nwritten: {path}  status: {record['status']}")
    sys.exit(0 if not failures else 1)


if __name__ == "__main__":
    main()
