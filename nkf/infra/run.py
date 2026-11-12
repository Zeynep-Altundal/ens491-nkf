"""infra.run: W01 synthetic pipeline. Sequence -> strapdown -> metrics -> SQLite -> run bundle.

The estimator is the strapdown baseline of docs/interfaces.md, section 4, with an oracle
initial state and no measurement update. It receives the inertial input and the initial
state only.

Usage (repo root, nkf env):
    python -m nkf.infra.run --platform ugv
    python -m nkf.infra.run --platform uav --seed 1
"""

import argparse
import json
import os
import sys
import traceback

import numpy as np
import yaml

from nkf.baselines.strapdown import oracle_initial_state, run_strapdown
from nkf.data.sequence import count_updates_by_source
from nkf.data.synthetic import make_synthetic
from nkf.eval.metrics import position_metrics
from nkf.infra import provenance, results_db


def deep_merge(base, override):
    out = dict(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(base_path, experiment_path, platform, seed):
    with open(base_path) as f:
        cfg = yaml.safe_load(f) or {}
    with open(experiment_path) as f:
        cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    cfg["platform"] = platform
    if seed is not None:
        cfg["seed"] = seed
    provenance.validate_config(cfg)
    return cfg


def write_bundle(bundle, cfg, seq, est, db_rows):
    with open(os.path.join(bundle, "resolved_config.yaml"), "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=True)
    with open(os.path.join(bundle, "metrics.csv"), "w") as f:
        f.write("run_id,sequence_id,split,outage_id,metric,statistic,value,unit,status\n")
        f.writelines(
            f"{r[0]},{r[1]},{r[2]},{r[3]},{r[4]},{r[5]},{r[6]:.6f},{r[7]},{r[10]}\n"
            for r in db_rows
        )
    np.savez_compressed(
        os.path.join(bundle, "trajectory.npz"),
        t_s=seq.imu.t_s,
        p_est=est["p_w"],
        p_ref=seq.reference.p_w,
    )
    return {
        name: provenance.sha256_file(os.path.join(bundle, name))
        for name in sorted(os.listdir(bundle))
    }


def run_synthetic(cfg):
    cfg_hash = provenance.config_hash(cfg)
    code_sha, dirty = provenance.git_info()
    run_id = provenance.make_run_id(cfg["platform"], cfg["method"], cfg["seed"], cfg_hash, code_sha)

    bundle = os.path.join(cfg["results_dir"], run_id)
    os.makedirs(bundle, exist_ok=True)
    os.makedirs(os.path.dirname(cfg["db_path"]) or ".", exist_ok=True)
    con = results_db.connect(cfg["db_path"])

    run_row = {
        "run_id": run_id,
        "parent_run_id": None,
        "status": "running",
        "start_utc": provenance.utc_now(),
        "owner": cfg.get("owner"),
        "issue": cfg.get("issue"),
        "purpose": cfg.get("purpose"),
        "protocol_version": cfg["protocol_version"],
        "platform": cfg["platform"],
        "method": cfg["method"],
        "seed": cfg["seed"],
        "split_id": cfg["split_id"],
        "config_hash": cfg_hash,
        "code_sha": code_sha,
        "git_dirty": int(dirty),
        "bundle_uri": bundle,
    }
    results_db.insert_run(con, run_row)
    con.commit()

    try:
        data_cfg = {**cfg["data"], **cfg["data"][cfg["platform"]]}
        seq = make_synthetic(
            cfg["platform"],
            seed=cfg["seed"],
            duration_s=data_cfg["duration_s"],
            imu_hz=data_cfg["imu_hz"],
            cfg={**data_cfg, "config_hash": cfg_hash},
        )
        initial = oracle_initial_state(seq.reference)
        # run_strapdown validates the time base and refuses a sequence with a gap.
        p_est, _v_est, _q_est = run_strapdown(seq.imu, initial)
        est = {"p_w": p_est}

        rows = position_metrics(p_est, seq.reference.p_w, seq.imu.t_s, seq.outage_s)
        n = len(seq.imu.t_s)
        db_rows = [
            (run_id, seq.id, seq.split_id, oid, m, s, v, u, "position_3d", n, "ok", None)
            for (m, s, v, u, oid) in rows
        ]
        files = write_bundle(bundle, cfg, seq, est, db_rows)

        end_utc = provenance.utc_now()
        manifest = {
            "run_id": run_id,
            "parent_run_id": None,
            "status": "completed",
            "start_utc": run_row["start_utc"],
            "end_utc": end_utc,
            "owner": cfg.get("owner"),
            "issue": cfg.get("issue"),
            "purpose": cfg.get("purpose"),
            "protocol_version": cfg["protocol_version"],
            "code": {"git_sha": code_sha, "git_dirty": dirty},
            "data": {
                "sequence_id": seq.id,
                "source_hash": seq.source_hash,
                "split_id": seq.split_id,
            },
            "method": {"name": cfg["method"], "seed": cfg["seed"], "config_hash": cfg_hash},
            "init": initial.init,
            "updates_by_source": count_updates_by_source([]),
            "observations": {
                "generated_available": len(seq.observations),
                "hidden_in_outage": seq.hidden_observations,
                "used_by_estimator": 0,
                "provenance": seq.observations[0].provenance if seq.observations else None,
            },
            "outputs": files,
            "note": "Synthetic smoke run; no estimator performance claim.",
        }
        manifest_path = os.path.join(bundle, "manifest.json")
        with open(manifest_path, "w") as f:
            json.dump(manifest, f, indent=2)

        results_db.insert_metrics(con, db_rows)
        results_db.insert_artifacts(con, run_id, files)
        results_db.update_run(
            con, run_id, status="completed", end_utc=end_utc, source_hash=seq.source_hash
        )
        con.commit()
        results_db.append_registry(
            cfg["registry"],
            {
                "run_id": run_id,
                "status": "completed",
                "artifact_uri": bundle,
                "sha256": provenance.sha256_file(manifest_path),
            },
        )
        return run_id, rows, dirty
    except Exception as e:
        results_db.update_run(
            con, run_id, status="failed", end_utc=provenance.utc_now(), failure_reason=repr(e)
        )
        con.commit()
        results_db.append_registry(
            cfg["registry"], {"run_id": run_id, "status": "failed", "artifact_uri": bundle}
        )
        raise
    finally:
        con.close()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--platform", choices=["ugv", "uav"], required=True)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--base", default="configs/base.yaml")
    parser.add_argument("--experiment", default="configs/experiments/smoke_synthetic.yaml")
    args = parser.parse_args(argv)

    cfg = load_config(args.base, args.experiment, args.platform, args.seed)
    try:
        run_id, rows, dirty = run_synthetic(cfg)
    except Exception:
        traceback.print_exc()
        sys.exit(1)
    print(f"run_id: {run_id}")
    for m, _s, v, u, oid in rows:
        print(f"  {m:15s} {oid:18s} {v:10.3f} {u}")
    if dirty:
        print("WARNING: git tree dirty -> exploratory run, not official evidence")


if __name__ == "__main__":
    main()
