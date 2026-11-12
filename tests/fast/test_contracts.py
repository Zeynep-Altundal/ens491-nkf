"""Import contract: every planned module exists and imports without optional dependencies."""

import importlib

import pytest

MODULES = [
    "nkf.data.kitti",
    "nkf.data.euroc",
    "nkf.data.synthetic",
    "nkf.data.sequence",
    "nkf.data.splits",
    "nkf.data.outage",
    "nkf.data.labels",
    "nkf.data.injectors",
    "nkf.baselines.strapdown",
    "nkf.baselines.eskf",
    "nkf.baselines.adaptive",
    "nkf.baselines.constraints_ugv",
    "nkf.baselines.constraints_uav",
    "nkf.filters.torch_eskf",
    "nkf.filters.noise_param",
    "nkf.filters.api",
    "nkf.models.uncertainty_net",
    "nkf.models.gain_net",
    "nkf.models.features",
    "nkf.train.loop",
    "nkf.train.losses",
    "nkf.train.curriculum",
    "nkf.train.tbptt",
    "nkf.eval.metrics",
    "nkf.eval.consistency",
    "nkf.eval.calibration",
    "nkf.eval.evo_export",
    "nkf.eval.failure_atlas",
    "nkf.infra.run",
    "nkf.infra.sweep",
    "nkf.infra.tracking",
    "nkf.infra.results_db",
    "nkf.infra.provenance",
    "nkf.figures.plots",
    "nkf.figures.tables",
]


@pytest.mark.fast
@pytest.mark.parametrize("name", MODULES)
def test_module_imports(name):
    importlib.import_module(name)
