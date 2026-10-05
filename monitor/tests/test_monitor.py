"""The hand monitor's fixtures through pytest (monitor venv). An UNKNOWN fixture is a skip with its reason."""
from __future__ import annotations

import pytest

pytest.importorskip("mediapipe", reason="the monitor's fixtures run in .venv_monitor (mediapipe is not in this env)")

from monitor import fixtures_monitor as fx  # noqa: E402
from monitor.fixture_registry import Unknown  # noqa: E402


@pytest.mark.parametrize("f", fx.REGISTRY, ids=lambda f: f.name)
def test_fixture(f):
    try:
        ok, detail = f.fn()
    except Unknown as e:
        pytest.skip(str(e))
    print(f"{f.name}: {detail}")
    assert ok, f"{f.name} ({f.what}): {detail}"
