"""The planner's fixtures through pytest: one test per fixture of planner/fixtures.py, the detail printed (-s / -rA)."""
from __future__ import annotations

import pytest

from planner import fixtures as fx


@pytest.mark.parametrize("fixture", fx.REGISTRY, ids=[f["name"] for f in fx.REGISTRY])
def test_fixture(fixture):
    ok, detail = fixture["fn"]()
    print(f"{fixture['name']} ({fixture['kind']}): {detail}")
    assert ok, f"{fixture['name']} ({fixture['kind']}): {detail}\n  {fixture['what']}"
