from pathlib import Path
import pytest
from robot_sim import WorldMap

ROOT=Path(__file__).resolve().parents[1]

@pytest.fixture
def room():
    return WorldMap({"version":1,"bounds":[0,0,5,5]})
