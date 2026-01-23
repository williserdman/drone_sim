# Nathaniel Hahn for cwru vtol
# 01-22-2026
# please run from root (src)
import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from common_types import *  # or import specific types you need
from drone.utils import position_smoother





class TestPositionSmoother:
    @pytest.fixture
    def smoother(self):
        return position_smoother.PositionSmoother(window=3)

    def test_init(self, smoother):
        assert smoother.window == 3
        assert smoother.num_els == 0
        assert smoother.sma == 0
        assert smoother.ema == 0
    
    def test_sma_calc(self, smoother):
        smoother.append(10.0)
        assert smoother.sma == 10.0
        
        smoother.append(20.0)
        assert smoother.sma == 15.0 # cause (10 + 20) / 2 = 15




