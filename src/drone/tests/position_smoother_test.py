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

    def test_ema_calculation(self, smoother):
        smoother.append(10.0)
        # ema = 10 * 0.5 + 0 * 0.5 = 5.0
        assert smoother.ema == pytest.approx(5.0)
        
        smoother.append(20.0)
        # ema = 20 * 0.5 + 5.0 * 0.5 = 12.5
        assert smoother.ema == pytest.approx(12.5)

    def test_window_limit_error(self, smoother):
        with pytest.raises(IndexError):
            for i in range(12):
                smoother.append(float(i))

class TestRelPosSmoother:
    @pytest.fixture
    def rel_smoother(self):
        return position_smoother.RelPosSmoother(window=10)

    def test_append_and_get_relative(self, rel_smoother):
        pos = RelativePosition(10.0, 20.0)
        rel_smoother.append(pos)
        
        ema_result = rel_smoother.get_ema()
        assert isinstance(ema_result, RelativePosition)
        assert ema_result.x != 0
        assert ema_result.y != 0

    def test_empty_get_returns_negative_one(self, rel_smoother):
        assert rel_smoother.get_ema() == -1
        assert rel_smoother.get_sma() == -1

    @pytest.mark.parametrize("x_val, y_val", [
        (1.0, 2.0),
        (100.5, 200.7),
        (-5.0, 0.0)
    ])
    def test_multiple_inputs(self, rel_smoother, x_val, y_val):
        rel_smoother.append(RelativePosition(x_val, y_val))
        sma = rel_smoother.get_sma()
        assert sma.x == x_val
        assert sma.y == y_val
