from ..common_types import RelativePosition


class PositionSmoother:
    def __init__(self, window=10):
        self.sum = 0
        self.window = window
        self.elements = []
        self.num_els = 0

        self.sma = 0
        self.ema = 0

        self.ema_mult = 2 / (window + 1)

    def append(self, el: float):
        # FIX 1: Actually store the element so we can pop it later!
        self.elements.append(el)

        self.sum += el
        self.num_els += 1

        # FIX 2: Use self.window instead of hardcoding 10
        if self.num_els > self.window:
            self.num_els -= 1
            self.sum -= self.elements.pop(0)

        self.sma = self.sum / self.num_els

        # Calculate EMA based on the previous EMA
        self.ema = el * self.ema_mult + self.get_ema() * (1 - self.ema_mult)

        return

    def get_ema(self):
        return self.ema

    def get_sma(self):
        return self.sma


class RelPosSmoother:
    def __init__(self, window=10):
        self.ps_x = PositionSmoother(window)
        self.ps_y = PositionSmoother(window)

    def append(self, el: RelativePosition):
        self.ps_x.append(el.x)
        self.ps_y.append(el.y)
        return

    def get_ema(self):
        if self.ps_x.num_els > 0:
            return RelativePosition(self.ps_x.get_ema(), self.ps_y.get_ema())
        else:
            return -1

    def get_sma(self):
        if self.ps_x.num_els > 0:
            return RelativePosition(self.ps_x.get_sma(), self.ps_y.get_sma())
        else:
            return -1
