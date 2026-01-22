import time


class MissonTracker:
    def __init__(self, mission_time_seconds=600):
        self.mission_begin = None
        self.timer_start = None
        self.mission_time_seconds = mission_time_seconds

    def begin_mission(self):
        self.mission_begin = time.time()
        return

    def begin_aux_timer(self):
        self.timer_start = time.time()
        return

    def end_aux_timer(self) -> float:
        if self.timer_start:
            return time.time() - self.timer_start
        return -1

    def time_left(self) -> float:
        if self.mission_begin:
            elapsed = time.time() - self.mission_begin
            return self.mission_time_seconds - elapsed
        return -1

    def end_mission(self) -> float:
        return 0


if __name__ == "__main__":
    mt = MissonTracker()
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5)
    print("timer went for:", mt.end_aux_timer())
    print("we have", mt.time_left(), "seconds left in mission")
