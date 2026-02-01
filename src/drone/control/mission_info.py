import time


class MissonTracker:
    def __init__(self, mission_time_seconds=600):
        self.mission_begin = None
        self.timer_start = None
        self.mission_time_seconds = mission_time_seconds
        # record of past passes using keyword: list of timed passes
        self.timed_passes = {}

    def begin_mission(self):
        self.mission_begin = time.time()
        return

    def begin_aux_timer(self):
        self.timer_start = time.time()
        return

    # record is the optional keyword for recording the pass
    def end_aux_timer(self, record = None) -> float:
        if self.timer_start:
        
            # record the pass in self.timed_passes if record
            # if keyword already recorded then adds to the keyword's list
            if record and record in self.timed_passes:
                self.timed_passes[record].append(time.time() - self.timer_start)

            # otherwise creates a new key word with the list[0] being the recorded time
            elif record:
                self.timed_passes[record] = [time.time() - self.timer_start]

            return time.time() - self.timer_start

        return -1

    def time_left(self) -> float:
        if self.mission_begin:
            elapsed = time.time() - self.mission_begin
            return self.mission_time_seconds - elapsed
        return -1

    def end_mission(self) -> float:
        return 0
    
    # returns the length of the last pass for the given record
    def get_last(self, record: str) -> float:
        if record in self.timed_passes:
            return self.timed_passes[record][-1]
        
        return -1
    
    # returns the length of the average pass for the given record
    def get_avg(self, record: str) -> float:
        if record in self.timed_passes:
            return sum(self.timed_passes[record])/len(self.timed_passes[record])
        
        return -1
    
    # returns the length of the worst pass for the given record
    def get_worst(self, record: str) -> float:
        if record in self.timed_passes:
            return max(self.timed_passes[record])
        
        return -1
    
    # returns the length of the best pass for the given record
    def get_best(self, record: str) -> float:
        if record in self.timed_passes:
            return min(self.timed_passes[record])
        
        return -1


if __name__ == "__main__":
    mt = MissonTracker()
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5)
    print("timer went for:", mt.end_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5.5)
    print("timer went for:", mt.end_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    print("the worst case time:", mt.get_worst("LF1"))
    print("the best case time:", mt.get_best("LF1"))
    print("the average case time:", mt.get_avg("LF1"))
    print("the last case time:", mt.get_last("LF1"))

