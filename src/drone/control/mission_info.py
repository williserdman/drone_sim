import time
from common_types import *
import json



class MissonTracker:
    def __init__(self, mission_time_seconds=600):
        self.mission_begin = None
        self.timer_start = None
        self.mission_time_seconds = mission_time_seconds
        # record of past passes using keyword: list of timed passes
        self.timed_passes = {}

    def setWaypoint(self, waypoint_ID:str, coords:GPSCoord):
        with open("mission_data/waypoints.json", "r") as waypointsJSON:
            waypointsData = json.load(waypointsJSON)
            waypointsData[waypoint_ID] = coords

        with open("mission_data/waypoints.json", "w") as waypointsWrite:
            json.dump(waypointsData, waypointsWrite)

    def getWaypoint(self, waypoint_ID:str) -> GPSCoord:
        with open("mission_data/waypoints.json", "r") as waypointsJSON:
            waypointsData = json.load(waypointsJSON)
            if waypoint_ID in waypointsData:
                return waypointsData[waypoint_ID]
            
        return None
    
    def getWaypointID(self) -> list[str]:
        waypointIDs = []
        with open("mission_data/waypoints.json", "r") as waypointsJSON:
            waypointsData = json.load(waypointsJSON)
            for elem in waypointsData:
                waypointIDs.append(elem)
            
        return waypointIDs
    
    def createPayloads(self, start:int, end:int):
        with open("mission_data/payloads.json", "w") as payloadsWrite:
            payloads = []
            for i in range(start, end):
                payloads.append({"id": i})
            
            json.dump(payloads, payloadsWrite)

    def getNextPayload(self) -> int:
        with open("mission_data/payloads.json", "r") as payloadsRead:
            jsonPayloads = json.load(payloadsRead)
            
        if not jsonPayloads:
            return None
            
        next_payload = jsonPayloads.pop(0)

        with open("mission_data/payloads.json", "w") as payloadsWrite:
            json.dump(jsonPayloads, payloadsWrite)

        return next_payload["id"]

    def begin_mission(self):
        self.mission_begin = time.time()
        return

    def begin_aux_timer(self):
        self.timer_start = time.time()
        return

    def end_aux_timer(self) -> float:
        if self.timer_start:
            return time.time() - self.timer_start
        return -1.0

    def time_left(self) -> float:
        if self.mission_begin:
            elapsed = time.time() - self.mission_begin
            return self.mission_time_seconds - elapsed
        return -1.0

    def end_mission(self) -> float:
        return 0.0
    
    # records and ends the timer
    def record_aux_timer(self, record) -> list:
        timer_length = self.end_aux_timer()
    
        if timer_length != -1:
        
            # record the pass in self.timed_passes if record
            # if keyword already recorded then adds to the keyword's list
            if record and record in self.timed_passes:
                self.timed_passes[record].append(timer_length)

            # otherwise creates a new key word with the list[0] being the recorded time
            elif record:
                self.timed_passes[record] = [timer_length]

            return self.timed_passes[record]

        return []
    
    # returns the length of the last pass for the given record
    def get_last(self, record: str) -> float:
        if record in self.timed_passes:
            return self.timed_passes[record][-1]
        
        return -1.0
    
    # returns the length of the average pass for the given record
    def get_avg(self, record: str) -> float:
        if record in self.timed_passes:
            return sum(self.timed_passes[record])/len(self.timed_passes[record])
        
        return -1.0
    
    # returns the length of the worst pass for the given record
    def get_worst(self, record: str) -> float:
        if record in self.timed_passes:
            return max(self.timed_passes[record])
        
        return -1.0
    
    # returns the length of the best pass for the given record
    def get_best(self, record: str) -> float:
        if record in self.timed_passes:
            return min(self.timed_passes[record])
        
        return -1.0


if __name__ == "__main__":
    mt = MissonTracker()
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5.0)
    print("timer went for:", mt.record_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5.5)
    print("timer went for:", mt.record_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    print("the worst case time:", mt.get_worst("LF1"))
    print("the best case time:", mt.get_best("LF1"))
    print("the average case time:", mt.get_avg("LF1"))
    print("the last case time:", mt.get_last("LF1"))

