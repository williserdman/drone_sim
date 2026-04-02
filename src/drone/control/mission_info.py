import time
from common_types import *
import json

"""
mission_info.py - This file allows tracking of time within a mission, storing waypoints (id, coord), and storing payload id's
"""

__author__ = "Vivian Chuang"

class MissonTracker:
    def __init__(self, mission_time_seconds=600):
        self.mission_begin = None
        self.timer_start = None
        self.mission_time_seconds = mission_time_seconds
        # record of past passes using keyword: list of timed passes
        self.timed_passes = {}

    # set a waypoint by giving the waypoint id and coordinate
    def set_waypoint(self, waypoint_ID:str, coords:GPSCoord):
        with open("mission_data/waypoints.json", "r+") as waypoints_JSON:
            waypoints_data = json.load(waypoints_JSON)
            waypoints_data[waypoint_ID] = coords

            # writing to file
            waypoints_JSON.seek(0)
            json.dump(waypoints_data, waypoints_JSON)
            waypoints_JSON.truncate()

    # get a waypoint's coordinates from its' id
    def get_waypoint(self, waypoint_ID:str) -> GPSCoord:
        with open("mission_data/waypoints.json", "r") as waypoints_JSON:
            waypoints_data = json.load(waypoints_JSON)
            if waypoint_ID in waypoints_data:
                return waypoints_data[waypoint_ID]
            
        return None
    
    # get a list of all waypoint id's
    def get_waypoint_id(self) -> list[str]:
        waypoint_IDs = []
        with open("mission_data/waypoints.json", "r") as waypoints_JSON:
            waypoints_data = json.load(waypoints_JSON)
            for elem in list(waypoints_data.keys()):
                waypoint_IDs.append(elem)
            
        return waypoint_IDs
    
    # add a payload id
    def add_payload(self, id:int):
        with open("mission_data/payloads.json", "r+") as payloads_write:
            payloads_data = json.load(payloads_write)
            if not payloads_data["id"]:
                payloads_data["id"] = []

            payloads_data["id"].append(id)
            json.dump(payloads_data, payloads_write)

    # remove a payload given its' id
    def remove_payload(self, id:int) -> bool:
        with open("mission_data/payloads.json", "r+") as payloads_write:
            payloads_data = json.load(payloads_write)
            if not payloads_data["id"]:
                return False

            payloads_data["id"].remove(id)
            json.dump(payloads_data, payloads_write)

            return True
        
    # returns all payload ids
    def get_payload_id(self) -> list[int]:
        with open("mission_data/payloads.json", "r") as payloads_read:
            payloads_data = json.load(payloads_read)
            return payloads_data["id"]

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

