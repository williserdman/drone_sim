"""
Docstring for src.drone.tests.mav_link_test
NOTES FOR VIVIAN: Connecting to rpy, remember to connect computer to tailscale using tailscale up in terminal
"""


from pymavlink import mavutil
import time


def send_time(connection):
    curr_time = int(time.time_ns()//1000)
    connection.mav.timesync_send(0, curr_time)
    print("Sending Timesync ts1:", curr_time)
    time_response = connection.recv_match(type='TIMESYNC', blocking=True, timeout=0.5)

    if not time_response:
        return None
    
    else:
        print("TIME:", time_response)

def send_status_text(connection):
    connection.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_INFO,  # severity level
        b"Sending message from Raspberry Pi"   # max length 50 characters     
    )

"""
Severity Options:

MAV_SEVERITY_EMERGENCY
MAV_SEVERITY_ALERT
MAV_SEVERITY_CRITICAL
MAV_SEVERITY_ERROR
MAV_SEVERITY_WARNING
MAV_SEVERITY_NOTICE
MAV_SEVERITY_INFO
MAV_SEVERITY_DEBUG
"""


print("Running")

# Change this to your port
connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

print("Expecting Heartbeat")

connection.wait_heartbeat()

print("CONNECTED: \n System ID:", connection.target_system, "\n Component ID:", connection.target_component)




while True:
    
    print("\n SENDING HEARTBEAT")
    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb_response = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
    send_status_text(connection)
    if hb_response:
        print("HEARTBEAT RESPONSE:", hb_response)

    send_time(connection)






