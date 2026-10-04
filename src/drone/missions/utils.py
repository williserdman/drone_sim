"""
These will send messages
"""

from pymavlink import mavutil


def send_log(
    text, master_mavlink_connection, severity=mavutil.mavlink.MAV_SEVERITY_INFO
):
    print(f"Sending: {text} (Severity: {severity})")
    # statustext_send automatically broadcasts to QGC
    master_mavlink_connection.mav.statustext_send(
        severity, text.encode("utf-8")  # MAVLink requires byte strings
    )


def log(msg: str, master):
    send_log(msg, master, mavutil.mavlink.MAV_SEVERITY_INFO)


def warn(msg: str, master):
    send_log(msg, master, mavutil.mavlink.MAV_SEVERITY_CRITICAL)
