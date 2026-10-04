import time
from pymavlink import mavutil

CONNECTION_STRING = "tcp:localhost:5763"
BAUD_RATE = 921600


def start_sender():
    print(f"[*] Connecting to {CONNECTION_STRING}...")

    master = mavutil.mavlink_connection(
        CONNECTION_STRING, baud=BAUD_RATE, source_system=1, source_component=191
    )

    print("[*] Waiting for heartbeat from Pixhawk...")
    master.wait_heartbeat()
    print("[+] Connected! Starting message transmission...\n")

    def send_log(text, severity=mavutil.mavlink.MAV_SEVERITY_INFO):
        print(f"Sending: {text} (Severity: {severity})")
        # statustext_send automatically broadcasts to QGC
        master.mav.statustext_send(
            severity, text.encode("utf-8")  # MAVLink requires byte strings
        )

    try:
        while True:
            # INFO: Usually just quietly logs in the QGC messages dropdown
            send_log(
                "RPi: System nominal, running checks.",
                mavutil.mavlink.MAV_SEVERITY_INFO,
            )
            time.sleep(5)

            """ # WARNING: Usually triggers a yellow pop-up alert in QGC
            send_log(
                "RPi: CPU temperature elevated.", mavutil.mavlink.MAV_SEVERITY_WARNING
            )
            time.sleep(5) """  # came up as red in the message chat, but didn't pop up

            # CRITICAL: Triggers a yellow, pop-up alert in QGC
            send_log("RPi: FM1 Initiated!", mavutil.mavlink.MAV_SEVERITY_CRITICAL)
            time.sleep(5)

    except KeyboardInterrupt:
        print("\n[*] Exiting Sender Script.")


if __name__ == "__main__":
    start_sender()
