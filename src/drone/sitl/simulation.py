"""
Basic SITL simulation test script for done_control functions.
Run directly: python simulation.py
"""
import sys
import os
import time
import collections

if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))

from dronekit import connect, VehicleMode
from src.drone.control.drone_control import arm_and_takeoff


# SITL home location (Van Horn)
HOME_LAT = 41.501900
HOME_LON = -81.604900
HOME_AMSL = 300

TARGET_ALT = 30

# start sitl with arducopter
def start_sitl():
    print("[SITL] Starting SITL simulator...")
    import dronekit_sitl
    sitl = dronekit_sitl.SITL()
    
    sitl.download('copter', 'stable')
    print("[SITL] Using stable ArduCopter build")
    
    # Launch SITL with home location
    sitl.launch(
        ['-I0', '--model', 'quad', f'--home={HOME_LAT},{HOME_LON},{HOME_AMSL},0'],
        await_ready=True,
        restart=True
    )

    connection_string = sitl.connection_string()
    print(f"[SITL] SITL started at {connection_string}")
    return sitl, connection_string

# Connect to vehicle
def connect_vehicle(connection_string):
    print(f"[CONNECT] Connecting to vehicle on {connection_string}...")
    vehicle = connect(connection_string, wait_ready=True)
    print("[CONNECT] Connected successfully!")
    print(f"[CONNECT] Vehicle mode: {vehicle.mode.name}")
    print(f"[CONNECT] GPS: {vehicle.gps_0}")
    print(f"[CONNECT] Battery: {vehicle.battery}")
    return vehicle

# Relax pre-arm checks for SITL
def relax_prearm_checks(vehicle):
    print("[SETUP] Relaxing pre-arm checks for SITL...")
    for param, value in [('ARMING_CHECK', 0), ('FS_THR_ENABLE', 0), ('BRD_SAFETYENABLE', 0)]:
        try:
            vehicle.parameters[param] = value
            time.sleep(0.05)
        except Exception:
            pass 
    print("[SETUP] Pre-arm checks relaxed")

# Simulation script
def main():
    sitl = None
    vehicle = None
    try:
        sitl, connection_string = start_sitl()
        vehicle = connect_vehicle(connection_string)
        relax_prearm_checks(vehicle)
        time.sleep(1)

        # Arm and takeoff
        arm_and_takeoff(vehicle, TARGET_ALT)
        
        # Hold at altitude for a few seconds
        print(f"\n[HOLD] Holding at {TARGET_ALT}m for 5 seconds...")
        time.sleep(5)
        
        # Land
        vehicle.mode = VehicleMode("RTL")
        
        print("\n[SUCCESS] Simulation completed successfully!")
        
    except Exception as e:
        print(f"\n[ERROR] Simulation failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if vehicle:
            print("\n[CLEANUP] Closing vehicle connection...")
            vehicle.close()
        
        if sitl:
            print("[CLEANUP] Stopping SITL...")
            sitl.stop()
        
        print("[CLEANUP] Done")

if __name__ == "__main__":
    main()
