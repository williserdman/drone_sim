"""
Basic SITL simulation test script for done_control functions.
Run from root directory: python -m src.drone.sitl.simulation
"""
import time
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect
from ..control.drone_control import DroneControl, arm_and_takeoff, horiz_distance_m
from ...common_types import GPSCoord
import dronekit_sitl


# SITL home location (Van Horn)
HOME_LAT = 41.501900
HOME_LON = -81.604900
HOME_AMSL = 300
TARGET_ALT = 10

wp = [41.501000, -81.604900, 30]

# start sitl with arducopter
def start_sitl():

    print("[SITL] Starting SITL simulator...")
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
    drone_control = None
    try:
        sitl, connection_string = start_sitl()
        
        # Connect and relax pre-arm checks before initializing DroneControl
        print(f"[CONNECT] Connecting to vehicle on {connection_string}...")
        temp_vehicle = connect(connection_string, wait_ready=True)
        print("[CONNECT] Connected successfully!")
        print("[SETUP] Relaxing pre-arm checks for SITL...")
        for param, value in [('ARMING_CHECK', 0), ('FS_THR_ENABLE', 0), ('BRD_SAFETYENABLE', 0)]:
            try:
                temp_vehicle.parameters[param] = value
                time.sleep(0.05)
            except Exception:
                pass 
        print("[SETUP] Pre-arm checks relaxed")
        temp_vehicle.close()
        time.sleep(1)

        # Instantiate DroneControl class
        print("\n[INIT] Initializing DroneControl...")
        drone_control = DroneControl(connection_string)
        
        # Arm and takeoff
        print(f"\n[TAKEOFF] Arming and taking off to {TARGET_ALT}m...")
        arm_and_takeoff(drone_control.vehicle, TARGET_ALT)
        
        # Hold at altitude for a few seconds
        print(f"\n[HOLD] Holding at {TARGET_ALT}m for 5 seconds...")
        time.sleep(5)
        
        # Navigate to waypoint 
        waypoint = GPSCoord(wp[0], wp[1], wp[2])
        print(f"\n[NAV] Going to waypoint: {waypoint}")
        result = drone_control.goto_waypoint(waypoint)

        if result == 0:
            print("[NAV] Reached waypoint successfully!")
        else:
            print("[NAV] Failed to reach waypoint")
        
        # Land using DroneControl
        print("\n[LAND] Landing...")
        drone_control.simple_land()
        
        print("\n[SUCCESS] Simulation completed successfully!")
        
    except Exception as e:
        print(f"\n[ERROR] Simulation failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if drone_control and hasattr(drone_control, 'vehicle'):
            print("\n[CLEANUP] Closing vehicle connection...")
            drone_control.vehicle.close()
        
        if sitl:
            print("[CLEANUP] Stopping SITL...")
            sitl.stop()
        
        print("[CLEANUP] Done")

if __name__ == "__main__":
    main()
