# SITL or QGC

from pymavlink import mavutil
import time
import random
import argparse
import glob
import sys
import os
NUM_SERVOS = 14

# Modeled from ash and nate RadioControl.py
def find_serial_ports():
    """
    Automatically detect serial ports on the system.
    Returns a list of potential serial ports.
    """
    if sys.platform.startswith('win'):
        # Windows
        ports = ['COM%s' % (i + 1) for i in range(32)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # Linux/Cygwin
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        # macOS
        ports = glob.glob('/dev/tty.*')
        ports.extend(glob.glob('/dev/cu.*'))
    else:
        raise EnvironmentError('Unsupported platform')
    
    result = []
    for port in ports:
        # Filter for common MAVLink-related port patterns
        if any(pattern in port for pattern in ['USB', 'ACM', 'serial', 'uart']):
            result.append(port)
    
    return result

def check_heartbeat(master):
    """Wait (blocking up to 5 seconds) for the next HEARTBEAT message"""
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print("Heartbeat: Sys ID %u, Comp ID %u, Type %u, Autopilot %u, Base mode %u, System status %u" %
                (msg.get_srcSystem(), msg.get_srcComponent(), msg.type, msg.autopilot,
                msg.base_mode, msg.system_status))
        return True
    else:
        print("No heartbeat received in the last 5 seconds")
        return False

def open_close(master, close: int):
    """
    close: True closes the servos, False opens the servos
    """
    master.mav.named_value_int_send(
        2000, b'open_close', close 
    )

def run_servo(master, servo_num=random.randint(0, 15)):
    master.mav.named_value_int_send(
        2000, b'servo_num', servo_num
    )

# Adjust to oour listener and sender QGC
def mavcmd_run_servo(master, servo_num=random.randint(0, 15), duty=1500):
    master.mav.mission_item_int_send(
    1,           # Target system ID (e.g. 1)
    201,        # Target component ID (typically 1)
    5789,                     # Sequence number for this waypoint
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Coordinate frame
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,               # MAV_CMD
    1,                 # 1 if this is the current waypoint, 0 otherwise
    1,            # 1 if the vehicle should continue to the next waypoint automatically
    servo_num,                  # Command-specific parameter 1 (float)
    duty,                   # Parameter 2
    5789,                   # Parameter 3
    5789,                   # Parameter 4
    1,                      # X parameter (latitude in degrees * 1e7 as int32)
    2,                      # Y parameter (longitude in degrees * 1e7 as int32)
    3,                      # Z parameter (altitude in meters; float)
    mavutil.mavlink.MAV_MISSION_TYPE_MISSION    # Mission type
)

def run_servos(master):
    # Check All Servos
    servos = [
        1, 2, 3, 5, 6, 8, 13,
        14, 15, 19, 39, 40, 41, 42, None
    ]
    for i in range(NUM_SERVOS):
        input(f"GPIO {servos[i]}: Enter to run servo #{i}")
        run_servo(master, i)

def mavcmd_run_servos(master):
    # Check All Servos
    servos = [
        1, 2, 3, 5, 6, 8, 13,
        14, 15, 19, 39, 40, 41, 42, None
    ]
    for i in range(NUM_SERVOS):
        input(f"GPIO {servos[i]}: Enter to run servo #{i}")
        mavcmd_run_servo(master, i, 2500)

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description='MAVLink Servo Control Tool')
    
    # Define command line arguments
    parser.add_argument('-p', '--port', type=str, help='Serial port to connect to')
    parser.add_argument('-b', '--baud', type=int, default=57600, help='Baud rate (default: 57600)')
    parser.add_argument('-a', '--auto-detect', action='store_true', help='Automatically detect serial port')
    parser.add_argument('-l', '--list-ports', action='store_true', help='List available serial ports')
    parser.add_argument('-c', '--close', action='store_true', help='Close all of the servos simultaneously')
    parser.add_argument('-o', '--open', action='store_true', help='Open all of the servos simultaneously')
    
    # Parse arguments
    args = parser.parse_args()
    
    # If list-ports flag is set, show available ports and exit
    if args.list_ports:
        ports = find_serial_ports()
        if ports:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No serial ports found")
        return
    
    # Determine which port to use
    connection_string = None
    
    if args.port:
        # Use user-specified port
        connection_string = args.port
        print(f"Using specified port: {connection_string}")
    elif args.auto_detect:
        # Try auto-detection
        ports = find_serial_ports()
        if not ports:
            print("Error: No serial ports found. Please specify a port with --port")
            return
        
        print("Attempting to connect to available ports...")
        
        for port in ports:
            try:
                print(f"Trying {port}...")
                master = mavutil.mavlink_connection(port, baud=args.baud, autoreconnect=False)
                
                # Try to get a heartbeat with a short timeout
                master.wait_heartbeat(timeout=2)
                connection_string = port
                print(f"Connected to {port} successfully!")
                # Close connection to reopen it properly later
                master.close()
                break
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
                continue
        
        if not connection_string:
            print("Error: Could not auto-detect a working MAVLink device.")
            print("Available ports were:", ports)
            print("Try specifying a port manually with --port")
            return
    else:
        # Default fallback values
        if sys.platform.startswith('darwin'):  # macOS
            connection_string = "/dev/tty.usbserial-DU0D58Y0"
        elif sys.platform.startswith('linux'):  # Linux
            connection_string = "/dev/ttyACM0"
        else:  # Windows or other
            connection_string = "COM1"
        
        print(f"No port specified. Using default: {connection_string}")
        print("You can specify a port with --port or use --auto-detect")
    
    # Connect to the flight controller
    try:
        master = mavutil.mavlink_connection(connection_string, baud=args.baud)
        print(f"Connecting to {connection_string} at {args.baud} baud...")
        
        print("Waiting for heartbeat from the flight controller...")
        master.wait_heartbeat(timeout=5)  # This blocks until a heartbeat is received
        print("Heartbeat detected!")
        print("System ID: %u, Component ID: %u" % (master.target_system, master.target_component))
        
        if check_heartbeat(master):
            # Run your control code here
            if args.close:
                open_close(master, 1)    # Close all servos before running
            elif args.open:
                open_close(master, 0)    # Close all servos before running
            mavcmd_run_servos(master)
            # open_close(master, 0)   # Open all servos after test
        else:
            print("Error: Lost connection to flight controller")
            
    except Exception as e:
        print(f"Error connecting to {connection_string}: {e}")
        return

if __name__ == "__main__":
    main()