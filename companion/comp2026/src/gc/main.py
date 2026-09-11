"""Explicit, synthetic MAVLink payload-development utility.

The named-value messages in this file are a development protocol, not standard
MAVLink actuator commands. The production companion listener does not consume
them. No connection or hardware driver is loaded when this module is imported.
"""

import argparse
import glob
import sys


NUM_SERVOS = 14


def find_serial_ports():
    """Return serial device names that commonly carry MAVLink."""
    if sys.platform.startswith("win"):
        ports = [f"COM{i + 1}" for i in range(32)]
    elif sys.platform.startswith(("linux", "cygwin")):
        ports = glob.glob("/dev/tty[A-Za-z]*")
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*") + glob.glob("/dev/cu.*")
    else:
        raise EnvironmentError("Unsupported platform")

    patterns = ("USB", "ACM", "serial", "uart")
    return [port for port in ports if any(pattern in port for pattern in patterns)]


def _validate_heartbeat(message, expected_identity, phase):
    if message is None:
        raise RuntimeError(f"{phase} heartbeat timeout")

    actual_identity = (message.get_srcSystem(), message.get_srcComponent())
    if actual_identity != expected_identity:
        raise RuntimeError(
            f"MAVLink identity mismatch during {phase} heartbeat: "
            f"expected {expected_identity}, received {actual_identity}"
        )

    print(
        "Heartbeat: Sys ID %u, Comp ID %u, Type %u, Autopilot %u, "
        "Base mode %u, System status %u"
        % (
            message.get_srcSystem(),
            message.get_srcComponent(),
            message.type,
            message.autopilot,
            message.base_mode,
            message.system_status,
        )
    )
    return message


def check_heartbeat(master, expected_identity):
    """Require the next heartbeat to come from the expected source."""
    message = master.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    return _validate_heartbeat(message, expected_identity, "subsequent")


def open_close(master, close: int):
    """Emit the synthetic named-value open or close selection."""
    master.mav.named_value_int_send(2000, b"open_close", close)


def run_servo(master, servo_num: int):
    """Emit a synthetic servo selection for a test receiver."""
    if not 0 <= servo_num < NUM_SERVOS:
        raise ValueError(f"servo_num must be between 0 and {NUM_SERVOS - 1}")
    master.mav.named_value_int_send(2000, b"servo_num", servo_num)


def mavcmd_run_servo(master, servo_num: int, duty: int = 1500):
    """Refuse the old NAV_WAYPOINT-as-servo encoding."""
    del master, servo_num, duty
    raise RuntimeError(
        "MAVLink actuator test disabled: no actuator receiver contract is "
        "defined. MAV_CMD_NAV_WAYPOINT must not be used as a servo command."
    )


def run_servos(master):
    """Interactively emit every synthetic servo selection."""
    gpio_labels = [1, 2, 3, 5, 6, 8, 13, 14, 15, 19, 39, 40, 41, 42]
    for servo_num, gpio_label in enumerate(gpio_labels):
        input(f"GPIO {gpio_label}: Enter to emit synthetic servo #{servo_num}")
        run_servo(master, servo_num)


def mavcmd_run_servos(master):
    """Refuse the old batch NAV_WAYPOINT operation."""
    mavcmd_run_servo(master, 0, 2500)


def _parser():
    parser = argparse.ArgumentParser(
        description="Synthetic MAVLink payload receiver development tool"
    )
    parser.add_argument("-p", "--port", help="Explicit test endpoint")
    parser.add_argument("-b", "--baud", type=int, default=57600)
    parser.add_argument("--target-system", type=int, help="Expected MAVLink system ID")
    parser.add_argument(
        "--target-component", type=int, help="Expected MAVLink component ID"
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="Acknowledge that named-value messages require a test receiver",
    )
    parser.add_argument("--servo", type=int, help="Synthetic servo selection")
    parser.add_argument("-c", "--close", action="store_true")
    parser.add_argument("-o", "--open", action="store_true")
    parser.add_argument("-l", "--list-ports", action="store_true")
    return parser


def _validate_live_args(parser, args):
    if args.list_ports:
        return
    if not args.port:
        parser.error("--port is required; no default live endpoint is permitted")
    if args.target_system is None or args.target_component is None:
        parser.error("--target-system and --target-component are required")
    if not args.synthetic:
        parser.error("--synthetic is required to acknowledge the test protocol")
    if args.servo is None and not args.open and not args.close:
        parser.error("select --servo, --open, or --close")
    if args.open and args.close:
        parser.error("--open and --close are mutually exclusive")


def main(argv=None):
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_live_args(parser, args)

    if args.list_ports:
        ports = find_serial_ports()
        if ports:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No serial ports found")
        return

    from pymavlink import mavutil

    master = None
    try:
        master = mavutil.mavlink_connection(args.port, baud=args.baud)
        expected_identity = (args.target_system, args.target_component)
        initial_heartbeat = master.wait_heartbeat(timeout=5)
        _validate_heartbeat(initial_heartbeat, expected_identity, "initial")
        check_heartbeat(master, expected_identity)

        if args.close:
            open_close(master, 1)
        elif args.open:
            open_close(master, 0)
        else:
            run_servo(master, args.servo)
    finally:
        if master is not None:
            master.close()


if __name__ == "__main__":
    main()
