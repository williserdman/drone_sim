import subprocess
import sys
import textwrap


def test_listener_import_does_not_load_hardware_drivers():
    script = textwrap.dedent(
        """
        import importlib.abc
        import sys

        blocked = {"gpiozero", "board", "busio", "adafruit_lidarlite"}

        class BlockHardwareDrivers(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split(".", 1)[0] in blocked:
                    raise ModuleNotFoundError(f"blocked hardware driver: {fullname}")
                return None

        sys.meta_path.insert(0, BlockHardwareDrivers())
        import drone.control.listener

        loaded = blocked.intersection(sys.modules)
        assert not loaded, f"listener imported hardware drivers: {sorted(loaded)}"
        """
    )

    subprocess.run([sys.executable, "-c", script], check=True)
