from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).parents[2] / "ardupilot_sitl/src"))


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "qgc_sitl_integration: opt-in isolated Docker SITL result validation",
    )
