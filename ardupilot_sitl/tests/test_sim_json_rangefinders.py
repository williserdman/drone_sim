from pathlib import Path
import shutil
import subprocess
import sys

import pytest


@pytest.mark.skipif(shutil.which("g++") is None, reason="source check requires g++")
@pytest.mark.parametrize("first_bit, succeeds", [(7, True), (10, False)])
def test_source_check_executes_the_declared_sensor_bit_mapping(
    tmp_path: Path, first_bit: int, succeeds: bool
) -> None:
    sitl = tmp_path / "libraries/SITL"
    sitl.mkdir(parents=True)
    (sitl / "SIM_JSON.h").write_text("\n".join(
        f"    RNG_{index + 1} = 1U << {first_bit + index},"
        for index in range(6)
    ))
    # The actual 4.6.3 update block. Moving the declared bits without updating
    # this loop reproduces the later upstream regression caught by the checker.
    (sitl / "SIM_JSON.cpp").write_text("""    // update range finder distances
    for (uint8_t i=7; i<13; i++) {
        if ((received_bitmask & 1U << i) == 0) {
            continue;
        }
        rangefinder_m[i-7] = state.rng[i-7];
    }

    // update wind vane
""")
    result = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("check_sim_json_rangefinders.py")),
         "--source-root", str(tmp_path)],
        capture_output=True, text=True, check=False,
    )
    assert (result.returncode == 0) is succeeds, result.stdout + result.stderr
    if not succeeds:
        assert "mapped incorrectly" in result.stderr
