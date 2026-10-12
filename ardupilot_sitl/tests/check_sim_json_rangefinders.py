#!/usr/bin/env python3
"""Compile and execute the range update block from an ArduPilot checkout."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import tempfile
import textwrap


def _range_update_block(source: str) -> str:
    start_marker = "    // update range finder distances\n"
    end_marker = "\n    // update wind vane"
    start = source.index(start_marker) + len(start_marker)
    end = source.index(end_marker, start)
    return textwrap.dedent(source[start:end]).strip()


def _range_masks(header: str) -> list[str]:
    masks = []
    for index in range(1, 7):
        match = re.search(
            rf"^\s*RNG_{index}\s*=\s*(0x[0-9A-Fa-f]+ULL)",
            header,
            flags=re.MULTILINE,
        )
        if match is None:
            raise RuntimeError(f"SIM_JSON.h has no RNG_{index} mask")
        masks.append(match.group(1))
    return masks


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", required=True, type=Path)
    args = parser.parse_args()
    sitl = args.source_root / "libraries/SITL"
    block = _range_update_block((sitl / "SIM_JSON.cpp").read_text())
    masks = _range_masks((sitl / "SIM_JSON.h").read_text())
    constants = "\n".join(
        f"constexpr uint64_t RNG_{index} = {mask};"
        for index, mask in enumerate(masks, 1)
    )
    program = f"""
        #include <cstdint>
        #include <cstdio>
        #include <algorithm>
        {constants}

        int main() {{
            struct {{ float rng[6]; }} state{{{{11, 22, 33, 44, 55, 66}}}};
            const uint64_t masks[6] = {{{', '.join(masks)}}};
            float rangefinder_m[10];
            for (uint8_t source = 0; source < 6; source++) {{
                std::fill_n(rangefinder_m, 10, -1.0f);
                const uint64_t received_bitmask = masks[source];
                {textwrap.indent(block, ' ' * 16).lstrip()}
                for (uint8_t destination = 0; destination < 10; destination++) {{
                    const float expected = destination == source ? state.rng[source] : -1.0f;
                    if (rangefinder_m[destination] != expected) {{
                        std::fprintf(stderr, "rng_%u mapped incorrectly at slot %u\\n",
                                     source + 1, destination);
                        return 1;
                    }}
                }}
            }}
            std::fill_n(rangefinder_m, 10, -1.0f);
            const uint64_t received_bitmask = 0;
            {textwrap.indent(block, ' ' * 12).lstrip()}
            for (float value : rangefinder_m) {{
                if (value != -1.0f) return 2;
            }}
            return 0;
        }}
    """
    with tempfile.TemporaryDirectory() as raw:
        directory = Path(raw)
        source = directory / "rangefinder_check.cpp"
        executable = directory / "rangefinder_check"
        source.write_text(textwrap.dedent(program), encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", source, "-o", executable],
            check=True,
        )
        subprocess.run([executable], check=True)
    print("SIM_JSON rangefinder mapping passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
