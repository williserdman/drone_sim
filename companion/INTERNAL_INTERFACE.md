# Companion Internal Interface

## Current interface

No same-process sibling interface exists in the initial scaffold.

## Future seams

Mission, vision, and vehicle-adapter submodules may use language-native imports. Their documented interfaces must preserve `run_id`, source frame identity, and simulation timestamps, and must not expose direct Gazebo control.

