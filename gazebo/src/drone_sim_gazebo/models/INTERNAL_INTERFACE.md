# Models Internal Interface

`model_uri_for_vehicle(vehicle_id)` maps the Phase 3 vehicle identity `iris`
to the conventional local Gazebo URI `model://iris_phase3`. Every other
vehicle is rejected. The mapping never produces a remote URI and assumes the
server sets `GZ_SIM_RESOURCE_PATH` to the resolver's absolute `resources/models`
directory.

This package owns identity-to-resource naming only. Resource-tree validation
and checksums belong to `worlds`; physics, ROS publication, and server control
belong to their respective sibling packages.
