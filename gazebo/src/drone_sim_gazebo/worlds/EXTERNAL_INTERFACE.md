# Worlds External Interface

The `worlds` package has no ROS, Gazebo Transport, subprocess, or durable-file
interface outside the Gazebo module. Other repository modules consume public
physical truth from `gazebo-runtime`; they do not load the world directly.
