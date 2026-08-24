# Models External Interface

The `models` package has no repository-wide runtime interface. Gazebo model
resources remain private, image-owned inputs to the `gazebo-runtime` service;
other modules must use the public ROS 2 physical-truth outputs instead of
loading model files.
