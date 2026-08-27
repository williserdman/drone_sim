"""Approved Gazebo and private ROS topic names for each selected world."""


PAYLOAD_IDS = (2, 3, 4)
_WORLDS = {"phase3_foundation", "vertical_descent", "competition_mission"}
_COMMON_PRIVATE_PUBLISHERS = (
    "/gazebo/private/clock",
    "/gazebo/private/camera/onboard/image",
    "/gazebo/private/camera/observer/image",
    "/gazebo/private/iris/odometry",
)


def _world(world_name: str) -> str:
    if world_name not in _WORLDS:
        raise ValueError("world_name must identify an approved local world")
    return world_name


def contact_topic_for_world(world_name: str) -> str:
    world_name = _world(world_name)
    return (
        f"/world/{world_name}/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )


def private_payload_topic(aruco_id: int, suffix: str) -> str:
    if aruco_id not in PAYLOAD_IDS:
        raise ValueError("aruco_id must identify an approved competition payload")
    if suffix not in {"pose", "contacts", "command", "joint_state", "result"}:
        raise ValueError("suffix must identify an approved private payload topic")
    return f"/gazebo/private/payload_{aruco_id}/{suffix}"


def private_publisher_topics_for_world(world_name: str) -> tuple[str, ...]:
    world_name = _world(world_name)
    topics = (*_COMMON_PRIVATE_PUBLISHERS, contact_topic_for_world(world_name))
    if world_name != "competition_mission":
        return topics
    competition = ["/gazebo/private/range/downward"]
    for aruco_id in PAYLOAD_IDS:
        competition.extend(
            (
                private_payload_topic(aruco_id, "pose"),
                private_payload_topic(aruco_id, "contacts"),
                private_payload_topic(aruco_id, "joint_state"),
                private_payload_topic(aruco_id, "result"),
            )
        )
    return (*topics, *competition)


def private_command_topics_for_world(world_name: str) -> tuple[str, ...]:
    world_name = _world(world_name)
    if world_name != "competition_mission":
        return ()
    return tuple(
        private_payload_topic(aruco_id, "command")
        for aruco_id in PAYLOAD_IDS
    )


def gazebo_topics_for_world(world_name: str) -> tuple[str, ...]:
    """Return the Gazebo publishers required before server readiness."""
    world_name = _world(world_name)
    topics = (
        "/clock",
        "/gazebo/private/camera/onboard/image",
        "/gazebo/private/camera/observer/image",
        "/gazebo/private/iris/odometry",
        contact_topic_for_world(world_name),
    )
    if world_name != "competition_mission":
        return topics
    competition = ["/gazebo/private/range/downward"]
    for aruco_id in PAYLOAD_IDS:
        competition.extend(
            (
                f"/model/payload_{aruco_id}/pose",
                f"/gazebo/private/payload/{aruco_id}/contacts",
                f"/gazebo/private/payload/{aruco_id}/joint_state",
                f"/gazebo/private/payload/{aruco_id}/result",
            )
        )
    return (*topics, *competition)
