"""Approved Gazebo and private ROS topic names for each selected world."""


PAYLOAD_IDS = (2, 3, 4)
_WORLDS = {"phase3_foundation", "vertical_descent", "competition_mission"}
_ONBOARD_CAMERA_TOPIC = "/gazebo/private/camera/onboard/image"
_COMPETITION_ONBOARD_CAMERA_TOPIC = (
    "/gazebo/private/camera/competition_onboard/image"
)
_OBSERVER_CAMERA_TOPIC = "/gazebo/private/camera/observer/image"


def _world(world_name: str) -> str:
    if world_name not in _WORLDS:
        raise ValueError("world_name must identify an approved local world")
    return world_name


def contact_topic_for_world(world_name: str) -> str:
    world_name = _world(world_name)
    if world_name == "competition_mission":
        return "/gazebo/private/iris/contact"
    return (
        f"/world/{world_name}/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )


def camera_topics_for_world(world_name: str) -> tuple[str, str]:
    world_name = _world(world_name)
    onboard = (
        _COMPETITION_ONBOARD_CAMERA_TOPIC
        if world_name == "competition_mission"
        else _ONBOARD_CAMERA_TOPIC
    )
    return onboard, _OBSERVER_CAMERA_TOPIC


def private_payload_topic(aruco_id: int, suffix: str) -> str:
    if aruco_id not in PAYLOAD_IDS:
        raise ValueError("aruco_id must identify an approved competition payload")
    if suffix not in {"pose", "contact_state", "command", "joint_state", "result"}:
        raise ValueError("suffix must identify an approved private payload topic")
    return f"/gazebo/private/payload_{aruco_id}/{suffix}"


def private_publisher_topics_for_world(world_name: str) -> tuple[str, ...]:
    world_name = _world(world_name)
    topics = (
        "/gazebo/private/clock",
        *camera_topics_for_world(world_name),
        "/gazebo/private/iris/odometry",
        contact_topic_for_world(world_name),
    )
    if world_name != "competition_mission":
        return topics
    competition = ["/gazebo/private/range/downward"]
    for aruco_id in PAYLOAD_IDS:
        competition.extend(
            (
                private_payload_topic(aruco_id, "pose"),
                private_payload_topic(aruco_id, "contact_state"),
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
        *camera_topics_for_world(world_name),
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
                f"/gazebo/private/payload/{aruco_id}/contact_state",
                f"/gazebo/private/payload/{aruco_id}/joint_state",
                f"/gazebo/private/payload/{aruco_id}/result",
            )
        )
    return (*topics, *competition)
