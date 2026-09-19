"""Approved Gazebo and private ROS topic names for each selected world."""


PAYLOAD_IDS = (2, 3, 4)
_PAYLOAD_IDS_BY_WORLD = {
    "competition_mission": PAYLOAD_IDS,
    "search_delivery": (3,),
}
_WORLDS = {
    "phase3_foundation",
    "vertical_descent",
    "competition_mission",
    "search_delivery",
}
_PAYLOAD_WORLDS = frozenset(_PAYLOAD_IDS_BY_WORLD)
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
    if world_name in _PAYLOAD_WORLDS:
        return "/gazebo/private/iris/contact"
    return (
        f"/world/{world_name}/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )


def camera_topics_for_world(world_name: str) -> tuple[str, str]:
    world_name = _world(world_name)
    onboard = (
        _COMPETITION_ONBOARD_CAMERA_TOPIC
        if world_name in _PAYLOAD_WORLDS
        else _ONBOARD_CAMERA_TOPIC
    )
    return onboard, _OBSERVER_CAMERA_TOPIC


def private_payload_topic(aruco_id: int, suffix: str) -> str:
    if aruco_id not in PAYLOAD_IDS:
        raise ValueError("aruco_id must identify an approved competition payload")
    if suffix not in {"pose", "contact_state", "command", "joint_state", "result"}:
        raise ValueError("suffix must identify an approved private payload topic")
    return f"/gazebo/private/payload_{aruco_id}/{suffix}"


def payload_ids_for_world(world_name: str) -> tuple[int, ...]:
    world_name = _world(world_name)
    return _PAYLOAD_IDS_BY_WORLD.get(world_name, ())


def private_publisher_topics_for_world(world_name: str) -> tuple[str, ...]:
    world_name = _world(world_name)
    topics = (
        "/gazebo/private/clock",
        *camera_topics_for_world(world_name),
        "/gazebo/private/iris/odometry",
        contact_topic_for_world(world_name),
    )
    payload_ids = payload_ids_for_world(world_name)
    if not payload_ids:
        return topics
    competition = ["/gazebo/private/range/downward"]
    for aruco_id in payload_ids:
        competition.extend(
            (
                private_payload_topic(aruco_id, "pose"),
                private_payload_topic(aruco_id, "contact_state"),
                private_payload_topic(aruco_id, "joint_state"),
                private_payload_topic(aruco_id, "result"),
            )
        )
    return (*topics, *competition)


def readiness_publisher_topics_for_world(world_name: str) -> tuple[str, ...]:
    """Private ROS publishers available before the world is released.

    Gazebo-to-ROS bridges advertise lazily after their first native message.
    Flight worlds are paused at startup, so native Gazebo Transport discovery
    and supervised bridge children establish that side of readiness instead.
    """
    world_name = _world(world_name)
    if world_name in {"vertical_descent", *_PAYLOAD_WORLDS}:
        return ()
    cameras = set(camera_topics_for_world(world_name))
    return tuple(
        topic
        for topic in private_publisher_topics_for_world(world_name)
        if topic not in cameras
    )


def recorder_topics_for_world(
    world_name: str, *, competition_evidence: bool
) -> tuple[str, ...]:
    _world(world_name)
    topics = (
        "/camera/onboard/image_raw",
        "/camera/onboard/frame_metadata",
        "/camera/observer/image_raw",
        "/camera/observer/frame_metadata",
        "/simulation/ground_truth",
    )
    if world_name in _PAYLOAD_WORLDS and competition_evidence:
        return (*topics, "/simulation/payload_state", "/competition/range/downward")
    return topics


def private_command_topics_for_world(world_name: str) -> tuple[str, ...]:
    world_name = _world(world_name)
    payload_ids = payload_ids_for_world(world_name)
    if not payload_ids:
        return ()
    return tuple(
        private_payload_topic(aruco_id, "command")
        for aruco_id in payload_ids
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
    payload_ids = payload_ids_for_world(world_name)
    if not payload_ids:
        return topics
    competition = ["/gazebo/private/range/downward"]
    for aruco_id in payload_ids:
        competition.extend(
            (
                f"/model/payload_{aruco_id}/pose",
                f"/gazebo/private/payload/{aruco_id}/contact_state",
                f"/gazebo/private/payload/{aruco_id}/joint_state",
                f"/gazebo/private/payload/{aruco_id}/result",
            )
        )
    return (*topics, *competition)
