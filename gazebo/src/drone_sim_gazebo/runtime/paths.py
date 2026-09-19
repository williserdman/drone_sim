"""Image-owned runtime paths selected by the immutable world identity."""

from pathlib import Path


def bridge_config_for_world(world_name: str) -> Path:
    paths = {
        "phase3_foundation": Path("/etc/drone_sim/gazebo-bridge.yaml"),
        "vertical_descent": Path("/etc/drone_sim/gazebo-bridge-flight.yaml"),
        "competition_mission": Path(
            "/etc/drone_sim/gazebo-bridge-competition.yaml"
        ),
        "search_delivery": Path(
            "/etc/drone_sim/gazebo-bridge-search-delivery.yaml"
        ),
    }
    try:
        return paths[world_name]
    except KeyError as error:
        raise ValueError("world_name must identify an approved local world") from error
