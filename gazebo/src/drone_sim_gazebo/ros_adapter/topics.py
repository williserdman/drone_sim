"""Approved private Gazebo topic names derived from the selected world."""


def contact_topic_for_world(world_name: str) -> str:
    if world_name not in {"phase3_foundation", "vertical_descent"}:
        raise ValueError("world_name must identify an approved local world")
    return (
        f"/world/{world_name}/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )
