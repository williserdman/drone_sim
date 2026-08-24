"""Stable mapping from repository vehicle IDs to local Gazebo model URIs."""


def model_uri_for_vehicle(vehicle_id: str) -> str:
    """Return the only image-owned model URI supported by Phase 3."""
    if vehicle_id != "iris":
        raise ValueError("Phase 3 supports only vehicle iris")
    return "model://iris_phase3"
