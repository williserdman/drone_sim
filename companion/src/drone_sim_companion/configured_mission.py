"""Fixed-sequence caller of the shared drone operation interface."""

from __future__ import annotations

from .mission_plan import MissionPlan
from .operations import DroneOperations


class ConfiguredMission:
    def __init__(self, plan: MissionPlan, operations: DroneOperations) -> None:
        self.plan = plan
        self.operations = operations
        self.state = "running"
        self.error = ""
        self.step_index = 0
        self.operation_id: str | None = None

    def tick(self, timestamp_ns: int) -> None:
        self.operations.tick(timestamp_ns)
        if self.state != "running":
            return
        if self.operation_id is not None:
            operation = self.operations.operation_status(self.operation_id)
            if operation.state == "running":
                return
            if operation.state != "succeeded":
                self.state = operation.state
                self.error = operation.error
                return
            self.step_index += 1
            self.operation_id = None
        if self.step_index == len(self.plan.steps):
            self.state = "succeeded"
            return
        step = self.plan.steps[self.step_index]
        self.operation_id = self.operations.start(step.tool, dict(step.args), timeout_sim_s=step.timeout_sim_s)
        operation = self.operations.operation_status(self.operation_id)
        if operation.state in {"failed", "cancelled"}:
            self.state = operation.state
            self.error = operation.error

    def abort(self, reason: str = "operator abort") -> None:
        self.operations.abort(reason)
        if self.state == "running":
            self.state = "cancelled"
            self.error = reason
