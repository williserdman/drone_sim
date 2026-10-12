"""Fixed-sequence caller of the shared drone operation interface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .mission_plan import (
    Condition,
    ConditionalGroup,
    ConditionResult,
    MissionPlan,
    MissionStep,
    PlanItem,
)
from .operations import DroneOperations


class ConditionEvaluator(Protocol):
    def evaluate(self, condition: Condition) -> bool | ConditionResult: ...


@dataclass
class _Cursor:
    items: tuple[PlanItem, ...]
    index: int
    path: str


class ConfiguredMission:
    def __init__(self, plan: MissionPlan, operations: DroneOperations, *,
                 conditions: ConditionEvaluator | None = None, emit=None) -> None:
        self.plan = plan
        self.operations = operations
        self.conditions = conditions
        self._emit = emit or (lambda _event, _stamp, _fields: None)
        self.state = "running"
        self.error = ""
        self.step_index = 0
        self.operation_id: str | None = None
        self._cursors = [_Cursor(plan.steps, 0, "steps")]

    def _fail(self, error: str) -> None:
        self.state = "failed"
        self.error = error

    @staticmethod
    def _condition_fields(condition: Condition) -> dict[str, object]:
        if condition.kind == "payload_eligible":
            return {condition.kind: condition.payload_id}
        return {condition.kind: True}

    def _evaluate_condition(self, condition: Condition) -> ConditionResult:
        if self.conditions is None:
            raise RuntimeError("conditional plan requires an eligibility evaluator")
        result = self.conditions.evaluate(condition)
        if type(result) is bool:
            if condition.kind == "payload_eligible":
                subject = f"payload {condition.payload_id}"
            else:
                subject = "any payload"
            return ConditionResult(result, f"{subject} is eligible" if result else f"{subject} is not eligible")
        if not isinstance(result, ConditionResult):
            raise RuntimeError("eligibility evaluator returned an invalid decision")
        if type(result.matched) is not bool or not isinstance(result.reason, str) or not result.reason.strip():
            raise RuntimeError("eligibility decision requires a boolean result and clear reason")
        return result

    def _next_step(self, timestamp_ns: int) -> MissionStep | None:
        while self._cursors:
            cursor = self._cursors[-1]
            if cursor.index == len(cursor.items):
                self._cursors.pop()
                continue
            item_index = cursor.index
            cursor.index += 1
            self.step_index += 1
            item = cursor.items[item_index]
            if isinstance(item, MissionStep):
                return item
            assert isinstance(item, ConditionalGroup)
            result = self._evaluate_condition(item.when)
            item_path = f"{cursor.path}[{item_index}]"
            if result.matched:
                self._cursors.append(_Cursor(item.steps, 0, f"{item_path}.steps"))
            else:
                self._emit("conditional_group_skipped", timestamp_ns, {
                    "path": item_path,
                    "condition": self._condition_fields(item.when),
                    "reason": result.reason,
                })
        return None

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
            self.operation_id = None
        try:
            step = self._next_step(timestamp_ns)
        except Exception as error:
            self._fail(f"condition evaluation failed: {error}")
            return
        if step is None:
            self.state = "succeeded"
            return
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
