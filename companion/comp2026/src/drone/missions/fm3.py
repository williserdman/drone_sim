"""Inactive historical FM3 route.

The former implementation combined unimplemented payload actions with a second
flight-control sequence. It is kept importable so legacy imports fail at an
explicit call boundary, but it is not an executable mission path.
"""


class InactiveMissionError(RuntimeError):
    """Raised whenever the quarantined FM3 route is invoked."""


_INACTIVE_MESSAGE = (
    "fm3 is inactive: payload pickup/drop behavior and supervisor integration "
    "are undefined"
)


def pickup() -> None:
    raise InactiveMissionError(_INACTIVE_MESSAGE)


def drop() -> None:
    raise InactiveMissionError(_INACTIVE_MESSAGE)


def fm3(*_args, **_kwargs) -> None:
    raise InactiveMissionError(_INACTIVE_MESSAGE)
