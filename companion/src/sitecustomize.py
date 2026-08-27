"""Python 3.12 aliases required by the pinned DroneKit 2.9.2 import."""

import collections
import collections.abc
import inspect


if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]

if not hasattr(inspect, "getargspec"):
    inspect.getargspec = inspect.getfullargspec  # type: ignore[attr-defined]
