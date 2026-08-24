"""Keep src-layout packages authoritative during importlib-mode collection."""

# Pytest otherwise synthesizes top-level ``artifacts`` and ``orchestration``
# namespace packages from their test paths before importing the real src-layout
# packages.  Preloading them preserves independent same-named test modules.
from importlib import import_module


for package in ("artifacts", "orchestration"):
    try:
        import_module(package)
    except ModuleNotFoundError as error:
        if error.name != package:
            raise
