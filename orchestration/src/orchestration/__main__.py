"""Allow ``python -m orchestration`` to invoke the operator CLI."""

from .cli import main


if __name__ == "__main__":
    raise SystemExit(main())
