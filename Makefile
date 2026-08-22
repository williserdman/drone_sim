.PHONY: test test-unit test-foundation

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test: test-unit test-foundation
