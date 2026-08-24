.PHONY: test test-unit test-foundation test-phase2

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test-phase2:
	docker compose --profile phase2 build
	DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v

test: test-unit test-foundation test-phase2
