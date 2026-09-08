.PHONY: test test-unit test-foundation test-phase2 inspect-competition

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test-phase2:
	docker compose --profile phase2 build
	DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v

inspect-competition:
	test -n "$(RUN_DIRECTORY)"
	uv run python scripts/inspect_competition_run.py "$(RUN_DIRECTORY)"

test: test-unit test-foundation test-phase2
