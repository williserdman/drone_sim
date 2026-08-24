.PHONY: test test-unit test-foundation test-phase2 inspect-phase3

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test-phase2:
	docker compose --profile phase2 build
	DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v

inspect-phase3:
	test -n "$(RUN_DIRECTORY)"
	uv run python -m artifacts.acceptance "$(RUN_DIRECTORY)" --rules-path scorekeeper/rules/descent_v1.json $(if $(filter 1 true yes,$(REQUIRE_MAXIMUM_SCORE)),--require-maximum-score,)

test: test-unit test-foundation test-phase2
