.PHONY: test test-unit test-foundation test-phase2 inspect-phase3 inspect-competition

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test-phase2:
	docker compose --profile phase2 build
	DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v

inspect-phase3:
	test -n "$(RUN_DIRECTORY)"
	test -n "$(EXPECTED_SOURCE_REVISION)"
	test -n "$(EXPECTED_SOURCE_DIRTY)"
	test -n "$(ORCHESTRATION_IMAGE_DIGEST)"
	test -n "$(ARTIFACTS_IMAGE_DIGEST)"
	test -n "$(COMPANION_IMAGE_DIGEST)"
	test -n "$(ARDUPILOT_IMAGE_DIGEST)"
	test -n "$(GAZEBO_IMAGE_DIGEST)"
	test -n "$(ELECTROMAGNET_IMAGE_DIGEST)"
	test -n "$(SCOREKEEPER_IMAGE_DIGEST)"
	uv run python -m artifacts.acceptance "$(RUN_DIRECTORY)" --rules-path scorekeeper/rules/descent_v1.json --expected-source-revision "$(EXPECTED_SOURCE_REVISION)" --expected-source-dirty "$(EXPECTED_SOURCE_DIRTY)" --expected-image-digest "drone-sim-orchestration-runtime:phase2=$(ORCHESTRATION_IMAGE_DIGEST)" --expected-image-digest "drone-sim-artifacts-runtime:phase2=$(ARTIFACTS_IMAGE_DIGEST)" --expected-image-digest "drone-sim-companion-runtime:phase3=$(COMPANION_IMAGE_DIGEST)" --expected-image-digest "drone-sim-ardupilot-runtime:phase3=$(ARDUPILOT_IMAGE_DIGEST)" --expected-image-digest "drone-sim-gazebo-runtime:phase3=$(GAZEBO_IMAGE_DIGEST)" --expected-image-digest "drone-sim-electromagnet-runtime:phase3=$(ELECTROMAGNET_IMAGE_DIGEST)" --expected-image-digest "drone-sim-scorekeeper-runtime:phase3=$(SCOREKEEPER_IMAGE_DIGEST)" $(if $(filter 1 true yes,$(REQUIRE_MAXIMUM_SCORE)),--require-maximum-score,)

inspect-competition:
	test -n "$(RUN_DIRECTORY)"
	uv run python scripts/inspect_competition_run.py "$(RUN_DIRECTORY)"

test: test-unit test-foundation test-phase2
