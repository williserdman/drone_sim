.PHONY: test test-unit test-foundation test-phase2 inspect-competition

test-unit:
	uv run pytest orchestration/tests artifacts/tests tests/contracts -v

test-foundation:
	uv run pytest tests/integration/test_foundation_compose.py -v

test-phase2:
	env -u COMPOSE_FILE -u COMPOSE_ENV_FILES -u COMPOSE_PATH_SEPARATOR \
		-u COMPOSE_PROFILES -u COMPOSE_PROJECT_NAME -u COMPOSE_PROJECT_DIR \
		-u COMPOSE_PROJECT_DIRECTORY -u COMPOSE_DISABLE_ENV_FILE \
		COMPOSE_DISABLE_ENV_FILE=1 docker compose --file compose.yaml \
		--project-directory "$(CURDIR)" --profile phase2 build
	DRONE_SIM_PHASE2_IMAGES_BUILT=1 uv run pytest tests/integration/test_phase2_compose.py -v

inspect-competition:
	test -n "$(RUN_DIRECTORY)"
	uv run python scripts/inspect_competition_run.py "$(RUN_DIRECTORY)"

test: test-unit test-foundation test-phase2
