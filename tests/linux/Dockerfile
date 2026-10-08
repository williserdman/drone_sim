FROM python:3.12-slim@sha256:05cda9777409a9c3ffddd94a4c476b79f0769a0b4857f0c7ed9226b6800b0d6f

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        ca-certificates \
        docker-compose \
        ffmpeg \
        git \
        zstd \
    && rm -rf /var/lib/apt/lists/* \
    && python -m pip install --no-cache-dir "uv==0.9.28" \
    && printf '%s\n' \
        '#!/bin/sh' \
        'if [ "${1:-}" = compose ]; then shift; exec docker-compose "$@"; fi' \
        'echo "Docker daemon access is unavailable in the Linux unit-test harness" >&2' \
        'exit 125' \
        > /usr/local/bin/docker \
    && chmod 0755 /usr/local/bin/docker

ENV PYTHONDONTWRITEBYTECODE=1 \
    UV_CACHE_DIR=/linux-test/uv-cache \
    UV_LINK_MODE=copy \
    UV_PROJECT_ENVIRONMENT=/linux-test/venv

WORKDIR /
