FROM ubuntu:24.04

RUN apt update
RUN apt install -y curl just sudo
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

COPY . .
RUN just setup

RUN apt install -y git ffmpeg xvfb xterm

RUN mkdir -p /usr/local/lib/gz-unitree/

RUN just install
RUN just setup-test

RUN useradd --create-home --shell /bin/bash --groups sudo --password gzrun gzrun 