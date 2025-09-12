FROM ubuntu:24.04

RUN apt update
RUN apt install -y curl just sudo
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
RUN source $HOME/.local/bin/env

COPY . .
RUN just setup

RUN apt install -y git ffmpeg xvfb xterm

RUN mkdir -p /usr/local/lib/gz-unitree/

RUN just install
RUN just setup-test
