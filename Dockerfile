FROM ubuntu:24.04

RUN apt update
RUN apt install -y curl just
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

COPY . .
RUN just setup

RUN apt install -y git ffmpeg xvfb xterm
