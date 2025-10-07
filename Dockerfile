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

# Create user and setup permissions on /etc/sudoers
RUN useradd -m -s /bin/bash -N -u 1001 gzrun && \
    echo "gzrun ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    chmod g+w /etc/passwd 