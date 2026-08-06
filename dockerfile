FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    python3 \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

COPY scripts/requirements.txt /tmp/requirements.txt

RUN uv pip install --system --break-system-packages -r /tmp/requirements.txt

WORKDIR /project

CMD ["tail", "-f", "/dev/null"]
