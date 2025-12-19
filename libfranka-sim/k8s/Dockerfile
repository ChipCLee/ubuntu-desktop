FROM --platform=$TARGETPLATFORM python:3.12-slim

RUN mkdir /src
COPY . /src
WORKDIR /src

ENV DEBIAN_FRONTEND=noninteractive \
    UV_PYTHON=python3 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PATH="/root/.local/bin:${PATH}" \
    UV_NO_PROGRESS=1 \
    HEADLESS=1

# Install minimal build tools and OpenGL libraries
RUN apt-get update && apt-get install -y \
    curl \
    ca-certificates \
    build-essential \
    libgl1 \
    vim-tiny \
    libglib2.0-0 \
    libx11-6 \
    libxrender1 \
    libxrandr2 libxcursor1 libxinerama1 libxi6 libxxf86vm1 \
    libglu1-mesa \
    libgles2-mesa \
    && rm -rf /var/lib/apt/lists/*

# Install uv and Python environment
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
RUN uv venv
RUN uv sync --python .venv/bin/python
RUN uv pip install -e .

EXPOSE 1337
CMD ["/src/.venv/bin/python", "-m", "franka_sim.run_server"]
