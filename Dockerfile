FROM ubuntu:latest

# Prevent interactive prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Update and install system dependencies
RUN apt-get update && apt-get install -y \
    bash \
    vim \
    curl \
    software-properties-common \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libffi-dev \
    git \
    openssl \
    wget \
    zlib1g-dev \
    libgl1 \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python and pip
RUN wget "https://www.python.org/ftp/python/3.9.0/Python-3.9.0.tgz"
RUN tar xvf Python-3.9.0.tgz
RUN cd Python-3.9.0 && ./configure --enable-optimizations && make && make install

RUN ln -s /usr/local/bin/pip3.9 /usr/local/bin/pip

# Install Python libraries
RUN pip install --no-cache-dir \
    numpy \
    opencv-python \
    pyrealsense2 \
    ultralytics \
    argparse \
    random2

# Set working directory
WORKDIR /app


RUN git clone https://github.com/DamianTadzik/embedded-systems-2-omni-robot.git



CMD ["/bin/bash"]
