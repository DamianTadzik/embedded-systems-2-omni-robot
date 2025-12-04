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
    openssl \
    wget \
    zlib1g-dev \
    libgl1 \
    libusb-1.0-0-dev \
    zip \
    autoconf \
    autoconf-archive \
    automake \
    libtool \
    pkg-config \
    libudev-dev \
    mosquitto \
    && rm -rf /var/lib/apt/lists/*

# Install Python and pip
RUN wget "https://www.python.org/ftp/python/3.9.0/Python-3.9.0.tgz" \
    && tar xvf Python-3.9.0.tgz \
    && cd Python-3.9.0 \
    && ./configure --enable-optimizations \
    && make \
    && make install \
    && ln -s /usr/local/bin/pip3.9 /usr/local/bin/pip

# Install Python libraries
RUN pip install --upgrade pip \
    && pip install --no-cache-dir \
    numpy \
    opencv-python \
    pyrealsense2 \
    ultralytics[export] \
    argparse \
    random2 \
    lap \
    paho-mqtt

# The above ultralytics installation will install Torch and Torchvision.
# However, these two packages installed via pip are not compatible with the Jetson platform, which is based on ARM64 architecture.
# Therefore, we need to manually install a pre-built PyTorch pip wheel and compile or install Torchvision from source.
RUN pip install --no-cache-dir \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl

# Install cuSPARSELt to fix a dependency issue with torch 2.5.0
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    && apt-get -y install libcusparselt0 libcusparselt-dev

# Install onnxruntime-gpu
RUN pip install --no-cache-dir \
    https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.19.0-cp310-cp310-linux_aarch64.whl

# Install RealSense SDK
RUN git clone https://github.com/Microsoft/vcpkg.git \
    && cd vcpkg \
    && ./bootstrap-vcpkg.sh \
    && ./vcpkg integrate install \
    && ./vcpkg install realsense2

# Set working directory
WORKDIR /app

# Download the code
RUN git clone https://github.com/DamianTadzik/embedded-systems-2-omni-robot.git

# Start Mosquitto broker
RUN cd /app/embedded-systems-2-omni-robot/L1 \
    && nohup mosquitto -v -c my_mosquitto.conf &

CMD ["/bin/bash"]
