# DontFall M4 (Zephyr) build container: west + Zephyr SDK (arm only).
#
# Built by scripts/init.sh. The SDK version matches meta-zephyr's
# zephyr-sdk-native in the DontFall Yocto build, so dev and release use the
# same compiler.
FROM ubuntu:22.04

ARG UID=1000
ARG GID=1000
ARG USERNAME=zephyr
ARG ZEPHYR_SDK_VERSION=0.16.9
# Python requirements are taken from the pinned Zephyr commit. Passed as a
# build arg from west.yml by scripts/lib.sh - no default, so a manual
# 'docker build' without it fails loudly instead of silently drifting.
ARG ZEPHYR_REV

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        git cmake ninja-build gperf device-tree-compiler wget curl xz-utils \
        file make gcc g++ python3 python3-dev python3-pip python3-setuptools \
        python3-wheel ca-certificates openssh-client rsync locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

RUN curl -fsSL "https://raw.githubusercontent.com/zephyrproject-rtos/zephyr/${ZEPHYR_REV}/scripts/requirements-base.txt" \
        -o /tmp/requirements-base.txt \
    && pip3 install --no-cache-dir west -r /tmp/requirements-base.txt \
    && rm /tmp/requirements-base.txt

# micro-ROS: micro_ros_zephyr_module builds libmicroros with colcon during
# the Zephyr build. Pinned so the container stays reproducible; empy and
# lark match ROS Jazzy's reference platform (Ubuntu 24.04).
RUN pip3 install --no-cache-dir \
        colcon-common-extensions==0.3.0 catkin_pkg==1.1.1 lark==1.1.9 empy==3.3.4

RUN cd /opt \
    && wget -q "https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz" \
    && tar xf "zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz" \
    && rm "zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz" \
    && "/opt/zephyr-sdk-${ZEPHYR_SDK_VERSION}/setup.sh" -t arm-zephyr-eabi -c
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZEPHYR_SDK_VERSION} \
    ZEPHYR_TOOLCHAIN_VARIANT=zephyr

# Same UID/GID as the host user so build output in the bind-mounted
# workspace keeps the right ownership.
RUN getent group "$GID" >/dev/null || groupadd -g "$GID" "$USERNAME"; \
    useradd -m -s /bin/bash -u "$UID" -g "$GID" "$USERNAME"

USER $USERNAME
CMD ["bash"]
