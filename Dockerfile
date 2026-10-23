# 23.0 is the minimum docker engine version that required to build.
# If your docker engine lower than this version, please upgrade it or manually enable the BuildKit.



# Sysroot source images for rat-develop-full.
# CI overrides these with digest-pinned references. For local builds, build
# rat-base for both architectures first and pass --build-arg explicitly;
# see docs/zh-cn/build_docker_image.md.
ARG SYSROOT_IMAGE_AMD64=rat-base:latest
ARG SYSROOT_IMAGE_ARM64=rat-base:latest

# Base container, provides a runtime environment
FROM ros:jazzy AS rat-base
ARG TARGETARCH

# Change bash as default shell instead of sh
SHELL ["/bin/bash", "-c"]

# Set timezone and non-interactive mode
ENV TZ=Asia/Shanghai \
    DEBIAN_FRONTEND=noninteractive

# If you are in China, you can use the following command to change the apt source to USTC mirror.
RUN sed -i -e 's|//archive.ubuntu.com|//mirrors.ustc.edu.cn|g' -e 's|//security.ubuntu.com|//mirrors.ustc.edu.cn|g' -e 's|//ports.ubuntu.com|//mirrors.ustc.edu.cn|g' /etc/apt/sources.list.d/ubuntu.sources

# Install tools and libraries.
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim wget curl unzip \
    zsh screen tmux \
    usbutils net-tools iputils-ping \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    ripgrep htop fzf \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    libceres-dev \
    ros-$ROS_DISTRO-rviz2 ros-$ROS_DISTRO-foxglove-bridge \
    ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-mavlink \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-osqp-vendor \
    lua5.4 liblua5.4-0 liblua5.4-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Mount rmcs source and install dependencies
# RUN --mount=type=bind,target=/rmcs_ws/src,source=rmcs_ws/src,readonly \
#     apt-get update && \
#     rosdep install --from-paths /rmcs_ws/src --ignore-src -r -y && \
#     apt-get clean && \
#     rm -rf /var/lib/apt/lists/* /tmp/*

# Developing container, works with devcontainer
FROM rat-base AS rat-dev
ARG TARGETARCH

# Install develop tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    libc6-dev gcc-14 g++-14 \
    cmake make ninja-build \
    openssh-client \
    lsb-release software-properties-common gnupg sudo \
    python3-colorama python3-dpkt \
    python3-pip websockify && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install Node.js 24 LTS (required by agent CLIs)
RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - && \
    apt-get install -y --no-install-recommends nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

# Install llvm-toolchain
ARG LLVM_VERSION=22
RUN mkdir -p /etc/apt/keyrings && \
    wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /etc/apt/keyrings/apt.llvm.org.gpg && \
    chmod 644 /etc/apt/keyrings/apt.llvm.org.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/apt.llvm.org.gpg] https://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
    > /etc/apt/sources.list.d/llvm.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    libomp-${LLVM_VERSION}-dev \
    clang-${LLVM_VERSION} clangd-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
    lldb-${LLVM_VERSION} lld-${LLVM_VERSION} llvm-${LLVM_VERSION} && \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/lldb lldb /usr/bin/lldb-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ar llvm-ar /usr/bin/llvm-ar-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/llvm-ranlib llvm-ranlib /usr/bin/llvm-ranlib-${LLVM_VERSION} 50 && \
    update-alternatives --install /usr/bin/ld.lld ld.lld /usr/bin/ld.lld-${LLVM_VERSION} 50 && \
    apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/*

# Install latest neovim
RUN case "${TARGETARCH}" in \
    amd64) nvim_arch=x86_64 ;; \
    arm64) nvim_arch=arm64 ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-${nvim_arch}.tar.gz && \
    rm -rf /opt/nvim && \
    tar -C /opt -xzf nvim-linux-${nvim_arch}.tar.gz && \
    mv /opt/nvim-linux-${nvim_arch} /opt/nvim && \
    rm nvim-linux-${nvim_arch}.tar.gz
ENV PATH="${PATH}:/opt/nvim/bin"

# Install latest stable cmake for user ubuntu
RUN case "${TARGETARCH}" in \
    amd64) cmake_arch=x86_64 ;; \
    arm64) cmake_arch=aarch64 ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    wget https://github.com/kitware/cmake/releases/download/v4.2.3/cmake-4.2.3-linux-${cmake_arch}.sh -O install.sh && \
    mkdir -p /opt/cmake/ && bash install.sh --skip-license --prefix=/opt/cmake/ --exclude-subdir && \
    rm install.sh

# Change user
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Precreate generic XDG-style parent directories for direct bind mounts under ubuntu's home.
RUN mkdir -p \
    /home/ubuntu/.agents \
    /home/ubuntu/.cache \
    /home/ubuntu/.config \
    /home/ubuntu/.local/share \
    /home/ubuntu/.local/state && \
    chown -R ubuntu:ubuntu /home/ubuntu/.agents /home/ubuntu/.cache /home/ubuntu/.config /home/ubuntu/.local
WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install oh my zsh, change theme to af-magic and setup environment of zsh
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" && \
    sed -i 's/ZSH_THEME=\"[a-z0-9\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc && \
    echo 'source ~/env_setup.zsh' >> ~/.zshrc

# Copy environment setup scripts
COPY --chown=1000:1000 .script/template/env_setup.bash env_setup.bash
COPY --chown=1000:1000 .script/template/env_setup.zsh env_setup.zsh

FROM --platform=linux/amd64 ${SYSROOT_IMAGE_AMD64} AS rat-sysroot-amd64
FROM --platform=linux/arm64 ${SYSROOT_IMAGE_ARM64} AS rat-sysroot-arm64

# Developing container with cross toolchains and opposite-arch sysroot.
FROM rat-dev AS rat-dev-full
ARG TARGETARCH

USER root

RUN apt-get update && \
    case "${TARGETARCH}" in \
    amd64) cross_triplet=aarch64-linux-gnu; cross_pkg_triplet=aarch64-linux-gnu ;; \
    arm64) cross_triplet=x86_64-linux-gnu; cross_pkg_triplet=x86-64-linux-gnu ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    apt-get install -y --no-install-recommends \
    "gcc-14-${cross_pkg_triplet}" "g++-14-${cross_pkg_triplet}" \
    "binutils-${cross_pkg_triplet}" && \
    update-alternatives --install "/usr/bin/${cross_triplet}-gcc" "${cross_triplet}-gcc" "/usr/bin/${cross_triplet}-gcc-14" 50 && \
    update-alternatives --install "/usr/bin/${cross_triplet}-g++" "${cross_triplet}-g++" "/usr/bin/${cross_triplet}-g++-14" 50 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg ca-certificates lsb-release && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends gz-harmonic cppzmq-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/*

RUN mkdir -p /opt/sysroots && \
    case "${TARGETARCH}" in \
    amd64) mkdir -p /opt/sysroots/arm64 ;; \
    arm64) mkdir -p /opt/sysroots/amd64 ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

RUN --mount=from=rat-sysroot-amd64,target=/mnt/sysroot-amd64,readonly \
    --mount=from=rat-sysroot-arm64,target=/mnt/sysroot-arm64,readonly \
    set -euo pipefail && \
    case "${TARGETARCH}" in \
    amd64) tar \
    --exclude='./dev/*' \
    --exclude='./proc/*' \
    --exclude='./sys/*' \
    --exclude='./run/*' \
    --exclude='./tmp/*' \
    -C /mnt/sysroot-arm64 -cf - . | tar -C /opt/sysroots/arm64 -xf - ;; \
    arm64) tar \
    --exclude='./dev/*' \
    --exclude='./proc/*' \
    --exclude='./sys/*' \
    --exclude='./run/*' \
    --exclude='./tmp/*' \
    -C /mnt/sysroot-amd64 -cf - . | tar -C /opt/sysroots/amd64 -xf - ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac

# Normalize the imported sysroot. Package postinst scripts (via update-alternatives,
# e.g. liblapack3/libblas3) create ABSOLUTE symlinks such as
#   usr/lib/aarch64-linux-gnu/liblapack.so.3 -> /etc/alternatives/...
# Absolute targets are resolved against the *host* root, not against --sysroot, so
# they dangle when cross-linking from the opposite architecture (ld: liblapack.so.3
# ... not found). Rewrite every absolute symlink as a relative one so that it
# resolves inside the sysroot.
RUN set -euo pipefail && \
    case "${TARGETARCH}" in \
    amd64) sysroot=/opt/sysroots/arm64 ;; \
    arm64) sysroot=/opt/sysroots/amd64 ;; \
    *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac && \
    find "$sysroot" -type l -print0 | while IFS= read -r -d '' link; do \
    target="$(readlink "$link")"; \
    case "$target" in /*) ;; *) continue ;; esac; \
    rel="$(realpath -m --relative-to="$(dirname "$link")" "$sysroot$target")"; \
    ln -sfn "$rel" "$link"; \
    done && \
    if [ -n "$(find "$sysroot" -type l -lname '/*' -print -quit)" ]; then \
    echo "Error: absolute symlinks remain under $sysroot:" >&2; \
    find "$sysroot" -type l -lname '/*' >&2; \
    exit 1; \
    fi && \
    echo "Normalized absolute symlinks under $sysroot"

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu