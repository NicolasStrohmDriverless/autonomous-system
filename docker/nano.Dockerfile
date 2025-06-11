FROM ros:humble-ros-base-jammy AS base

ARG USERNAME=strohmo
ARG UID=1000
ARG GID=$UID

# Install some dependencies packages
RUN apt update -q \
    && apt upgrade -q -y \
    && apt-get install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    clang-12 \
    clang++-12 \ 
    libc++-12-dev \  
    libc++abi-12-dev \
    unzip \
    libpcl-dev \
    wget \
    doxygen \
    ros-humble-sick-scan-xd \
    ros-humble-depthai-ros \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g "$GID" "$USERNAME" \
    && useradd -lm -u "$UID" -g "$USERNAME" -s /bin/bash "$USERNAME" \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/"$USERNAME"/autonomous-system
WORKDIR /home/$USERNAME/autonomous-system

# Copy configuration files
RUN echo 'source /opt/ros/'"$ROS_DISTRO"'/setup.bash' >> /home/"$USERNAME"/.bashrc \
    && echo 'source /home/'"$USERNAME"'/autonomous-system/install/setup.bash' >> /home/"$USERNAME"/.bashrc \
    && echo 'export PATH="/home/'"$USERNAME"'/.local/bin:\$PATH"' \
    && pip3 install mkdocs \
    && pip3 install mkdocs-material \
    && pip3 install mkdoxy

# Setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
