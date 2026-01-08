# kudos to https://github.com/pablomalvido/rmcp_ros2_humble_pkg/tree/main

FROM ghcr.io/open-rmf/rmf/rmf_demos:jazzy-rmf-latest

ARG USERNAME=admin
ARG UID=2000
ARG GID=2000

ENV ROS_DOMAIN_ID=5

RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    software-properties-common \
    python3-pip \
    xauth \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    gdb \
    git \
    nano \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    sudo \
    vim \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /rmf_demos_ws/install/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'if [ -f /home/'$USERNAME'/ros2_ws/install/setup.bash ]; then source /home/'$USERNAME'/ros2_ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc

CMD ["bash"]