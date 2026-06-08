FROM osrf/ros:foxy-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV AUTOCAR_WORKSPACE=/workspace
ENV AUTOCAR_SHARED_DIR=/workspace/shared
ENV AUTOCAR_LOG_DIR=/workspace/runtime/logs
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1
ENV VNC_RESOLUTION=1280x800

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    curl \
    dbus-x11 \
    gazebo11 \
    git \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    mesa-utils \
    novnc \
    procps \
    psmisc \
    python3-colcon-common-extensions \
    python3-pip \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-joint-state-publisher \
    ros-foxy-slam-toolbox \
    ros-foxy-vision-msgs \
    websockify \
    x11vnc \
    xfce4 \
    xfce4-terminal \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY container/api/requirements.txt /tmp/api-requirements.txt
RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir -r /tmp/api-requirements.txt

COPY src/AutoCarROS2 /workspace/src/AutoCarROS2
RUN printf "empy<4\n" > /tmp/ros-foxy-constraints.txt \
    && python3 -m pip install --no-cache-dir --ignore-installed "empy>=3.3.4,<4" \
    && python3 -m pip install --no-cache-dir -c /tmp/ros-foxy-constraints.txt -r /workspace/src/AutoCarROS2/requirements.txt \
    && source /opt/ros/foxy/setup.bash \
    && colcon build --symlink-install

COPY container/api /workspace/api
COPY container/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && mkdir -p /workspace/shared /workspace/runtime/logs

EXPOSE 6080 8001

ENTRYPOINT ["/entrypoint.sh"]
