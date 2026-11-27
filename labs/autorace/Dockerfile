FROM ros:jazzy-ros-base@sha256:103b6c417973394659af6804d4917cd489b91e5ed22eb7bef263025a3bed2410
RUN sed -i 's|http://ports.ubuntu.com|https://ports.ubuntu.com|' /etc/apt/sources.list.d/ubuntu.sources \
    && apt-get -o Acquire::Retries=4 -o APT::Update::Error-Mode=any update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-robot-state-publisher ros-jazzy-xacro \
    ros-jazzy-rviz2 ros-jazzy-ament-lint-auto \
    python3-opencv python3-pytest python3-matplotlib python3-yaml \
    xvfb x11-utils imagemagick libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*
ENV LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /root/autorace_ws
