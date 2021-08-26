FROM ros:noetic
ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip && \
  : "gstreamer dependencies" && \
  apt-get install -y -qq \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev && \
  : "libuvc dependencies" && \
  apt-get install -y -qq \
    libusb-1.0-0-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG NUM_THREADS=1

RUN set -x && \
  apt-get update -y -qq && \
  : "install ROS packages" && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone https://github.com/ricohapi/libuvc-theta.git && \
  cd libuvc-theta && \
  mkdir build && \
  cd build && \
  cmake -DCMAKE_BUILD_TYPE=Release .. && \
  make && \
  make install && \
  ldconfig && \
  cd ../.. && \
  rm -rf libuvc-theta

RUN set -x && \
  apt-get update -y -qq && \
  : "ci dependencies" && \
  apt-get install -y -qq \
    ccache \
    clang-format-6.0 && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN set -x && \
  apt-get update -y -qq && \
  : "dev dependencies" && \
  apt-get install -y -qq \
    rsync && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws
COPY . /catkin_ws/src/theta_driver

RUN set -x && \
  : "build ROS packages" && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  catkin_make -j${NUM_THREADS}"

RUN set -x && \
  sh -c "echo 'source /opt/ros/${ROS_DISTRO}/setup.bash\nsource /catkin_ws/devel/setup.bash' >> ~/.bashrc"

CMD ["bash"]
