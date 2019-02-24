#       _   _   _    ____      ____    _   _  ____ _        _
#      | | | | / \  / ___|    / __ \  | | | |/ ___| |      / \
#      | | | |/ _ \ \___ \   / / _` | | | | | |   | |     / _ \
#      | |_| / ___ \ ___) | | | (_| | | |_| | |___| |___ / ___ \
#       \___/_/   \_\____/   \ \__,_|  \___/ \____|_____/_/   \_\
#                             \____/

FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt-get update     -qq                                                  && \
    apt-get install -y -qq curl                                             && \
    apt-get install -y -qq                                                     \
    git                                                                        \
    gnupg                                                                      \
    gnupg2

# Add apt-get repositories containing required packages. #######################
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb http://deb.debian.org/debian testing main" \
  | tee /etc/apt/sources.list.d/python3_6.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt " \
  "stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

# Install apt dependencies. ####################################################
RUN apt-get update -qq                                                      && \
    apt-get upgrade -y -qq                                                  && \
    apt-get install -y                                                         \
    bmon                                                                       \
    build-essential                                                            \
    ca-certificates-java                                                       \
    check                                                                      \
    clang-3.9                                                                  \
    clang-format-7                                                             \
    cmake                                                                      \
    g++                                                                        \
    git                                                                        \
    google-perftools                                                           \
    gphoto2                                                                    \
    gphotofs                                                                   \
    htop                                                                       \
    libc++-dev                                                                 \
    libc++abi-dev                                                              \
    lib32z1-dev                                                                \
    libc6-dev-i386                                                             \
    libgoogle-perftools-dev                                                    \
    librsync-dev                                                               \
    libssl-dev                                                                 \
    lsb-release                                                                \
    libtool                                                                    \
    libyajl-dev                                                                \
    libz-dev                                                                   \
    make                                                                       \
    nmap                                                                       \
    openjdk-11-jdk                                                             \
    pkg-config                                                                 \
    protobuf-compiler                                                          \
    python2.7                                                                  \
    python2.7-lib                                                              \
    python2.7-dev                                                              \
    python3.7                                                                  \
    python3.7-dev                                                              \
    python3.7-distutils                                                        \
    python-matplotlib                                                          \
    python-empy                                                                \
    python-jinja2                                                              \
    screen                                                                     \
    sudo                                                                       \
    tmux                                                                       \
    uthash-dev                                                                 \
    unzip                                                                      \
    vim

# Install pip and python dependencies. #########################################
RUN curl https://bootstrap.pypa.io/get-pip.py | python2.7
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.7
RUN pip2.7 install --upgrade pip                                            && \
    pip3.7 install --upgrade pip

# Set up workspace. ############################################################
# Fix xlocale.h renamed to locale.h
# https://bugs.launchpad.net/ubuntu/+source/libc++/+bug/1725858
RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

RUN useradd -ms /bin/bash uas
RUN echo 'uas:spinny' | chpasswd
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

# User env setup. ##############################################################
RUN mkdir /external_libs

RUN apt-get install -y openssh-client openssh-server

USER uas
RUN mkdir -p /home/uas/code_env
ENV WORKSPACE /home/uas/code_env WORKDIR /home/uas/code_env

WORKDIR /home/uas

# Install python requirements. #################################################
RUN pip2.7 install --user future
RUN git clone https://github.com/intel/mavlink-router.git
WORKDIR /home/uas/mavlink-router
RUN git submodule update --init --recursive
RUN ./autogen.sh && ./configure CFLAGS='-g -O2' \
    --sysconfdir=/etc \
    --localstatedir=/var \
    --libdir=/usr/lib64 \
    --prefix=/usr
RUN make
RUN sudo make install

RUN pip2.7 install --user                                                      \
  future==0.16.0                                                               \
  MAVProxy==1.6.1                                                              \
  pymavlink==2.2.4                                                             \
  pyserial==3.4                                                                \
  websocket-client==0.44.0                                                     \
  dronekit==2.9.1                                                              \
  dronekit-sitl==3.2.0                                                         \
  Pillow==4.3.0                                                                \
  socketIO-client==0.7.2                                                       \
  python-socketio==1.8.1                                                       \
  Flask==0.12.2                                                                \
  Flask-SocketIO==2.9.2                                                        \
  futures==3.1.1                                                               \
  socketIO-client==0.7.2                                                       \
  numpy==1.16                                                                  \
  shap==0.23.0                                                                 \
  empy==3.3.2                                                                  \
  toml==0.9.4                                                                  \
  eventlet==0.23.0                                                             \
  protobuf==3.5.2.post1                                                        \
  pyzmq==17.0.0                                                                \
  zmq==0.0.0

RUN rm -rf /home/uas/mavlink-router

# Use a stable version of bazel. ###############################################
WORKDIR /tmp
USER root
RUN curl -L https://github.com/bazelbuild/bazel/releases/download/0.22.0/bazel_0.22.0-linux-x86_64.deb > bazel_0.22.0-linux-x86_64.deb
RUN apt-get install -y bash-completion
RUN dpkg -i bazel_0.22.0-linux-x86_64.deb

#RUN echo "source /home/uas/.bazel/bin/bazel-complete.bash" > /home/uas/.bashrc

# ROS dependencies.
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y ros-melodic-ros-base ros-melodic-mavros

RUN printf "source /opt/ros/melodic/setup.bash" > /home/uas/.bashrc

WORKDIR /tmp
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
  chmod +x install_geographiclib_datasets.sh && \
  ./install_geographiclib_datasets.sh

USER uas
RUN ssh-keygen -t rsa -N '' -f ~/.ssh/id_rsa
USER root

# Set up working directory and permissions for later scripts to use. ###########
WORKDIR /home/uas/code_env

ENV ROS_MASTER_URI http://0.0.0.0:11311

EXPOSE 8085/udp
EXPOSE 8086/udp
EXPOSE 8090/udp
EXPOSE 8091/udp
