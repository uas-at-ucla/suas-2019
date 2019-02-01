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
    aria2                                                                      \
    git                                                                        \
    gnupg                                                                      \
    gnupg2

# Install apt-fast for faster APT downloads. ###################################
WORKDIR /tmp
RUN git clone https://github.com/ilikenwf/apt-fast.git
RUN cp apt-fast/apt-fast /bin                                               && \
    chmod +x /bin/apt-fast                                                  && \
    cp apt-fast/apt-fast.conf /etc                                          && \
    echo "VERBOSE_OUTPUT=n" > /etc/apt-fast.conf

# Add apt-get repositories containing required packages. #######################
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN echo "deb http://deb.debian.org/debian testing main" \
  | tee /etc/apt/sources.list.d/python3_6.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt " \
  "stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

# Install apt dependencies. ####################################################
RUN apt-fast update -qq                                                     && \
    apt-fast upgrade -y -qq                                                 && \
    apt-fast install -y                                                        \
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
    nodejs                                                                     \
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
    ros-base                                                                   \
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

RUN apt-fast install -y openssh-client openssh-server

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

# ROS dependencies.
RUN pip install --user rosinstall_generator wstool
RUN sudo rosdep init && rosdep update
RUN sudo apt-fast install -y libxmlrpcpp-dev librosconsole-dev

# Use a stable version of bazel. ###############################################
WORKDIR /tmp
USER root
RUN curl -L https://github.com/bazelbuild/bazel/releases/download/0.22.0/bazel_0.22.0-linux-x86_64.deb > bazel_0.22.0-linux-x86_64.deb
RUN dpkg -i bazel_0.22.0-linux-x86_64.deb

#RUN echo "source /home/uas/.bazel/bin/bazel-complete.bash" > /home/uas/.bashrc

RUN ln /usr/lib/x86_64-linux-gnu/libroscpp.so.2d /usr/lib/x86_64-linux-gnu/libroscpp.so
RUN ln /usr/lib/x86_64-linux-gnu/libroslib.so.1.14.4 /usr/lib/x86_64-linux-gnu/libroslib.so
RUN ln /usr/lib/x86_64-linux-gnu/librospack.so.2.5.2 /usr/lib/x86_64-linux-gnu/librospack.so

# Set up working directory and permissions for later scripts to use. ###########
WORKDIR /home/uas/code_env

EXPOSE 8085/udp
EXPOSE 8086/udp
EXPOSE 8090/udp
EXPOSE 8091/udp
