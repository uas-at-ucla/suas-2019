#       _   _   _    ____      ____    _   _  ____ _        _
#      | | | | / \  / ___|    / __ \  | | | |/ ___| |      / \
#      | | | |/ _ \ \___ \   / / _` | | | | | |   | |     / _ \
#      | |_| / ___ \ ___) | | | (_| | | |_| | |___| |___ / ___ \
#       \___/_/   \_\____/   \ \__,_|  \___/ \____|_____/_/   \_\
#                             \____/

FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt-get update     -qq                                                  && \
    apt-get install -y -qq curl gnupg

# Add apt-get repositories containing required packages. ###########################
#RUN echo "deb http://deb.debian.org/debian buster main contrib non-free\n" \
#  "deb-src http://deb.debian.org/debian buster main\n\n" \
#  "deb http://deb.debian.org/debian buster-updates main contrib non-free\n" \
#  "deb-src http://deb.debian.org/debian buster-updates main\n\n" \
#  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
#  "deb-src http://deb.debian.org/debian sid main" > \
#  /etc/apt/sources.list

RUN curl -sL https://bazel.build/bazel-release.pub.gpg | apt-key add -      && \
    curl -sL https://deb.nodesource.com/setup_8.x | bash -                  && \
    echo "deb http://deb.debian.org/debian testing main" |                     \
      tee /etc/apt/sources.list.d/python3_6.list                            && \
    curl -sL https://bazel.build/bazel-release.pub.gpg | apt-key add -      && \
    echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt " \
  "stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

# Install apt dependencies. ####################################################
RUN apt-get update -qq                                                      && \
    apt-get upgrade -y -qq                                                  && \
    apt-get install -y                                                         \
    ant                                                                        \
    autoconf                                                                   \
    autoconf                                                                   \
    automake                                                                   \
    bazel                                                                      \
    bmon                                                                       \
    build-essential                                                            \
    ca-certificates-java                                                       \
    check                                                                      \
    clang-7                                                                    \
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
    openjdk-8-jdk-headless                                                     \
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
    ruby                                                                       \
    screen                                                                     \
    sudo                                                                       \
    tmux                                                                       \
    uthash-dev                                                                 \
    vim

# Fix git certificate bug with open JDK. #######################################
RUN sed -i -e 's/keystore.type=pkcs12/keystore.type=jks/g'                     \
      /etc/java-8-openjdk/security/java.security                            && \
    rm /etc/ssl/certs/java/cacerts                                          && \
    update-ca-certificates -f

# Install pip and python dependencies. #########################################
RUN curl -sL https://bootstrap.pypa.io/get-pip.py | python2.7               && \
    curl -sL https://bootstrap.pypa.io/get-pip.py | python3.6               && \
    pip2.7 install --upgrade pip                                            && \
    pip3.6 install --upgrade pip

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
RUN git clone --recursive https://github.com/intel/mavlink-router.git
WORKDIR /home/uas/mavlink-router
RUN ./autogen.sh && ./configure CFLAGS='-g -O2' \
    --sysconfdir=/etc \
    --localstatedir=/var \
    --libdir=/usr/lib64 \
    --prefix=/usr
RUN make -j4
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
  numpy==1.15.2                                                                \
  empy==3.3.2                                                                  \
  toml==0.9.4                                                                  \
  eventlet==0.23.0                                                             \
  protobuf==3.5.2.post1                                                        \
  pyzmq==17.0.0                                                                \
  zmq==0.0.0

# Set up working directory and permissions for later scripts to use. ###########
WORKDIR /home/uas/code_env

RUN rm -rf /home/uas/mavlink-router

EXPOSE 8085/udp
EXPOSE 8086/udp
EXPOSE 8090/udp
EXPOSE 8091/udp

# Set user to root so that we can set the appropriate permissions for file #####
# access later on. #############################################################
USER root
