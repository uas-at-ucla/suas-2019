FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt update     -qq                                                      && \
    apt install -y -qq curl                                                 && \
    apt install -y -qq gnupg

# Add apt repositories containing required packages. ###########################
RUN echo "deb http://deb.debian.org/debian buster main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster main\n\n" \
  "deb http://deb.debian.org/debian buster-updates main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster-updates main\n\n" \
  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian sid main" > \
  /etc/apt/sources.list

RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -

RUN apt update     -qq                                                      && \
    apt upgrade -y -qq

# Install apt dependencies. ####################################################
RUN apt install -y -qq                                                         \
  bmon                                                                         \
  build-essential                                                              \
  git                                                                          \
  htop                                                                         \
  nodejs                                                                       \
  python3.6                                                                    \
  python3.6-dev                                                                \
  python3.6-distutils                                                          \
  sudo                                                                         \
  tmux                                                                         \
  vim

RUN apt upgrade -y -qq

# Install pip and python dependencies. #########################################
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.6
RUN pip3.6 install --upgrade pip

# Set up workspace. ############################################################
RUN useradd -ms /bin/bash uas
RUN echo 'uas:spinny' | chpasswd
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

# User env setup. ##############################################################
RUN mkdir /external_libs

RUN apt-get install -y libjpeg-dev zlib1g-dev

USER uas
RUN mkdir -p /home/uas/code_env
ENV WORKSPACE /home/uas/code_env WORKDIR /home/uas/code_env

WORKDIR /home/uas

# Install python requirements. #################################################
RUN pip3.6 install --user future

RUN pip3.6 install --user                                                      \
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
  numpy==1.13.1                                                                \
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