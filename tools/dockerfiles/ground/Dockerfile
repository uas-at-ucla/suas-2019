FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt update                                                              && \
    apt install -y curl                                                     && \
    apt install -y gnupg

# Add apt repositories containing required packages. ###########################
RUN echo "deb http://deb.debian.org/debian stretch main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian stretch main\n\n" \
  "deb http://deb.debian.org/debian stretch-updates main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian stretch-updates main\n\n" \
  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian sid main" > \
  /etc/apt/sources.list

RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb http://deb.debian.org/debian testing main" \
  | tee /etc/apt/sources.list.d/python3_6.list
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash -

RUN apt update                                                              && \
    apt upgrade -y

# Install apt dependencies. ####################################################
RUN apt install -y                                                             \
  ant                                                                          \
  build-essential                                                              \
  git                                                                          \
  htop                                                                         \
  nodejs                                                                       \
  python2.7                                                                    \
  python2.7-lib                                                                \
  python2.7-dev                                                                \
  python3.7                                                                    \
  python3.7-dev                                                                \
  python3.7-distutils                                                          \
  sudo                                                                         \
  tmux                                                                         \
  vim

RUN apt upgrade -y

# Fix git certificate bug with open JDK. #######################################
RUN sed -i -e 's/keystore.type=pkcs12/keystore.type=jks/g' \
  /etc/java-10-openjdk/security/java.security
RUN rm /etc/ssl/certs/java/cacerts
RUN update-ca-certificates -f

# Install pip and python dependencies. #########################################
RUN curl https://bootstrap.pypa.io/get-pip.py | python2.7
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.7
RUN pip2.7 install --upgrade pip                                            && \
    pip3.7 install --upgrade pip

RUN apt-get install -y libtiff5-dev libjpeg62-turbo-dev zlib1g-dev libfreetype6-dev liblcms2-dev libwebp-dev
RUN ln -s /usr/lib/x86_64-linux-gnu/libz.so /lib/
RUN ln -s /usr/lib/x86_64-linux-gnu/libjpeg.so /lib/

# Set up workspace. ############################################################
RUN useradd -ms /bin/bash uas
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

# User env setup. ##############################################################
USER uas
RUN mkdir -p /home/uas/code_env
ENV WORKSPACE /home/uas/code_env WORKDIR /home/uas/code_env

WORKDIR /home/uas

# Install python requirements. #################################################
RUN pip3.7 install --user                                                      \
  future                                                                       \
  futures                                                                      \
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
  socketIO-client==0.7.2                                                       \
  numpy                                                                        \
  empy==3.3.2                                                                  \
  toml==0.9.4                                                                  \
  eventlet==0.23.0                                                             \
  protobuf==3.5.2.post1                                                        \
  pyzmq==17.0.0                                                                \
  zmq==0.0.0

# Set up working directory and permissions for later scripts to use. ###########
WORKDIR /home/uas/code_env

# Set user to root so that we can set the appropriate permissions for file #####
# access later on. #############################################################
USER root
