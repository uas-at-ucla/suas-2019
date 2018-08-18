#       _   _   _    ____      ____    _   _  ____ _        _
#      | | | | / \  / ___|    / __ \  | | | |/ ___| |      / \
#      | | | |/ _ \ \___ \   / / _` | | | | | |   | |     / _ \
#      | |_| / ___ \ ___) | | | (_| | | |_| | |___| |___ / ___ \
#       \___/_/   \_\____/   \ \__,_|  \___/ \____|_____/_/   \_\
#                             \____/

FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt update
RUN apt install -y curl
RUN apt install -y gnupg

RUN echo "deb http://deb.debian.org/debian buster main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster main\n\n" \
  "deb http://deb.debian.org/debian buster-updates main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster-updates main\n\n" \
  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian sid main" > \
  /etc/apt/sources.list

### Add external repos
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN echo "deb http://deb.debian.org/debian testing main" \
  | tee /etc/apt/sources.list.d/python3_6.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" \
  | tee /etc/apt/sources.list.d/bazel.list

RUN apt update
RUN apt upgrade -y

### Install dependencies from apt
RUN apt install -y python3.7
RUN apt install -y python3.7-dev
RUN apt install -y python3.7-distutils

RUN apt install -y ant
RUN apt install -y bazel
RUN apt install -y bmon
RUN apt install -y build-essential
RUN apt install -y ca-certificates-java
RUN apt install -y clang-3.9
RUN apt install -y clang-format-3.9
RUN apt install -y cmake
RUN apt install -y git
RUN apt install -y gphoto2
RUN apt install -y gphotofs
RUN apt install -y htop
RUN apt install -y libc++-dev
RUN apt install -y libc++abi-dev
RUN apt install -y lib32z1-dev
RUN apt install -y libc6-dev-i386
RUN apt install -y libssl-dev
RUN apt install -y lsb-release
RUN apt install -y nodejs
RUN apt install -y openjdk-10-jdk
RUN apt install -y protobuf-compiler
RUN apt install -y python2.7
RUN apt install -y python2.7-lib
RUN apt install -y python2.7-dev
RUN apt install -y python-empy
RUN apt install -y python-jinja2
RUN apt install -y ruby
RUN apt install -y screen
RUN apt install -y sudo
RUN apt install -y tmux
RUN apt install -y vim
#RUN apt install -y libgazebo7-dev
#RUN apt install -y libopencv-dev
#RUN apt install -y libeigen3-dev
#RUN apt install -y liblz4-dev
#RUN apt install -y lib32stdc++6

RUN apt upgrade -y

# Fix git certificate bug with open JDK
RUN sed -i -e 's/keystore.type=pkcs12/keystore.type=jks/g' /etc/java-10-openjdk/security/java.security
RUN rm /etc/ssl/certs/java/cacerts
RUN update-ca-certificates -f

# Install pip
RUN curl https://bootstrap.pypa.io/get-pip.py | python2.7
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.6
RUN pip2.7 install --upgrade pip
RUN pip3.6 install --upgrade pip

# Set up workspace
# Fix xlocale.h renamed to locale.h
# https://bugs.launchpad.net/ubuntu/+source/libc++/+bug/1725858
RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

RUN useradd -ms /bin/bash uas
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -a -G sudo uas

# User env setup.
RUN mkdir /external_libs

USER uas
RUN mkdir -p /home/uas/code_env
ENV WORKSPACE /home/uas/code_env WORKDIR /home/uas/code_env

WORKDIR /home/uas

# Install mavlink-router
RUN sudo apt install -y autoconf autoconf automake libtool make pkg-config check g++ librsync-dev libz-dev libssl-dev uthash-dev libyajl-dev
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

RUN pip2.7 install --user mavproxy

WORKDIR /home/uas/code_env

RUN rm -rf /home/uas/mavlink-router

EXPOSE 8085/udp
EXPOSE 8086/udp

USER root
