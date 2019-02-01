FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt-get update     -qq                                                  && \
    apt-get install -y -qq curl                                             && \
    apt-get install -y -qq gnupg

# Add apt-get repositories containing required packages. ###########################
RUN echo "deb http://deb.debian.org/debian stretch main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian stretch main\n\n" \
  "deb http://deb.debian.org/debian stretch-updates main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian stretch-updates main\n\n" \
  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian sid main" > \
  /etc/apt/sources.list

RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -

RUN apt-get update                                                          && \
    apt-get upgrade -y

# Install apt dependencies. ####################################################
RUN apt-get install -y                                                         \
  ant                                                                          \
  autoconf                                                                     \
  autoconf                                                                     \
  automake                                                                     \
  bmon                                                                         \
  build-essential                                                              \
  ca-certificates-java                                                         \
  check                                                                        \
  cmake                                                                        \
  g++                                                                          \
  git                                                                          \
  htop                                                                         \
  libc++-dev                                                                   \
  libc++abi-dev                                                                \
  lib32z1-dev                                                                  \
  libc6-dev-i386                                                               \
  librsync-dev                                                                 \
  libssl-dev                                                                   \
  lsb-release                                                                  \
  libtool                                                                      \
  libyajl-dev                                                                  \
  libz-dev                                                                     \
  make                                                                         \
  nodejs                                                                       \
  pkg-config                                                                   \
  protobuf-compiler                                                            \
  software-properties-common                                                   \
  sudo                                                                         \
  tmux                                                                         \
  uthash-dev                                                                   \
  xvfb                                                                         \
  vim

# Set up Oracle Java8. #########################################################
RUN add-apt-repository                                                         \
     "deb http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main"
RUN apt-get update                                                          && \
    echo debconf shared/accepted-oracle-license-v1-1 select true             | \
    debconf-set-selections                                                  && \
    apt-get install -y                                                         \
      oracle-java8-installer


# Set up workspace. ############################################################
RUN useradd -ms /bin/bash uas
RUN echo 'uas:spinny' | chpasswd
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

# User env setup. ##############################################################
RUN apt-get install -y \
    bzip2 \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    doxygen \
    g++ \
    gcc \
    gdb \
    git \
    gnupg \
    gosu \
    lcov \
    make \
    ninja-build \
    openjdk-8-jdk \
    openjdk-8-jre \
    openssh-client \
    python-pip \
    python-pygments \
    python-setuptools \
    rsync \
    shellcheck \
    unzip \
    wget \
    xsltproc \
    zip

RUN python -m pip install --upgrade pip \
      && pip install setuptools wheel \
      && pip install argparse coverage jinja2 empy numpy requests serial toml pyyaml cerberus

RUN apt-get install -y \
      libboost-all-dev \
      cmake \
      libconfig++-dev \
      libreadline-dev \
      xvfb

USER uas
WORKDIR /home/uas
RUN git clone https://github.com/MonashUAS/cmavnode.git && \
    cd cmavnode && \
    git submodule update --init && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    sudo make install

RUN git clone https://github.com/intel/mavlink-router.git && \
    cd mavlink-router && \
    git submodule update --init --recursive && \
    ./autogen.sh && ./configure CFLAGS='-g -O2' \
        --sysconfdir=/etc --localstatedir=/var --libdir=/usr/lib64 \
    --prefix=/usr && \
    make && \
    sudo make install

RUN pip install --user mavproxy

WORKDIR /home/uas

EXPOSE 8085/udp
EXPOSE 8086/udp
EXPOSE 8095/udp
EXPOSE 8096/udp
EXPOSE 14540/udp
EXPOSE 14550/udp

# Set user to root so that we can set the appropriate permissions for file #####
# access later on. #############################################################
USER root
