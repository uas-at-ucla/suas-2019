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

# Set up workspace. ############################################################
RUN useradd -ms /bin/bash uas
RUN echo 'uas:spinny' | chpasswd
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

# User env setup. ##############################################################
RUN apt-get update && apt-get install -y \
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

# needed for cmavnode
RUN apt-get update && apt-get install -y libpthread-stubs0-dev

# needed for mavlink-router
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install --upgrade pip && pip3 install future

# Get Oracle Java 8
RUN apt-get update && apt-get install -y locales && \
    echo oracle-java8-installer shared/accepted-oracle-license-v1-1            \
      select true | debconf-set-selections                                  && \
    wget -O oracle-java8-installer.deb http://debian.opennms.org/dists/opennms-23/main/binary-all/oracle-java8-installer_8u131-1~webupd8~2_all.deb && \
    dpkg -i ./oracle-java8-installer.deb && apt-get install -f && \
    wget -O oracle-java8-set-default.deb http://debian.opennms.org/dists/opennms-23/main/binary-all/oracle-java8-set-default_8u131-1~webupd8~2_all.deb && \
    dpkg -i ./oracle-java8-set-default.deb && apt-get install -f && \
    apt autoremove -y && rm ./oracle-java8-installer.deb ./oracle-java8-set-default.deb

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

RUN pip install --upgrade pip && pip install --user mavproxy

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
