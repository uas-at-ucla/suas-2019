FROM debian:stretch

RUN apt update

RUN apt install -y curl
RUN apt install -y gnupg
##RUN apt install -y vim

# Install node/NPM dependencies ################################################
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt install -y nodejs;

# Install Python3.6 ############################################################
RUN echo "deb http://ftp.de.debian.org/debian testing main" \
         | tee /etc/apt/sources.list.d/python3_6.list
RUN echo 'APT::Default-Release "stable";' | tee -a /etc/apt/apt.conf.d/00local
RUN apt update
RUN apt -t testing install -y python3.6
RUN apt -t testing install -y python3.6-dev
RUN apt -t testing install -y python3.6-distutils
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.6
RUN pip install --upgrade pip

# Install Bazel ################################################################
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable " \
         "jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN apt update
RUN apt install -y bazel

# Set up workspace
RUN mkdir -p /usr/src/app
ENV WORKSPACE /usr/src/app WORKDIR /usr/src/app

RUN bazel version

# Install compilation and testing dependencies #################################
RUN apt install -y libc++abi-dev
RUN apt install -y libc++-dev
RUN apt install -y clang-format-3.9
RUN apt install -y clang-3.9
RUN apt install -y ruby
RUN apt install -y git
RUN apt install -y lsb-release build-essential libssl-dev 
RUN apt install -y build-essential
RUN apt install -y libssl-dev 
RUN apt install -y ant
##RUN apt install -y libgazebo7-dev
##RUN apt install -y libopencv-dev
##RUN apt install -y libeigen3-dev
RUN apt install -y protobuf-compiler
##RUN apt install -y liblz4-dev
##RUN apt install -y libc6-i386
##RUN apt install -y lib32z1-dev
##RUN apt install -y lib32stdc++6

# Fix git certificate bug with open JDK
RUN apt install -y ca-certificates-java
RUN sed -i -e 's/keystore.type=pkcs12/keystore.type=jks/g' /etc/java-11-openjdk/security/java.security
RUN rm /etc/ssl/certs/java/cacerts
RUN update-ca-certificates -f

# Fix xlocale.h renamed to locale.h
# https://bugs.launchpad.net/ubuntu/+source/libc++/+bug/1725858
RUN ln -s /usr/include/locale.h /usr/include/xlocale.h

WORKDIR /usr/src/app
