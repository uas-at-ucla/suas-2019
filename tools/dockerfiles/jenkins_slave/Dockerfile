FROM debian:stretch

# Install compilation and testing dependencies #################################
RUN apt-get update     -qq                                                  && \
    apt-get install -y -qq                                                     \
    curl                                                                       \
    gnupg2

# Add apt repositories containing required packages. ###########################
RUN echo "deb http://deb.debian.org/debian buster main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster main\n\n" \
  "deb http://deb.debian.org/debian buster-updates main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian buster-updates main\n\n" \
  "deb http://deb.debian.org/debian sid main contrib non-free\n" \
  "deb-src http://deb.debian.org/debian sid main" > \
  /etc/apt/sources.list

RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN echo "deb http://deb.debian.org/debian testing main" \
  | tee /etc/apt/sources.list.d/python3_6.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt " \
  "stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list

RUN apt update     -qq                                                      && \
    apt upgrade -y -qq

# Install apt dependencies. ####################################################
RUN apt-get install -y -qq                                                     \
  apt-transport-https                                                          \
  ca-certificates                                                              \
  ca-certificates-java                                                         \
  netcat                                                                       \
  openjdk-11-jdk                                                               \
  openssh-client                                                               \
  openssh-server                                                               \
  python2.7                                                                    \
  software-properties-common                                                   \
  tmux                                                                         \
  htop                                                                         \
  bmon                                                                         \
  nmap                                                                         \
  tree                                                                         \
  sudo                                                                         \
  vim

RUN apt upgrade -y -qq

# Install docker ###############################################################
RUN curl -fsSL https://download.docker.com/linux/debian/gpg | apt-key add -
RUN apt-key fingerprint 0EBFCD88
RUN add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/debian \
   stretch \
   stable"
RUN apt-get update -y -qq
RUN apt-get install -y -qq docker-ce

# User env setup. ##############################################################
RUN useradd -ms /bin/bash jenkins_uasatucla
RUN echo 'jenkins_uasatucla:spinny' | chpasswd
RUN usermod -aG docker jenkins_uasatucla
RUN usermod -aG sudo jenkins_uasatucla

WORKDIR /home/jenkins_uasatucla/

RUN curl -o /home/jenkins_uasatucla/jenkins-cli.jar \
  https://uasatucla.org/jenkins/jnlpJars/jenkins-cli.jar

RUN curl -o /home/jenkins_uasatucla/sonar-scanner.zip \
  https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.0.0.1744-linux.zip &&
  unzip sonar-scanner.zip

# Set user to root so that we can set the appropriate permissions for file #####
# access later on. #############################################################
USER root
