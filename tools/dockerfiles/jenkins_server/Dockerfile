FROM jenkins/jenkins:latest

USER root
RUN apt-get update -y
RUN apt-get install -y \
     apt-transport-https \
     ca-certificates \
     curl \
     gnupg2 \
     software-properties-common

RUN curl -fsSL https://download.docker.com/linux/debian/gpg | apt-key add -
RUN apt-key fingerprint 0EBFCD88
RUN add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/debian \
   stretch \
   stable"

RUN apt-get update -y
RUN apt-get install -y docker-ce

RUN usermod -aG docker jenkins

RUN apt-get install -y tmux

USER jenkins
