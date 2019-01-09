FROM python:3.6-slim

RUN apt-get update
RUN apt-get install -y curl
RUN apt-get install -y gnupg

RUN curl -sL https://deb.nodesource.com/setup_10.x | bash -
RUN apt-get install -y nodejs

RUN useradd -ms /bin/bash uas
RUN usermod -aG sudo uas
RUN mkdir -p /home/uas/code_env
WORKDIR /home/uas/code_env
USER root
