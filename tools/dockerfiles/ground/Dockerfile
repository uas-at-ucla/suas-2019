FROM python:3.6-slim

RUN apt-get update && apt-get install -y curl
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash - && \
    apt-get update && apt-get install -y nodejs
RUN apt-get update && apt-get install -y gnupg
RUN apt-get update && apt-get install -y build-essential
RUN apt-get update && apt-get install -y git
RUN apt-get update && apt-get install -y sudo
RUN apt-get update && apt-get install -y iputils-ping

# ROS
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list'
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# RUN apt-get update && apt-get install -y ros-melodic-ros-base
# RUN mkdir -p /home/uas
# RUN printf "source /opt/ros/melodic/setup.bash" > /home/uas/.bashrc

RUN useradd -ms /bin/bash uas
RUN echo 'uas:spinny' | chpasswd
RUN echo "%sudo  ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG sudo uas

RUN mkdir -p /home/uas/code_env
WORKDIR /home/uas/code_env
USER root
