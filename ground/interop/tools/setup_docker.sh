#!/bin/bash
# Installs Docker.

SETUP=$(readlink -f $(dirname ${BASH_SOURCE[0]}))
LOG_NAME=setup_docker
source ${SETUP}/common.sh

log "Installing packages for APT HTTPS."
apt-get -qq update
apt-get -qq install -y apt-transport-https ca-certificates

log "Adding Docker GPG key."
apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 58118E89F3A912897C070ADBF76221572C52609D

log "Adding Docker repository."
bash -c "echo 'deb https://apt.dockerproject.org/repo ubuntu-${RELEASE} main' > /etc/apt/sources.list.d/docker.list"
apt-get -qq update

log "Installing APT packages for Docker."
apt-get -qq install -y \
    docker-engine \
    linux-image-extra-$(uname -r) \
    linux-image-extra-virtual

log "Starting Docker daemon."
service docker restart
