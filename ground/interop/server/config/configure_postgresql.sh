#!/bin/bash
# Configures the postgresql server.

CONFIG=$(readlink -f $(dirname ${BASH_SOURCE[0]}))

set -e

sudo cp ${CONFIG}/postgresql.conf /etc/postgresql/9.3/main/
sudo service postgresql restart

sudo -u postgres psql -c "CREATE USER postgresql_user WITH CREATEDB PASSWORD 'postgresql_pass';"
sudo -u postgres psql -c "CREATE DATABASE auvsi_suas_db;"
