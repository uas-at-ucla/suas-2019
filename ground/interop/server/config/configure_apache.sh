#!/bin/bash
# Configures the apache server.

CONFIG=$(readlink -f $(dirname ${BASH_SOURCE[0]}))

set -e

sudo chown -R www-data /var/www
sudo cp ${CONFIG}/xsendfile.conf /etc/apache2/conf-enabled/
sudo cp ${CONFIG}/limit_upload.conf /etc/apache2/conf-enabled/
sudo cp ${CONFIG}/interop_apache.conf /etc/apache2/sites-enabled/
sudo cp ${CONFIG}/apache2.conf /etc/apache2/
sudo service apache2 restart
