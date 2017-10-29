#!/bin/bash
# Tests the Interop Server Docker image.

docker run -it auvsisuas/interop-server bash -c \
    "sudo service postgresql start && \
     sudo service memcached start && \
     cd /interop/server && \
     python manage.py test && \
     cd /interop/server/auvsi_suas/static/auvsi_suas && \
     bash test_with_phantomjs.sh"
