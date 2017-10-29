"""
WSGI config for the interop server.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/1.6/howto/deployment/wsgi/
"""

from django.core.wsgi import get_wsgi_application
from django.test import Client
import os
import sys

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "server.settings")

# Add parent directory to Python path
server_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path = [server_dir] + sys.path

application = get_wsgi_application()

# Django puts off loading many relevant modules until the first request
# arrives. If the disk is very slow (e.g., when using Vagrant, see #8),
# this introduces significant latency in the first request. By making a dummy
# request here, we force Django to import everything it needs and improve
# latency for the first real request.
Client().get("/")
