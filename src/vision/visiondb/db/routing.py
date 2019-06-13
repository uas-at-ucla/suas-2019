from django.urls import path
from rest_framework import routers
from . import consumers

websocket_urlpatterns = [
    path('ws/image-updates/', consumers.ImageConsumer),
]
