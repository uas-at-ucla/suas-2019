from django.urls import path
from . import consumers

websocket_urlpatterns = [
    path('ws/image-updates/', consumers.ImageConsumer),
]
