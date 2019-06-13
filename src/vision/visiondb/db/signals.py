from channels.layers import get_channel_layer
from django.db.models.signals import post_save
from django.dispatch import receiver
from asgiref.sync import async_to_sync
from .serializers import DataSerializer

from . import models

channel_layer = get_channel_layer()


@receiver(post_save, sender=models.Data)
def push_img_update(sender, instance, created, **kwargs):
    async_to_sync(channel_layer.group_send)('image-updates', {
        'type': 'event_newupdate',
        'instance': DataSerializer(instance).data
    })
    print('database updated!')
