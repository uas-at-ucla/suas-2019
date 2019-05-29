from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer
import json


class ImageConsumer(WebsocketConsumer):
    def connect(self):
        async_to_sync(self.channel_layer.group_add)('image-updates',
                                                    self.channel_name)
        self.accept()
        self.send(text_data='{"test": "youve connected"}')

    def event_newupdate(self, event):
        self.send(text_data=json.dumps(event['instance']))
        print('newupdate event!')
