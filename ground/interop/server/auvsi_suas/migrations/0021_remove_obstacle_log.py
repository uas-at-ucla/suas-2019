# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0020_remove_server_info'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='obstacleaccesslog',
            name='user', ),
        migrations.DeleteModel(
            name='ObstacleAccessLog', ),
    ]
