# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0021_remove_obstacle_log'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='missionconfig',
            name='sric_pos', ),
    ]
