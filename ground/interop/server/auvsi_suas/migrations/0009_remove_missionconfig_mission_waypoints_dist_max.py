# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0008_missionconfig_obstacles'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='missionconfig',
            name='mission_waypoints_dist_max', ),
    ]
