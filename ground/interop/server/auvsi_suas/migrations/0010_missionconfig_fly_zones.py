# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0009_remove_missionconfig_mission_waypoints_dist_max'),
    ]

    operations = [
        migrations.AddField(
            model_name='missionconfig',
            name='fly_zones',
            field=models.ManyToManyField(to='auvsi_suas.FlyZone'), ),
    ]
