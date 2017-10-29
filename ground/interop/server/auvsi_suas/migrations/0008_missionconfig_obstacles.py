# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [('auvsi_suas', '0007_target_thumbnail')]

    operations = [
        migrations.AddField(
            model_name='missionconfig',
            name='moving_obstacles',
            field=models.ManyToManyField(to='auvsi_suas.MovingObstacle')),
        migrations.AddField(
            model_name='missionconfig',
            name='stationary_obstacles',
            field=models.ManyToManyField(to='auvsi_suas.StationaryObstacle')),
    ]
