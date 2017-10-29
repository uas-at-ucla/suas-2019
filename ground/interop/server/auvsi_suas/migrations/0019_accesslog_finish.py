# -*- coding: utf-8 -*-
"""See 0018_accesslog for the first stage of this migration."""

from __future__ import print_function
from __future__ import unicode_literals

from django.db import models, migrations
import datetime
from django.utils import timezone
from django.conf import settings
import functools
import logging


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('auvsi_suas', '0018_accesslog'),
    ]

    operations = [
        migrations.RenameModel(
            old_name='NewMissionClockEvent', new_name='MissionClockEvent'),
        migrations.RenameModel(
            old_name='NewObstacleAccessLog', new_name='ObstacleAccessLog'),
        migrations.RenameModel(
            old_name='NewServerInfoAccessLog', new_name='ServerInfoAccessLog'),
        migrations.RenameModel(
            old_name='NewTakeoffOrLandingEvent',
            new_name='TakeoffOrLandingEvent'),
        migrations.RenameModel(
            old_name='NewUasTelemetry', new_name='UasTelemetry'),

        # Back to auto_now_add.
        migrations.AlterField(
            model_name='missionclockevent',
            name='timestamp',
            field=models.DateTimeField(auto_now_add=True, db_index=True)),
        migrations.AlterField(
            model_name='obstacleaccesslog',
            name='timestamp',
            field=models.DateTimeField(auto_now_add=True, db_index=True)),
        migrations.AlterField(
            model_name='serverinfoaccesslog',
            name='timestamp',
            field=models.DateTimeField(auto_now_add=True, db_index=True)),
        migrations.AlterField(
            model_name='takeofforlandingevent',
            name='timestamp',
            field=models.DateTimeField(auto_now_add=True, db_index=True)),
        migrations.AlterField(
            model_name='uastelemetry',
            name='timestamp',
            field=models.DateTimeField(auto_now_add=True, db_index=True)),
    ]
