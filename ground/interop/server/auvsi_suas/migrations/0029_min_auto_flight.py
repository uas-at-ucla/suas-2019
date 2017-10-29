# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0028_mission_judge_feedback_optional_delivery'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='missionjudgefeedback',
            name='manual_flight_time', ),
        migrations.AddField(
            model_name='missionjudgefeedback',
            name='min_auto_flight_time',
            field=models.BooleanField(default=True),
            preserve_default=False, ),
    ]
