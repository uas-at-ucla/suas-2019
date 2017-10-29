# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0029_min_auto_flight'),
    ]

    operations = [
        migrations.AddField(
            model_name='missionjudgefeedback',
            name='crashed',
            field=models.BooleanField(default=False),
            preserve_default=False, ),
        migrations.AddField(
            model_name='missionjudgefeedback',
            name='things_fell_off_uas',
            field=models.BooleanField(default=False),
            preserve_default=False, ),
    ]
