# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0016_add_target_time'),
    ]

    operations = [
        migrations.AddField(
            model_name='missionconfig',
            name='targets',
            field=models.ManyToManyField(
                related_name='missionconfig_targets', to='auvsi_suas.Target'),
        ),
    ]
