# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0001_initial'),
    ]

    operations = [
        migrations.RemoveField(
            model_name='missionconfig',
            name='ir_target_pos', ),
        migrations.AddField(
            model_name='missionconfig',
            name='ir_primary_target_pos',
            field=models.ForeignKey(
                related_name='missionconfig_ir_primary_target_pos',
                to='auvsi_suas.GpsPosition'),
            preserve_default=False, ),
        migrations.AddField(
            model_name='missionconfig',
            name='ir_secondary_target_pos',
            field=models.ForeignKey(
                related_name='missionconfig_ir_secondary_target_pos',
                to='auvsi_suas.GpsPosition'),
            preserve_default=False, ),
    ]
