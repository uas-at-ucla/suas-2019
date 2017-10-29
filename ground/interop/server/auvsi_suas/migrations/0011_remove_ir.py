# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [('auvsi_suas', '0010_missionconfig_fly_zones')]

    operations = [
        migrations.RemoveField(
            model_name='missionconfig', name='ir_primary_target_pos'),
        migrations.RemoveField(
            model_name='missionconfig', name='ir_secondary_target_pos'),
    ]
