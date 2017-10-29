# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [('auvsi_suas', '0002_ir_primary_secondary_targets')]

    operations = [
        migrations.AddField(
            model_name='missionconfig',
            name='is_active',
            field=models.BooleanField(default=False)),
    ]
