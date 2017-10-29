# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0022_remove_missionconfig_sric_pos'),
    ]

    operations = [
        migrations.AlterIndexTogether(
            name='missionclockevent',
            index_together=set([('user', 'timestamp')]), ),
        migrations.AlterIndexTogether(
            name='takeofforlandingevent',
            index_together=set([('user', 'timestamp')]), ),
        migrations.AlterIndexTogether(
            name='uastelemetry',
            index_together=set([('user', 'timestamp')]), ),
    ]
