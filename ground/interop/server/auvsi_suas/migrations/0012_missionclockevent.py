# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0011_remove_ir'),
    ]

    operations = [
        migrations.CreateModel(
            name='MissionClockEvent',
            fields=[
                ('accesslog_ptr', models.OneToOneField(
                    parent_link=True,
                    auto_created=True,
                    primary_key=True,
                    serialize=False,
                    to='auvsi_suas.AccessLog')),
                ('team_on_clock', models.BooleanField()),
                ('team_on_timeout', models.BooleanField()),
            ],
            bases=('auvsi_suas.accesslog', ), ),
    ]
