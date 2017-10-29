# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0019_accesslog_finish'),
    ]

    operations = [
        migrations.RemoveField(model_name='serverinfoaccesslog', name='user'),
        migrations.RemoveField(model_name='missionconfig', name='server_info'),
        migrations.DeleteModel(name='ServerInfo'),
        migrations.DeleteModel(name='ServerInfoAccessLog'),
    ]
