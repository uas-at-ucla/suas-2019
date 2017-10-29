# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import datetime
from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [('auvsi_suas', '0015_target_thumbnail_approved')]

    operations = [
        # Default applies only to existing rows. Actual field has no default.
        migrations.AddField(
            model_name='target',
            name='creation_time',
            field=models.DateTimeField(
                auto_now_add=True,
                default=datetime.datetime(year=2016, month=1, day=1)),
            preserve_default=False),
        migrations.AddField(
            model_name='target',
            name='last_modified_time',
            field=models.DateTimeField(
                auto_now=True,
                default=datetime.datetime(year=2016, month=1, day=1)),
            preserve_default=False),
    ]
