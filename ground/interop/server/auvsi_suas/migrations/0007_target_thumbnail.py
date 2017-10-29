# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0006_target_blank'),
    ]

    operations = [
        migrations.AddField(
            model_name='target',
            name='thumbnail',
            field=models.ImageField(upload_to=b'targets', blank=True), ),
    ]
