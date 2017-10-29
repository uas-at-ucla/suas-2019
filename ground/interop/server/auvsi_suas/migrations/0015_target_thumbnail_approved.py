# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0014_add_autonomous_targets'),
    ]

    operations = [
        migrations.AddField(
            model_name='target',
            name='thumbnail_approved',
            field=models.NullBooleanField(), ),
    ]
