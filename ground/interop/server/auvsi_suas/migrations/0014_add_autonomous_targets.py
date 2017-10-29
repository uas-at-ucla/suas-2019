# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0013_remove_ir_as_target_type'),
    ]

    operations = [
        migrations.AddField(
            model_name='target',
            name='autonomous',
            field=models.BooleanField(default=False), ),
    ]
