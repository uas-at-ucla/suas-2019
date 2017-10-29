# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0012_missionclockevent'),
    ]

    operations = [
        migrations.AlterField(
            model_name='target',
            name='target_type',
            field=models.IntegerField(choices=[(1, b'standard'), (2, b'qrc'),
                                               (3, b'off_axis'),
                                               (4, b'emergent')]), ),
    ]
