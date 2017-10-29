# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models
from django.conf import settings


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('auvsi_suas', '0030_crash_tfoa'),
    ]

    operations = [
        migrations.RenameModel('Target', 'Odlc'),
        migrations.RenameField('Odlc', 'target_type', 'odlc_type'),
        migrations.RenameField('MissionConfig', 'off_axis_target_pos',
                               'off_axis_odlc_pos'),
        migrations.RenameField('MissionConfig', 'targets', 'odlcs'),
    ]
