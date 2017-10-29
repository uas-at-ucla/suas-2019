# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations
from django.conf import settings


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('auvsi_suas', '0004_missionconfig_server_info'),
    ]

    operations = [
        migrations.CreateModel(
            name='Target',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('target_type', models.IntegerField(
                    choices=[(1, b'standard'), (2, b'qrc'), (3, b'off_axis'),
                             (4, b'emergent'), (5, b'ir')])),
                ('alphanumeric', models.TextField(default=b'')),
                ('shape', models.IntegerField(
                    null=True,
                    choices=[(1, b'circle'), (2, b'semicircle'),
                             (3, b'quarter_circle'), (4, b'triangle'),
                             (5, b'square'), (6, b'rectangle'), (7,
                                                                 b'trapezoid'),
                             (8, b'pentagon'), (9, b'hexagon'), (10,
                                                                 b'heptagon'),
                             (11, b'octagon'), (12, b'star'), (13,
                                                               b'cross')])),
                ('background_color', models.IntegerField(
                    null=True,
                    choices=[(1, b'white'), (2, b'black'), (3, b'gray'),
                             (4, b'red'), (5, b'blue'), (6, b'green'),
                             (7, b'yellow'), (8, b'purple'), (9, b'brown'),
                             (10, b'orange')])),
                ('alphanumeric_color', models.IntegerField(
                    null=True,
                    choices=[(1, b'white'), (2, b'black'), (3, b'gray'),
                             (4, b'red'), (5, b'blue'), (6, b'green'),
                             (7, b'yellow'), (8, b'purple'), (9, b'brown'),
                             (10, b'orange')])),
                ('orientation', models.IntegerField(
                    null=True,
                    choices=[(1, b'n'), (2, b'ne'), (3, b'e'), (4, b'se'),
                             (5, b's'), (6, b'sw'), (7, b'w'), (8, b'nw')])),
                ('description', models.TextField(default=b'')),
                ('location', models.ForeignKey(
                    to='auvsi_suas.GpsPosition', null=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
            ], ),
    ]
