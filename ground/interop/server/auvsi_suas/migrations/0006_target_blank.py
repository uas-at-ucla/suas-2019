# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations


class Migration(migrations.Migration):

    dependencies = [
        ('auvsi_suas', '0005_target'),
    ]

    operations = [
        migrations.AlterField(
            model_name='target',
            name='alphanumeric',
            field=models.TextField(default=b'', blank=True), ),
        migrations.AlterField(
            model_name='target',
            name='alphanumeric_color',
            field=models.IntegerField(
                blank=True,
                null=True,
                choices=[(1, b'white'), (2, b'black'), (3, b'gray'),
                         (4, b'red'), (5, b'blue'), (6, b'green'), (7,
                                                                    b'yellow'),
                         (8, b'purple'), (9, b'brown'), (10, b'orange')]), ),
        migrations.AlterField(
            model_name='target',
            name='background_color',
            field=models.IntegerField(
                blank=True,
                null=True,
                choices=[(1, b'white'), (2, b'black'), (3, b'gray'),
                         (4, b'red'), (5, b'blue'), (6, b'green'), (7,
                                                                    b'yellow'),
                         (8, b'purple'), (9, b'brown'), (10, b'orange')]), ),
        migrations.AlterField(
            model_name='target',
            name='description',
            field=models.TextField(default=b'', blank=True), ),
        migrations.AlterField(
            model_name='target',
            name='location',
            field=models.ForeignKey(
                blank=True, to='auvsi_suas.GpsPosition', null=True), ),
        migrations.AlterField(
            model_name='target',
            name='orientation',
            field=models.IntegerField(
                blank=True,
                null=True,
                choices=[(1, b'n'), (2, b'ne'), (3, b'e'), (4, b'se'),
                         (5, b's'), (6, b'sw'), (7, b'w'), (8, b'nw')]), ),
        migrations.AlterField(
            model_name='target',
            name='shape',
            field=models.IntegerField(
                blank=True,
                null=True,
                choices=[(1, b'circle'), (2, b'semicircle'),
                         (3, b'quarter_circle'), (4, b'triangle'),
                         (5, b'square'), (6, b'rectangle'), (7, b'trapezoid'),
                         (8, b'pentagon'), (9, b'hexagon'), (10, b'heptagon'),
                         (11, b'octagon'), (12, b'star'), (13, b'cross')]), ),
    ]
