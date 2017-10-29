# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import models, migrations
from django.conf import settings


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
    ]

    operations = [
        migrations.CreateModel(
            name='AccessLog',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(
                    auto_now_add=True, db_index=True)),
            ], ),
        migrations.CreateModel(
            name='AerialPosition',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('altitude_msl', models.FloatField()),
            ], ),
        migrations.CreateModel(
            name='FlyZone',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('altitude_msl_min', models.FloatField()),
                ('altitude_msl_max', models.FloatField()),
            ], ),
        migrations.CreateModel(
            name='GpsPosition',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('latitude', models.FloatField()),
                ('longitude', models.FloatField()),
            ], ),
        migrations.CreateModel(
            name='MissionConfig',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('mission_waypoints_dist_max', models.FloatField()),
                ('air_drop_pos', models.ForeignKey(
                    related_name='missionconfig_air_drop_pos',
                    to='auvsi_suas.GpsPosition')),
                ('emergent_last_known_pos', models.ForeignKey(
                    related_name='missionconfig_emergent_last_known_pos',
                    to='auvsi_suas.GpsPosition')),
                ('home_pos', models.ForeignKey(
                    related_name='missionconfig_home_pos',
                    to='auvsi_suas.GpsPosition')),
                ('ir_target_pos', models.ForeignKey(
                    related_name='missionconfig_ir_target_pos',
                    to='auvsi_suas.GpsPosition')),
            ], ),
        migrations.CreateModel(
            name='MovingObstacle',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('speed_avg', models.FloatField()),
                ('sphere_radius', models.FloatField()),
            ], ),
        migrations.CreateModel(
            name='ServerInfo',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(auto_now_add=True)),
                ('team_msg', models.CharField(max_length=100)),
            ], ),
        migrations.CreateModel(
            name='StationaryObstacle',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('cylinder_radius', models.FloatField()),
                ('cylinder_height', models.FloatField()),
                ('gps_position',
                 models.ForeignKey(to='auvsi_suas.GpsPosition')),
            ], ),
        migrations.CreateModel(
            name='Waypoint',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('order', models.IntegerField(db_index=True)),
                ('position',
                 models.ForeignKey(to='auvsi_suas.AerialPosition')),
            ], ),
        migrations.CreateModel(
            name='ObstacleAccessLog',
            fields=[
                ('accesslog_ptr', models.OneToOneField(
                    parent_link=True,
                    auto_created=True,
                    primary_key=True,
                    serialize=False,
                    to='auvsi_suas.AccessLog')),
            ],
            bases=('auvsi_suas.accesslog', ), ),
        migrations.CreateModel(
            name='ServerInfoAccessLog',
            fields=[
                ('accesslog_ptr', models.OneToOneField(
                    parent_link=True,
                    auto_created=True,
                    primary_key=True,
                    serialize=False,
                    to='auvsi_suas.AccessLog')),
            ],
            bases=('auvsi_suas.accesslog', ), ),
        migrations.CreateModel(
            name='TakeoffOrLandingEvent',
            fields=[
                ('accesslog_ptr', models.OneToOneField(
                    parent_link=True,
                    auto_created=True,
                    primary_key=True,
                    serialize=False,
                    to='auvsi_suas.AccessLog')),
                ('uas_in_air', models.BooleanField()),
            ],
            bases=('auvsi_suas.accesslog', ), ),
        migrations.CreateModel(
            name='UasTelemetry',
            fields=[
                ('accesslog_ptr', models.OneToOneField(
                    parent_link=True,
                    auto_created=True,
                    primary_key=True,
                    serialize=False,
                    to='auvsi_suas.AccessLog')),
                ('uas_heading', models.FloatField()),
            ],
            bases=('auvsi_suas.accesslog', ), ),
        migrations.AddField(
            model_name='movingobstacle',
            name='waypoints',
            field=models.ManyToManyField(to='auvsi_suas.Waypoint'), ),
        migrations.AddField(
            model_name='missionconfig',
            name='mission_waypoints',
            field=models.ManyToManyField(
                related_name='missionconfig_mission_waypoints',
                to='auvsi_suas.Waypoint'), ),
        migrations.AddField(
            model_name='missionconfig',
            name='off_axis_target_pos',
            field=models.ForeignKey(
                related_name='missionconfig_off_axis_target_pos',
                to='auvsi_suas.GpsPosition'), ),
        migrations.AddField(
            model_name='missionconfig',
            name='search_grid_points',
            field=models.ManyToManyField(
                related_name='missionconfig_search_grid_points',
                to='auvsi_suas.Waypoint'), ),
        migrations.AddField(
            model_name='missionconfig',
            name='sric_pos',
            field=models.ForeignKey(
                related_name='missionconfig_sric_pos',
                to='auvsi_suas.GpsPosition'), ),
        migrations.AddField(
            model_name='flyzone',
            name='boundary_pts',
            field=models.ManyToManyField(to='auvsi_suas.Waypoint'), ),
        migrations.AddField(
            model_name='aerialposition',
            name='gps_position',
            field=models.ForeignKey(to='auvsi_suas.GpsPosition'), ),
        migrations.AddField(
            model_name='accesslog',
            name='user',
            field=models.ForeignKey(to=settings.AUTH_USER_MODEL), ),
        migrations.AddField(
            model_name='uastelemetry',
            name='uas_position',
            field=models.ForeignKey(to='auvsi_suas.AerialPosition'), ),
    ]
