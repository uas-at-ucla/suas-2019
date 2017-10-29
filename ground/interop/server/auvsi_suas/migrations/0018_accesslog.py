# -*- coding: utf-8 -*-
"""This two-part migration converts AccessLog to an abstract base class.

Django is not capable of running a simple migration to move the AccessLog
fields into the respective subtypes, so we must do an elaborate song-and-dance.

1. Create new models in the final form we desire.
2. Manually copy each entry from the old model to the new model.
3. Delete the old models.

In the next migration:
4. Rename the new models to their final name (same as the old models).
5. Delete the AccessLog model.

The last two steps must be done in the next model because Django cannot handle
both deleting the old models and renaming the new models to the old names
within the same migration.
"""

from __future__ import print_function
from __future__ import unicode_literals

from django.db import models, migrations
import datetime
from django.utils import timezone
from django.conf import settings
import functools
import logging

big_migration = False


def migration_warning(apps, scheme_editor):
    ObstacleAccessLog = apps.get_model('auvsi_suas', 'obstacleaccesslog')
    ServerInfoAccessLog = apps.get_model('auvsi_suas', 'serverinfoaccesslog')
    UasTelemetry = apps.get_model('auvsi_suas', 'uastelemetry')

    global big_migration
    if ObstacleAccessLog.objects.count() > 10000:
        big_migration = True
    elif ServerInfoAccessLog.objects.count() > 10000:
        big_migration = True
    elif UasTelemetry.objects.count() > 10000:
        big_migration = True

    if big_migration:
        print("""
Warning: If you have a large database (e.g., >100k UasTelemetry objects)
then this migration may take a long time and require a large amount of
memory (>2GB).

If the following migration asks for confirmation to remove the stale accesslog
table, say yes.""")


def copy_mission_clock_event(apps, scheme_editor):
    if big_migration:
        print('Copying MissionClockEvent')

    MissionClockEvent = apps.get_model('auvsi_suas', 'missionclockevent')
    NewMissionClockEvent = apps.get_model('auvsi_suas', 'newmissionclockevent')

    for entry in MissionClockEvent.objects.all():
        new = NewMissionClockEvent(
            timestamp=entry.timestamp,
            user=entry.user,
            team_on_clock=entry.team_on_clock,
            team_on_timeout=entry.team_on_timeout)
        new.save()


def copy_obstacle_access_log(apps, scheme_editor):
    if big_migration:
        print('Copying ObstacleAccessLog')

    ObstacleAccessLog = apps.get_model('auvsi_suas', 'obstacleaccesslog')
    NewObstacleAccessLog = apps.get_model('auvsi_suas', 'newobstacleaccesslog')

    for entry in ObstacleAccessLog.objects.all():
        new = NewObstacleAccessLog(timestamp=entry.timestamp, user=entry.user)
        new.save()


def copy_server_info_access_log(apps, scheme_editor):
    if big_migration:
        print('Copying ServerInfoAccessLog')

    ServerInfoAccessLog = apps.get_model('auvsi_suas', 'serverinfoaccesslog')
    NewServerInfoAccessLog = apps.get_model('auvsi_suas',
                                            'newserverinfoaccesslog')

    for entry in ServerInfoAccessLog.objects.all():
        new = NewServerInfoAccessLog(
            timestamp=entry.timestamp, user=entry.user)
        new.save()


def copy_takeoff_or_landing_event(apps, scheme_editor):
    if big_migration:
        print('Copying TakeoffOrLandingEvent')

    TakeoffOrLandingEvent = apps.get_model('auvsi_suas',
                                           'takeofforlandingevent')
    NewTakeoffOrLandingEvent = apps.get_model('auvsi_suas',
                                              'newtakeofforlandingevent')

    for entry in TakeoffOrLandingEvent.objects.all():
        new = NewTakeoffOrLandingEvent(
            timestamp=entry.timestamp,
            user=entry.user,
            uas_in_air=entry.uas_in_air)
        new.save()


def copy_uas_telemetry(apps, scheme_editor):
    if big_migration:
        print('Copying UasTelemetry')

    UasTelemetry = apps.get_model('auvsi_suas', 'uastelemetry')
    NewUasTelemetry = apps.get_model('auvsi_suas', 'newuastelemetry')

    for entry in UasTelemetry.objects.all():
        new = NewUasTelemetry(
            timestamp=entry.timestamp,
            user=entry.user,
            uas_position=entry.uas_position,
            uas_heading=entry.uas_heading)
        new.save()


class Migration(migrations.Migration):

    dependencies = [
        migrations.swappable_dependency(settings.AUTH_USER_MODEL),
        ('auvsi_suas', '0017_missionconfig_targets'),
    ]

    operations = [
        migrations.RunPython(migration_warning),
        migrations.CreateModel(
            name='NewMissionClockEvent',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                # Use default instead of auto_now_add so that this can be
                # overridden.
                ('timestamp', models.DateTimeField(
                    default=timezone.now, db_index=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
                ('team_on_clock', models.BooleanField()),
                ('team_on_timeout', models.BooleanField()),
            ]),
        migrations.RunPython(copy_mission_clock_event),
        migrations.CreateModel(
            name='NewObstacleAccessLog',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(
                    default=timezone.now, db_index=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
            ]),
        migrations.RunPython(copy_obstacle_access_log),
        migrations.CreateModel(
            name='NewServerInfoAccessLog',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(
                    default=timezone.now, db_index=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
            ]),
        migrations.RunPython(copy_server_info_access_log),
        migrations.CreateModel(
            name='NewTakeoffOrLandingEvent',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(
                    default=timezone.now, db_index=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
                ('uas_in_air', models.BooleanField()),
            ]),
        migrations.RunPython(copy_takeoff_or_landing_event),
        migrations.CreateModel(
            name='NewUasTelemetry',
            fields=[
                ('id', models.AutoField(
                    verbose_name='ID',
                    serialize=False,
                    auto_created=True,
                    primary_key=True)),
                ('timestamp', models.DateTimeField(
                    default=timezone.now, db_index=True)),
                ('user', models.ForeignKey(to=settings.AUTH_USER_MODEL)),
                ('uas_position',
                 models.ForeignKey(to='auvsi_suas.AerialPosition')),
                ('uas_heading', models.FloatField()),
            ]),
        migrations.RunPython(copy_uas_telemetry),
        migrations.DeleteModel(name='MissionClockEvent'),
        migrations.DeleteModel(name='ObstacleAccessLog'),
        migrations.DeleteModel(name='ServerInfoAccessLog'),
        migrations.DeleteModel(name='TakeoffOrLandingEvent'),
        migrations.DeleteModel(name='UasTelemetry'),
        migrations.DeleteModel(name='AccessLog'),
    ]
