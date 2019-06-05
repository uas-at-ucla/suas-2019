# Generated by Django 2.2.1 on 2019-06-05 02:41

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Character',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('char', models.IntegerField()),
            ],
        ),
        migrations.CreateModel(
            name='CharColor',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('color', models.IntegerField()),
            ],
        ),
        migrations.CreateModel(
            name='Data',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('Lat', models.FloatField()),
                ('Lng', models.FloatField()),
                ('Alt', models.FloatField()),
                ('Heading', models.IntegerField()),
                ('Modified', models.BooleanField()),
                ('PipeStage', models.CharField(max_length=50)),
                ('img', models.CharField(max_length=200)),
            ],
        ),
        migrations.CreateModel(
            name='Orientation',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('orientation', models.IntegerField()),
            ],
        ),
        migrations.CreateModel(
            name='Shape',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('shape', models.IntegerField()),
            ],
        ),
        migrations.CreateModel(
            name='ShapeColor',
            fields=[
                ('id', models.IntegerField(primary_key=True, serialize=False, unique=True)),
                ('color', models.IntegerField()),
            ],
        ),
    ]
