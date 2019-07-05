from django.db import models

# Create your models here.


class Data(models.Model):
    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    Lat = models.FloatField()
    Lng = models.FloatField()
    Alt = models.FloatField()
    Heading = models.IntegerField()
    # N = 1, NE = 2, E = 3.. so on
    Modified = models.BooleanField()
    PipeStage = models.CharField(max_length=50)
    img = models.CharField(max_length=200)


class Shape(models.Model):
    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    shape = models.IntegerField()


class Character(models.Model):

    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    char = models.IntegerField()


class Orientation(models.Model):

    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    orientation = models.IntegerField()


class ShapeColor(models.Model):

    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    color = models.IntegerField()


class Character(models.Model):
    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    char = models.IntegerField()


class CharColor(models.Model):
    id = models.IntegerField(blank=False, unique=True, primary_key=True)
    color = models.IntegerField()
