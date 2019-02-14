from django.db import models

# Create your models here.

class Data(models.Model):
    Lat = models.FloatField()
    Lng = models.FloatField()
    Alt = models.FloatField()
    Heading = models.IntegerField()
    # N = 1, NE = 2, E = 3.. so on
    Modified = models.BooleanField()
    PipeStage = models.TextField()
    img = models.BinaryField()
class Shape(models.Model):
    shape = models.IntegerField()

class Character(models.Model):
    char = models.IntegerField()
class Orientation(models.Model):
    orientation = models.IntegerField()
class Color(models.Model):
    color = models.IntegerField()
