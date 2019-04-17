from django.db import models

# Create your models here.

class Data(models.Model):
    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    Lat = models.FloatField()
    Lng = models.FloatField()
    Alt = models.FloatField()
    Heading = models.IntegerField()
    # N = 1, NE = 2, E = 3.. so on
    Modified = models.BooleanField()
    PipeStage = models.TextField()
    img = models.BinaryField()
class Shape(models.Model):
    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    shape = models.IntegerField()

class Character(models.Model):

    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    char = models.IntegerField()
class Orientation(models.Model):

    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    orientation = models.IntegerField()
class Color(models.Model):

    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    shape = models.IntegerField()

class Character(models.Model):
    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    char = models.IntegerField()
class Orientation(models.Model):
    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    orientation = models.IntegerField()
class Color(models.Model):
    id = models.IntegerField(blank = False, unique = True, primary_key = True);
    color = models.IntegerField()
