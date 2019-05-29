from django.shortcuts import render

from rest_framework import viewsets
from .serializers import DataSerializer, ShapeSerializer, CharacterSerializer, OrientationSerializer, ColorSerializer
from .models import Data, Shape, Character, Orientation, Color

# Create your views here.

class CreateData(viewsets.ModelViewSet):
    queryset =  Data.objects.all()
    serializer_class = DataSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateShape(viewsets.ModelViewSet):
    queryset =  Shape.objects.all()
    serializer_class = ShapeSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateColor(viewsets.ModelViewSet):
    queryset =  Color.objects.all()
    serializer_class = ColorSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateCharacter(viewsets.ModelViewSet):
    queryset =  Character.objects.all()
    serializer_class = CharacterSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateOrientation(viewsets.ModelViewSet):
    queryset =  Orientation.objects.all()
    serializer_class = OrientationSerializer

    def perform_create(self, serializer):
        serializer.save()
