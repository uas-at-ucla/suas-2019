from django.shortcuts import render

from rest_framework import generics
from .serializers import DataSerializer, ShapeSerializer, CharacterSerializer, OrientationSerializer, ColorSerializer
from .models import Data, Shape, Character, Orientation, Color

# Create your views here.

class CreateData(generics.ListCreateAPIView):
    queryset =  Data.objects.all()
    serializer_class = DataSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateShape(generics.ListCreateAPIView):
    queryset =  Shape.objects.all()
    serializer_class = ShapeSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateColor(generics.ListCreateAPIView):
    queryset =  Color.objects.all()
    serializer_class = ColorSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateCharacter(generics.ListCreateAPIView):
    queryset =  Character.objects.all()
    serializer_class = CharacterSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateOrientation(generics.ListCreateAPIView):
    queryset =  Orientation.objects.all()
    serializer_class = OrientationSerializer

    def perform_create(self, serializer):
        serializer.save()

