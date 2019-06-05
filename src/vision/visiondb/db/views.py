from django.shortcuts import render

from rest_framework import viewsets
from .serializers import DataSerializer, ShapeSerializer, CharacterSerializer, OrientationSerializer, ShapeColorSerializer, CharColorSerializer
from .models import Data, Shape, Character, Orientation, ShapeColor, CharColor

# Create your views here.


class CreateData(viewsets.ModelViewSet):
    serializer_class = DataSerializer

    def perform_create(self, serializer):
        serializer.save()

    def get_queryset(self):
        min_lat = self.request.query_params.get('min_lat')
        min_lng = self.request.query_params.get('min_lng')
        max_lat = self.request.query_params.get('max_lat')
        max_lng = self.request.query_params.get('max_lng')

        if (min_lat is None or min_lng is None or max_lat is None
                or max_lng is None):
            return Data.objects.all()
        else:
            return Data.objects.filter(
                Lat__gte=min_lat, Lng__gte=min_lng).filter(Lat__lte=max_lat,
                                                           Lng__lte=max_lng)


class CreateShape(viewsets.ModelViewSet):
    queryset = Shape.objects.all()
    serializer_class = ShapeSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateShapeColor(viewsets.ModelViewSet):
    queryset = ShapeColor.objects.all()
    serializer_class = ShapeColorSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateCharColor(viewsets.ModelViewSet):
    queryset = CharColor.objects.all()
    serializer_class = CharColorSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateCharacter(viewsets.ModelViewSet):
    queryset = Character.objects.all()
    serializer_class = CharacterSerializer

    def perform_create(self, serializer):
        serializer.save()


class CreateOrientation(viewsets.ModelViewSet):
    queryset = Orientation.objects.all()
    serializer_class = OrientationSerializer

    def perform_create(self, serializer):
        serializer.save()
