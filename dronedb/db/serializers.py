from .models import Data,Shape,Character,Orientation,Color
from rest_framework import serializers


class DataSerializer(serializers.HyperlinkedModelSerializer):
    image = serializers.ImageField(max_length=None, use_url=True,
                                   )
    class Meta:
        model = Data
        fields = '__all__'


class ShapeSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = Shape
        fields = '__all__'

class CharacterSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = Character
        fields = '__all__'

class OrientationSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = Orientation
        fields = '__all__'

class ColorSerializer(serializers.HyperlinkedModelSerializer):
    class Meta:
        model = Color
        fields = '__all__'