from .models import Data,Shape,Character,Orientation,Color
from rest_framework import serializers


class DataSerializer(serializers.ModelSerializer):
    img = serializers.ImageField(max_length=None, use_url=True,
                                   )
    class Meta:
        model = Data
        fields = '__all__'


class ShapeSerializer(serializers.ModelSerializer):
    class Meta:
        model = Shape
        fields = '__all__'

class CharacterSerializer(serializers.ModelSerializer):
    class Meta:
        model = Character
        fields = '__all__'

class OrientationSerializer(serializers.ModelSerializer):
    class Meta:
        model = Orientation
        fields = '__all__'

class ColorSerializer(serializers.ModelSerializer):
    class Meta:
        model = Color
        fields = '__all__'