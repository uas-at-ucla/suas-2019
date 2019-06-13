from .models import Data, Shape, Character, Orientation, CharColor, ShapeColor
from rest_framework import serializers


class DataSerializer(serializers.ModelSerializer):
    img = serializers.ImageField(
        max_length=None,
        use_url=True,
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


class ShapeColorSerializer(serializers.ModelSerializer):
    class Meta:
        model = ShapeColor
        fields = '__all__'


class CharColorSerializer(serializers.ModelSerializer):
    class Meta:
        model = CharColor
        fields = '__all__'
