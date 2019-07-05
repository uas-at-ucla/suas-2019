from django.contrib import admin
from .models import Data, Shape, Character, Orientation, ShapeColor, CharColor

# Register your models here.
admin.site.register(Data)
admin.site.register(Shape)
admin.site.register(Character)
admin.site.register(Orientation)
admin.site.register(ShapeColor)
admin.site.register(CharColor)
