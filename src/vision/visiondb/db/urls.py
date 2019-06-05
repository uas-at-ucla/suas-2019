from django.conf.urls import url, include
from rest_framework.urlpatterns import format_suffix_patterns
from rest_framework import routers
from .views import CreateData, CreateShape, CreateShapeColor, CreateCharColor, CreateCharacter, CreateOrientation

router = routers.DefaultRouter(trailing_slash=False)
router.register('data', CreateData, basename='data')
router.register('shape', CreateShape)
router.register('shape_color', CreateShapeColor)
router.register('char_color', CreateCharColor)
router.register('character', CreateCharacter)
router.register('orientation', CreateOrientation)

urlpatterns = router.urls
