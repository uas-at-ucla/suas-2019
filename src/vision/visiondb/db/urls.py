from django.conf.urls import url, include
from rest_framework.urlpatterns import format_suffix_patterns
from rest_framework import routers
from .views import CreateData, CreateShape, CreateColor, CreateCharacter, CreateOrientation


router = routers.DefaultRouter(trailing_slash = False)
router.register('data', CreateData)
router.register('shape', CreateShape)
router.register('color', CreateColor)
router.register('character', CreateCharacter)
router.register('orientation', CreateOrientation)

urlpatterns = router.urls
