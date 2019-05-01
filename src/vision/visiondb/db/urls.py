from django.conf.urls import url, include
from rest_framework.urlpatterns import format_suffix_patterns
from .views import CreateData, CreateShape, CreateColor, CreateCharacter, CreateOrientation


urlpatterns={
    url(r'^data/$', CreateData.as_view(), name="data"),
    url(r'^shape/$', CreateShape.as_view(), name="shape"),
    url(r'^char/$', CreateCharacter.as_view(), name="char"),
    url(r'^orientation/$', CreateOrientation.as_view(), name="orient"),
    url(r'^color/$', CreateColor.as_view(), name="color"),

}

urlpatterns = format_suffix_patterns(urlpatterns)