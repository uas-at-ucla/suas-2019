from django.conf.urls import url, include
from rest_framework import format_suffix_patterns
import .views


urlpatterns={
    url(r'^data/$', Data.as_view(), name="data")
    url(r'^shape/$', Shape.as_view(), name="shape")
    url(r'^char/$', Character.as_view(), name="char")
    url(r'^orientation/$', Orientation.as_view(), name="orient")
    url(r'^color/$', Color.as_view(), name="color")

}
