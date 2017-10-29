from django.conf.urls import patterns, include, url
from django.contrib import admin

admin.autodiscover()

# yapf: disable
urlpatterns = patterns(
    '',
    url(r'^admin/', include(admin.site.urls)),
    url(r'^', include('auvsi_suas.views.urls', namespace="auvsi_suas"))
)
# yapf: enable
