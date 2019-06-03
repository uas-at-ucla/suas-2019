from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter

import db.routing

application = ProtocolTypeRouter({
    # http -> django views is added by default
    'websocket':
    AuthMiddlewareStack(URLRouter(db.routing.websocket_urlpatterns)),
})
