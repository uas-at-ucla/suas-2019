Client Library
==============

The competition provides a Python client library for communicating with the
interoperability server. It can be found in ``/client/interop/`` in the
`repository
<https://github.com/auvsi-suas/interop/tree/master/client/interop>`_ and in the
client Docker image.

Note this library does not provide client-side request validation. It relies on
the server to reject invalid requests, like rejecting objects with an invalid
shape.

.. automodule:: interop
   :members: Client,
             AsyncClient,
             InteropError,
             Mission,
             FlyZone,
             Waypoint,
             GpsPosition,
             Telemetry,
             StationaryObstacle,
             MovingObstacle,
             Odlc
   :special-members: __init__
