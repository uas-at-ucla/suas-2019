Automation
==========

This section describes how to write admin automation for the interop server by
writing scripts which connect directly to the database and bypass the
webserver. Note that the database and Django configurations only permit local
access, so you'll need to run any such scripts locally. This may be useful
while testing to automatically setup test cases (e.g. test mission) or analyze
results (e.g. turning radius).

**Configure Django**. The first step when writing a script is to import Django
and have it setup for programmatic access. Start the script with the following:

.. code-block:: python

    import os
    import sys

    # Add server code to Python PATH for imports.
    sys.path.append('/path/to/server')
    # Add environment variable to get Django settings file.
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "server.settings")

    # Setup Django.
    from django.core.wsgi import get_wsgi_application
    application = get_wsgi_application()

**Import SUAS Code**. The next step is to import the various models that you
want to access. The following shows how to import the ``MissionConfig`` and
``GpsPosition`` models.

.. code-block:: python

    from auvsi_suas.models.gps_position import GpsPosition
    from auvsi_suas.models.mission_config import MissionConfig

**Read & Write Objects**. The following shows how to read all ``MissionConfig``
objects and save a ``GpsPosition`` object. For details on how to perform
actions like this on Django models, see the `Django Tutorials
<https://docs.djangoproject.com/en/1.10/intro/>`__.

.. code-block:: python

    # Print all mission objects.
    print MissionConfig.objects.all()

    # Create and save a GPS position.
    gpos = GpsPosition(latitudel=38.145335, longitude=-76.427512)
    gpos.save()

**Full Example**. See the `create_teams.py
<https://github.com/auvsi-suas/interop/blob/master/server/tools/team_creator/create_teams.py>`__
script for an end-to-end example of how the judges use this programmatic access
to automate team account creation.
