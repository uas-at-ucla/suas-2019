Debugging
=========

This document contains useful means of debugging problems.

**Server Log File**. The competition server logs important events and debug
information to a temporary log file. The server writes useful debugging
information to ``/var/log/apache2/interop_server_error.log``. Users can inspect
this file during or after competition server execution to debug
interoperability. For example, if your implementation is getting denied due to
invalid user credentials, the log will contain a message stating such and what
request parameters were provided at time of denial.

From a bash shell in the container, print out the log:

.. code-block:: bash

    cat /var/log/apache2/interop_server_error.log

Watch for changes to the log:

.. code-block:: bash

    tail -f /var/log/apache2/interop_server_error.log

**Debug Mode**. The `settings.py
<https://github.com/auvsi-suas/interop/blob/master/server/server/settings.py>`__
file contains useful settings. One such setting is ``Debug``, which you can set
to ``True`` to get additional debugging information. You need to restart the
server for it to take affect.

.. code-block:: bash

    sudo service apache2 restart
