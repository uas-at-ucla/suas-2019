AUVSI SUAS Interoperability: Server Source
==========================================

This folder contains the interoperability web server. This directory is a
Django project, with the following components:

  1. `auvsi_suas`: The Django application that contains the main code and logic
     for the server. This includes various models, views, and control logic.
  2. `server`: The Django website that uses the `auvsi_suas` application to
     service user requests.
  3. `manage.py`: A utility program provided by Django which can be used to
     manage the application. This includes creating databases, running the
     development server, etc.
  4. `requirements.txt`: The dependencies required by the server.
  5. `fixtures`: Data which can be loaded into the server.

If the automated setup is utilized, this server is installed in a
[virtualenv](https://virtualenv.pypa.io/en/latest/). Before running any of the
commands below, you must activate the virtualenv with:

```sh
source venv/bin/activate
```


Development Server
------------------

The development server can be used for testing during development. This must
not be used at competition (performance, security, etc.). Teams do not need to
run the development server while testing their integration. Instead, teams
should use the Apache server which is enabled as part of automated setup.

To start the development web server, execute:

``` sh
python manage.py runserver 8080
```

To access the web server from external machines, execute:

``` sh
python manage.py runserver 0.0.0.0:8080
```

The server will start on the local address (localhost, 127.0.0.1) with port
8080. This means the web server is providing a web page at:
http://localhost:8080/

To stop the web server, use Control-C.


Django Administrative Commands
------------------------------

The Django web framework provides the `manage.py` script which can perform many
actions to manage the system. These are documented in the command and on the
Django project's website. Some useful commands:

* `createsuperuser`: Used to create a new superuser. The superuser can login to
  the admin website to edit data and create other users.
* `dumpdata`: Used to dump data. This can be used to backup data, save it for
  later use, or export it for use in other systems.
* `flush`: Used to wipe the database. This will erase all data.
* `loaddata`: Used to load data previously exported via the `dump` command.
* `shell`: Used to open a Python shell with server library access &
  configuration.
* `test`: Used to execute the unittests against a test database.
