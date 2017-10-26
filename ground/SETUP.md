# Installing the Ground Control Software

# Content
* Cloning Procedure
* Installing Packages
* Compiling

## Cloning Procedure
This assumes you have a [Git account and SSH
key](https://github.com/uas-at-ucla/suas_2018/blob/master/SETUP.md) already.

   ```bash
   git clone git@github.com:uas-at-ucla/ground.git
   ```

## Installing Packages
We are assuming you have shell access to your computer. For Windows, I would
recommend looking into [Visual Studio Code](https://code.visualstudio.com/) for
its [embedded
terminal](https://code.visualstudio.com/docs/editor/integrated-terminal); it
uses the Ubuntu bash shell.

For Ubuntu, use the apt-get package manager.
   ```bash
   sudo apt-get install python-venv
   sudo apt-get install python3-flask
   ```

On MacOS, run the following command on the terminal in the ground station directory.
   ```bash
   sudo pip install virtualenv
   pip install Flask
   ```

If all the above was successful, then you are done. Otherwise, you may want to
refer to a [more descriptive
documentation](http://flask.pocoo.org/docs/0.12/installation/).

## Compiling

STILL NEEDS TO BE WORKED ON. I will work on developing a bash script to get
everything we need running.

To launch at your localhost, run the following command.
   ```bash
   cd /path/to/ground/
   export FLASK_APP=start_interface.py
   flask run
   ```
