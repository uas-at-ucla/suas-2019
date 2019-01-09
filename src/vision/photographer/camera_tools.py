import logging
import os
import signal
import subprocess
import sys

import gphoto2 as gp


# (add context if error)
def get_summary(camera):
    # list summary of camera
    text = gp.check_result(gp.gp_camera_get_summary(camera))
    print('Summary')
    print('=======')
    print('\n')
    print(text.text)


def take_picture(camera):
    # you can change to continuous shooting by changing the settings and then using this command
    #print(gp.check_result(gp.list_config(camera)))

    # take a picture
    print('Capturing single image')

    # mark down file path
    file_path = gp.check_result(
        gp.gp_camera_capture(camera, gp.GP_CAPTURE_IMAGE))


def get_pid(name):
    return subprocess.check_output(["pidof", name])


def main():
    # logging
    logging.basicConfig(
        filename='camera_test.log',
        format='%(levelname)s: %(name)s: %(message)s',
        level=logging.WARNING)
    gp.check_result(gp.use_python_logging())

    killProcess = "gvfsd-photo2"
    pid = get_pid(killProcess)

    if (pid == 0):
        os.kill(pid, signal.SIGTERM)

    # instantiate camera and context
    #context = gp.Context()

    # if can autodetect camera
    #if hasattr(gp, 'gp_camera_autodetect'):
    #camera = gp.check_result(gp.gp_camera_autodetect())
    #else:
    camera = gp.check_result(gp.gp_camera_new())

    print('Please connect and switch on your camera')
    '''
    # error checking the camera (add context if error)
    while True:
        error = gp.gp_camera_init(camera, context)
        if error >= gp.GP_OK:
            # found camera, may continue
            break
        if error != gp.GP_ERROR_MODEL_NOT_FOUND:
            # some other problem that it will anounce
            raise gp.GPhoto2Error(error)
        # if there's no camera try again in 5 seconds
        time.sleep(5)
    '''

    #look at configs and deal with them appropriately. Look at continuous script and replicate it in terms of python code into this file here. experiment with shit.

    gp.check_result(gp.gp_camera_init(camera))

    # get summary (can implement a choice later on) (add context if error)
    get_summary(camera)

    # take a picture
    #take_picture(camera)
    config = gp.gp_camera_get_config(camera)
    camera_list = gp.camera.CameraListConfigFunc()
    print(config)

    # exit camera (add context if error)
    gp.check_result(gp.gp_camera_exit(camera))


if __name__ == "__main__":
    sys.exit(main())
