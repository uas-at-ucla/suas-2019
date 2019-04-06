import numpy as np
import math

from config import Config

def calculate_target_coordinates(
        target_pos_pixel,
        parent_img_real_coords,
        parent_img_dimensions_pixel,
        altitude,
        heading,
        focal_length=Config.LENS_FOCAL_LENGTH.value,
        sensor_dimensions=Config.CAMERA_SENSOR_DIMENSIONS.value):
    """ Calculate the coordinates of a target in an image.

    Arguments:
    target_pos_pixel -- (pixels) the tuple(x, y) position of the target 
                        (top-left origin)
    parent_img_real_coords -- (ISO 6709, degrees) the (lat, lng) position of 
                              the center of the image
    parent_img_dimensions_pixel -- (pixels) the dimensions of the image
    altitude -- (m) the altitude when the image was taken
    heading -- (+CW degrees) the direction of the top of the picture
    sensor_dimensions -- (mm) the dimensions of the sensor 
                         !!! ratio must match image !!!
    focal_length -- (mm) the focal_length of the lens
    """  # noqa
    # yapf: disable

    # the algorithm assumes positive degrees are CCW
    # but the heading is given with +CW
    heading = -heading

    # get the average of ratio from the two dimensions
    pixel_to_sensor_ratio = 0
    for pixel_dim in parent_img_dimensions_pixel:
        for sensor_dim in sensor_dimensions:
            pixel_to_sensor_ratio += sensor_dim / pixel_dim
    pixel_to_sensor_ratio /= 2

    scaling_ratio = altitude / focal_length * pixel_to_sensor_ratio
    parent_img_center = tuple(int(component / 2)
                              for component in parent_img_dimensions_pixel)
    target_vec = np.array([
        (scaling_ratio * (parent_img_center[0] - target_pos_pixel[0])),
        (scaling_ratio * (parent_img_center[1] - target_pos_pixel[1]))
        ],
        dtype=np.float64)
    rotation_matrix = np.array([[math.cos(heading), -math.sin(heading)],
                                [math.sin(heading),  math.cos(heading)]])
    # yapf: enable
    target_vec = rotation_matrix @ target_vec
    new_lat = parent_img_real_coords[0] + (
        -target_vec[1] / Config.R_EARTH.value) * 180 / math.pi
    # this is an approximation assuming the latitude remains constant
    # or is very small compared to the radius of the earth
    new_lng = parent_img_real_coords[1] + (target_vec[0] / (
        Config.R_EARTH.value / math.cos(new_lat))) * 180 / math.pi
    return (new_lat, new_lng)
