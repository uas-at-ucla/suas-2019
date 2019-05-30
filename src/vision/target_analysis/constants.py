# Histogram bin size (must divide 180)
BIN_SIZE = 4

# Camera parameters
IMAGE_WIDTH = 5184
SENSOR_WIDTH = 22.3e-3
FOCAL_LEN = 18e-3

# Pixels to sensor size scale factor
PX_TO_SENSOR = SENSOR_WIDTH / IMAGE_WIDTH

# Pixels to meters scale factor at specified altitude
def px_to_gnd(alt):
    return PX_TO_SENSOR * alt / FOCAL_LEN

# Valid contour area range (meters squared)
GND_AREA_MIN = 0.25 ** 2
GND_AREA_MAX = 1.00 ** 2

# TODO: currently hardcoded for testing
ALTITUDE = 30

# Maximum number of points used to describe contour
DETAIL_MAX = 1000

# Create overlap of one bin size and keep threshold centered on hue
def range_lower(hue):
    return (hue - BIN_SIZE, 0, 32)

def range_upper(hue):
    return (hue + BIN_SIZE, 255, 255)


