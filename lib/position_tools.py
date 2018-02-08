from math import radians, cos, sin, asin, sqrt

COORD_TO_METERS = 111132.954

class Geocoord3D:
    def __init__(self, lat, lng, alt):
        self.lat = lat
        self.lng = lng
        self.alt = alt

class Vector3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, vec):
        return Vector3D((self.x + vec.x), (self.y + vec.y), (self.z + vec.z))

    def __sub__(self, vec):
        return Vector3D((self.x - vec.x), (self.y - vec.y), (self.z - vec.z))

    def __mul__(self, scalar):
        return Vector3D((scalar * self.x), (scalar * self.y), (scalar * self.z))

    def __rmul__(self, scalar):
        return Vector3D((scalar * self.x), (scalar * self.y), (scalar * self.z))

    def __div__(self, scalar):
        return Vector3D((self.x / scalar), (self.y / scalar), (self.z / scalar))

    def set(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def set(self, vec):
        self.x = vec.x
        self.y = vec.y
        self.z = vec.z

    def dot(self, vec):
        return (self.x * vec.x) + (self.y * vec.y) + (self.z * vec.z)

    def norm(self):
        return sqrt(self.dot(self))

    def print_vec(self):
        print("(" + str(self.x)   + \
              ",\t" + str(self.y) + \
              ",\t" + str(self.z) + ')')

def distance(coord1, coord2):
    d_ground = distance_2D(coord1.lng, coord1.lat, \
            coord2.lng, coord2.lat)
    d_alt = coord2.alt - coord1.alt
    return sqrt(d_ground ** 2 + d_alt ** 2)

def distance_2D(lon1, lat1, lon2, lat2):
    # Uses haversine
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371 * 1000 # Radius of earth in meters. Use 3956 for miles
    return c * r

def point_to(home, remote):
    # In NED coordinates
    x = (remote.lat - home.lat) * COORD_TO_METERS
    y = (remote.lng - home.lng) * COORD_TO_METERS
    z = -(remote.alt - home.alt) #because NED

    vec = Vector3D(x, y, z)

    length = vec.norm()

    return (vec / length)

def mix_directions(unit1, unit2, ratio):
    comp = 1 - ratio
    weightSum = Vector3D( \
            unit1.x*ratio + unit2.x*comp, \
            unit1.y*ratio + unit2.y*comp, \
            unit1.z*ratio + unit2.z*comp  )
    length = sqrt(weightSum.x**2 + weightSum.y**2 + weightSum.z**2)

    return Vector3D( \
            weightSum.x / length, \
            weightSum.y / length, \
            weightSum.z / length  )


