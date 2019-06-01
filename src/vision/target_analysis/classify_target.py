import cv2 as cv
import shapely
import shapely.ops
import shapely.geometry as sg
import numpy as np
import matplotlib.pyplot as plt
fig, ax = plt.subplots()

__all__ = ['classifyShape', 'classifyLetter', 'classifyColor']

class customShape:
    def __init__(self, poly, name, delSym):
        self.poly = poly
        self.sides = len(poly.exterior.coords) - 1
        self.name = name
        self.delSym = delSym

def plotcoords(coords, label = None):
    pts = list(coords)
    x, y = zip(*pts)
    ax.plot(x, y, label = label)


def carttopol(x, y):
    r = np.sqrt(x**2 + y**2)
    omega = np.arctan2(y, x)
    return (r, omega)


def poltocart(r, omega):
    x = r * np.cos(omega)
    y = r * np.sin(omega)
    return (x, y)


def genRegPoly(n, area = 1):
    vert = []
    delomega = 2*np.pi/n
    for v in range(n):
        vert.append(poltocart(1, v*delomega))

    poly = sg.Polygon(vert)
    sf = area/(poly.area**0.5)

    return shapely.affinity.scale(poly, sf, sf)

def moveToOrigin(contour):
    M = cv.moments(contour)
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    return contour - (cx, cy)

# Input: degrees 0-179 on color wheel
# Output: string indicating which color it is

def classifyColor(hue):
    hue = int(hue)
    cdict = {
        'red': range(-7, 15),
        'orange': range(15, 23),
        'yellow': range(23, 38),
        'green': range(38, 83),
        'blue': range(83, 135),
        'purple': range(135, 150),
        'magenta': range(150, 173),
    }

    while(hue >= 173):
        hue -= 180
    while(hue < -7):
        hue += 180

    for color in cdict:
        if hue in cdict[color]:
            return color

    return None

# Input: list of lists containing 2 integers (i.e. list of xy-pairs)
# Output: string indicating which shape it is

# current implementation of classify shape should be able to recognize any shape in any rotation
#   this also covers classifying letters because letters can be represented as shapes

# uses pyplot to display and show the given contour and the believed shape it matches

def classifyShape(contour):
    contour = [tuple(point) for point in moveToOrigin(contour).reshape(-1, 2)]
    shapes = []

    for sides in range(3, 10):
        shapes.append(customShape(genRegPoly(sides), str(sides) + '-gon', 360/sides))

    # for shape in shapes:
    #     print(shape.name, shape.sides, shape.delSym)

    mPoly = sg.Polygon(sg.MultiPoint(contour))
    sf = 1/(mPoly.area**0.5)
    mPoly = shapely.affinity.scale(mPoly, sf, sf, origin=(0,0))

    best, b_shape = mPoly.difference(shapes[0].poly).area, shapes[0]
    for shape in shapes:
        for i in range(1, int(360/shape.delSym) + 1):
            diff = mPoly.difference(shapely.affinity.rotate(shape.poly, i)).area
            if diff < best:
                best, b_shape = diff, shape

    plotcoords(mPoly.exterior.coords, 'givenpoly')
    plotcoords(b_shape.poly.exterior.coords, 'matchedpoly')
    ax.legend(loc='upper left')
    ax.axis('equal')
    plt.show()
    return b_shape.name

# Input: list of lists containing 2 integers (i.e. list of xy-pairs)
# Output: character indicating which letter it is

def classifyLetter(contour):
    pass

#currently testing using a near square quadrilateral
def main():
    testContour = np.array([[[1, 0]], [[-.1, .9]], [[-1.2, -.1]], [[-.1, -.8]]])
    print(classifyShape(testContour))

if __name__ == '__main__':
    main()
