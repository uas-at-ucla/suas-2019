from PIL import Image
import numpy
import math

#order: topleft, topright, botright, botleft
#transforms so that pb goes to pa
def find_coeffs(pa, pb):
	matrix = []
	for p1, p2 in zip(pa, pb):
		matrix.append([p1[0], p1[1], 1, 0, 0, 0, -p2[0]*p1[0], -p2[0]*p1[1]])
		matrix.append([0, 0, 0, p1[0], p1[1], 1, -p2[1]*p1[0], -p2[1]*p1[1]])

	A = numpy.matrix(matrix, dtype=numpy.float)
	B = numpy.array(pb).reshape(8)

	res = numpy.dot(numpy.linalg.inv(A.T * A) * A.T, B)
	return numpy.array(res).reshape(8)

#angles in degrees xD
#arguments:
#src: source image file name, dest: destination file name 
#angleOfDepression: angle downward from horizontal
#verticalFieldOfView: vertical angle of camera field of view
#altitude, latitude, longitude: initial camera altitude, latitude, longitude
#heading: degrees EAST of NORTH
def rectify(src, dest, angleOfDepression, verticalFieldOfView, altitude, latitude, longitude, heading):
	img = Image.open(src)
	w,h = img.size
	pi = math.pi
	b = verticalFieldOfView*pi/180
	a = (angleOfDepression-(verticalFieldOfView/2))*pi/180

	w1 = int(w*math.sin(a+b)/math.cos((pi/2.0)-a)) #a != 0
	h1 = int(h*(math.sin(a+b)*math.tan((pi/2.0)-a)-math.cos(a+b))/math.sqrt(2-2*math.cos(b)))

	pa = [((w-w1)/2,0),((w+w1)/2,0),(w,h1),(0,h1)]
	pb = [(0,0),(w,0),(w,h),(0,h)]

	coeffs = find_coeffs(pa,pb)
	img.transform((w,h1), Image.PERSPECTIVE, coeffs, Image.BICUBIC).save(dest)

	head = heading * pi/180
	h2 = altitude * (math.tan(pi/2-a)-1/math.tan(a+b))
	displacement = h2/2 + altitude/math.tan(a+b)
	
	newHeight = h2/(2*math.tan(b/2))
	newLatitude = latitude + displacement * math.cos(head)
	newLongitude = longitude + displacement * math.sin(head)

	return (newHeight, newLatitude, newLongitude)
