import math
gs_lat = float(input("Latitude: "))
gs_lon = float(input("Longitude: "))

# getting telemetry from drone/ground station
dr_lat = 0 #TODO
dr_lon = 0 #TODO

#Yaw
yaw_angle = 0.00 # using telemetry and lat/long to get angles
R = 6371.00 * 10 ** 6
phi1 = math.radians(gs_lat)
phi2 = math.radians(dr_lat) # height comes in as h, drone co-ordinates as dr_lat/dr_lon
delta_phi = math.radians(dr_lat - gs_lat)
delta_lambda = math.radians(dr_lon - gs_lon)

a = math.sin(delta_phi / 2) * math.sin(delta_phi / 2) + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) * math.sin(delta_lambda / 2)

c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

dist_dr_gs = R * c

yaw_angle = yaw_angle + math.atan(h/dist_dr_gs)

# controlling the servo (calculations of how far it needs to go)

# intialize servo_ext and int_yaw
# k =  dist. bw hinge and connecting pt / sin(angle bw servo and base)
# servo_ext = servo_ext + (sin (yaw_angle) - sin (int_yaw))/k

#Pitch
x_gs = R * math.cos(math.radians(gs_lat)) * math.cos(math.radians(gs_lon))
y_gs = R * math.cos(math.radians(gs_lat)) * math.sin(math.radians(gs_lon))

x_dr = R * math.cos(math.radians(dr_lat)) * math.cos(math.radians(dr_lon))
y_dr = R * math.cos(math.radians(dr_lat)) * math.sin(math.radians(dr_lon))

#int_pitch = 0.00
pitch_angle = 0.00
change_pos = (y_dr - y_gs) / (x_dr - x_gs)
pitch_angle = pitch_angle - int_pitch + atan(change_pos)

# output into motors to change to those angles
