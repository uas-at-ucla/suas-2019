# THIS FILE CANNOT CONTAIN BACKSLASHES OR DOUBLE QUOTES
mission1 = {
    'altitude_msl_min': 0,
    'altitude_msl_max': 750, #feet
    'boundary_pts': [
        { 'latitude': 38.14627, 'longitude': -76.42816 },
        { 'latitude': 38.15162, 'longitude': -76.42868 },
        { 'latitude': 38.15189, 'longitude': -76.43147 },
        { 'latitude': 38.15059, 'longitude': -76.43536 },
        { 'latitude': 38.14757, 'longitude': -76.43234 },
        { 'latitude': 38.14467, 'longitude': -76.43295 },
        { 'latitude': 38.14326, 'longitude': -76.43477 },
        { 'latitude': 38.14046, 'longitude': -76.43264 },
        { 'latitude': 38.14072, 'longitude': -76.42601 },
        { 'latitude': 38.14376, 'longitude': -76.42121 },
        { 'latitude': 38.14735, 'longitude': -76.42321 },
        { 'latitude': 38.14613, 'longitude': -76.42665 }
    ],
    'search_grid_points': [
        { 'latitude': 38.14576, 'longitude': -76.42969 },
        { 'latitude': 38.14323, 'longitude': -76.43379 },
        { 'latitude': 38.14123, 'longitude': -76.43233 },
        { 'latitude': 38.14139, 'longitude': -76.42709 },
        { 'latitude': 38.14221, 'longitude': -76.42611 }
    ],
    'waypoints': [
        { 'latitude': 38.15079, 'longitude': -76.43044, 'altitude_msl': 150 },
        { 'latitude': 38.14961, 'longitude': -76.43295, 'altitude_msl': 200 },
        { 'latitude': 38.14218, 'longitude': -76.42564, 'altitude_msl': 200 },
        { 'latitude': 38.14388, 'longitude': -76.42263, 'altitude_msl': 200 },
        { 'latitude': 38.14564, 'longitude': -76.42424, 'altitude_msl': 200 },
        { 'latitude': 38.14400, 'longitude': -76.42875, 'altitude_msl': 250 }
    ],
    'emergent_object': { 'latitude': 38.145762, 'longitude': -76.423065 },
    'off_axis_object': { 'latitude': 38.147635, 'longitude': -76.427249 },
    'home': { 'latitude': 38.145323, 'longitude': -76.428000 },
    'air_drop': { 'latitude': 38.145830, 'longitude': -76.426391 },
    'stationary_obstacles': [
        { 'latitude': 38.15079, 'longitude': -76.43044, 'radius': 100, 'height': 100 }
    ],
    'moving_obstacles': [
        { 
            'radius': 300,
            'speed_avg': 30,
            'waypoints': [
                { 'latitude': 38.15079, 'longitude': -76.43044, 'altitude_msl': 100},
                { 'latitude': 38.14139, 'longitude': -76.42709, 'altitude_msl': 150}
            ]
        }
    ]
}


mission2 = {
    'altitude_msl_min': 0,
    'altitude_msl_max': 750,
    'boundary_pts': [
        { 'latitude': 34.17403661272721, 'longitude':   -118.48180172782406 }, 
        { 'latitude': 34.17369931099332, 'longitude': -118.48175881247982 }, 
        { 'latitude': 34.1730247034811, 'longitude': -118.48220942359433},
        { 'latitude': 34.17126714805826, 'longitude': -118.4779178891705 },
        { 'latitude': 34.171853003932654, 'longitude': -118.47630856376156 },
        { 'latitude': 34.17511951970793, 'longitude': -118.47753165107235 }
    ],
    'search_grid_points': [
        { 'latitude': 34.17408936092683, 'longitude': -118.48073169311522 },
        { 'latitude': 34.17352127337439, 'longitude': -118.48036691268919 }, 
        { 'latitude': 34.17343250934888, 'longitude': -118.47729846557615 },
        { 'latitude': 34.17453317666632, 'longitude': -118.47770616134642 }
    ],
    'waypoints': [
        { 'latitude': 34.17272239378375, 'longitude':  -118.48094626983641, 'altitude_msl': 150 },
        { 'latitude': 34.17327273386768, 'longitude': -118.48047420104979, 'altitude_msl': 200 }, 
        { 'latitude': 34.17329048671387, 'longitude':  -118.47819968780516, 'altitude_msl': 200 },
        { 'latitude': 34.17316621671217, 'longitude': -118.47740575393675, 'altitude_msl': 200 }, 
        { 'latitude':  34.17242059285922, 'longitude':  -118.4775774153137, 'altitude_msl': 200 },
        { 'latitude': 34.171870247218436, 'longitude': -118.47845717987059, 'altitude_msl': 250 }   
    ],
    'emergent_object': { 'latitude': 34.17311547855627, 'longitude': -118.48121668488153 },
    'off_axis_object': { 'latitude': 34.17377032199672, 'longitude': -118.47536036387527 },
    'home': { 'latitude': 34.17147967716717, 'longitude': -118.47792073806761 },
    'air_drop': { 'latitude': 34.17230771859926, 'longitude':  -118.47723628670343 },
    'stationary_obstacles': [
        { 'latitude': 34.17481721751467, 'longitude': -118.4776847036743, 'radius': 100, 'height': 100 }
    ],
    'moving_obstacles': [
        { 
            #'radius': 300,
            'radius': 50,
            'speed_avg': 30,
            'waypoints': [
                { 'latitude': 34.1721010377628, 'longitude': -118.47663327774046, 'altitude_msl': 100},
                { 'latitude': 34.173574531744876, 'longitude': -118.47714826187132, 'altitude_msl': 150}
            ]
        }
    ]
}

mission = mission2

altitude_msl_min = mission['altitude_msl_min']
altitude_msl_max = mission['altitude_msl_max']
boundary_pts = mission['boundary_pts']
search_grid_points = mission['search_grid_points']
waypoints = mission['waypoints']
emergent_object = mission['emergent_object']
off_axis_object = mission['off_axis_object']
home = mission['home']
air_drop = mission['air_drop']
stationary_obstacles = mission['stationary_obstacles']
moving_obstacles = mission['moving_obstacles']

import os
import sys

# Add environment variable to get Django settings file.
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'server.settings')

# Setup Django.
from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

from auvsi_suas.models.aerial_position import AerialPosition
from auvsi_suas.models.fly_zone import FlyZone
from auvsi_suas.models.gps_position import GpsPosition
from auvsi_suas.models.waypoint import Waypoint

from auvsi_suas.models.mission_config import MissionConfig

from auvsi_suas.models.moving_obstacle import MovingObstacle
from auvsi_suas.models.stationary_obstacle import StationaryObstacle


StationaryObstacle.objects.all().delete()
MovingObstacle.objects.all().delete()
MissionConfig.objects.all().delete()

config = MissionConfig()

emergent_last_known_pos = GpsPosition(latitude = emergent_object['latitude'], longitude = emergent_object['longitude'])
emergent_last_known_pos.save()
config.emergent_last_known_pos = emergent_last_known_pos

off_axis_odlc_pos = GpsPosition(latitude = off_axis_object['latitude'], longitude = off_axis_object['longitude'])
off_axis_odlc_pos.save()
config.off_axis_odlc_pos = off_axis_odlc_pos

home_pos = GpsPosition(latitude = home['latitude'], longitude = home['longitude'])
home_pos.save()
config.home_pos = home_pos

air_drop_pos = GpsPosition(latitude = air_drop['latitude'], longitude = air_drop['longitude'])
air_drop_pos.save()
config.air_drop_pos = air_drop_pos

config.save()

fly_zone = FlyZone()
fly_zone.altitude_msl_min = altitude_msl_min
fly_zone.altitude_msl_max = altitude_msl_max
fly_zone.save()
for i in range(len(boundary_pts)):
    pt = boundary_pts[i]
    pos = GpsPosition(latitude=pt['latitude'], longitude=pt['longitude'])
    pos.save()
    apos = AerialPosition(gps_position=pos, altitude_msl=0)
    apos.save()
    wpt = Waypoint(position=apos, order=(i+1))
    wpt.save()
    fly_zone.boundary_pts.add(wpt)
fly_zone.save()
config.fly_zones.add(fly_zone)

for i in range(len(search_grid_points)):
    pt = search_grid_points[i]
    pos = GpsPosition(latitude=pt['latitude'], longitude=pt['longitude'])
    pos.save()
    apos = AerialPosition(gps_position=pos, altitude_msl=0)
    apos.save()
    wpt = Waypoint(position=apos, order=(i+1))
    wpt.save()
    config.search_grid_points.add(wpt)

for i in range(len(waypoints)):
    pt = waypoints[i]
    pos = GpsPosition(latitude=pt['latitude'], longitude=pt['longitude'])
    pos.save()
    apos = AerialPosition(gps_position=pos, altitude_msl=pt['altitude_msl'])
    apos.save()
    wpt = Waypoint(position=apos, order=(i+1))
    wpt.save()
    config.mission_waypoints.add(wpt)

for obstacle in moving_obstacles:
    m_obstacle = MovingObstacle()
    m_obstacle.speed_avg = obstacle['speed_avg']
    m_obstacle.sphere_radius = obstacle['radius']
    m_obstacle.save()
    for i in range(len(obstacle['waypoints'])):
        pt = obstacle['waypoints'][i]
        pos = GpsPosition(latitude=pt['latitude'], longitude=pt['longitude'])
        pos.save()
        apos = AerialPosition(gps_position = pos, altitude_msl = pt['altitude_msl'])
        apos.save()
        wpt = Waypoint(position = apos, order = (i+1))
        wpt.save()
        m_obstacle.waypoints.add(wpt)
    m_obstacle.save()
    config.moving_obstacles.add(m_obstacle)

for obstacle in stationary_obstacles:
    pos = GpsPosition(latitude=obstacle['latitude'], longitude=obstacle['longitude'])
    pos.save()
    s_obstacle = StationaryObstacle(gps_position=pos, cylinder_radius=obstacle['radius'], cylinder_height=obstacle['height'])
    s_obstacle.save()
    config.stationary_obstacles.add(s_obstacle)

config.is_active = True
config.save()