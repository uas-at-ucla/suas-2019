from django.contrib import admin
from auvsi_suas.models.aerial_position import AerialPosition
from auvsi_suas.models.fly_zone import FlyZone
from auvsi_suas.models.gps_position import GpsPosition
from auvsi_suas.models.mission_clock_event import MissionClockEvent
from auvsi_suas.models.mission_judge_feedback import MissionJudgeFeedback
from auvsi_suas.models.mission_config import MissionConfig
from auvsi_suas.models.moving_obstacle import MovingObstacle
from auvsi_suas.models.stationary_obstacle import StationaryObstacle
from auvsi_suas.models.takeoff_or_landing_event import TakeoffOrLandingEvent
from auvsi_suas.models.odlc import Odlc
from auvsi_suas.models.uas_telemetry import UasTelemetry
from auvsi_suas.models.waypoint import Waypoint


@admin.register(GpsPosition)
class GpsPositionModelAdmin(admin.ModelAdmin):
    show_full_result_count = False


@admin.register(AerialPosition)
class AerialPositionModelAdmin(admin.ModelAdmin):
    raw_id_fields = ("gps_position", )
    show_full_result_count = False


@admin.register(MissionConfig)
class MissionConfigModelAdmin(admin.ModelAdmin):
    raw_id_fields = ("home_pos", "emergent_last_known_pos",
                     "off_axis_odlc_pos", "air_drop_pos")
    filter_horizontal = ("fly_zones", "mission_waypoints",
                         "search_grid_points", "odlcs", "stationary_obstacles",
                         "moving_obstacles")


@admin.register(FlyZone)
class FlyZoneModelAdmin(admin.ModelAdmin):
    filter_horizontal = ("boundary_pts", )


@admin.register(MovingObstacle)
class MovingObstacleModelAdmin(admin.ModelAdmin):
    filter_horizontal = ("waypoints", )


@admin.register(StationaryObstacle)
class StationaryObstacleModelAdmin(admin.ModelAdmin):
    raw_id_fields = ("gps_position", )


@admin.register(Odlc)
class OdlcModelAdmin(admin.ModelAdmin):
    show_full_result_count = False
    raw_id_fields = ("location", )


@admin.register(UasTelemetry)
class UasTelemetryModelAdmin(admin.ModelAdmin):
    show_full_result_count = False
    raw_id_fields = ("uas_position", )


@admin.register(Waypoint)
class WaypointModelAdmin(admin.ModelAdmin):
    show_full_result_count = False
    raw_id_fields = ("position", )


# These don't require any raw fields.
admin.site.register(MissionClockEvent)
admin.site.register(MissionJudgeFeedback)
admin.site.register(TakeoffOrLandingEvent)
