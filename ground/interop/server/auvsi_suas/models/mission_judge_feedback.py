"""Feedback from judges on mission performance."""

from django.conf import settings
from django.db import models

from auvsi_suas.proto import mission_pb2
from mission_config import MissionConfig


class MissionJudgeFeedback(models.Model):
    """Stores feedback from judges on a team's mission performance.

    Attributes:
        mission: The mission for which this is feedback.
        user: The user for which this is feedback.
        flight_time: Time spent occupying runway and airspace.
        post_process_time: Time spent handling data on mission clock.
        used_timeout: Whether the team used their single timeout.
        min_auto_flight_time: Whether the team had the min auto flight time.
        safety_pilot_takeovers: The number of times the pilot took over.
        waypoints_captured: Number of waypoints that were captured.
        out_of_bounds: Number of times the UAS went out of bounds.
        unsafe_out_of_bounds: Number of times out of bounds compromised safety.
        things_fell_off_uas: Whether something fell off UAS during flight.
        crashed: Whether the UAS crashed.
        air_delivery_accuracy_ft: Accuracy of delivery in feet.
        operational_excellence_percent: Grade of team performance [0, 100].

    """
    mission = models.ForeignKey(MissionConfig)
    user = models.ForeignKey(settings.AUTH_USER_MODEL)

    flight_time = models.DurationField()
    post_process_time = models.DurationField()
    used_timeout = models.BooleanField()

    min_auto_flight_time = models.BooleanField()
    safety_pilot_takeovers = models.IntegerField()
    waypoints_captured = models.IntegerField()
    out_of_bounds = models.IntegerField()
    unsafe_out_of_bounds = models.IntegerField()
    things_fell_off_uas = models.BooleanField()
    crashed = models.BooleanField()

    air_delivery_accuracy_ft = models.FloatField(null=True)

    operational_excellence_percent = models.FloatField()

    class Meta:
        unique_together = (('mission', 'user'), )

    def proto(self):
        """Get the proto formatted feedback."""
        feedback = mission_pb2.MissionJudgeFeedback()

        feedback.flight_time_sec = self.flight_time.total_seconds()
        feedback.post_process_time_sec = self.post_process_time.total_seconds()
        feedback.used_timeout = self.used_timeout

        feedback.min_auto_flight_time = self.min_auto_flight_time
        feedback.safety_pilot_takeovers = self.safety_pilot_takeovers
        feedback.waypoints_captured = self.waypoints_captured
        feedback.out_of_bounds = self.out_of_bounds
        feedback.unsafe_out_of_bounds = self.unsafe_out_of_bounds
        feedback.things_fell_off_uas = self.things_fell_off_uas
        feedback.crashed = self.crashed

        if self.air_delivery_accuracy_ft:
            feedback.air_delivery_accuracy_ft = self.air_delivery_accuracy_ft

        feedback.operational_excellence_percent = self.operational_excellence_percent

        return feedback
