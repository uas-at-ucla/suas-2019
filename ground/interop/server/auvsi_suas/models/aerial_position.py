"""Aerial position model."""

from auvsi_suas.models import distance
from django.db import models
from gps_position import GpsPosition


class AerialPosition(models.Model):
    """Aerial position which consists of a GPS position and an altitude.

    Attributes:
        gps_position: GPS position.
        altitude_msl: Altitude (MSL) in feet.
    """
    gps_position = models.ForeignKey(GpsPosition)
    altitude_msl = models.FloatField()

    def __unicode__(self):
        """Descriptive text for use in displays."""
        return unicode("AerialPosition (pk:%s, alt:%s, gps:%s)" %
                       (str(self.pk), str(self.altitude_msl),
                        self.gps_position.__unicode__()))

    def distance_to(self, other):
        """Computes distance to another position.

        Args:
          other: The other position.
        Returns:
          Distance in feet.
        """
        return distance.distance_to(
            self.gps_position.latitude, self.gps_position.longitude,
            self.altitude_msl, other.gps_position.latitude,
            other.gps_position.longitude, other.altitude_msl)

    def duplicate(self, other):
        """Determines whether this AerialPosition is equivalent to another.

        This differs from the Django __eq__() method which simply compares
        primary keys. This method compares the field values.

        Args:
            other: The other position for comparison.
        Returns:
            True if they are equal.
        """
        return (self.gps_position.duplicate(other.gps_position) and
                self.altitude_msl == other.altitude_msl)
