"""GPS position model."""

from auvsi_suas.models import distance
from django.db import models


class GpsPosition(models.Model):
    """GPS position consisting of a latitude and longitude degree value.

    Attributes:
        latitude: Latitude in degrees.
        longitude: Longitude in degrees.
    """
    latitude = models.FloatField()
    longitude = models.FloatField()

    def __unicode__(self):
        """Descriptive text for use in displays."""
        return unicode("GpsPosition (pk:%s, lat:%s, lon:%s)" %
                       (str(self.pk), str(self.latitude), str(self.longitude)))

    def distance_to(self, other):
        """Computes distance to another position.

        Args:
          other: The other position.
        Returns:
          Distance in feet.
        """
        return distance.distance_to(self.latitude, self.longitude, 0,
                                    other.latitude, other.longitude, 0)

    def duplicate(self, other):
        """Determines whether this GpsPosition is equivalent to another.

        This differs from the Django __eq__() method which simply compares
        primary keys. This method compares the field values.

        Args:
            other: The other position for comparison.
        Returns:
            True if they are equal.
        """
        return (self.latitude == other.latitude and
                self.longitude == other.longitude)
