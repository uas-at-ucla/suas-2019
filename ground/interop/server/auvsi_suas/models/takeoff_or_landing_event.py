"""Takeoff or landing event model."""

from access_log import AccessLog
from time_period import TimePeriod
from django.db import models
from django.utils import timezone


class TakeoffOrLandingEvent(AccessLog):
    """Marker for a UAS takeoff/landing. UAS must interop during that time.

    Attributes:
        uas_in_air: Whether the UAS is now in the air.
    """
    uas_in_air = models.BooleanField()

    def __unicode__(self):
        """Descriptive text for use in displays."""
        return unicode('TakeoffOrLandingEvent (pk:%s, user:%s, timestamp:%s, '
                       'uas_in_air:%s)' %
                       (str(self.pk), self.user.__unicode__(),
                        str(self.timestamp), str(self.uas_in_air)))

    @classmethod
    def flights(cls, user):
        """Gets the time periods for which the given user was in flight.

        Args:
            user: The user for which to get flight periods for.
        Returns:
            A list of TimePeriod objects corresponding to individual flights.
        """
        return TimePeriod.from_events(
            TakeoffOrLandingEvent.by_user(user),
            is_start_func=lambda x: x.uas_in_air,
            is_end_func=lambda x: not x.uas_in_air)

    @classmethod
    def user_in_air(cls, user, time=None):
        """Determine if given user is currently in-air

        Args:
            user: User to get in-flight status for
            time: Time to check in-air status; default now
        Returns:
            True if user is currently in-flight, False otherwise
        """
        event = cls.last_for_user(user, end_time=time)
        if event:
            return event.uas_in_air
        return False
