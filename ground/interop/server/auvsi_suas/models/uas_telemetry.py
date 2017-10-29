"""UAS Telemetry model."""

import itertools
from collections import defaultdict

from django.conf import settings
from django.contrib.auth.models import User
from django.db import models
from django.utils import timezone

from auvsi_suas.models import distance
from auvsi_suas.models import units
from auvsi_suas.models.access_log import AccessLog
from auvsi_suas.models.aerial_position import AerialPosition
from auvsi_suas.models.gps_position import GpsPosition
from auvsi_suas.models.moving_obstacle import MovingObstacle
from auvsi_suas.models.takeoff_or_landing_event import TakeoffOrLandingEvent
from auvsi_suas.patches.simplekml_patch import AltitudeMode
from auvsi_suas.patches.simplekml_patch import Color
from auvsi_suas.proto import mission_pb2


class UasTelemetry(AccessLog):
    """UAS telemetry reported by teams.

    Attributes:
        uas_position: The position of the UAS.
        uas_heading: The (true north) heading of the UAS in degrees.
    """
    uas_position = models.ForeignKey(AerialPosition)
    uas_heading = models.FloatField()

    def __unicode__(self):
        """Descriptive text for use in displays."""
        return unicode("UasTelemetry (pk:%s, user:%s, timestamp:%s, "
                       "heading:%s, pos:%s)" %
                       (str(self.pk), self.user.__unicode__(),
                        str(self.timestamp), str(self.uas_heading),
                        self.uas_position.__unicode__()))

    def duplicate(self, other):
        """Determines whether this UasTelemetry is equivalent to another.

        This differs from the Django __eq__() method which simply compares
        primary keys. This method compares the field values.

        Args:
            other: The other log for comparison.
        Returns:
            True if they are equal.
        """
        return (self.uas_position.duplicate(other.uas_position) and
                self.uas_heading == other.uas_heading)

    def json(self):
        ret = {
            'id': self.pk,
            'user': self.user.pk,
            'timestamp': self.timestamp.isoformat(),
            'latitude': self.uas_position.gps_position.latitude,
            'longitude': self.uas_position.gps_position.longitude,
            'altitude_msl': self.uas_position.altitude_msl,
            'heading': self.uas_heading,
        }

        return ret

    @classmethod
    def by_user(cls, *args, **kwargs):
        """Gets the time-sorted list of access log for the given user.

        Note: This prefetches the related AerialPosition and GpsPosition
        for each entry.  Thus, any subsequent changes to those entries in
        the database after fetch may not be reflected in the objects.

        Args:
            user: The user to get the access log for.
        Returns:
            A list of access log objects for the given user sorted by timestamp.
        """
        # Almost every user of UasTelemetry.by_user wants to use
        # the related AerialPosition and GpsPosition.  To avoid excessive
        # database queries, we select these values from the database up front.
        return super(UasTelemetry, cls).by_user(*args, **kwargs) \
                .select_related('uas_position__gps_position')

    @classmethod
    def dedupe(cls, logs):
        """Dedupes a set of UAS telemetry logs.

        For every set of sequential telemetry logs that are duplicates, it will
        filter all but the first log. Sensors and autopilots are unlikely to
        provide exactly the same data, even if the system is stationary, so
        logs which have exactly the same values are likely duplicates. Duplicate
        telemetry data is not allowed per the rules, so it is filtered.

        Args:
            logs: A sorted list of UasTelemetry logs.
        Returns:
            A list containing the non-duplicate logs in the original list.
        """
        # Check that logs were provided.
        if not logs:
            return logs

        # For each log, compare to previous. If different, add to output.
        filtered = []
        prev_log = None
        for log in logs:
            if prev_log is None or not prev_log.duplicate(log):
                # New unique log.
                filtered.append(log)
                prev_log = log

        return filtered

    @classmethod
    def kml(cls, user, logs, kml, kml_doc):
        """
        Appends kml nodes describing the given user's flight as described
        by the log array given.

        Args:
            user: A Django User to get username from
            logs: A list of UasTelemetry elements
            kml: A simpleKML Container to which the flight data will be added
            kml_doc: The simpleKML Document to which schemas will be added
        Returns:
            None
        """
        # KML Compliant Datetime Formatter
        kml_datetime_format = "%Y-%m-%dT%H:%M:%S.%fZ"
        icon = 'http://maps.google.com/mapfiles/kml/shapes/airports.png'
        threshold = 1  # Degrees

        kml_folder = kml.newfolder(name=user.username)

        flights = TakeoffOrLandingEvent.flights(user)
        if len(flights) == 0:
            return

        logs = filter(lambda log: cls._is_bad_position(log, threshold), logs)
        for i, flight in enumerate(flights):
            label = 'Flight {}'.format(i + 1)  # Flights are one-indexed
            kml_flight = kml_folder.newfolder(name=label)

            flight_logs = filter(lambda x: flight.within(x.timestamp), logs)
            if len(flight_logs) < 2:
                continue

            coords = []
            angles = []
            when = []
            for entry in flight_logs:
                pos = entry.uas_position.gps_position
                # Spatial Coordinates
                coord = (pos.longitude, pos.latitude,
                         units.feet_to_meters(entry.uas_position.altitude_msl))
                coords.append(coord)

                # Time Elements
                time = entry.timestamp.strftime(kml_datetime_format)
                when.append(time)

                # Degrees heading, tilt, and roll
                angle = (entry.uas_heading, 0.0, 0.0)
                angles.append(angle)

            # Create a new track in the folder
            trk = kml_flight.newgxtrack(name='Flight Path')
            trk.altitudemode = AltitudeMode.absolute

            # Append flight data
            trk.newwhen(when)
            trk.newgxcoord(coords)
            trk.newgxangle(angles)

            # Set styling
            trk.extrude = 1  # Extend path to ground
            trk.style.linestyle.width = 2
            trk.style.linestyle.color = Color.blue
            trk.iconstyle.icon.href = icon

            for obstacle in MovingObstacle.objects.all():
                obstacle.kml(path=flight_logs, kml=kml_flight, kml_doc=kml_doc)

    @classmethod
    def live_kml(cls, kml, timespan):
        users = User.objects.all()
        for user in users:
            period_logs = UasTelemetry.by_user(user)\
                .filter(timestamp__gt=timezone.now() - timespan)

            if len(period_logs) < 1:
                continue

            linestring = kml.newlinestring(name=user.username)
            coords = []
            for entry in period_logs:
                pos = entry.uas_position.gps_position
                # Spatial Coordinates
                coord = (pos.longitude, pos.latitude,
                         units.feet_to_meters(entry.uas_position.altitude_msl))
                coords.append(coord)
            linestring.coords = coords
            linestring.altitudemode = AltitudeMode.absolute
            linestring.extrude = 1
            linestring.style.linestyle.color = Color.blue
            linestring.style.polystyle.color = Color.changealphaint(
                100, Color.blue)

    @staticmethod
    def closest_interpolated_distance(start_log, end_log, waypoint, utm):
        """Finds the closest distance to the waypoint by interpolating between
        start_log and end_log.

        Args:
            start_log: A UAS telemetry log.
            end_log: A UAS telemetry log.
            waypoint: The waypoint for which to find the closest distance.
            utm: The UTM Proj projection to project into.
        Returns:
            The closest distance to the waypoint in feet.
        """
        # If no provided end, use start distance.
        if end_log is None:
            return start_log.uas_position.distance_to(waypoint.position)

        # Verify that aircraft velocity is within bounds. Don't interpolate if
        # it isn't because the data is probably erroneous.
        d = start_log.uas_position.distance_to(end_log.uas_position)
        t = (end_log.timestamp - start_log.timestamp).total_seconds()
        if (t > settings.MAX_TELMETRY_INTERPOLATE_INTERVAL_SEC or
            (d / t) > settings.MAX_AIRSPEED_FT_PER_SEC):
            return start_log.uas_position.distance_to(waypoint.position)

        def uas_position_to_tuple(pos):
            return (pos.gps_position.latitude, pos.gps_position.longitude,
                    pos.altitude_msl)

        # Linearly interpolate between start and end telemetry and find the
        # closest distance to the waypoint.
        start = uas_position_to_tuple(start_log.uas_position)
        end = uas_position_to_tuple(end_log.uas_position)
        point = uas_position_to_tuple(waypoint.position)
        return distance.distance_to_line(start, end, point, utm)

    @staticmethod
    def score_waypoint(distance):
        """Scores a single waypoint distance."""
        return max(0,
                   float(settings.SATISFIED_WAYPOINT_DIST_MAX_FT - distance) /
                   settings.SATISFIED_WAYPOINT_DIST_MAX_FT)

    @classmethod
    def satisfied_waypoints(cls, home_pos, waypoints, uas_telemetry_logs):
        """Determines whether the UAS satisfied the waypoints.

        Waypoints must be satisfied in order. The entire pattern may be
        restarted at any point. The best (most waypoints satisfied) attempt
        will be returned.

        Assumes that waypoints are at least
        settings.SATISFIED_WAYPOINT_DIST_MAX_FT apart.

        Args:
            home_pos: The home position for projections.
            waypoints: A list of waypoints to check against.
            uas_telemetry_logs: A list of UAS Telemetry logs to evaluate.
        Returns:
            A list of auvsi_suas.proto.WaypointEvaluation.
        """
        # Form utm for use as projection in distance calcluations.
        zone, north = distance.utm_zone(home_pos.latitude, home_pos.longitude)
        utm = distance.proj_utm(zone, north)

        # Reduce telemetry from telemetry to waypoint hits.
        # This will make future processing more efficient via data reduction.
        # While iterating, compute the best distance seen for feedback.
        best = {}
        hits = []
        for iu, start_log in enumerate(uas_telemetry_logs):
            end_log = None
            if iu + 1 < len(uas_telemetry_logs):
                end_log = uas_telemetry_logs[iu + 1]
            for iw, waypoint in enumerate(waypoints):
                dist = cls.closest_interpolated_distance(
                    start_log, end_log, waypoint, utm)
                best[iw] = min(best.get(iw, dist), dist)
                score = cls.score_waypoint(dist)
                if score > 0:
                    hits.append((iw, dist, score))
        # Remove redundant hits which wouldn't be part of best sequence.
        # This will make future processing more efficient via data reduction.
        hits = [
            max(g, key=lambda x: x[2])
            for _, g in itertools.groupby(hits, lambda x: x[0])
        ]

        # Find highest scoring sequence via dynamic programming.
        # Implement recurrence relation:
        #   S(iw, ih) = s[iw, ih] + max_{k=[0,ih)} S(iw-1, k)
        dp = defaultdict(lambda: defaultdict(lambda: (0, None, None)))
        highest_total = None
        highest_total_pos = (None, None)
        for iw in xrange(len(waypoints)):
            for ih, (hiw, hdist, hscore) in enumerate(hits):
                # Compute score for assigning current hit to current waypoint.
                score = hscore if iw == hiw else 0.0
                # Compute best total score, which includes this match score and
                # best of all which could come before it.
                prev_iw = iw - 1
                total_score = score
                total_score_back = (None, None)
                if prev_iw >= 0:
                    for prev_ih in xrange(ih + 1):
                        (prev_total_score, _) = dp[prev_iw][prev_ih]
                        new_total_score = prev_total_score + score
                        if new_total_score > total_score:
                            total_score = new_total_score
                            total_score_back = (prev_iw, prev_ih)
                dp[iw][ih] = (total_score, total_score_back)
                # Track highest score seen.
                if total_score > highest_total:
                    highest_total = total_score
                    highest_total_pos = (iw, ih)
        # Traceback sequence to get scores and distance for score.
        scores = defaultdict(lambda: (0, None))
        cur_pos = highest_total_pos
        while cur_pos != (None, None):
            cur_iw, cur_ih = cur_pos
            hiw, hdist, hscore = hits[cur_ih]
            if cur_iw == hiw:
                scores[cur_iw] = (hscore, hdist)
            _, cur_pos = dp[cur_iw][cur_ih]

            # Convert to evaluation.
        waypoint_evals = []
        for iw, waypoint in enumerate(waypoints):
            score, dist = scores[iw]
            waypoint_eval = mission_pb2.WaypointEvaluation()
            waypoint_eval.id = iw
            waypoint_eval.score_ratio = score
            if dist is not None:
                waypoint_eval.closest_for_scored_approach_ft = dist
            if iw in best:
                waypoint_eval.closest_for_mission_ft = best[iw]
            waypoint_evals.append(waypoint_eval)
        return waypoint_evals

    @staticmethod
    def _is_bad_position(log, threshold):
        """
        Determine whether entry is not near latitude and longitude of 0,0.

        Args:
            x: UasTelemetry element
        Returns:
            Boolean: True if position is not near 0,0, else False
        """
        pos = log.uas_position.gps_position
        if max(abs(pos.latitude), abs(pos.longitude)) < threshold:
            return False
        return True
