"""Object detection, localization, and classification model."""

import collections
import enum
import networkx as nx
import operator
from auvsi_suas.proto import odlc_pb2
from django.conf import settings
from django.db import models
from gps_position import GpsPosition
from takeoff_or_landing_event import TakeoffOrLandingEvent
from mission_clock_event import MissionClockEvent


class Choices(enum.IntEnum):
    """Base class for enums used to limit Django field choices,
    plus other helper methods.

    Item names should be lowercase to work properly with lookup().
    """

    @classmethod
    def choices(cls):
        """Provide choices for Django's IntField.choices.

        Returns:
            Enum values in an iterator to be passed to IntField.choices.
            The enum value is used as the field key, and the name as the
            description.
        """
        return [(int(v), k) for k, v in cls.__members__.items()]

    @classmethod
    def lookup(cls, s):
        """Lookup value from name.

        Args:
            s: name to lookup; case insensitive

        Returns:
            Value associated with name

        Raises:
            KeyError: name not valid
        """
        return cls.__members__[str(s).lower()]

    @classmethod
    def names(cls):
        """Names of choices

        Returns:
            List of names of values
        """
        return cls.__members__.keys()


@enum.unique
class OdlcType(Choices):
    """Valid object types.

    Warning: DO NOT change/reuse values, or compatibility will be lost with
    old data sets. Only add new values to the end. Next value is 5.
    """
    standard = 1
    off_axis = 3
    emergent = 4


@enum.unique
class Orientation(Choices):
    """Valid object orientations.

    Warning: DO NOT change/reuse values, or compatibility will be lost with
    old data sets. Only add new values to the end.
    """
    n = 1
    ne = 2
    e = 3
    se = 4
    s = 5
    sw = 6
    w = 7
    nw = 8


@enum.unique
class Shape(Choices):
    """Valid object shapes.

    Warning: DO NOT change/reuse values, or compatibility will be lost with
    old data sets. Only add new values to the end. Next value is 14.
    """
    circle = 1
    semicircle = 2
    quarter_circle = 3
    triangle = 4
    square = 5
    rectangle = 6
    trapezoid = 7
    pentagon = 8
    hexagon = 9
    heptagon = 10
    octagon = 11
    star = 12
    cross = 13


@enum.unique
class Color(Choices):
    """Valid object colors.

    Warning: DO NOT change/reuse values, or compatibility will be lost with
    old data sets. Only add new values to the end. Next value is 11.
    """
    white = 1
    black = 2
    gray = 3
    red = 4
    blue = 5
    green = 6
    yellow = 7
    purple = 8
    brown = 9
    orange = 10


class Odlc(models.Model):
    """Object detection submission for a team.

    Attributes:
        user: The user which submitted and owns this object detection.
        odlc_type: Object type.
        location: Object location.
        orientation: Object orientation.
        shape: Object shape.
        background_color: Object background color.
        alphanumeric: Object alphanumeric.
        alphanumeric_color: Object alphanumeric color.
        description: Free-form object description.
        description_approved: Whether judge considers description valid.
        autonomous: Objcet is an ADLC submission.
        thumbnail: Uploaded object image thumbnail.
        thumbnail_approved: Whether judge considers thumbnail valid for object.
        creation_time: Time that this object was first created.
        last_modified_time: Time that this object was last modified.
    """
    user = models.ForeignKey(settings.AUTH_USER_MODEL, db_index=True)
    odlc_type = models.IntegerField(choices=OdlcType.choices())
    location = models.ForeignKey(GpsPosition, null=True, blank=True)
    orientation = models.IntegerField(
        choices=Orientation.choices(), null=True, blank=True)
    shape = models.IntegerField(choices=Shape.choices(), null=True, blank=True)
    background_color = models.IntegerField(
        choices=Color.choices(), null=True, blank=True)
    alphanumeric = models.TextField(default='', blank=True)
    alphanumeric_color = models.IntegerField(
        choices=Color.choices(), null=True, blank=True)
    description = models.TextField(default='', blank=True)
    description_approved = models.NullBooleanField()
    autonomous = models.BooleanField(default=False)
    thumbnail = models.ImageField(upload_to='objects', blank=True)
    thumbnail_approved = models.NullBooleanField()
    creation_time = models.DateTimeField(auto_now_add=True)
    last_modified_time = models.DateTimeField(auto_now=True)
    actionable_override = models.BooleanField(default=False)

    def __unicode__(self):
        """Descriptive text for use in displays."""
        d = self.json(is_superuser=True)
        return unicode('{name}({fields})'.format(
            name=self.__class__.__name__,
            fields=', '.join('%s=%s' % (k, v) for k, v in d.iteritems())))

    def json(self, is_superuser=False):
        """Odlc as dict, for JSON."""
        odlc_type = None
        if self.odlc_type is not None:
            odlc_type = OdlcType(self.odlc_type).name

        latitude = None
        longitude = None
        if self.location is not None:
            latitude = self.location.latitude
            longitude = self.location.longitude

        orientation = None
        if self.orientation is not None:
            orientation = Orientation(self.orientation).name

        shape = None
        if self.shape is not None:
            shape = Shape(self.shape).name

        background_color = None
        if self.background_color is not None:
            background_color = Color(self.background_color).name

        alphanumeric = None
        if self.alphanumeric != '':
            alphanumeric = self.alphanumeric

        alphanumeric_color = None
        if self.alphanumeric_color is not None:
            alphanumeric_color = Color(self.alphanumeric_color).name

        description = None
        if self.description != '':
            description = self.description

        d = {
            'id': self.pk,
            'user': self.user.pk,
            'type': odlc_type,
            'latitude': latitude,
            'longitude': longitude,
            'orientation': orientation,
            'shape': shape,
            'background_color': background_color,
            'alphanumeric': alphanumeric,
            'alphanumeric_color': alphanumeric_color,
            'description': description,
            'autonomous': self.autonomous,
        }

        if is_superuser:
            d['description_approved'] = self.description_approved
            d['thumbnail'] = self.thumbnail.name if self.thumbnail else None
            d['thumbnail_approved'] = self.thumbnail_approved
            d['creation_time'] = self.creation_time
            d['last_modified_time'] = self.last_modified_time
            d['actionable_override'] = self.actionable_override

        return d

    def similar_classifications_ratio(self, other):
        """Counts the number of similar classification attributes.

        Args:
            other: Another object for which to compare.
        Returns:
            The ratio of attributes which are the same.
        """
        # Cannot have similar fields with different type objects.
        if self.odlc_type != other.odlc_type:
            return 0

        standard_fields = [
            'orientation', 'shape', 'background_color', 'alphanumeric',
            'alphanumeric_color'
        ]
        classify_fields = {
            OdlcType.standard: standard_fields,
            OdlcType.off_axis: standard_fields,
            OdlcType.emergent: ['description_approved'],
        }
        fields = classify_fields[self.odlc_type]
        count = 0
        for field in fields:
            if getattr(self, field) == getattr(other, field):
                count += 1
        return float(count) / len(fields)

    def actionable_submission(self, flights=None):
        """Checks if Odlc meets Actionable Intelligence submission criteria.

        A object is "actionable" if one of the following conditions is met:
            (a) If it was submitted over interop and last updated during the
                aircraft's first flight.
            (b) If the object was submitted via USB, the object's
                actionable_override flag was set by an admin.

        Args:
            flights: Optional memoized flights for this object's user. If
                     omitted, the flights will be looked up.

        Returns:
            True if object may be considered an "actionable" submission.
        """
        if flights is None:
            flights = TakeoffOrLandingEvent.flights(self.user)

        actionable = False
        if len(flights) > 0:
            flight = flights[0]
            if flight.within(self.creation_time) and \
                flight.within(self.last_modified_time):
                actionable = True

        return self.actionable_override or actionable

    def interop_submission(self, missions=None):
        """Checks if Odlc meets Interoperability submission criteria.

        A object counts as being submitted over interoperability system if it
        was submitted and last updated while the team was on the mission clock.

        Args:
            missions: Optional memoized missions for this object's user. If
                     omitted, the missions will be looked up.

        Returns:
            True if object may be considered an "interoperability" submission.
        """
        if missions is None:
            missions = MissionClockEvent.missions(self.user)

        for mission in missions:
            if mission.within(self.creation_time) and \
                mission.within(self.last_modified_time):
                return True

        return False


class OdlcEvaluator(object):
    """Evaluates submitted objects against real judge-made objects."""

    def __init__(self, submitted_objects, real_objects):
        """Creates an evaluation of submitted objects against real objects.

        Args:
            submitted_objects: List of submitted Odlc objects, all from
                               the same user.
            real_objects: List of real objects made by judges.

        Raises:
            AssertionError: not all submitted objects are from the same user.
        """
        self.submitted_objects = submitted_objects
        self.real_objects = real_objects

        if self.submitted_objects:
            self.user = self.submitted_objects[0].user
            for t in self.submitted_objects:
                if t.user != self.user:
                    raise AssertionError(
                        "All submitted objects must be from the same user")

            self.flights = TakeoffOrLandingEvent.flights(self.user)
            self.missions = MissionClockEvent.missions(self.user)

        self.matches = self.match_odlcs(submitted_objects, real_objects)
        self.unmatched = self.find_unmatched(submitted_objects, real_objects,
                                             self.matches)

    def range_lookup(self,
                     ranges,
                     key,
                     start_operator=operator.ge,
                     end_operator=operator.le):
        """Performs a range based lookup.

        Args:
            ranges: A list of maps containing start,end,value keys.
            key: The key for the range lookup.
            start_operator: The operator to compare the start key against.
            end_operator: The operator to compare the end key against.
        Returns:
            The value associated for the range lookup, or None if not in range.
        """
        for r in ranges:
            if start_operator(key, r['start']) and end_operator(key, r['end']):
                return r['value']
        return None

    def evaluate_match(self, submitted, real):
        """Evaluates the match if the two objects were to be paired.

        Args:
            submitted: The team submitted object. Must be one of
                self.submitted_objects.
            real: The real object made by the judges. Must be one of
                self.real_objects.
        Returns:
            auvsi_suas.proto.OdlcEvaluation. The match evaluation.
        """
        object_eval = odlc_pb2.OdlcEvaluation()
        object_eval.real_odlc = real.pk
        object_eval.submitted_odlc = submitted.pk
        object_eval.score_ratio = 0

        # Odlcs which are not the same type have no match value.
        if submitted.odlc_type != real.odlc_type:
            return object_eval
        # Odlcs which don't have an approved thumbnail have no value.
        if not submitted.thumbnail_approved:
            return object_eval

        # Compute values which influence score and are provided as feedback.
        if submitted.thumbnail_approved is not None:
            object_eval.image_approved = submitted.thumbnail_approved
        if (submitted.odlc_type == OdlcType.emergent and
                submitted.description_approved is not None):
            object_eval.description_approved = submitted.description_approved
        object_eval.classifications_ratio = real.similar_classifications_ratio(
            submitted)
        if submitted.location:
            object_eval.geolocation_accuracy_ft = \
                    submitted.location.distance_to(real.location)
        object_eval.actionable_submission = submitted.actionable_submission(
            flights=self.flights)
        object_eval.autonomous_submission = submitted.autonomous
        object_eval.interop_submission = submitted.interop_submission(
            missions=self.missions)

        # Compute score.
        object_eval.classifications_score_ratio = \
                object_eval.classifications_ratio
        if object_eval.HasField('geolocation_accuracy_ft'):
            object_eval.geolocation_score_ratio = max(
                0, (float(settings.TARGET_LOCATION_THRESHOLD) -
                    object_eval.geolocation_accuracy_ft) /
                float(settings.TARGET_LOCATION_THRESHOLD))
        else:
            object_eval.geolocation_score_ratio = 0
        object_eval.actionable_score_ratio = \
                1 if object_eval.actionable_submission else 0
        object_eval.autonomous_score_ratio = \
                1 if object_eval.autonomous_submission else 0
        object_eval.interop_score_ratio = \
                1 if object_eval.interop_submission else 0
        object_eval.score_ratio = (
            (settings.CHARACTERISTICS_WEIGHT *
             object_eval.classifications_score_ratio) +
            (settings.GEOLOCATION_WEIGHT * object_eval.geolocation_score_ratio)
            +
            (settings.ACTIONABLE_WEIGHT * object_eval.actionable_score_ratio) +
            (settings.AUTONOMY_WEIGHT * object_eval.autonomous_score_ratio) +
            (settings.INTEROPERABILITY_WEIGHT *
             object_eval.interop_score_ratio))

        return object_eval

    def match_odlcs(self, submitted_objects, real_objects):
        """Matches the objects to maximize match value.

        Args:
            submitted_objects: List of submitted object detections.
            real_objects: List of real objects made by judges.
        Returns:
            A map from submitted object to real object, and real object to
            submitted object, if they are matched.
        """
        # Create a bipartite graph from submitted to real objects with match
        # values (score ratio) as edge weights. Skip edges with no match value.
        g = nx.Graph()
        g.add_nodes_from(submitted_objects)
        g.add_nodes_from(real_objects)
        for submitted in submitted_objects:
            for real in real_objects:
                match_value = self.evaluate_match(submitted, real).score_ratio
                if match_value:
                    g.add_edge(submitted, real, weight=match_value)
        # Compute the full matching.
        return nx.algorithms.matching.max_weight_matching(g)

    def find_unmatched(self, submitted_objects, real_objects, matches):
        """Finds unmatched objects, filtering double-counts by autonomy.

        Args:
            submitted_objects: List of submitted object detections.
            real_objects: List of real objects made by judges.
            matches: Map from submitted to real objects indicating matches.
        Returns:
            List of objects which are unmatched after filtering autonomy
            duplicates.
        """
        # Create a bipartite graph from unsubmitted to real objects with match
        # values (score ratio) as edge weights. Skip edges with no match value.
        # Skip edges if not inverse autonomy for existing match.
        remaining_objects = [t for t in submitted_objects if t not in matches]
        g = nx.Graph()
        g.add_nodes_from(remaining_objects)
        g.add_nodes_from(real_objects)
        for submitted in remaining_objects:
            for real in real_objects:
                match_value = self.evaluate_match(submitted, real).score_ratio
                inverted_autonomy = (
                    real in matches and
                    submitted.autonomous != matches[real].autonomous)
                if match_value and inverted_autonomy:
                    # We care about minimiznig unmatched, not match weight, so
                    # use weight of 1.
                    g.add_edge(submitted, real, weight=1)
        # Compute the matching to find unused objects.
        unused_match = nx.algorithms.matching.max_weight_matching(g)
        # Difference between remaining and unused is unmatched.
        return [t for t in remaining_objects if t not in unused_match]

    def evaluate(self):
        """Evaluates the submitted objects.

        Returns:
            auvsi_suas.proto.MultiOdlcEvaluation.
        """
        multi_eval = odlc_pb2.MultiOdlcEvaluation()
        # Compute match value.
        for real in self.real_objects:
            object_eval = multi_eval.odlcs.add()
            object_eval.real_odlc = real.pk
            object_eval.score_ratio = 0
            submitted = self.matches.get(real)
            if submitted:
                object_eval.CopyFrom(self.evaluate_match(submitted, real))
        if self.real_objects:
            multi_eval.matched_score_ratio = sum(
                [e.score_ratio
                 for e in multi_eval.odlcs]) / len(self.real_objects)
        else:
            multi_eval.matched_score_ratio = 0
        # Compute extra object penalty.
        multi_eval.unmatched_odlc_count = len(self.unmatched)
        multi_eval.extra_object_penalty_ratio = \
                (multi_eval.unmatched_odlc_count *
                        settings.EXTRA_OBJECT_PENALTY_RATIO)
        # Compute total score.
        multi_eval.score_ratio = (multi_eval.matched_score_ratio -
                                  multi_eval.extra_object_penalty_ratio)
        return multi_eval
