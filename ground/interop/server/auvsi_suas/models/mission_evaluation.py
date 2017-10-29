"""Mission evaluation."""

import datetime
import itertools
import logging
from django.conf import settings
from django.contrib.auth.models import User

from auvsi_suas.models import distance
from auvsi_suas.models import units
from auvsi_suas.models.fly_zone import FlyZone
from auvsi_suas.models.mission_clock_event import MissionClockEvent
from auvsi_suas.models.mission_judge_feedback import MissionJudgeFeedback
from auvsi_suas.models.takeoff_or_landing_event import TakeoffOrLandingEvent
from auvsi_suas.models.odlc import Odlc
from auvsi_suas.models.odlc import OdlcEvaluator
from auvsi_suas.models.uas_telemetry import UasTelemetry
from auvsi_suas.proto import mission_pb2

# Logging for the module
logger = logging.getLogger(__name__)


def generate_feedback(mission_config, user, team_eval):
    """Generates mission feedback for the given team and mission.

    Args:
        mission_config: The mission to evaluate the team against.
        user: The team user object for which to evaluate and provide feedback.
        team_eval: The team evaluation to fill.
    """
    feedback = team_eval.feedback

    # Calculate the total mission clock time.
    missions = MissionClockEvent.missions(user)
    mission_clock_time = datetime.timedelta(seconds=0)
    for mission in missions:
        duration = mission.duration()
        if duration is None:
            team_eval.warnings.append('Infinite mission clock.')
        else:
            mission_clock_time += duration
    feedback.mission_clock_time_sec = mission_clock_time.total_seconds()

    # Calculate total time in air.
    flight_periods = TakeoffOrLandingEvent.flights(user)
    if flight_periods:
        flight_time = reduce(lambda x, y: x + y,
                             [p.duration() for p in flight_periods])
        feedback.flight_time_sec = flight_time.total_seconds()
    else:
        feedback.flight_time_sec = 0
    # Find the user's flights.
    for period in flight_periods:
        if period.duration() is None:
            team_eval.warnings.append('Infinite flight period.')
    uas_period_logs = [
        UasTelemetry.dedupe(logs)
        for logs in UasTelemetry.by_time_period(user, flight_periods)
    ]
    uas_logs = list(itertools.chain.from_iterable(uas_period_logs))
    if not uas_logs:
        team_eval.warnings.append('No UAS telemetry logs.')

    # Determine interop telemetry rates.
    telem_max, telem_avg = UasTelemetry.rates(
        user, flight_periods, time_period_logs=uas_period_logs)
    if telem_max:
        feedback.uas_telemetry_time_max_sec = telem_max
    if telem_avg:
        feedback.uas_telemetry_time_avg_sec = telem_avg

    # Determine if the uas went out of bounds. This must be done for
    # each period individually so time between periods isn't counted as
    # out of bounds time. Note that this calculates reported time out
    # of bounds, not actual or possible time spent out of bounds.
    out_of_bounds = datetime.timedelta(seconds=0)
    feedback.boundary_violations = 0
    for logs in uas_period_logs:
        bv, bt = FlyZone.out_of_bounds(mission_config.fly_zones.all(), logs)
        feedback.boundary_violations += bv
        out_of_bounds += bt
    feedback.out_of_bounds_time_sec = out_of_bounds.total_seconds()

    # Determine if the uas hit the waypoints.
    feedback.waypoints.extend(
        UasTelemetry.satisfied_waypoints(
            mission_config.home_pos,
            mission_config.mission_waypoints.order_by('order'), uas_logs))

    # Evaluate the object detections.
    user_odlcs = Odlc.objects.filter(user=user).all()
    evaluator = OdlcEvaluator(user_odlcs, mission_config.odlcs.all())
    feedback.odlc.CopyFrom(evaluator.evaluate())

    # Determine collisions with stationary and moving obstacles.
    for obst in mission_config.stationary_obstacles.all():
        obst_eval = feedback.stationary_obstacles.add()
        obst_eval.id = obst.pk
        obst_eval.hit = obst.evaluate_collision_with_uas(uas_logs)
    for obst in mission_config.moving_obstacles.all():
        obst_eval = feedback.moving_obstacles.add()
        obst_eval.id = obst.pk
        obst_eval.hit = obst.evaluate_collision_with_uas(uas_logs)

    # Add judge feedback.
    try:
        judge_feedback = MissionJudgeFeedback.objects.get(
            mission=mission_config.pk, user=user.pk)
        feedback.judge.CopyFrom(judge_feedback.proto())
    except MissionJudgeFeedback.DoesNotExist:
        team_eval.warnings.append('No MissionJudgeFeedback for team.')

    # Sanity check mission time.
    judge_mission_clock = (
        feedback.judge.flight_time_sec + feedback.judge.post_process_time_sec)
    if abs(feedback.mission_clock_time_sec - judge_mission_clock) > 30:
        team_eval.warnings.append(
            'Mission clock differs between interop and judge.')


def score_team(team_eval):
    """Generates a score from the given feedback.

    Args:
        team_eval: A auvsi_suas.proto.MissionEvaluation for the team.
    """
    feedback = team_eval.feedback
    score = team_eval.score

    # Can't score without judge feedback.
    if not feedback.HasField('judge'):
        team_eval.warnings.append('Cant score due to no judge feedback.')
        return

    # Determine telemetry prerequisite.
    telem_prereq = False
    if (feedback.HasField('uas_telemetry_time_avg_sec') and
            feedback.uas_telemetry_time_avg_sec > 0):
        telem_prereq = feedback.uas_telemetry_time_avg_sec <= settings.INTEROP_TELEM_THRESHOLD_TIME_SEC
    # Score timeline.
    timeline = score.timeline
    flight_points = feedback.judge.flight_time_sec * settings.FLIGHT_TIME_SEC_TO_POINTS
    process_points = feedback.judge.post_process_time_sec * settings.PROCESS_TIME_SEC_TO_POINTS
    total_time_points = max(
        0, settings.MISSION_TIME_TOTAL_POINTS - flight_points - process_points)
    timeline.mission_time = total_time_points / settings.MISSION_TIME_TOTAL_POINTS
    total_time = feedback.judge.flight_time_sec + feedback.judge.post_process_time_sec
    over_time = max(0, total_time - settings.MISSION_MAX_TIME_SEC)
    timeline.mission_penalty = over_time * settings.MISSION_TIME_PENALTY_FROM_SEC
    timeline.timeout = 0 if feedback.judge.used_timeout else 1
    timeline.score_ratio = (
        (settings.MISSION_TIME_WEIGHT * timeline.mission_time) +
        (settings.TIMEOUT_WEIGHT * timeline.timeout) -
        timeline.mission_penalty)

    # Score autonomous flight.
    flight = score.autonomous_flight
    if feedback.judge.min_auto_flight_time:
        takeovers = feedback.judge.safety_pilot_takeovers
        flight.flight = max(0, 1 -
                            (takeovers * settings.AUTONOMOUS_FLIGHT_TAKEOVER))
    else:
        flight.flight = 0
    flight.telemetry_prerequisite = telem_prereq
    flight.waypoint_capture = (
        float(feedback.judge.waypoints_captured) / len(feedback.waypoints))
    wpt_scores = [w.score_ratio for w in feedback.waypoints]
    if telem_prereq:
        flight.waypoint_accuracy = (
            reduce(lambda x, y: x + y, wpt_scores) / len(feedback.waypoints))
    else:
        flight.waypoint_accuracy = 0
    flight.out_of_bounds_penalty = (
        feedback.judge.out_of_bounds * settings.BOUND_PENALTY +
        feedback.judge.unsafe_out_of_bounds * settings.SAFETY_BOUND_PENALTY)
    if feedback.judge.things_fell_off_uas:
        flight.things_fell_off_penalty = settings.TFOA_PENALTY
    else:
        flight.things_fell_off_penalty = 0
    if feedback.judge.crashed:
        flight.crashed_penalty = settings.CRASH_PENALTY
    else:
        flight.crashed_penalty = 0
    flight.score_ratio = (
        settings.AUTONOMOUS_FLIGHT_FLIGHT_WEIGHT * flight.flight +
        settings.WAYPOINT_CAPTURE_WEIGHT * flight.waypoint_capture +
        settings.WAYPOINT_ACCURACY_WEIGHT * flight.waypoint_accuracy -
        flight.out_of_bounds_penalty - flight.things_fell_off_penalty -
        flight.crashed_penalty)

    # Score obstacle avoidance.
    avoid = score.obstacle_avoidance
    avoid.telemetry_prerequisite = telem_prereq
    if telem_prereq:
        avoid.stationary_obstacle = (reduce(lambda x, y: x + y, [
            0.0 if o.hit else 1.0 for o in feedback.stationary_obstacles
        ]) / len(feedback.stationary_obstacles))
        avoid.moving_obstacle = (reduce(lambda x, y: x + y, [
            0.0 if o.hit else 1.0 for o in feedback.moving_obstacles
        ]) / len(feedback.moving_obstacles))
    else:
        avoid.stationary_obstacle = 0
        avoid.moving_obstacle = 0
    avoid.score_ratio = (
        avoid.stationary_obstacle * settings.STATIONARY_OBST_WEIGHT +
        avoid.moving_obstacle * settings.STATIONARY_OBST_WEIGHT)

    # Score objects.
    objects = score.object
    object_eval = feedback.odlc
    object_field_mapping = [
        ('classifications_score_ratio', 'characteristics'),
        ('geolocation_score_ratio', 'geolocation'),
        ('actionable_score_ratio', 'actionable'),
        ('autonomous_score_ratio', 'autonomy'),
        ('interop_score_ratio', 'interoperability'),
    ]
    for eval_field, score_field in object_field_mapping:
        total = reduce(lambda x, y: x + y,
                       [getattr(o, eval_field) for o in object_eval.odlcs])
        setattr(objects, score_field, float(total) / len(object_eval.odlcs))
    objects.extra_object_penalty = object_eval.extra_object_penalty_ratio
    objects.score_ratio = object_eval.score_ratio

    # Score air delivery.
    air = score.air_delivery
    air.delivery_accuracy = feedback.judge.air_delivery_accuracy_ft
    air.score_ratio = max(
        0, (settings.AIR_DELIVERY_THRESHOLD_FT - air.delivery_accuracy) /
        settings.AIR_DELIVERY_THRESHOLD_FT)

    # Score operational excellence.
    score.operational_excellence.score_ratio = (
        feedback.judge.operational_excellence_percent / 100.0)

    # Compute total score.
    if feedback.judge.min_auto_flight_time:
        score.score_ratio = (
            settings.TIMELINE_WEIGHT * score.timeline.score_ratio +
            settings.AUTONOMOUS_WEIGHT * score.autonomous_flight.score_ratio +
            settings.OBSTACLE_WEIGHT * score.obstacle_avoidance.score_ratio +
            settings.OBJECT_WEIGHT * score.object.score_ratio +
            settings.AIR_DELIVERY_WEIGHT * score.air_delivery.score_ratio +
            settings.OPERATIONAL_WEIGHT *
            score.operational_excellence.score_ratio)
    else:
        team_eval.warnings.append(
            'Insufficient flight time to receive any mission points.')
        score.score_ratio = 0


def evaluate_teams(mission_config, users=None):
    """Evaluates the teams (non admin users) of the competition.

    Args:
        mission_config: The mission to evaluate users against.
        users: Optional list of users to eval. If None will evaluate all.
    Returns:
        A auvsi_suas.proto.MultiUserMissionEvaluation.
    """
    # Start a results map from user to MissionEvaluation.
    mission_eval = mission_pb2.MultiUserMissionEvaluation()

    # If not provided, eval all users.
    if users is None:
        users = User.objects.all()

    logger.info('Starting team evaluations.')
    for user in users:
        # Ignore admins.
        if user.is_superuser:
            continue

        # Start the evaluation data structure.
        logger.info('Evaluation starting for user: %s.' % user.username)
        team_eval = mission_eval.teams.add()
        team_eval.team = user.username
        # Generate feedback.
        generate_feedback(mission_config, user, team_eval)
        # Generate score from feedback.
        score_team(team_eval)

    return mission_eval
