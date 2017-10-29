"""Tests for the mission_judge_feedback module."""

import datetime
from mission_judge_feedback import MissionJudgeFeedback
from django.test import TestCase


class TestMissionJudgeFeedback(TestCase):
    def test_proto(self):
        """Tests proto()."""
        feedback = MissionJudgeFeedback(
            flight_time=datetime.timedelta(seconds=1),
            post_process_time=datetime.timedelta(seconds=2),
            used_timeout=True,
            min_auto_flight_time=True,
            safety_pilot_takeovers=3,
            waypoints_captured=5,
            out_of_bounds=6,
            unsafe_out_of_bounds=7,
            things_fell_off_uas=False,
            crashed=False,
            air_delivery_accuracy_ft=8,
            operational_excellence_percent=9)
        pb = feedback.proto()

        self.assertAlmostEqual(1, pb.flight_time_sec)
        self.assertAlmostEqual(2, pb.post_process_time_sec)
        self.assertTrue(pb.used_timeout)
        self.assertTrue(pb.min_auto_flight_time)
        self.assertEqual(3, pb.safety_pilot_takeovers)
        self.assertEqual(5, pb.waypoints_captured)
        self.assertEqual(6, pb.out_of_bounds)
        self.assertEqual(7, pb.unsafe_out_of_bounds)
        self.assertFalse(pb.things_fell_off_uas)
        self.assertFalse(pb.crashed)
        self.assertAlmostEqual(8, pb.air_delivery_accuracy_ft)
        self.assertAlmostEqual(9, pb.operational_excellence_percent)

        feedback.air_delivery_accuracy_ft = None
        pb = feedback.proto()

        self.assertFalse(pb.HasField('air_delivery_accuracy_ft'))
