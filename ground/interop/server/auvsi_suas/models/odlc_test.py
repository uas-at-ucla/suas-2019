"""Tests for the odlc module."""

import os.path
from auvsi_suas.models.gps_position import GpsPosition
from auvsi_suas.models.mission_clock_event import MissionClockEvent
from auvsi_suas.models.takeoff_or_landing_event import TakeoffOrLandingEvent
from auvsi_suas.models.odlc import Color
from auvsi_suas.models.odlc import Odlc
from auvsi_suas.models.odlc import OdlcEvaluator
from auvsi_suas.models.odlc import OdlcType
from auvsi_suas.models.odlc import Shape
from auvsi_suas.models.odlc import Orientation
from django.conf import settings
from django.contrib.auth.models import User
from django.core.files.uploadedfile import SimpleUploadedFile
from django.test import TestCase
from django.utils import timezone


class TestOdlc(TestCase):
    """Tests for the Odlc model."""

    def setUp(self):
        """Sets up the tests."""
        super(TestOdlc, self).setUp()
        self.user = User.objects.create_user('user', 'email@example.com',
                                             'pass')

    def test_valid(self):
        """Test creating a valid odlc."""
        with open(
                os.path.join(settings.BASE_DIR,
                             'auvsi_suas/fixtures/testdata/S.jpg')) as f:
            thumb = SimpleUploadedFile('thumb.jpg', f.read())

        l = GpsPosition(latitude=38, longitude=-76)
        l.save()

        t = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Test odlc',
            thumbnail=thumb)
        t.save()

    def test_unicode(self):
        """Test unicode conversion."""
        with open(
                os.path.join(settings.BASE_DIR,
                             'auvsi_suas/fixtures/testdata/S.jpg')) as f:
            thumb = SimpleUploadedFile('thumb.jpg', f.read())

        l = GpsPosition(latitude=38, longitude=-76)
        l.save()

        t = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Test odlc',
            thumbnail=thumb)
        t.save()

        self.assertTrue(t.__unicode__())

    def test_minimal_unicode(self):
        """Unicode with only user and odlc."""
        t = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t.save()

        self.assertTrue(t.__unicode__())

    def test_null_fields(self):
        """Only user and odlc type."""
        t = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t.save()

    def test_creation_time(self):
        """Creation time is set on creation and doesn't change on update."""
        t = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t.save()

        orig = t.creation_time
        self.assertIsNotNone(orig)

        t.alphanumeric = 'A'
        t.save()

        self.assertEqual(orig, t.creation_time)

    def test_last_modified_time(self):
        """Last modified time is set on creation and changes every update."""
        t = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t.save()

        orig = t.last_modified_time
        self.assertIsNotNone(orig)

        t.alphanumeric = 'A'
        t.save()

        self.assertGreater(t.last_modified_time, orig)

    def test_json(self):
        """Test odlc JSON."""
        l = GpsPosition(latitude=38, longitude=-76)
        l.save()

        t = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Test odlc',
            autonomous=True)
        t.save()

        d = t.json()

        self.assertIn('id', d)
        self.assertEqual(self.user.pk, d['user'])
        self.assertEqual('standard', d['type'])
        self.assertEqual(38, d['latitude'])
        self.assertEqual(-76, d['longitude'])
        self.assertEqual('s', d['orientation'])
        self.assertEqual('square', d['shape'])
        self.assertEqual('white', d['background_color'])
        self.assertEqual('ABC', d['alphanumeric'])
        self.assertEqual('black', d['alphanumeric_color'])
        self.assertEqual('Test odlc', d['description'])
        self.assertEqual(True, d['autonomous'])
        self.assertNotIn('thumbnail_approved', d)

        d = t.json(is_superuser=True)
        self.assertIn('description_approved', d)
        self.assertIn('thumbnail_approved', d)

        t.description_approved = True
        t.thumbnail_approved = True
        t.actionable_override = True
        t.save()
        d = t.json(is_superuser=True)
        self.assertEqual(True, d['description_approved'])
        self.assertEqual(None, d['thumbnail'])
        self.assertEqual(True, d['thumbnail_approved'])
        self.assertEqual(True, d['actionable_override'])

    def test_minimal_json(self):
        """Test odlc JSON with minimal data."""
        t = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t.save()

        d = t.json()

        self.assertIn('id', d)
        self.assertEqual(self.user.pk, d['user'])
        self.assertEqual('standard', d['type'])
        self.assertEqual(None, d['latitude'])
        self.assertEqual(None, d['longitude'])
        self.assertEqual(None, d['orientation'])
        self.assertEqual(None, d['shape'])
        self.assertEqual(None, d['background_color'])
        self.assertEqual(None, d['alphanumeric'])
        self.assertEqual(None, d['alphanumeric_color'])
        self.assertEqual(None, d['description'])
        self.assertEqual(False, d['autonomous'])

    def test_similar_classifications_ratio(self):
        """Tests similar classification ratios are computed correctly."""
        # Test equal standard odlcs.
        l = GpsPosition(latitude=38, longitude=-76)
        l.save()
        t1 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Test odlc',
            description_approved=True,
            autonomous=True)
        t1.save()
        t2 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Test other odlc',
            description_approved=False,
            autonomous=True)
        t2.save()
        self.assertAlmostEqual(1.0, t1.similar_classifications_ratio(t2))

        # Test unequal standard odlcs.
        t1.alphanumeric = 'DEF'
        t1.alphanumeric_color = Color.blue
        t1.save()
        self.assertAlmostEqual(3.0 / 5.0, t1.similar_classifications_ratio(t2))
        t1.shape = Shape.circle
        t1.background_color = Color.orange
        t1.save()
        self.assertAlmostEqual(1.0 / 5.0, t1.similar_classifications_ratio(t2))

        # Test different types.
        t1.odlc_type = OdlcType.off_axis
        t1.save()
        self.assertAlmostEqual(0, t1.similar_classifications_ratio(t2))

        # Test off_axis is same as standard.
        t2.odlc_type = OdlcType.off_axis
        t2.alphanumeric = 'DEF'
        t2.save()
        self.assertAlmostEqual(2.0 / 5.0, t1.similar_classifications_ratio(t2))

        # Test emergent type based on description approval.
        t1.odlc_type = OdlcType.emergent
        t1.save()
        t2.odlc_type = OdlcType.emergent
        t2.save()
        self.assertAlmostEqual(0.0, t1.similar_classifications_ratio(t2))
        t2.description_approved = True
        t2.save()
        self.assertAlmostEqual(1.0, t1.similar_classifications_ratio(t2))

    def test_actionable_submission(self):
        """Tests actionable_submission correctly filters submissions."""
        # t1 created and updated before take off.
        t1 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t1.save()
        t1.alphanumeric = 'A'
        t1.save()

        # t2 created before take off and updated in flight.
        t2 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t2.save()

        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=True)
        event.save()

        t2.alphanumeric = 'A'
        t2.save()

        # t3 created and updated in flight.
        t3 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t3.save()
        t3.alphanumeric = 'A'
        t3.save()

        # t4 created in flight and updated after landing.
        t4 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t4.save()

        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=False)
        event.save()

        t4.alphanumeric = 'A'
        t4.save()

        # t5 created and updated after landing.
        t5 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t5.save()
        t5.alphanumeric = 'A'
        t5.save()

        # t6 created and updated in second flight.
        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=True)
        event.save()
        t6 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t6.save()
        t6.alphanumeric = 'A'
        t6.save()
        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=False)
        event.save()

        # t7 with actionable_override set.
        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=True)
        event.save()
        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=False)
        event.save()
        t7 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            actionable_override=True)
        t7.save()

        self.assertFalse(t1.actionable_submission())
        self.assertFalse(t2.actionable_submission())
        self.assertTrue(t3.actionable_submission())
        self.assertFalse(t4.actionable_submission())
        self.assertFalse(t5.actionable_submission())
        self.assertFalse(t6.actionable_submission())
        self.assertTrue(t7.actionable_submission())

    def test_interop_submission(self):
        """Tests interop_submission correctly filters submissions."""
        # t1 created and updated before mission time starts.
        t1 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t1.save()
        t1.alphanumeric = 'A'
        t1.save()

        # t2 created before mission time starts and updated once it does.
        t2 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t2.save()

        # Mission time starts.
        event = MissionClockEvent(
            user=self.user, team_on_clock=True, team_on_timeout=False)
        event.save()

        t2.alphanumeric = 'A'
        t2.save()

        # t3 created and updated during mission time.
        t3 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t3.save()
        t3.alphanumeric = 'A'
        t3.save()

        # t4 created in in mission time and updated during timeout.
        t4 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t4.save()

        # Team takes timeout. Mission time stops.
        event = MissionClockEvent(
            user=self.user, team_on_clock=False, team_on_timeout=True)
        event.save()

        t4.alphanumeric = 'A'
        t4.save()

        # t5 created and updated during timeout.
        t5 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t5.save()
        t5.alphanumeric = 'A'
        t5.save()

        # t6 created and updated once mission time resumes.
        event = MissionClockEvent(
            user=self.user, team_on_clock=True, team_on_timeout=False)
        event.save()
        t6 = Odlc(user=self.user, odlc_type=OdlcType.standard)
        t6.save()
        t6.alphanumeric = 'A'
        t6.save()
        event = MissionClockEvent(
            user=self.user, team_on_clock=False, team_on_timeout=False)
        event.save()

        self.assertFalse(t1.interop_submission())
        self.assertFalse(t2.interop_submission())
        self.assertTrue(t3.interop_submission())
        self.assertFalse(t4.interop_submission())
        self.assertFalse(t5.interop_submission())
        self.assertTrue(t6.interop_submission())


class TestOdlcEvaluator(TestCase):
    """Tests for the OdlcEvaluator."""

    def setUp(self):
        """Setup the test case."""
        super(TestOdlcEvaluator, self).setUp()
        self.maxDiff = None
        self.user = User.objects.create_user('user', 'email@example.com',
                                             'pass')

        l1 = GpsPosition(latitude=38, longitude=-76)
        l1.save()
        l2 = GpsPosition(latitude=38.0003, longitude=-76)
        l2.save()
        l3 = GpsPosition(latitude=-38, longitude=76)
        l3.save()

        event = MissionClockEvent(
            user=self.user, team_on_clock=True, team_on_timeout=False)
        event.save()

        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=True)
        event.save()

        # A odlc worth full points.
        self.submit1 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Submit test odlc 1',
            description_approved=True,
            autonomous=True,
            thumbnail_approved=True)
        self.submit1.save()
        self.real1 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            orientation=Orientation.s,
            shape=Shape.square,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Real odlc 1')
        self.real1.save()

        event = MissionClockEvent(
            user=self.user, team_on_clock=False, team_on_timeout=False)
        event.save()

        # A odlc worth less than full points.
        self.submit2 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            orientation=Orientation.n,
            shape=Shape.circle,
            background_color=Color.white,
            # alphanumeric set below
            alphanumeric_color=Color.black,
            description='Submit test odlc 2',
            autonomous=False,
            thumbnail_approved=True)
        self.submit2.save()
        self.real2 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l2,
            orientation=Orientation.s,
            shape=Shape.triangle,
            background_color=Color.white,
            alphanumeric='ABC',
            alphanumeric_color=Color.black,
            description='Real test odlc 2')
        self.real2.save()

        # A odlc worth no points, so unmatched.
        self.submit3 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            orientation=Orientation.nw,
            shape=Shape.pentagon,
            background_color=Color.gray,
            alphanumeric='XYZ',
            alphanumeric_color=Color.orange,
            description='Incorrect description',
            autonomous=False,
            thumbnail_approved=True)
        self.submit3.save()
        self.real3 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            orientation=Orientation.e,
            shape=Shape.semicircle,
            background_color=Color.yellow,
            alphanumeric='LMN',
            # alphanumeric_color set below
            location=l3,
            description='Test odlc 3')
        self.real3.save()

        # Odlcs without approved image has no match value.
        self.submit4 = Odlc(
            user=self.user,
            odlc_type=OdlcType.emergent,
            location=l1,
            description='Test odlc 4',
            autonomous=False,
            thumbnail_approved=False)
        self.submit4.save()
        self.real4 = Odlc(
            user=self.user,
            odlc_type=OdlcType.emergent,
            location=l1,
            description='Test odlc 4')
        self.real4.save()

        # A odlc without location worth fewer points.
        self.submit5 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            orientation=Orientation.n,
            shape=Shape.trapezoid,
            background_color=Color.purple,
            alphanumeric='PQR',
            alphanumeric_color=Color.blue,
            description='Test odlc 5',
            autonomous=False,
            thumbnail_approved=True)
        self.submit5.save()
        self.real5 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            orientation=Orientation.n,
            shape=Shape.trapezoid,
            background_color=Color.purple,
            alphanumeric='PQR',
            alphanumeric_color=Color.blue,
            description='Test odlc 5')
        self.real5.save()

        # Emergent odlc with correct description.
        self.submit6 = Odlc(
            user=self.user,
            odlc_type=OdlcType.emergent,
            location=l1,
            description='Submit test odlc 6',
            description_approved=True,
            autonomous=True,
            thumbnail_approved=True)
        self.submit6.save()
        self.real6 = Odlc(
            user=self.user,
            odlc_type=OdlcType.emergent,
            location=l1,
            description='Real odlc 1',
            description_approved=True)
        self.real6.save()

        event = TakeoffOrLandingEvent(user=self.user, uas_in_air=False)
        event.save()

        # submit2 updated after landing.
        self.submit2.alphanumeric = 'ABC'
        self.submit2.save()
        self.submit3.alphanumeric_color = Color.yellow
        self.submit3.save()
        # Unused but not unmatched odlc.
        self.submit7 = Odlc(
            user=self.user,
            odlc_type=OdlcType.standard,
            location=l1,
            alphanumeric_color=Color.black,
            description='Submit unused test odlc 1',
            autonomous=False,
            thumbnail_approved=True)
        self.submit7.save()

        self.submitted_odlcs = [
            self.submit7, self.submit6, self.submit5, self.submit4,
            self.submit3, self.submit2, self.submit1
        ]
        self.real_odlcs = [
            self.real1, self.real2, self.real3, self.real4, self.real5,
            self.real6
        ]
        self.real_matched_odlcs = [
            self.real1, self.real2, self.real4, self.real5
        ]

    def test_match_value(self):
        """Tests the match value for two odlcs."""
        e = OdlcEvaluator(self.submitted_odlcs, self.real_odlcs)
        self.assertAlmostEqual(
            1.0,
            e.evaluate_match(self.submit1, self.real1).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.174,
            e.evaluate_match(self.submit2, self.real2).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.0,
            e.evaluate_match(self.submit3, self.real3).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.0,
            e.evaluate_match(self.submit4, self.real4).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.3,
            e.evaluate_match(self.submit5, self.real5).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.7,
            e.evaluate_match(self.submit6, self.real6).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.240,
            e.evaluate_match(self.submit7, self.real1).score_ratio,
            places=3)

        self.assertAlmostEqual(
            0.814,
            e.evaluate_match(self.submit1, self.real2).score_ratio,
            places=3)
        self.assertAlmostEqual(
            0.32,
            e.evaluate_match(self.submit2, self.real1).score_ratio,
            places=3)

    def test_match_odlcs(self):
        """Tests that matching odlcs produce maximal matches."""
        e = OdlcEvaluator(self.submitted_odlcs, self.real_odlcs)
        self.assertDictEqual({
            self.submit1: self.real1,
            self.submit2: self.real2,
            self.submit5: self.real5,
            self.submit6: self.real6,
            self.real1: self.submit1,
            self.real2: self.submit2,
            self.real5: self.submit5,
            self.real6: self.submit6,
        }, e.match_odlcs(self.submitted_odlcs, self.real_odlcs))

    def test_evaluate(self):
        """Tests that the evaluation is generated correctly."""
        e = OdlcEvaluator(self.submitted_odlcs, self.real_odlcs)
        d = e.evaluate()
        td = {t.real_odlc: t for t in d.odlcs}

        self.assertEqual(self.submit1.pk, td[self.real1.pk].submitted_odlc)
        self.assertEqual(True, td[self.real1.pk].image_approved)
        self.assertEqual(1.0, td[self.real1.pk].classifications_ratio)
        self.assertEqual(0.0, td[self.real1.pk].geolocation_accuracy_ft)
        self.assertEqual(True, td[self.real1.pk].actionable_submission)
        self.assertEqual(True, td[self.real1.pk].interop_submission)
        self.assertEqual(1.0, td[self.real1.pk].classifications_score_ratio)
        self.assertEqual(1.0, td[self.real1.pk].geolocation_score_ratio)
        self.assertEqual(1.0, td[self.real1.pk].actionable_score_ratio)
        self.assertEqual(1.0, td[self.real1.pk].autonomous_score_ratio)
        self.assertEqual(1.0, td[self.real1.pk].interop_score_ratio)
        self.assertAlmostEqual(1.0, td[self.real1.pk].score_ratio)

        self.assertEqual(self.submit2.pk, td[self.real2.pk].submitted_odlc)
        self.assertEqual(True, td[self.real2.pk].image_approved)
        self.assertEqual(0.6, td[self.real2.pk].classifications_ratio)
        self.assertAlmostEqual(
            109.444, td[self.real2.pk].geolocation_accuracy_ft, places=3)
        self.assertEqual(False, td[self.real2.pk].actionable_submission)
        self.assertEqual(False, td[self.real2.pk].interop_submission)
        self.assertEqual(0.6, td[self.real2.pk].classifications_score_ratio)
        self.assertAlmostEqual(
            0.270, td[self.real2.pk].geolocation_score_ratio, places=3)
        self.assertEqual(0.0, td[self.real2.pk].actionable_score_ratio)
        self.assertEqual(0.0, td[self.real2.pk].interop_score_ratio)
        self.assertAlmostEqual(0.174, td[self.real2.pk].score_ratio, places=3)

        self.assertEqual(True, td[self.real6.pk].description_approved)
        self.assertAlmostEqual(0.262, d.score_ratio, places=3)
        self.assertEqual(2, d.unmatched_odlc_count)

    def test_evaluate_no_submitted_odlcs(self):
        """Tests that evaluation works with no submitted odlcs."""
        e = OdlcEvaluator([], self.real_odlcs)
        d = e.evaluate()

        self.assertEqual(0, d.matched_score_ratio)
        self.assertEqual(0, d.unmatched_odlc_count)
        self.assertEqual(6, len(d.odlcs))

    def test_evaluate_no_real_odlcs(self):
        """Tests that evaluation works with no real odlcs."""
        e = OdlcEvaluator(self.submitted_odlcs, [])
        d = e.evaluate()

        self.assertEqual(0, d.matched_score_ratio)
        self.assertEqual(7, d.unmatched_odlc_count)
        self.assertAlmostEqual(-0.35, d.score_ratio, places=3)
        self.assertEqual(0, len(d.odlcs))
