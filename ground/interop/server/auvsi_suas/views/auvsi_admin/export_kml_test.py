"""Tests for the evaluate_teams module."""

import logging
from django.contrib.auth.models import User
from django.core.urlresolvers import reverse
from django.test import TestCase
from django.test.client import Client
from xml.etree import ElementTree

from auvsi_suas.models import units


class TestGenerateKMLCommon(TestCase):
    """Tests the generateKML view."""

    # String formatter for KML format that expects lon, lat, alt arguments
    coord_format = '<gx:coord>{} {} {}</gx:coord>'

    def setUp(self):
        """Sets up the tests."""
        # Create nonadmin user
        self.nonadmin_user = User.objects.create_user(
            'testuser', 'testemail@x.com', 'testpass')
        self.nonadmin_user.save()

        # Create admin user
        self.admin_user = User.objects.create_superuser(
            'testuser2', 'testemail@x.com', 'testpass')
        self.admin_user.save()

        # Create URLs for testing
        self.login_url = reverse('auvsi_suas:login')
        self.eval_url = reverse('auvsi_suas:export_data')

    def validate_kml(self, kml_data, folders, users, coordinates):
        ElementTree.fromstring(kml_data)
        for folder in folders:
            tag = '<name>{}</name>'.format(folder)
            self.assertTrue(tag in kml_data)

        for user in users:
            tag = '<name>{}</name>'.format(user)
            self.assertTrue(tag in kml_data)

        for coord in coordinates:
            coord_str = self.coord_format.format(coord[0], coord[1], coord[2])
            self.assertIn(coord_str, kml_data)


class TestGenerateKMLNoFixture(TestGenerateKMLCommon):
    """Tests the generateKML view."""

    def __init__(self, *args, **kwargs):
        super(TestGenerateKMLNoFixture, self).__init__(*args, **kwargs)
        self.folders = ['Teams', 'Missions']
        self.users = ['testuser']
        self.coordinates = []

    def test_generateKML_not_logged_in(self):
        """Tests the generate KML method."""
        response = self.client.get(self.eval_url)
        self.assertEqual(403, response.status_code)

    def test_generateKML_nonadmin(self):
        """Tests the generate KML method."""
        self.client.post(self.login_url,
                         {'username': 'testuser',
                          'password': 'testpass'})
        response = self.client.get(self.eval_url)
        self.assertEqual(403, response.status_code)

    def test_generateKML(self):
        """Tests the generate KML method."""
        self.client.post(self.login_url,
                         {'username': 'testuser2',
                          'password': 'testpass'})
        response = self.client.get(self.eval_url)
        self.assertEqual(200, response.status_code)

        kml_data = response.content
        self.validate_kml(kml_data, self.folders, self.users, self.coordinates)


class TestGenerateKMLWithFixture(TestGenerateKMLCommon):
    """Tests the generateKML view."""
    fixtures = ['testdata/sample_mission.json']

    def __init__(self, *args, **kwargs):
        super(TestGenerateKMLWithFixture, self).__init__(*args, **kwargs)
        self.folders = ['Teams', 'Missions']
        self.users = ['testuser', 'user0', 'user1']
        self.coordinates = [(lat, lon, units.feet_to_meters(alt))
                            for lat, lon, alt in [
                                (-76.0, 38.0, 0.0),
                                (-76.0, 38.0, 10.0),
                                (-76.0, 38.0, 20.0),
                                (-76.0, 38.0, 30.0),
                                (-76.0, 38.0, 100.0),
                                (-76.0, 38.0, 30.0),
                                (-77.0, 38.0, 60.0),
                            ]]

    def test_generateKML_not_logged_in(self):
        """Tests the generate KML method."""
        response = self.client.get(self.eval_url)
        self.assertEqual(403, response.status_code)

    def test_generateKML_nonadmin(self):
        """Tests the generate KML method."""
        self.client.post(self.login_url,
                         {'username': 'testuser',
                          'password': 'testpass'})
        response = self.client.get(self.eval_url)
        self.assertEqual(403, response.status_code)

    def test_generateKML(self):
        """Tests the generate KML method."""
        self.client.post(self.login_url,
                         {'username': 'testuser2',
                          'password': 'testpass'})
        response = self.client.get(self.eval_url)
        self.assertEqual(200, response.status_code)

        kml_data = response.content
        self.validate_kml(kml_data, self.folders, self.users, self.coordinates)
