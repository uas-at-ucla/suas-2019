"""Tests for the login module."""

from django.contrib.auth.models import User
from django.core.urlresolvers import reverse
from django.test import TestCase

login_url = reverse('auvsi_suas:login')


class TestLoginUserView(TestCase):
    """Tests the login_user view."""

    def setUp(self):
        """Sets up the test by creating a test user."""
        self.user = User.objects.create_user('testuser', 'testemail@x.com',
                                             'testpass')
        self.user.save()

    def test_invalid_request(self):
        """Tests an invalid request by mis-specifying parameters."""
        # Test GET instead of POST, without parameters
        response = self.client.get(login_url)
        self.assertEqual(405, response.status_code)

        # Test GET instrad of POST, with proper parameters
        response = self.client.get(login_url, {
            'username': 'testuser',
            'password': 'testpass',
        })
        self.assertEqual(405, response.status_code)

        # Test POST with no parameters
        response = self.client.post(login_url)
        self.assertEqual(400, response.status_code)

        # Test POST with a missing parameter
        response = self.client.post(login_url, {'username': 'test'})
        self.assertEqual(400, response.status_code)
        response = self.client.post(login_url, {'password': 'test'})
        self.assertEqual(400, response.status_code)

    def test_invalid_credentials(self):
        """Tests invalid credentials for login."""
        response = self.client.post(login_url,
                                    {'username': 'a',
                                     'password': 'b'})
        self.assertEqual(401, response.status_code)

    def test_correct_credentials(self):
        """Tests correct credentials for login."""
        response = self.client.post(login_url, {
            'username': 'testuser',
            'password': 'testpass',
        })
        self.assertEqual(200, response.status_code)
