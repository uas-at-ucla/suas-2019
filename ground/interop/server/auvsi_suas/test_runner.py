"""Test runner which uses a different filepattern."""

import logging
import shutil
import tempfile
from django.conf import settings
from django.core.cache import cache
from django.test import runner


class AuvsiSuasTestRunner(runner.DiscoverRunner):
    def __init__(self, *args, **kwargs):
        """Initializes the parent with a different filepattern."""
        kwargs['pattern'] = '*_test.py'
        super(AuvsiSuasTestRunner, self).__init__(*args, **kwargs)

    def setup_test_environment(self):
        """Create a custom MEDIA_ROOT and configure sendfile."""
        super(AuvsiSuasTestRunner, self).setup_test_environment()

        # Scratch MEDIA_ROOT for odlc uploads
        self.media_root = settings.MEDIA_ROOT
        settings.MEDIA_ROOT = tempfile.mkdtemp()

        # We don't have Apache during testing, we need to have Django send
        # files directly.
        self.sendfile_backend = settings.SENDFILE_BACKEND
        settings.SENDFILE_BACKEND = 'sendfile.backends.development'

        # Disable logging
        logging.disable(logging.CRITICAL)

        # Clear any stale data in cache.
        cache.clear()

    def teardown_test_environment(self):
        shutil.rmtree(settings.MEDIA_ROOT)

        settings.MEDIA_ROOT = self.media_root
        settings.SENDFILE_BACKEND = self.sendfile_backend

        logging.disable(logging.NOTSET)

        # Clear any test data in the cache.
        cache.clear()

        super(AuvsiSuasTestRunner, self).teardown_test_environment()
