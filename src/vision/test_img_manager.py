import signal
import sys
import time
import unittest
from unittest import mock

import img_manager

class TestImgManager(unittest.TestCase):

    def setUp(self):
        self.mock_truth = {'user': 'mock_user', 'addr': 'mock_addr', 'remote_dir': 'mock_remote_dir', 'os_type': 'nt'}

    @mock.patch('img_manager.PureWindowsPath', autospec=True)
    @mock.patch('img_manager.PurePosixPath', autospec=True)
    @mock.patch('img_manager.process_manager')
    def test_init(self, mock_pm, mock_ppp, mock_pwp):

        # Initialize mock parameters
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # Check attributes
        self.assertEqual(manager.dir, 'mock_par')
        self.assertEqual(manager.truth_src, self.mock_truth)
        self.assertTrue(mock_pm.ProcessManager.called)

        # Check paths
        mock_pwp.assert_called_with(self.mock_truth['remote_dir'])
        mock_ppp.assert_not_called()

        # Reset for next set of parameters
        mock_pwp.reset_mock()
        mock_ppp.reset_mock()

        # Try posix #####
        self.mock_truth['os_type'] = 'posix'
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # Check paths
        mock_pwp.assert_not_called()
        mock_ppp.assert_called_with(self.mock_truth['remote_dir'])

        # Reset for next set of parameters
        mock_pwp.reset_mock()
        mock_ppp.reset_mock()

        # Try other #####
        self.mock_truth['os_type'] = 'other'
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # Check paths
        mock_pwp.assert_not_called()
        mock_ppp.assert_called_with(self.mock_truth['remote_dir'])

    @mock.patch('img_manager.process_manager')
    def test_stop(self, mock_pm):
        # initialize
        manager = img_manager.ImgManager('mock_par', self.mock_truth)
        
        # check stop
        manager.stop()
        self.assertTrue(mock_pm.ProcessManager.called)

    # TODO test the rest of the methods 

if __name__ == '__main__':
    unittest.main()
