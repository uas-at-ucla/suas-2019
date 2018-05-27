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

    def test_gen_id(self):
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        ids = set()
        for i in range(5000):
            new_id = img_manager.ImgManager.gen_id()
            self.assertFalse(new_id in ids)
            ids.add(new_id)

    @mock.patch('img_manager.open')
    @mock.patch('img_manager.json.load')
    @mock.patch('img_manager.ImgManager._retrieve_img')
    def test_get_prop(self, mock_retrieve, mock_load, mock_open):
        def reset_mocks():
            mock_retrieve.reset_mock()
            mock_load.reset_mock()
            mock_open.reset_mock()

        # init manager
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # test regular call
        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}

        self.assertEqual(manager.get_prop('mock_id', 'b'), 2)

        mock_retrieve.assert_not_called()
        mock_load.assert_called_with('mockfile')
        mock_open.return_value.__enter__.assert_called_with(os.path.join('mock_par', 'mock_id.json'), 'r')
        self.assertTrue(mock_open.return_value.__exit__.called)

        reset_mocks()
        
        # test retry error
        mock_open.side_effect = OSError()
        try:
            manager.get_prop('mock_id', 'b')
        except OSError:
            self.assertEqual(mock_retrieve.call_count, 2)
        else:
            self.assertTrue(False) # did not properly throw error

        mock_open.side_effect = None
        reset_mocks()

        # test key error
        mock_load.side_effect = KeyError()
        try:
            manager.get_prop('mock_id', 'd')
        except KeyError:
            pass
        else:
            self.assertTrue(False) # did not properly throw error
        
    @mock.patch('img_manager.open')
    @mock.patch('img_manager.json.dump')
    @mock.patch('img_manager.json.load')
    @mock.patch('img_manager.ImgManager._retrieve_img')
    def test_set_prop(self, mock_retrieve, mock_load, mock_dump, mock_open):
        def reset_mocks():
            mock_retrieve.reset_mock()
            mock_load.reset_mock()
            mock_dump.reset_mock()
            mock_open.reset_mock()

        # init manager
        manager = img_manager.ImgManager('mock_par', self.mock_truth)
        
        # test regular call
        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}

        manager.set_prop('mock_id', 'b', 0)

        mock_retrieve.assert_not_called()
        mock_load.assert_called_with('mockfile')
        mock_open.return_value.__enter__assert_called_with(os.path.join('mock_par', 'mock_id.json'), 'r')
        mock_open.return_value.__enter__.assert_called_with(os.path.join('mock_par', 'mock_id.json'), 'w')
        self.assertEqual(mock_open.return_value.__exit__.call_count, 2)
        mock_dump.assert_called_with({'a': 1, 'b': 0, 'c': 3}, 'mockfile')

        reset_mocks()

        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}
        manager.get_prop('mock_id', 'd', 0)
        mock_retrieve.assert_not_called()
        mock_load.assert_called_with('mockfile')
        mock_open.return_value.__enter__.assert_called_with(os.path.join('mock_par', 'mock_id.json'), 'r')
        mock_open.return_value.__enter__.assert_called_with(os.path.join('mock_par', 'mock_id.json'), 'w')
        self.assertEqual(mock_open.__exit__.call_count, 2)
        mock_dump.assert_called_with({'a': 1, 'b': 2, 'c': 3, 'd': 0}, 'mockfile')

        reset_mocks()

        # test oserror
        mock_open.side_effect = OSError()

        try:
            manager.set_prop('mock_id', 'b', 0)
        except OSError:
            self.assertEqual(mock_retrieve.call_count, 2)
        else:
            self.assertTrue(False) # error not properly thrown

    # TODO test the rest of the methods 

if __name__ == '__main__':
    unittest.main()
