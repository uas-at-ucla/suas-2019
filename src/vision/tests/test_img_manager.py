import signal
import sys
import os
import time
import unittest
import json
from unittest import mock

os.chdir(os.path.dirname(os.path.realpath(__file__)))
os.chdir('..')

sys.path.insert(0, 'util')
import img_manager

# Move to the vision directory


class TestImgManagerOthers(unittest.TestCase):
    def setUp(self):
        self.mock_truth = {
            'user': 'mock_user',
            'addr': 'mock_addr',
            'remote_dir': 'mock_remote_dir',
            'os_type': 'nt'
        }

    @mock.patch('img_manager.PureWindowsPath', autospec=True)
    @mock.patch('img_manager.PurePosixPath', autospec=True)
    @mock.patch('img_manager.process_manager')
    def test_init(self, mock_pm, mock_ppp, mock_pwp):

        # Initialize mock parameters
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # Check attributes
        self.assertEqual(manager.dir, os.path.abspath('mock_par'))
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
            with self.subTest(id=new_id):
                self.assertFalse(new_id in ids)
                self.assertFalse(new_id[0] == '-')
                ids.add(new_id)


@mock.patch('img_manager.print')
@mock.patch('img_manager.open')
@mock.patch('img_manager.json.load')
@mock.patch('img_manager.ImgManager._update_props')
class TestImgManagerGetProp(unittest.TestCase):
    def setUp(self):
        self.mock_truth = {
            'user': 'mock_user',
            'addr': 'mock_addr',
            'remote_dir': 'mock_remote_dir',
            'os_type': 'nt'
        }
        self.manager = img_manager.ImgManager('mock_par', self.mock_truth)

    def test_get_prop_reg_call(self, mock_update, mock_load, mock_open, mock_print):
        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}

        self.assertEqual(self.manager.get_prop('mock_id', 'b'), 2)

        mock_update.assert_not_called()
        mock_load.assert_called_with('mockfile')
        mock_open.assert_called_with(
            os.path.join(os.path.abspath('mock_par'), 'mock_id.json'), 'r')
        self.assertTrue(mock_open.return_value.__exit__.called)

    def test_get_prop_retry_err(self, mock_update, mock_load, mock_open, mock_print):
        mock_open.side_effect = OSError()
        self.manager.get_prop('mock_id', 'b')
        self.assertEqual(mock_update.call_count, 1)
        self.assertEqual(mock_open.call_count, 2)

    def test_get_prop_key_err(self, mock_update, mock_load, mock_open, mock_print):
        mock_load.side_effect = KeyError()
        self.assertEqual(self.manager.get_prop('mock_id', 'd'), None)


class TestImgManagerGetImg(unittest.TestCase):
    @mock.patch('img_manager.open')
    @mock.patch('img_manager.ImgManager._retrieve_img')
    def test_get_img_existing(self, mock_retrieve, mock_open):
        # init manager
        self.mock_truth = {
            'user': 'mock_user',
            'addr': 'mock_addr',
            'remote_dir': 'mock_remote_dir',
            'os_type': 'nt'
        }
        manager = img_manager.ImgManager('mock_par', self.mock_truth)

        # test getting an existing image


@mock.patch('img_manager.print')
@mock.patch('img_manager.open')
@mock.patch('img_manager.json.dump')
@mock.patch('img_manager.json.load')
@mock.patch('img_manager.ImgManager._update_props')
class TestImgManagerSetProp(unittest.TestCase):
    def setUp(self):
        self.mock_truth = {
            'user': 'mock_user',
            'addr': 'mock_addr',
            'remote_dir': 'mock_remote_dir',
            'os_type': 'nt'
        }
        self.manager = img_manager.ImgManager('mock_par', self.mock_truth)

    def test_set_prop_update(self, mock_update, mock_load, mock_dump, mock_open, mock_print):
        # test regular call
        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}

        self.manager.set_prop('mock_id', 'b', 0)

        mock_update.assert_not_called()
        mock_load.assert_called_with('mockfile')
        self.assertEqual(mock_open.return_value.__exit__.call_count, 2)
        mock_dump.assert_called_with({'a': 1, 'b': 0, 'c': 3}, 'mockfile')

    def test_set_prop_new(self, mock_update, mock_load, mock_dump, mock_open, mock_print):
        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}

        self.manager.set_prop('mock_id', 'd', 0)

        mock_open.return_value.__enter__.return_value = 'mockfile'
        mock_load.return_value = {'a': 1, 'b': 2, 'c': 3}
        self.manager.get_prop('mock_id', 'd', 0)
        mock_update.assert_not_called()
        mock_load.assert_called_with('mockfile')
        self.assertEqual(mock_open.return_value.__exit__.call_count, 3)
        mock_dump.assert_called_with({
            'a': 1,
            'b': 2,
            'c': 3,
            'd': 0
        }, 'mockfile')

    def test_set_prop_os_err(self, mock_update, mock_load, mock_dump,
                             mock_open, mock_print):
        mock_open.side_effect = OSError()
        self.manager.set_prop('mock_id', 'b', 0)
        self.assertEqual(mock_update.call_count, 1)
        self.assertEqual(mock_open.call_count, 2)


if __name__ == '__main__':
    unittest.main()
