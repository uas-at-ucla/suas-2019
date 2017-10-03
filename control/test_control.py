import os
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)

import sys
sys.dont_write_bytecode = True
sys.path.insert(0, dname + '/../util')
sys.path.insert(0, dname + '/flight_control')
import time
import signal
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner

import process_manager
import copter_interface

class TestControl(unittest.TestCase):
    def setUp(self):
        self.test_drone = process_manager.ProcessManager()
        signal.signal(signal.SIGINT, self.test_drone.killall_signalled)

    def tearDown(self):
        self.test_drone.killall()

    def tearDownModule(self):
        self.test_drone.killall()

    def test_init(self):
        drone_address = self.spawn_simulated_drone(0.0, 0.0, 0.0, 0)
        copter = copter_interface.CopterInterface(drone_address)

        time.sleep(13.0)

        sensors = copter.sensor_reader.sensors.get()
        copter.stop()

        # Make sure we are filling in all our sensor values after initialization
        # of the flight controller.
        self.assertTrue(sensors["state"].get() is not None)
        self.assertTrue(sensors["armed"].get() is not None)
        self.assertTrue(sensors["voltage"].get() is not None)
        self.assertTrue(sensors["last_heartbeat"].get() is not None)
        self.assertTrue(sensors["velocity_x"].get() is not None)
        self.assertTrue(sensors["velocity_y"].get() is not None)
        self.assertTrue(sensors["velocity_z"].get() is not None)
        self.assertTrue(sensors["pitch"].get() is not None)
        self.assertTrue(sensors["roll"].get() is not None)
        self.assertTrue(sensors["yaw"].get() is not None)
        self.assertTrue(sensors["heading"].get() is not None)
        self.assertTrue(sensors["ground_speed"].get() is not None)
        self.assertTrue(sensors["air_speed"].get() is not None)

    def test_sensors(self):
        drone_address = self.spawn_simulated_drone(0.0, 0.0, 0.0, 0)
        copter = copter_interface.CopterInterface(drone_address)

        time.sleep(20.0)

        # Make sure we get stable values after flight controller initialization.
        for i in range(0, 10):
            sensors = copter.sensor_reader.sensors.get()

            self.assertTrue(sensors["state"].get() == "STANDBY")
            self.assertTrue(sensors["armed"].get() == "False")
            self.assertGreater(sensors["voltage"].get(), 12.3)
            self.assertLess(abs(sensors["velocity_x"].get()), 0.3)
            self.assertLess(abs(sensors["velocity_y"].get()), 0.3)
            self.assertLess(abs(sensors["velocity_z"].get()), 0.3)
            self.assertLess(abs(sensors["gps_rel_alt"].get()), 0.1)
            self.assertGreater(sensors["gps_satellites"].get(), 8)
            self.assertLess(abs(sensors["ground_speed"].get()), 0.1)
            self.assertLess(abs(sensors["air_speed"].get()), 0.1)
            time.sleep(1)

        copter.stop()

    def spawn_simulated_drone(self, lat, lng, alt, instance):
        self.test_drone.spawn_process("python " + \
                dname + "/flight_control/simulate_copter.py " + \
                "copter " + \
                "--home " + str(lat) + "," \
                          + str(lng) + "," \
                          + str(alt) + ",0 " + \
                "--instance " + str(instance))

        port = 5760 + 10 * instance
        return "tcp:127.0.0.1:" + str(port)

    def spawn_drone_interface(self, drone_address):
        self.proc_manager.spawn_process("python " + \
                "./flight_control/copter_interface.py " + \
                "--address " + drone_address)


if __name__ == "__main__":
    loader = TestLoader()
    suite = TestSuite((
        loader.loadTestsFromTestCase(TestControl),
    ))

    runner = TextTestRunner(verbosity = 2)
    runner.run(suite)
