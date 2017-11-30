import os
# Start off fresh by making sure that our working directory is the same as the
# directory that this script is in.
dname = os.path.dirname(os.path.realpath(__file__))
os.chdir(dname)

import pickle
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, '../util')
sys.path.insert(0, 'flight_control')
sys.path.insert(0, 'commander')
import time
import signal
import unittest
from unittest import TestLoader, TestSuite, TextTestRunner

import process_manager
import copter_interface
import commander

class TestControl(unittest.TestCase):
    def setUp(self):
        self.RUN_GROUND = True;

        self.test_drone = process_manager.ProcessManager()

        signal.signal(signal.SIGINT, self.kill_processes_and_exit)

    def tearDown(self):
        self.kill_processes()

    def tearDownModule(self):
        self.kill_processes()

    def test_commander(self):
        if self.RUN_GROUND:
            self.test_drone.run_command("python ../ground/client/build.py")
            self.test_drone.spawn_process("python ../ground/run_ground.py")

        self.test_drone.spawn_process( \
                "python commander/drone_communications.py")

        init_lat = 38.1470000;
        init_lng = -76.4284722;
        drone_address = self.spawn_simulated_drone(init_lat, init_lng, 0.0, 0)

        test_commander = commander.Commander(drone_address)
        unittest.registerResult(test_commander)

        test_commander.add_command(commander.TakeoffCommand())

        DEVIATION = 0.005

        test_commander.add_command(commander.GotoCommand(init_lat + DEVIATION, \
                                                         init_lng + DEVIATION, \
                                                         100))

        test_commander.start_mission()

        test_commander.stop()
        pickle_location = dname+'/commander/mission.pickle'
        if not os.path.isfile(pickle_location):
            print("Did not find mission pickle!")
        with open(pickle_location, "rb") as f:
            mission_commands = pickle.load(f)
            print("Found pickle")

    def test_copter_interface_init(self):
        drone_address = self.spawn_simulated_drone(0.0, 0.0, 0.0, 0)
        copter = copter_interface.CopterInterface(drone_address)

        while not copter.vehicle.is_armable:
            time.sleep(1.0)

        sensors = copter.sensor_reader.sensors.get()

        # Make sure we are filling in all our sensor values after initialization
        # of the flight controller.
        self.assertTrue(sensors["state"].get() is not None)
        self.assertTrue(sensors["pixhawk_state"].get() is not None)
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

        copter.stop()

    def spawn_simulated_drone(self, lat, lng, alt, instance):
        self.test_drone.spawn_process("python " + \
                "flight_control/dronekit-sitl/dronekit_sitl/__init__.py " + \
                "copter " + \
                "--home " + str(lat) + "," \
                          + str(lng) + "," \
                          + str(alt) + ",0 " + \
                "--instance " + str(instance))

        # Wait to make sure the simulated drone is completely set up before
        # continuing.
        time.sleep(2.0)

        port = 5760 + 10 * instance
        return "tcp:127.0.0.1:" + str(port)

    def kill_processes(self):
        self.test_drone.killall()

    def kill_processes_and_exit(self, signal, frame):
        print("Exiting")
        self.kill_processes()
        exit(0)

if __name__ == "__main__":
    loader = TestLoader()
    suite = TestSuite((
        loader.loadTestsFromTestCase(TestControl),
    ))

    runner = TextTestRunner(verbosity = 2)

    ret = not runner.run(suite).wasSuccessful()
    sys.exit(ret)
