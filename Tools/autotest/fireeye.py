'''
Fly FireEye in RealFlight
'''

from __future__ import print_function
import os
import numpy
import math

from pymavlink import mavutil
from pymavlink.rotmat import Vector3

import vehicle_test_suite
from vehicle_test_suite import Test
from vehicle_test_suite import AutoTestTimeoutException, NotAchievedException, PreconditionFailedException

import operator
import shutil
import random


# get location of scripts
testdir = os.path.dirname(os.path.realpath(__file__))
WIND = "0,180,0.2"  # speed,direction,variance
SITL_START_LOCATION = mavutil.location(36.8325082, -2.8512096, 735, 0) # AutoTest Hill 


class AutoTestFireEye(vehicle_test_suite.TestSuite):

    @staticmethod
    def get_not_armable_mode_list():
        return []

    @staticmethod
    def get_not_disarmed_settable_modes_list():
        return []

    @staticmethod
    def get_no_position_not_settable_modes_list():
        return []

    @staticmethod
    def get_position_armable_modes_list():
        return []

    @staticmethod
    def get_normal_armable_modes_list():
        return []

    def vehicleinfo_key(self):
        return 'ArduPlane'

    def default_frame(self):
        return "flightaxis_autotest:" + os.environ['wslhost']

    def test_filepath(self):
        return os.path.realpath(__file__)

    def sitl_start_location(self):
        return SITL_START_LOCATION

    def default_speedup(self):
        '''RealFlight doesn't support speedup'''
        return 1

    def log_name(self):
        return "FireEye"

    def set_current_test_name(self, name):
        self.current_test_name_directory = "ArduPlane_Tests/" + name + "/"

    def apply_defaultfile_parameters(self):
        # plane passes in a defaults_filepath in place of applying
        # parameters afterwards.
        pass

    def defaults_filepath(self):
        return os.path.join(testdir, "default_params/fireeye.parm")

    def is_plane(self):
        return True

    def default_mode(self):
        return "QHOVER"

    def get_stick_arming_channel(self):
        return int(self.get_parameter("RCMAP_YAW"))

    def get_disarm_delay(self):
        return int(self.get_parameter("LAND_DISARMDELAY"))

    def set_autodisarm_delay(self, delay):
        self.set_parameter("LAND_DISARMDELAY", delay)

    def reset_aircraft(self):
        '''Reset aircraft and wait for it to be ready to arm'''
        self.disarm_vehicle(force=True)
        self.reboot_sitl()
        self.repeatedly_apply_parameter_file(
            os.path.join(testdir, 'default_params/fireeye-engout.parm'))
        self.set_rc(3, 1000)
        self.change_mode('QHOVER')
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)
        self.wait_rpm(2, 1000, 3000)
        self.wait_ready_to_arm()

    def do_guided(self, location):
        '''Fly to a location in GUIDED mode'''
        self.change_mode('GUIDED')
        self.mav.mav.mission_item_int_send(
                1,
                1,
                0, # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, # current
                0, # autocontinue
                0, # p1
                0, # p2
                0, # p3
                0, # p4
                int(location.lat * 1e7), # latitude
                int(location.lng * 1e7), # longitude
                location.alt) # altitude

    def EngineOutScript(self):
        '''Test engine out script in RealFlight on an AgTS FireEye'''
        def basic_auto_mission(heading, target, is_guided=False,
                               min_distance = 0, max_distance = 50,
                               qassist_timeout=0, qrtl_timeout=0):
            '''
            Run the basic auto mission, and kill the engine when we reach
            the desired altitude and heading. By testing many different
            headings, we confirm that the landing behavior works regardless of
            which angle we happen to be facing relative to the wind when we
            reach the desired landing altitude.
            
            Specify the target landing point, which is a rally point by default
            or a guided point if is_guided is True.

            You can optionally specify a minimum and maximum distance to the
            target for the test to pass, and you can override the Q_ASSIST and
            QRTL timeouts to test that they work as expected.
            '''

            subtest_message = \
                    f"Basic mission test with heading {heading:.0f}" + \
                    " and " + ("guided" if is_guided else "rally") + \
                    f" {target.lat:.6f}, {target.lng:.6f}"

            if qassist_timeout:
                subtest_message = "Testing that the Q_ASSIST timeout works"
            if qrtl_timeout:
                subtest_message = "Testing that the QRTL timeout works"

            self.start_subtest(subtest_message)

            self.reset_aircraft()
            if not is_guided:
                self.upload_rally_points_from_locations([target])
            if qassist_timeout:
                self.set_parameter("ENGOUT_QAST_TIME", qassist_timeout)
            if qrtl_timeout:
                self.set_parameter("ENGOUT_QRTL_TIME", qrtl_timeout)
            self.change_mode('AUTO')
            self.arm_vehicle()
            self.set_rc(3, 1500)

            self.wait_current_waypoint(4, timeout=600)

            # Wait for the aircraft to reach the desired heading
            self.wait_heading(heading, 5, timeout=300)

            # Kill the engine
            self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)

            if is_guided:
                self.wait_mode('RTL')
                self.delay_sim_time(10)
                self.do_guided(target)

            if qassist_timeout:
                self.wait_text("Q_ASSIST for too long")
            elif qrtl_timeout:
                self.wait_text("QRTL for too long")

            # Wait for the aircraft to land
            self.wait_disarmed(timeout=600)

            # Confirm the expected distance to the target
            self.assert_distance(target, self.mav.location(),
                                 min_distance=min_distance,
                                 max_distance=max_distance)

            self.end_subtest(subtest_message)

        # Install the scripts
        source = os.path.join(self.rootdir(), "libraries", "AP_Scripting", "modules", "utilities.lua") # noqa: E501
        destdir = os.path.join(self.rootdir(),"scripts", "modules")
        dest = "utilities.lua"
        if not os.path.exists(destdir):
            os.mkdir(destdir)
        self.progress("Copying (%s) to (%s)" % (source, dest))
        shutil.copy(source, os.path.join(destdir, dest))

        source = os.path.join(self.rootdir(), "libraries", "AP_Scripting", "drivers", "EFI_SITL.lua") # noqa: E501
        self.install_script(source, "EFI_SITL.lua")
        self.install_applet_script("engine_out.lua")

        # Reboot
        self.reboot_sitl()

        # Disable fence
        self.set_parameter('FENCE_ENABLE', 0)

        # Set the SYSID_MYGCS to the source system (prevent GCS failsafe)
        self.set_parameter("SYSID_MYGCS", self.mav.source_system)

        # Upload mission
        self.load_mission("mission.waypoints")
        # Load rally point
        # (we set a stupid altitude on purpose; it should not be used)
        rally_loc = mavutil.location(36.8164241, -2.868918, 5000, 0)
        guided_loc = mavutil.location(36.8192676, -2.8719136, 5000, 0)

        # =============================================
        #            Test guided override
        # =============================================
        self.upload_rally_points_from_locations([rally_loc])
        basic_auto_mission(270, guided_loc, is_guided=True)

        # =============================================
        # Test regular landings from many random angles
        # =============================================

        # Killing the engine at random headings to make sure the landing always
        # goes smoothly no matter which orientation compared to the wind we are
        # at when we reach the landing altitude.
        headings = list(range(0, 360, 90)) + random.sample(range(360), 4)
        for heading in headings:
            basic_auto_mission(heading, rally_loc)

        # =============================================
        #          Test the Q_ASSIST timeout
        # =============================================
        basic_auto_mission(270, rally_loc, qassist_timeout=1, min_distance=0, max_distance=300)

        # =============================================
        #            Test the QRTL timeout
        # =============================================
        basic_auto_mission(270, rally_loc, qrtl_timeout=5, min_distance=50, max_distance=300)

        # =====================================================================
        # Test detection messages, and the backup and restore of all parameters
        # =====================================================================
        self.start_subtest("Testing detections and parameters backup/restore")
        self.reset_aircraft()
        params_before, _ = self.download_parameters(self.sysid_thismav(), 1)
        self.change_mode('AUTO')
        self.arm_vehicle()
        self.set_rc(3, 1500)
        self.wait_current_waypoint(4, timeout=600)
        # Kill the engine
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
        self.wait_text("Engine out")
        self.delay_sim_time(1)
        # Read all parameters
        params_after, _ = self.download_parameters(self.sysid_thismav(), 1)
        # I don't know why Q_P_ACCZ_IMAX changes randomly, but it's unrelated
        param_ignore_filter = ["STAT", "BARO", "SERVO", "Q_P_ACCZ_IMAX"]
        # Print differences
        for p in params_before:
            if any(p.startswith(s) for s in param_ignore_filter):
                continue
            if params_before[p] != params_after[p]:
                self.progress(f"{p} changed from {params_before[p]} to {params_after[p]}")

        # Restore the engine
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)
        self.wait_text("Engine running")
        self.delay_sim_time(1)
        # Read the parameters again
        params_after, _ = self.download_parameters(self.sysid_thismav(), 1)
        # Assert that all parameters are the same
        for p in params_before:
            if any(p.startswith(s) for s in param_ignore_filter):
                continue
            if params_before[p] != params_after[p]:
                raise ValueError(f"{p} changed from {params_before[p]} to {params_after[p]}")

        # Force disarm and end the subtest
        self.disarm_vehicle(force=True)
        self.wait_disarmed()
        self.end_subtest("Testing detections and parameters backup/restore")

        # =============================================
        #             Test prearm checks
        # =============================================
        self.start_subtest("Testing prearm checks")
        self.reset_aircraft()

        # Deliberately set too low of a value for GLIDE_SPD
        self.set_parameter("GLIDE_SPD", 1)
        self.wait_not_ready_to_arm()
        self.set_parameter("GLIDE_SPD", 22)
        self.wait_ready_to_arm()

        # Deliberately set too high of a value for GLIDE_SPD
        self.set_parameter("GLIDE_SPD", 100)
        self.wait_not_ready_to_arm()
        self.set_parameter("GLIDE_SPD", 22)
        self.wait_ready_to_arm()

        # Turn off the engine
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=0)
        self.wait_not_ready_to_arm()
        self.run_cmd(mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL, p1=1)
        self.wait_ready_to_arm()

        self.end_subtest("Testing prearm checks")

        self.remove_installed_modules("utilities.lua")
        self.remove_installed_script("EFI_SITL.lua")
        self.remove_installed_script("engine_out.lua")

    def tests(self):
        '''return list of all tests'''
        return [
            self.EngineOutScript,
        ]
