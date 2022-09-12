#!/usr/bin/env python

# standard imports
import sys
import time

# third-party imports
import rospy

# app imports
from drone_utils import Drone


class SummerMapping:
    def __init__(self):
        rospy.init_node("summer_mapping")
        self.iris = Drone()

    def takeoff_iris(self):
        max_altitude = 18
        gp = self.iris.global_position
        while gp == None:
            if rospy.is_shutdown():
                sys.exit(1)
            time.sleep(1)
            rospy.loginfo("Waiting for global position to be updated...")
            gp = self.iris.global_position
        rospy.loginfo("Got new global position of the drone: ")
        rospy.loginfo(gp)
        srv_call = self.iris.switch_to_guided_mode()
        if not srv_call.mode_sent:
            rospy.logerr("Unable to switch to GUIDED mode")
            sys.exit(1)
        srv_call = self.iris.arm()
        if not srv_call.success:
            rospy.logerr("Unable to ARM drone")
            sys.exit(1)
        srv_call = self.iris.takeoff(gp.latitude, gp.longitude, max_altitude)
        if not srv_call.success:
            rospy.logerr("Unable to Takeoff drone")
            sys.exit(1)
        # TODO: check to ensure drone has reached the 18m height
        # TODO: start with ransac

    def map_road(self):
        self.takeoff_iris()

if __name__ == "__main__":
    node = SummerMapping()
    node.map_road()

