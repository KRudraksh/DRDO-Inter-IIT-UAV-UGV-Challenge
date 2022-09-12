#!/usr/bin/env python

# third-party imports
import rospy
from mavros_msgs.srv import (
    SetMode,
    SetModeRequest,
    CommandBool,
    CommandBoolRequest,
    CommandTOL,
    CommandTOLRequest,
)
from sensor_msgs.msg import NavSatFix


class Drone:
    def __init__(self):
        rospy.loginfo("Initializing Drone class")
        self.global_position = None
        rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.feedback_global_position
        )

    def feedback_global_position(self, msg):
        self.global_position = msg

    def arm(self):
        rospy.loginfo("Arming the drone")
        call_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        data = CommandBoolRequest()
        data.value = True
        return call_srv(data)

    def takeoff(self, latitude, longitude, altitude):
        rospy.loginfo("Initiating takeoff")
        call_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        data = CommandTOLRequest()
        data.latitude = latitude
        data.longitude = longitude
        data.altitude = altitude
        return call_srv(data)

    def land(self, latitude, longitude, altitude):
        rospy.loginfo("Initiating landing")
        call_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        data = CommandTOLRequest()
        data.latitude = latitude
        data.longitude = longitude
        data.altitude = altitude
        return call_srv(data)

    def mavros_set_mode(self, mode):
        rospy.loginfo("Switching to mode: " + mode)
        call_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        data = SetModeRequest()
        data.custom_mode = mode
        return call_srv(data)

    def switch_to_guided_mode(self):
        mode = "GUIDED"
        return self.mavros_set_mode(mode)

    def switch_to_stabilize_mode(self):
        mode = "STABILIZE"
        return self.mavros_set_mode(mode)

