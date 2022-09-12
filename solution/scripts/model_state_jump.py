#!/usr/bin/env python

# standard imports
import sys
import time

# third-party imports
import rospy
from gazebo_msgs.msg import ModelState

class SummerMapping:
    def __init__(self):
        rospy.init_node("model_state_jump")
        self.model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)

    def teleport(self):
        model_state = ModelState()
        with open("setpoints_w3_with_z.txt") as file:
            lines = file.readlines()
            for line in lines:
                if rospy.is_shutdown():
                    sys.exit(1)
                data = line.split(",")
                model_state = ModelState()
                x, y, z = float(data[0]), float(data[1]), float(data[2])

                model_state.model_name = 'iris'
                model_state.pose.position.x = x
                model_state.pose.position.y = y
                model_state.pose.position.z = z + 18
                self.model_state_pub.publish(model_state)
                rospy.loginfo("Teleporting iris to (" + str(x) + ", " +  str(y) + ", " + str(z) + ")")

                model_state.model_name = 'prius'
                model_state.pose.position.x = x
                model_state.pose.position.y = y
                model_state.pose.position.z = z
                model_state.pose.orientation.x = 0.0183965500998
                model_state.pose.orientation.y = -0.0148245630069
                model_state.pose.orientation.z = 0.714295990227
                model_state.pose.orientation.w = 0.699444806702
                self.model_state_pub.publish(model_state)
                rospy.loginfo("Teleporting prius to (" + str(x) + ", " +  str(y) + ", " + str(z) + ")")

                time.sleep(2)


if __name__ == "__main__":
    node = SummerMapping()
    node.teleport()

