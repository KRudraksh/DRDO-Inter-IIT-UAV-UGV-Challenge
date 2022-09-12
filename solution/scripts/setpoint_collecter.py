#! /usr/bin/python
import rospy 

from gazebo_msgs.msg import *
import os
output_file = open("setpoints.txt", "w")

def cb_function(msg):
	# print("%s,%s" msg.pose[1].position.x,msg.pose[1].position.y)
	output_file.write("{},{}\n".format(msg.pose[1].position.x,msg.pose[1].position.y))


rospy.init_node('setpoint_extractor')
rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, cb_function)

def main():
	while not rospy.is_shutdown():
		pass
if __name__ == "__main__":
    main()