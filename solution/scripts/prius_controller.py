#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
import cubic_spline_planner
from geometry_msgs.msg import Point, Quaternion
from prius_msgs.msg import Control
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
inf = 1000000000000
rospy.init_node('PRIUS_CONTROLLER',anonymous = True)

class CONTROLLER:
    def __init__(self, x_waypoints, y_waypoints):
        self.rate = rospy.Rate(20)
        self.k_e = 0.5 # STANLEY GAIN
        self.k_s = 0.1 # SOFTNESS CONSTANT
        self.k_p = 1 # PROPORTIONAL GAIN
        self.max_steer = 0.6458
        self.l_f = 1.41 # LENGTH OF FRONT AXLE FROM THE CENTRE OF THE CHASSIS
        self.max_speed = 65 # KM/H
        self.target_speed = self.max_speed / 3.6 # M/S
        self.min_speed = 15 / 3.6
        self.a = 1
        self.x_waypoints = x_waypoints
        self.y_waypoints = y_waypoints
        self.get_trajectory()
        self.v_to_be_followed = []
        self.last_traj_index = len(self.x_trajectory) - 1
        self.prius_position = Point()
        self.prius_orientation = Quaternion()
        self.get_velocity_array()
        self.control_publisher = rospy.Publisher('/prius', Control, queue_size = 10)
        #*****************************************************************************************************
        rospy.Subscriber('/gazebo/model_states',ModelStates, self.get_pose) # integrate with feedback subsystem
        #****************************************************************************************************
        time.sleep(1)
    
    def get_trajectory(self):
        self.x_trajectory, self.y_trajectory, self.yaw_trajectory, _, _ = cubic_spline_planner.calc_spline_course(self.x_waypoints, self.y_waypoints, ds = 0.1)

    def get_roc(self,x1, y1, x2, y2, x3, y3) :
        try :
            x12 = x1 - x2
            x13 = x1 - x3
            y12 = y1 - y2
            y13 = y1 - y3
            y31 = y3 - y1
            y21 = y2 - y1
            x31 = x3 - x1
            x21 = x2 - x1
            # x1^2 - x3^2
            sx13 = pow(x1, 2) - pow(x3, 2)
            # y1^2 - y3^2
            sy13 = pow(y1, 2) - pow(y3, 2)
            sx21 = pow(x2, 2) - pow(x1, 2)
            sy21 = pow(y2, 2) - pow(y1, 2)
            f = (((sx13) * (x12) + (sy13) *
                  (x12) + (sx21) * (x13) +
                  (sy21) * (x13)) // (2 *
                  ((y31) * (x12) - (y21) * (x13))))
            g = (((sx13) * (y12) + (sy13) * (y12) +
                  (sx21) * (y13) + (sy21) * (y13)) //
                  (2 * ((x31) * (y12) - (x21) * (y13))))
            c = (-pow(x1, 2) - pow(y1, 2) -
                  2 * g * x1 - 2 * f * y1)
            h = -g
            k = -f
            sqr_of_r = abs(h * h + k * k - c)
            r = round(math.sqrt(sqr_of_r), 5)
            return r
        except ZeroDivisionError:
            return inf
    
    def get_velocity_array(self):
        for i in range(len(self.x_trajectory)-5):
            rochere = self.get_roc(self.x_trajectory[i],self.y_trajectory[i],self.x_trajectory[i+4],self.y_trajectory[i+4],self.x_trajectory[i-4],self.y_trajectory[i-4])
            if rochere < 15:
                self.v_to_be_followed.append(math.sqrt(self.a * float(rochere)**(float(rochere)-12)))
            else:
                self.v_to_be_followed.append(math.sqrt(self.a * float(rochere)**1.1))
        self.v_to_be_followed = np.clip(self.v_to_be_followed, self.min_speed, self.target_speed)

#***************************************************************************************************
    def get_pose(self,data): # integrate with feedback subsystem
        for i in range (len(data.name)):
            if data.name[i] == 'prius':
                idx = i
        self.prius_position.x = data.pose[idx].position.x
        self.prius_position.y = data.pose[idx].position.y     
        self.prius_position.z = data.pose[idx].position.z
        self.prius_orientation.x = data.pose[idx].orientation.x
        self.prius_orientation.y = data.pose[idx].orientation.y
        self.prius_orientation.z = data.pose[idx].orientation.z
        self.prius_orientation.w = data.pose[idx].orientation.w
        self.v_x = data.twist[idx].linear.x
        self.v_y = data.twist[idx].linear.y
        self.v_z = data.twist[idx].linear.z
        self.velocity = math.sqrt(self.v_x**2 + self.v_y**2 + self.v_z**2)
        self.yaw = self.get_yaw()
        self.x_front_axle = self.prius_position.x + self.l_f * math.cos(self.yaw)
        self.y_front_axle = self.prius_position.y + self.l_f * math.sin(self.yaw)
#**************************************************************************************

    def normalize(self,angle):
        if angle > math.pi:
            angle -= 2*math.pi
        if angle < -math.pi:
            angle += 2*math.pi
        return angle
    
    def get_yaw(self):
        quaternion = (self.prius_orientation.x , self.prius_orientation.y, self.prius_orientation.z,self.prius_orientation.w)
        euler_angles = euler_from_quaternion(quaternion)
        yaw = euler_angles[2]
        return self.normalize(yaw)
    
    def control_publish(self, throttle, brake, steer, gear):
        ctrl = Control()
        ctrl.header.frame_id = 'world'
        ctrl.header.stamp = rospy.Time.now()
        ctrl.throttle = throttle
        ctrl.brake = brake
        ctrl.steer = steer
        if gear == 'NEUTRAL':
            msg = 1
        elif gear == 'FORWARD':
            msg = 2
        elif gear == 'REVERSE':
            msg = 3 
        else:
            msg = 0
        ctrl.shift_gears = msg
        self.control_publisher.publish(ctrl)
        self.rate.sleep()

    def slow_down(self, target_velocity):
        while abs(self.velocity - target_velocity) > 0.01:
            self.control_publish(0,1,0,'NEUTRAL')
    
    def get_target_index(self):    
        x_distances = [self.x_front_axle - x_trajectory_points for x_trajectory_points in self.x_trajectory]
        y_distances = [self.y_front_axle - y_trajectory_points for y_trajectory_points in self.y_trajectory]
        distances = np.hypot(x_distances, y_distances)
        target_index = np.argmin(distances)
        front_axle_vector = [-np.cos(self.yaw + math.pi / 2), -np.sin(self.yaw) + math.pi / 2]
        cross_track_error = np.dot([x_distances[target_index], y_distances[target_index]], front_axle_vector)
        return target_index, cross_track_error        

    def steer_control(self, last_target_index): 
        current_target_index, cross_track_error = self.get_target_index()
        if last_target_index >= current_target_index :
            current_target_index = last_target_index
        theta_heading_error = self.normalize(self.yaw_trajectory[current_target_index] - self.yaw)
        theta_cross_track = np.arctan2(self.k_e * cross_track_error , (self.k_s + self.velocity))
        if self.velocity < 13:
            steer_out = 3*theta_heading_error + 4*theta_cross_track
        else :
            steer_out = theta_heading_error + 3*theta_cross_track
        return 2*steer_out, current_target_index
    
    def velocity_control(self, target_speed): 
        return self.k_p * (target_speed - self.velocity)

    def start(self):
        target_index, _ = self.get_target_index()
        while (self.last_traj_index - target_index > 3) :
            a_i = self.velocity_control(self.v_to_be_followed[target_index]) 
            steer_out, target_index = self.steer_control(target_index)
            steer_input = steer_out / self.max_steer
            if self.velocity <= self.target_speed :
                self.control_publish(a_i, 0, steer_input,'FORWARD')
            else :
                self.control_publish(-1.2*a_i, 0, steer_input, 'REVERSE')
        self.slow_down(0)

def openfile(ins):
    file = open(ins, "r")
    lines = file.readlines()
    X = []
    Y = []
    for line in lines:
        line = line.split(',')
        x = float(line[0])
        y = float(line[1])
        X.append(x)
        Y.append(y)
    file.close()
    return X,Y


if __name__ == '__main__':
    x_waypoints,y_waypoints = openfile("setpoints_w2.txt")
    control = CONTROLLER(x_waypoints, y_waypoints)
    time.sleep(0.2)
    control.start()