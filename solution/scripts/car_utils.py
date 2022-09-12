#!/usr/bin/env python
from cProfile import label
import rospy
import math
import numpy as np
import time
import cubic_spline_planner
from geometry_msgs.msg import Point, Quaternion
from prius_msgs.msg import Control
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
rospy.init_node('PRIUS_CONTROLLER',anonymous = True)

class CONTROLLER:
    def __init__(self, x_waypoints, y_waypoints):
        self.rate = rospy.Rate(20)
        self.k_e = 0.16 # STANLEY GAIN
        self.k_s = 0.2 # SOFTNESS CONSTANT
        self.k_p = 1 # PROPORTIONAL GAIN

        # Speed Control Parameters
        self.v_avg = 15
        self.a = self.v_avg
        self.b = 1
        self.k = 0.08
        self.double_derivative_threshold = 100
        self.min_velocity = 10
        self.max_velocity = 37.99

        self.max_steer = 0.6458
        self.l_f = 1.41 # LENGTH OF FRONT AXLE FROM THE CENTRE OF THE CHASSIS
        self.max_speed = 15 # KM/H
        self.target_speed = self.max_speed / 3.6 # M/S
        self.x_waypoints = x_waypoints
        self.y_waypoints = y_waypoints
        self.get_trajectory()
        self.x_followed = []
        self.y_followed = []
        self.last_traj_index = len(self.x_trajectory) - 1
        self.prius_position = Point()
        self.prius_orientation = Quaternion()
        self.derivative_computation()
        self.compute_desired_speed_array()
        self.control_publisher = rospy.Publisher('/prius', Control, queue_size = 10)
        rospy.Subscriber('/gazebo/model_states',ModelStates, self.get_pose)
        time.sleep(1)
    
    def get_trajectory(self):
        self.x_trajectory, self.y_trajectory, self.yaw_trajectory, _, _ = cubic_spline_planner.calc_spline_course(self.x_waypoints, self.y_waypoints, ds = 0.1)
    

    def get_pose(self,data):
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
    
    def normalize(self,angle):
        if angle > math.pi:
            angle -= 2*math.pi
        if angle < -math.pi:
            angle += 2*math.pi
        return angle
 
    def get_yaw(self):
        x,y,z,w = self.prius_orientation.x , self.prius_orientation.y, self.prius_orientation.z,self.prius_orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return self.normalize(yaw_z)
    
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

    def derivative_computation(self):
        self.x_dot = np.gradient(np.array(self.x_trajectory))
        self.y_dot = np.gradient(np.array(self.y_trajectory))
        self.y_prime_x = self.y_dot / self.x_dot
        self.y_prime_x_dot = np.gradient(self.y_prime_x)
        self.y_prime_prime_x = self.y_prime_x_dot / self.x_dot

    def compute_desired_speed_array(self):
        self.desired_speed = abs(self.a / (self.b + self.k * (self.y_prime_prime_x - self.double_derivative_threshold)))
        self.desired_speed = np.clip(self.desired_speed, self.min_velocity, self.max_velocity)
        
    def steer_control(self, last_target_index): 
        current_target_index, cross_track_error = self.get_target_index()
        if last_target_index >= current_target_index :
            current_target_index = last_target_index
        theta_heading_error = self.normalize(self.yaw_trajectory[current_target_index] - self.yaw)
        theta_cross_track = np.arctan2(self.k_e * cross_track_error , (self.k_s + self.velocity))
        steer_out = theta_heading_error + theta_cross_track
        return steer_out, current_target_index
    
    def velocity_control(self, target_speed): 
        return self.k_p * (target_speed - self.velocity)

    def start(self):
        target_index, _ = self.get_target_index()
        while (self.last_traj_index - target_index > 3) :
            self.x_followed.append(self.prius_position.x)
            self.y_followed.append(self.prius_position.y)
            steer_out, target_index = self.steer_control(target_index)
            a_i = self.velocity_control(self.desired_speed[target_index]) 
            steer_input = steer_out / self.max_steer
            if self.velocity <= self.target_speed :
                self.control_publish(a_i, 0, steer_input,'FORWARD')
            else :
                self.control_publish(-a_i, 0, steer_input, 'REVERSE')
        self.slow_down(0)
        self.plot()
    
               
    def plot(self):
        plt.figure()
        plt.subplots(1)
        plt.plot(self.x_trajectory, self.y_trajectory, color = 'red', label='trajectory')
        plt.legend()

        plt.subplots(1)
        plt.plot(self.x_trajectory, self.y_prime_x, color = 'blue', label="first derivative")
        plt.legend()

        plt.subplots(1)
        plt.plot(self.x_trajectory, self.y_prime_prime_x, color = 'green', label = 'second derivative')
        plt.legend()

        plt.subplots(1)
        plt.plot(self.x_followed, self.y_followed, color = 'red', label = 'tracked trajectory')
        plt.plot(self.x_trajectory, self.y_trajectory, color = 'blue', label='desired trajectory')
        plt.legend()
        
        plt.show()

def openfile(ins):
    file = open(ins, "r")
    lines = file.readlines()
    X = []
    Y = []
    Z = []
    for line in lines:
        line = line.split(',')
        x = float(line[0])
        y = float(line[1])
        z = float(line[2])
        X.append(x)
        Y.append(y)
        Z.append(z)
    file.close()
    return X,Y,Z


if __name__ == '__main__':
    x_waypoints, y_waypoints, z_waypoints = openfile("setpoints_w3_with_z.txt")
    
    control = CONTROLLER(x_waypoints, y_waypoints)
    # control.plot()
    time.sleep(0.2)
    control.start()
    

    

