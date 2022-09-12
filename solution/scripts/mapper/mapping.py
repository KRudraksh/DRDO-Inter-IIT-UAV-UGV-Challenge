#!/usr/bin/env python3
import numpy as np
# from ardu_c import FLIGHT_CONTROLLER, eular_quad, quad_eular
from mapper.camera import Camera_output
from mapper.road import Road
import math
import time
import rospy
import matplotlib.pyplot as plt



class Mapping():
    def __init__(self) -> None:
        np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning)
        # self.iris = FLIGHT_CONTROLLER()

        self.road = Road(554.254691191187, 554.254691191187)
        self.height_th = 19.5
        self.x_th = 1.5
        self.x_factor = -2
        self.yaw_factor = 0.6
        self.forward_const = -10
        self.n_points = 10            #no of points to be compaired from s to append new points
        self.pt_dist_min = 2
        self.pt_dist_max = 8

        self.s0 = np.array([])
        self.s1 = np.array([])
        self.waypt_global = None                                 #waypoint wrt origin drone

        self.mid_points = []



    def start_camera(self):
        self.cam.get_rgb_image
        self.cam.get_depth_image
        self.cam.depth_camera_info



    def road_data(self):
        self.road.cal( debug = False)
        self.sense = self.road.curr_sense
        self.curr_s0 = self.road.curr_s0
        self.curr_s1 = self.road.curr_s1
        self.height = self.road.curr_h_wrtroad
        self.x_offset = self.road.curr_x_road
        self.dyaw = self.road.d_yaw_drone

    # def Rx(self, t):
    #     self.Rxm = [[1, 0, 0, 0],
    #                 [0, math.cos(t), math.sin(t), 0],
    #                 [0, -1*math.sin(t), math.cos(t), 0],
    #                 [0, 0, 0, 1]]


    # def Ry(self, t):

    #     self.Rym = [[math.cos(t), 0, -1*math.sin(t), 0],
    #                 [0, 1, 0, 0],
    #                 [math.sin(t), 0, math.cos(t), 0],
    #                 [0, 0, 0, 1]]

    # def Rz(self, t):
    #     self.Rzm = [[math.cos(t), -1*math.sin(t), 0, 0],
    #                 [math.sin(t), math.cos(t), 0, 0],
    #                 [0, 0, 1, 0],
    #                 [0, 0, 0, 1]]


    # def transformation(self, mat):
    #     txyz = mat.transform.translation
    #     tq = mat.transform.rotation
    #     roll, pitch, yaw = quad_eular(tq.x, tq.y, tq.z, tq.w)
    #     self.Rx(roll)
    #     self.Ry(pitch)
    #     self.Rz(yaw)
    #     self.T=[[1, 0, 0, 0],
    #             [0, 1, 0, 0],
    #             [0, 0, 1, 0],
    #             [txyz.x, txyz.y, txyz.z, 1]]

    #     temp = np.matmul(self.Rxm, self.Rym)
    #     temp = np.matmul(temp, self.Rzm)
    #     self.tm = np.matmul(temp, self.T)



    def cnvt_global(self):
        self.curr_s0_local = self.curr_s0.copy()
        self.curr_s1_local = self.curr_s1.copy()
        if len(self.curr_s0) > 0:
            temp = self.road.cam.get_global(self.curr_s0[:, 0], self.curr_s0[:, 1], self.curr_s0[:, 2])
            self.curr_s0 = np.array([temp[0], temp[1], temp[2]]).T

        if len(self.curr_s1) > 0:
            temp = self.road.cam.get_global(self.curr_s1[:, 0], self.curr_s1[:, 1], self.curr_s1[:, 2])
            self.curr_s1 = np.array([temp[0], temp[1], temp[2]]).T


    def map(self):
        if len(self.curr_s0) > 0:
            if len(self.s0) == 0:
                self.s0 = self.curr_s0.copy()                   #initialising the map

            elif len(self.s0) > 0 and len(self.s0) < self.n_points:                 #map doesn't have enough point
                for i in range(0, len(self.curr_s0)):
                    temp = np.sum(np.square(self.s0.T - np.reshape(self.curr_s0[i, :], (3,1))), axis = 0).min()
                    if temp > self.pt_dist_min and temp < self.pt_dist_max:
                        self.s0 = np.concatenate((self.s0, np.reshape(self.curr_s0[i, :], (1, 3))), axis = 0)

            elif len(self.s0) > self.n_points and len(self.curr_s0) > 1:                #map has enough points  ###have to optimize
                for i in range(0, len(self.curr_s0)):
                    temp = np.sum(np.square(self.s0.T - np.reshape(self.curr_s0[i, :], (3,1))), axis = 0).min()
                    if temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s0 = np.concatenate((self.s0, np.reshape(self.curr_s0[i, :], (1, 3))), axis = 0)

            if len(self.s1) > 0 and len(self.s1) < self.n_points:
                for i in range(0, len(self.curr_s0)):
                    temp = np.sum(np.square(self.s1.T - np.reshape(self.curr_s0[i, :], (3,1))), axis = 0).min()
                    if temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s1 = np.concatenate((self.s1, np.reshape(self.curr_s0[i, :], (1, 3))) , axis = 0)

            elif len(self.s1) > self.n_points and len(self.curr_s0) > 1:
                for i in range(0, len(self.curr_s0)):
                    temp = np.sum(np.square(self.s1.T - np.reshape(self.curr_s0[i, :], (3,1))), axis = 0).min()
                    if temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s1 = np.concatenate((self.s1, np.reshape(self.curr_s0[i, :], (1,3))), axis = 0)

        if len(self.curr_s1) > 0:

            if len(self.s1) == 0:
                self.s1 = self.curr_s1.copy()

            elif len(self.s1) > 0 and len(self.s1) < self.n_points:
                for i in range(0, len(self.curr_s1)):
                    temp = np.sum(np.square(self.s1.T - np.reshape(self.curr_s1[i, :], (3,1))), axis = 0).min()
                    if temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s1 = np.concatenate((self.s1, np.reshape(self.curr_s1[i, :], (1,3))), axis = 0)

            elif len(self.s1) > self.n_points:
                for i in range(0, len(self.curr_s1)):
                    temp = np.sum(np.square(self.s1.T - np.reshape(self.curr_s1[i, :], (3,1))), axis = 0).min()
                    if  temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s1 = np.concatenate((self.s1, np.reshape(self.curr_s1[i, :], (1,3))), axis = 0)


            if len(self.s0) > 0 and len(self.s0) < self.n_points:                 #map doesn't have enough point
                for i in range(0, len(self.curr_s1)):
                    temp = np.sum(np.square(self.s0.T - np.reshape(self.curr_s1[i, :], (3,1))), axis = 0).min()
                    if temp > self.pt_dist_min and temp < self.pt_dist_max:
                        self.s0 = np.concatenate((self.s0, np.reshape(self.curr_s1[i, :], (1, 3))), axis = 0)

            elif len(self.s0) > self.n_points and len(self.curr_s1) > 1:                #map has enough points  ###have to optimize
                for i in range(0, len(self.curr_s1)):
                    temp = np.sum(np.square(self.s0.T - np.reshape(self.curr_s1[i, :], (3,1))), axis = 0).min()
                    if temp < self.pt_dist_max and temp > self.pt_dist_min:
                        self.s0 = np.concatenate((self.s0, np.reshape(self.curr_s1[i, :], (1, 3))), axis = 0)

        # print(self.s0)
        # print(self.s1)


    def map_v2(self):
        if len(self.road.waypt_global) > 0:
            if len(self.mid_points) > 0:
                temp = np.array(self.mid_points[-1])
                if (np.dot(self.road.waypt_global, temp) > 0):
                    self.mid_points.append(self.road.waypt_global)

    def main(self):
        self.road_data()
        self.cnvt_global()
        self.map()
        self.waypt_global = self.road.waypt_global        #####
        # if self.road.p0_found:
        #     print(self.road.p0)
        #     print(self.road.cam.get_global(self.road.p0[0], self.road.p0[1], self.road.p0[2]))
        # if self.road.p1_found:
        #     print(self.road.p1)
        #     print(self.road.cam.get_global(self.road.p1[0], self.road.p1[1], self.road.p1[2]))
        # print(self.road.curr_h_wrtroad, self.road.curr_sense, self.road.curr_x_road, self.road.d_yaw_drone)
        self.return_stuff = [self.waypt_global,self.road.d_yaw_drone]
        # print(self.iris.curr_yaw)
        # print('waypt'+str(self.road.waypt))
        # print('waypt global' + str(self.road.waypt_global))
        # if self.road.road_not_found:
        #     np.save('road_0.npy', np.array(self.mid_points))
        '''if self.road.curr_h_wrtroad > self.height_th:
            self.dz = self.height_th - self.height - 0.5
        else:
            self.dz = self.height_th - self.road.curr_h_wrtroad  - 0.5
            if self.dz < 0:
                self.dz = 0

        if self.x_offset > self.x_th or self.x_offset < -1*self.x_th:
            self.dy = self.x_offset*self.x_factor
        else:
            self.dy = 0

        if self.road.d_yaw_drone > 10 or self.road.d_yaw_drone < -10:
            self.dyaw_out = self.dyaw*self.yaw_factor

            self.ox, self.oy, self.oz, self.ow = eular_quad(0,0,self.iris.curr_yaw + self.dyaw_out)

        else:
            self.ox, self.oy, self.oz, self.ow = eular_quad(0,0,self.iris.curr_yaw)

        rotation_matrix = np.array([[math.cos(self.iris.curr_yaw),math.sin(self.iris.curr_yaw)],
                                    [-math.sin(self.iris.curr_yaw),math.cos(self.iris.curr_yaw)]])

        y, x = -1*self.forward_const, self.dy
        x_out, y_out = np.matmul(rotation_matrix, np.array([x, y]))

        # self.x, self.y, self.z = self.iris.pt.x + x_out, self.iris.pt.y + y_out, self.iris.pt.z +self.dz
        self.x, self.y, self.z = self.road.waypt_global[0], self.road.waypt_global[1], self.iris.pt.z +self.dz
        print(self.dz)

        print(self.road.curr_h_wrtroad, self.road.curr_sense, self.road.curr_x_road, self.road.d_yaw_drone)
        print(self.iris.curr_yaw)
        print('waypt'+str(self.road.waypt))
        print('waypt global' + str(self.road.waypt_global))

        self.iris.set_orientation(self.ox, self.oy, self.oz, self.ow)
        self.iris.gotopose(self.x, self.y, self.z)'''






        # if len(self.curr_s1) > 1:
        #     # print(self.curr_s1)
        #     # np.save('s11.npy', self.curr_s1)
        #     a = np.ones((len(self.curr_s1), 1))
        #     self.curr_s1 = np.append(self.curr_s1, a, axis = -1)
        #     self.curr_s1 = np.matmul(self.curr_s1, self.tm)[:, 0:3]
        #     # print(self.curr_s1)

        # if len(self.curr_s0):
        #     # print(self.curr_s0)
        #     # np.save('s01.npy', self.curr_s0)
        #     a = np.ones((len(self.curr_s0), 1))
        #     self.curr_s0 = np.append(self.curr_s0, a, axis = -1)
        #     self.curr_s0 = np.matmul(self.curr_s0, self.tm)[:, 0:3]
        #     # print(self.curr_s0)
        # print(self.s0.shape)
        # print(self.s1.shape)
        # plt.imshow(self.road.road_b)
        # plt.show()









# if __name__ == '__main__':
#     try:
#         mav = FLIGHT_CONTROLLER()
#         time.sleep(5)
#         map = Mapping()
#     except:
#         pass
#     time.sleep(5)


#     r = rospy.Rate(10)
#     # map.iris.set_Guided_mode()
#     # map.iris.set_Altitude_Hold_mode()
#     # map.iris.takeoff(10)
#     # time.sleep(2)
#     # for i in range(0, 100):
#     #     map.iris.gotopose(map.iris.pt.x, map.iris.pt.y, 20, curr_orientation= True)
#     # map.iris.set_Guided_mode()
#     mav.d_yaw(0)
#     # if (mav.within_rad() < 500):
#     mav.set_mode('STABILIZE')
#     mav.toggle_arm(1)
#     time.sleep(3)
#     mav.set_Guided_mode()
#     mav.takeoff(19) #cannot takeoff using gotopose
#     print("take-off successfull")
#     time.sleep(2)
#     while True:
#         map.main()
#         mav.d_yaw(0)
#         print(map.return_stuff)
#         # if (map.return_stuff[0] == np.nan):
#         #     break
#         map.return_stuff[0] =  np.nan_to_num(map.return_stuff[0], nan = 0)
#         mav.gotopose_yaw_1(map.return_stuff[0][0], map.return_stuff[0][1] ,map.return_stuff[0][2] + 19, mav.curr_yaw + map.return_stuff[1])
#         r.sleep()






