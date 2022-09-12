#!/usr/bin/env python3
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import time
from sklearn.linear_model import LinearRegression
from numpy import percentile as pt

from mapper.camera import Camera_output

def backbone(kx, ky, Fx, Fy):
    A = np.arange(kx)
    A = A-kx
    x = []
    for i in range (ky):
        x.append(A)
    x = np.array(x)
    x = x + (int(kx/2 ) + 1)
    A = np.arange(ky)
    A = A-ky
    y = []
    for i in range (kx):
        y.append(A)
    y = np.array(y)*(-1)
    y = y.T
    y = y - int(ky/2) -1
    x = x/Fx
    y = y/Fy

    return x, y




###Useful outputs
#curr_h_wrtroad
#curr_x_road        By how much road is not in center of frame, +ve right side, -ve left side
#curr_sense         where is rode wrt drone 0: center, 1: right, -1: left
#curr_s0         wrt camera frame (edge of road)
#curr_s1
#d_yaw_drone


def slope(coff, p):
    return coff[1] + 2*coff[0]*p



class Road:
    def __init__(self, fx, fy, live = True):
        self.live = live
        if live:
            self.cam = Camera_output()
        self.z = None
        self.height, self.width = (480, 640)
        self.k = 35                                        #box size for plane computation
        self.reg = LinearRegression()
        self.Fx = fx                                #camera parameter
        self.Fy = fy
        self.nh = int(self.height/self.k)                            #width and length of sampled space
        self.nw = int(self.width/self.k)
        self.d = 20                             #distance between two points of same side
        self.n_p = 6
        self.x, self.y = backbone(self.k,self.k,self.Fx, self.Fy)
        self.curr_sense = 0     #0: road in center, 1: right side, -1: left side
        self.road_loc_thrs = 1.5             #if avg x of road > thrs then road is on right side of frame
        self.curr_h_wrtroad = 10
        self.curr_x_road = 0
        self.d_yaw_drone = 0
        # self.d_yaw_drone_linear = 0
        self.slope_threshold = 5
        self.x_thrs = 14






        #for debuging
        self.data0 = []
        self.data1 = []
        self.index_left = []




    def depth_sampling(self):
        data = np.array(self.z)
        sample = []
        height, width = data.shape
        nh = int(height/self.k)
        nw = int(width/self.k)
        for i in range(0, nh):
            for j in range(0, nw):
                sample.append(data[self.k*i:self.k*(i+1), self.k*j:self.k*(j+1)])
        return np.array(sample)

    def seg(self):
        sampled_space = self.depth_sampling()

        x = (sampled_space*self.x).copy()
        y = (sampled_space*self.y).copy()
        x_ = np.reshape(x, (x.shape[0], x.shape[2]*x.shape[1]))
        y_ = np.reshape(y, (y.shape[0], y.shape[2]*y.shape[1]))
        sampled_space_ = np.reshape(sampled_space, (sampled_space.shape[0], sampled_space.shape[2]*sampled_space.shape[1]))

        frame_data = []
        for i in range(sampled_space.shape[0]):

            X = np.array([x_[i,:], y_[i,:]]).T
            self.reg.fit(X, sampled_space_[i,:])
            z = self.reg.predict(X)
            avg_error = np.average(np.abs(z - sampled_space_[i, :]))

            r = [self.reg.coef_[0], self.reg.coef_[1], self.reg.intercept_, avg_error, i]
            frame_data.append(r)
        self.curr_frame_data = np.array(frame_data)

        self.curr_frame_p_ = np.array(self.curr_frame_data[:,3] > self.curr_frame_data[:,3].max()*0.01, dtype = int)
        #height, width = self.z.shape
        #self.nh = int(height/self.k)
        #self.nw = int(width/self.k)
        self.t = np.reshape(self.curr_frame_p_, (self.nh, self.nw))
        self.g_box = self.curr_frame_data[self.curr_frame_p_ < 1, :]




    def i_to_xy(self,i):
        x = np.array(i/self.nw, dtype = int)
        y = np.array(i%self.nw, dtype = int)
        return x, y



    def boundary_cal(self):
        self.road_not_found = False
        const = int(0)
        ki = 25 + const
        kf = 75 - const
        while True:
            if kf - ki < 30:
                break
            prt = pt(self.g_box[:, 0], range(ki,kf))
            avg = np.average(prt)
            std = np.std(prt)
            if std > 0.01:
                const = int(std*100)
                ki += const
                kf -= const
            else:
                break

        f = 20
        index0 = self.g_box[self.g_box[:, 0] >  avg + std*f , 4]
        index1 = self.g_box[self.g_box[:, 0] < avg - std*f, 4]


        if kf - ki < 30:
            ki = int(0)
            r = 5
            lstd = []
            lavg = []
            for i in range(0, 17):
                prt = pt(self.g_box[:, 0], range(ki + r*i ,ki + 20 +r*i -1))
                avg = np.average(prt)
                std = np.std(prt)
                lstd.append(std)
                lavg.append(avg)

            i_ = np.argmin(np.array(lstd))
            if lstd[i_] < 0.0005:
                f = 100
            if lstd[i_] > 0.002:
                print('No road found')
                self.road_not_found = True
            index0 = self.g_box[self.g_box[:, 0] >  lavg[i_] + lstd[i_]*f , 4]
            index1 = self.g_box[self.g_box[:, 0] < lavg[i_] - lstd[i_]*f, 4]


        x, y = self.i_to_xy(index0)
        x1, y1 = self.i_to_xy(index1)


        self.t[x, y] = 1
        self.t[x1, y1] = 1


        #self.road_info()



        t_boundary = (self.t*0).copy()
        t_boundary[:, (0, -1)] += self.t[:, (0,-1)]
        t_boundary[(0, -1), :] += self.t[(0,-1), :]
        t_boundary[t_boundary > 1] = 1
        t_boundary = np.array(t_boundary, dtype = 'uint8')

        kernel = np.array([[1,1,1],
                        [1,4,1],
                        [1,1,1]])/8

        t_ = np.array(self.t, dtype = 'uint8').copy()        #Edge of the road
        t_out = cv2.filter2D(src = t_.copy(), ddepth = -1, kernel = kernel)
        t_[t_out < 1] = 0
        t_ += t_boundary
        t_[t_ > 1] = 1

        kernel = np.array([[1,1,1],
                        [1,1,1],
                        [1,1,1]])

        t_ = np.array(t_, dtype = 'uint8').copy()        #Edge of the road
        t_out = cv2.filter2D(src = t_.copy(), ddepth = -1, kernel = kernel)
        t_[t_out == 0] = 1
        self.road_b = t_.copy()

        self.boundary_points = np.argwhere(t_ == 0)

        self.center_bp = self.center_finder_live(self.boundary_points[:,1], self.boundary_points[:,0])

    def road_info(self):
        road_index = np.argwhere(self.t == 0)
        self.mean_xyz_road = np.average(self.center_finder_live(road_index[:, 1], road_index[:, 0]), axis = 1)
        if self.mean_xyz_road[0] > self.road_loc_thrs:
            self.curr_sense = 1
        elif self.mean_xyz_road[0] < -1*self.road_loc_thrs:
            self.curr_sense = -1
        else:
            self.curr_sense = 0

        self.curr_h_wrtroad = self.mean_xyz_road[2]
        self.curr_x_road = self.mean_xyz_road[0]



    def center_finder(self,h_x, w_y):
        x_frame, y_frame = backbone(self.z.shape[1], self.z.shape[0], self.Fx, self.Fy)
        x = x_frame*self.z
        y = y_frame*self.z
        h_ = np.array((h_x + 0.5)*self.k, dtype = int)
        w_ = np.array((w_y + 0.5)*self.k, dtype = int)
        self.curr_frame_x, self.curr_frame_y ,self.curr_frame_z = x[w_, h_], y[w_, h_], self.z[w_, h_]
        return np.array([self.curr_frame_x, self.curr_frame_y ,self.curr_frame_z])


    def center_finder_live(self, h_x, w_y):
        h_ = np.array((h_x + 0.5)*self.k, dtype = int)
        w_ = np.array((w_y + 0.5)*self.k, dtype = int)
        return self.cam.get_xyz(w_, h_)





    def edge_cluster(self, debug = False):
        # if len(self.center_bp) < 1:
            # np.save('error.npy', self.z)
            # print('hmmm error time')
            # time.sleep(2)
        s0 = np.array(self.center_bp[:, -1])
        s0 = np.reshape(s0, (1, s0.shape[0]))
        s1 = np.reshape(np.array([0,0,0]), (1, 3))
        if debug:
            self.data0 = []
            self.data1 = []
            self.index_left = []
        for i in range(1, self.center_bp.shape[1] +1):
            i = -1*i
            if len(s0) > 1:
                if len(s0) < self.n_p -1:
                    d0 = np.sum(np.square(s0[0:len(s0), :] - self.center_bp[:, i]), axis = 1).min()
                    if debug:
                        self.data0.append([d0, i])
                else:
                    d0 = np.sum(np.square(s0[-1*self.n_p:-1, :] - self.center_bp[:, i]), axis = 1).min()
                    if debug:
                        self.data0.append([d0, i])
            else:
                d0 = np.sum(np.square(s0[-1, :] - self.center_bp[:, i]))
                if debug:
                        self.data0.append([d0, i])
                if d0  < self.d:
                    s0 = np.concatenate([s0, np.reshape(self.center_bp[:, i], (1,3))], axis = 0)
                    continue
            if len(s1) > 2:
                if len(s1) < self.n_p:
                    d1 = np.sum(np.square(s1[1:len(s0), :] - self.center_bp[:, i]), axis = 1).min()
                    if debug:
                        self.data1.append([d1, i])
                else:
                    d1 = np.sum(np.square(s1[-1*self.n_p:-1, :] - self.center_bp[:, i]), axis = 1).min()
                    if debug:
                        self.data1.append([d1, i])
            elif len(s1) > 1:
                d1 =  np.sum(np.square(s1[-1, :] - self.center_bp[:, i]))
                if debug:
                        self.data1.append([d1, i])
                if d1 < self.d:
                    s1 = np.concatenate((s1, np.reshape(self.center_bp[:, i], (1,3))), axis = 0)
                    continue

            if len(s0) > 1 and len(s1) > 2:
                if d0 < d1 and d0 < self.d:
                    s0 = np.concatenate([s0, np.reshape(self.center_bp[:, i], (1,3))], axis = 0)
                    continue
                elif d1 < self.d:
                    s1 = np.concatenate((s1, np.reshape(self.center_bp[:, i], (1,3))), axis = 0)
                    continue
            elif len(s0) > 1 and d0 < self.d:
                s0 = np.concatenate([s0, np.reshape(self.center_bp[:, i], (1,3))], axis = 0)
                continue
            elif len(s1) > 2 and d1 < self.d:
                s1 = np.concatenate((s1, np.reshape(self.center_bp[:, i], (1,3))), axis = 0)
                continue

            if len(s1) == 1:
                s1 = np.concatenate((s1, np.reshape(self.center_bp[:, i], (1,3))), axis = 0)
            else:
                if debug:
                        self.index_left.append(int(i))
        self.curr_s0 = s0[1:, :].copy()
        self.curr_s1 = s1[2:, :].copy()


    def cal(self, depth = None, debug = False, no_return = True, vis = False):
        if self.live:
            self.z = self.cam.img_depth.copy()
        else:
            self.z = depth.copy()
        self.seg()
        self.boundary_cal()

        if len(self.center_bp) > 0:
            self.edge_cluster(debug = debug)
            self.cal_dir()
            # self.cal_dir_linear()
            self.boundary_found = True
        else:
            self.boundary_found = False
        self.road_info()
        self.cal_nxt_waypoint()

        if not no_return:
            return self.center_bp, [self.curr_s0, self.curr_s1], [self.road_b]
        r = np.array(self.t).copy()
        r[r==0] = 255
        if vis:
            cv2.imshow('YOLO output', np.array(r, dtype= 'uint8'))
            if cv2.waitKey(25) & 0xFF == ord('q'):

                cv2.destroyAllWindows()


    def cal_dir(self):
        if len(self.curr_s0) > 0:
            self.ts_e0 = True
            coff0 = np.polyfit(self.curr_s0[:, 1], self.curr_s0[:, 0], 2)
            self.slope_e0 = slope(coff0, self.curr_s0[:, 1].max())
            # print('coff0' + str(coff0))
            # print(self.curr_s0[:, 1].max())
        else:
            self.ts_e0 = False
        if len(self.curr_s1) > 0:
            self.ts_e1 = True
            coff1 = np.polyfit(self.curr_s1[:, 1], self.curr_s1[:, 0], 2)
            self.slope_e1 = slope(coff1, self.curr_s1[:, 1].max())
            # print('coff1' + str(coff1))
            # print(self.curr_s1[:, 1].max())
        else:
            self.ts_e1 = False

        self.cal_dyaw()


    def cal_nxt_waypoint(self):
        if len(self.curr_s0) > 0:
            n0 = int(len(self.curr_s0)*0.25)
        else:
            n0 = 0
        if len(self.curr_s1) > 0:
            n1 = int(len(self.curr_s1)*0.25)
        else:
            n1 = 0

        if not n0 == 0 and  not n1 == 0:
            if n0 > n1:
                n = n1
            else:
                n = n0
        elif n0 == 0 or n1 == 0:
            if n0 > n1 :
                n = n0
            elif n1 > n0:
                n = n1
        else:
            print('Unable to find next way point')
            return

        # print(n)
        # if len(self.curr_s0) > 0 and not n == 0:
        #     self.p0_found = True
        #     p0_index = np.argwhere(self.curr_s0[:, 1] == self.curr_s0[:, 1].min())
        #     p0_index = p0_index[0,0]
        #     i0 = int(p0_index - n)
        #     self.p0 = np.array([self.curr_s0[i0:p0_index, 0], self.curr_s0[i0:p0_index, 1], self.curr_s0[i0:p0_index, 2]])
        #     self.p0 = np.average(self.p0, axis= -1)

        #     print('p0' + str(self.p0))
        # else:
        #     self.p0_found = False
        # if len(self.curr_s1) > 0 and not n == 0:
        #     self.p1_found = True
        #     p1_index = np.argwhere(self.curr_s1[:, 1] == self.curr_s1[:, 1].min())
        #     p1_index = p1_index[0,0]
        #     i1 = int(p1_index - n)
        #     self.p1 = [self.curr_s1[i1:p1_index, 0], self.curr_s1[i1:p1_index, 1], self.curr_s1[i1:p1_index, 2]]
        #     self.p1 = np.average(self.p1, axis= -1)
        #     print('p1' + str(self.p1))
        # else:
        #     self.p1_found = False


        if len(self.curr_s0) > 0 and not n == 0:
            self.p0_found = True
            p0_index = np.argwhere(self.curr_s0[:, 1] == self.curr_s0[:, 1].max())
            p0_index = p0_index[0,0]
            i0 = int(p0_index + n)
            self.p0 = np.array([self.curr_s0[p0_index:i0, 0], self.curr_s0[p0_index:i0, 1], self.curr_s0[p0_index:i0, 2]])
            self.p0 = np.average(self.p0, axis= -1)

            print('p0' + str(self.p0))
        else:
            self.p0_found = False
        if len(self.curr_s1) > 0 and not n == 0:
            self.p1_found = True
            p1_index = np.argwhere(self.curr_s1[:, 1] == self.curr_s1[:, 1].max())
            p1_index = p1_index[0,0]
            i1 = int(p1_index + n)
            self.p1 = [self.curr_s1[p1_index:i1, 0], self.curr_s1[p1_index:i1, 1], self.curr_s1[p1_index:i1, 2]]
            self.p1 = np.average(self.p1, axis= -1)
            print('p1' + str(self.p1))
        else:
            self.p1_found = False


        if self.p0_found and self.p1_found:
            if np.sum(np.abs(self.p0[1] - self.p1[1])) < self.x_thrs:
                self.waypt = np.average([self.p0, self.p1], axis = 0)
            elif len(self.curr_s0) > len(self.curr_s1):
                self.waypt = self.p0.copy()
                self.waypt[0] -= self.curr_x_road
            else:
                self.waypt = self.p1.copy()
                self.waypt[0] -= self.curr_x_road


        elif not self.p0_found and self.p1_found:
            # pass            #work with p1
            self.waypt = self.p1.copy()
            self.waypt[0] -= self.curr_x_road

        elif not self.p1_found and self.p0_found:
            # pass                #work with p0
            self.waypt = self.p0.copy()
            self.waypt[0] -= self.curr_x_road

        else:
            print('Unable to compute waypt')


        self.waypt_global = self.cam.get_global(self.waypt[0], self.waypt[1], self.waypt[2])




    # def cal_dir_linear(self):
    #     if len(self.curr_s0) > 0:
    #         self.ts_e0 = True
    #         coff0 = np.polyfit(self.curr_s0[:, 1], self.curr_s0[:, 0], 1)
    #         self.slope_e0 = coff0[0]
    #         # print('coff0' + str(coff0))

    #     else:
    #         self.ts_e0 = False
    #     if len(self.curr_s1) > 0:
    #         self.ts_e1 = True
    #         coff1 = np.polyfit(self.curr_s1[:, 1], self.curr_s1[:, 0], 1)
    #         self.slope_e1 = coff1[0]
    #         # print('coff1' + str(coff1))

    #     else:
    #         self.ts_e1 = False


    #     # print('Liner calculation')
    #     self.cal_dyaw_linear()


    def cal_dyaw(self):
        if self.ts_e0 and self.ts_e1:
            avg_slope = (self.slope_e0 + self.slope_e1)/2
            if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
                self.d_yaw_drone = 0
            else:
                self.d_yaw_drone = np.abs(math.degrees(math.atan(avg_slope)))
        elif self.ts_e0:
            avg_slope = self.slope_e0
            if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
                self.d_yaw_drone = 0
            else:
                self.d_yaw_drone = np.abs(math.degrees(math.atan(avg_slope)))

        elif self.ts_e1:
            avg_slope = self.slope_e1
            if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
                self.d_yaw_drone = 0
            else:
                self.d_yaw_drone = np.abs(math.degrees(math.atan(avg_slope)))


        # print('avg_slope'+str(avg_slope))

        if avg_slope < 0:
            self.d_yaw_drone = -1*self.d_yaw_drone


    # def cal_dyaw_linear(self):
    #     if self.ts_e0 and self.ts_e1:
    #         avg_slope = (self.slope_e0 + self.slope_e1)/2
    #         if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
    #             self.d_yaw_drone_linear = 0
    #         else:
    #             self.d_yaw_drone_linear = np.abs(math.degrees(math.atan(avg_slope)))
    #     elif self.ts_e0:
    #         avg_slope = self.slope_e0
    #         if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
    #             self.d_yaw_drone_linear = 0
    #         else:
    #             self.d_yaw_drone_linear = np.abs(math.degrees(math.atan(avg_slope)))

    #     elif self.ts_e1:
    #         avg_slope = self.slope_e1
    #         if avg_slope > self.slope_threshold or avg_slope < -self.slope_threshold:
    #             self.d_yaw_drone_linear = 0
    #         else:
    #             self.d_yaw_drone_linear = np.abs(math.degrees(math.atan(avg_slope)))


    #     # print('avg_slope'+str(avg_slope))

    #     if avg_slope < 0:
    #         self.d_yaw_drone_linear = -1*self.d_yaw_drone_linear


