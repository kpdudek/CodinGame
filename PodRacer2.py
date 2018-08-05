# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 00:28:38 2018

@author: kpdud
"""
import sys
import math
from math import sqrt
from math import acos

# To debug: print("Debug messages...", file=sys.stderr)

laps = int(input())
num_cp = int(input())
chk_pts = []
for i in range(num_cp):
    cp = ([int(j) for j in input().split()])
    chk_pts.append(cp)

class Pod(object):

    def __init__(self):
        #Parameters given on each turn
        self.x = 0
        self.y = 0
        self.x_vel = 0
        self.y_vel = 0
        self.ang = 0
        self.cp_id = 0

        #Turn Counter
        self.turn_num = 0

        #Checkpoint coordinates
        self.nextcpx = 0
        self.nextcpy = 0

        #Angle between pods velocity and the euclidian distance
        self.xerror = 0
        self.yerror = 0
        self.dist = 0
        self.velocity = 0

        self.cp_ang = 0
        self.vel_ang = 0

    def prep(self,xpose,ypose,x_v,y_v,ang,next_check_id):
        self.x = xpose
        self.y = ypose
        self.x_vel = x_v
        self.y_vel = y_v
        self.ang = ang
        self.cp_id = next_check_id

    def cp_params(self):
        self.nextcpx,self.nextcpy = chk_pts[self.cp_id]

    def count_turn(self):
        self.turn_num += 1

    def angles(self):
        self.xerror = self.nextcpx - self.x
        self.yerror = self.nextcpy - self.y

        self.dist = sqrt(self.xerror**2 + self.yerror**2)
        self.velocity = sqrt(self.x_vel**2 + self.y_vel**2)

        if self.x_vel == 0:
            theta = 0
        else:
            theta = math.degrees(math.atan(self.yerror/self.xerror))

        if self.xerror > 0 and self.yerror < 0:
            self.cp_ang = 360-theta
        elif self.xerror > 0 and self.yerror > 0:
            self.cp_ang = theta
        elif self.xerror < 0 and self.yerror < 0:
            self.cp_ang = 180 + theta
        elif self.xerror < 0 and self.yerror > 0:
            self.cp_ang = 180 - theta
        elif self.xerror < 0 and self.yerror == 0:
            self.cp_ang = 180
        elif self.xerror > 0 and self.yerror == 0:
            self.cp_ang = 0
        elif self.xerror == 0 and self.yerror > 0:
            self.cp_ang = 90
        elif self.xerror == 0 and self.yerror < 0:
            self.cp_ang = 270

        if self.x_vel == 0:
            theta = 0
        else:
            theta = math.degrees(math.atan(self.y_vel/self.x_vel))

        if self.x_vel > 0 and self.y_vel < 0:
            self.vel_ang = 360-theta
        elif self.x_vel > 0 and self.y_vel > 0:
            self.vel_ang = theta
        elif self.x_vel < 0 and self.y_vel < 0:
            self.vel_ang = 180 + theta
        elif self.x_vel < 0 and self.y_vel > 0:
            self.vel_ang = 180 - theta
        elif self.x_vel < 0 and self.y_vel == 0:
            self.vel_ang = 180
        elif self.x_vel > 0 and self.y_vel == 0:
            self.vel_ang = 0
        elif self.x_vel == 0 and self.y_vel > 0:
            self.vel_ang = 90
        elif self.x_vel == 0 and self.y_vel < 0:
            self.vel_ang = 270

    def control(self):
        # if theta_v - theta_cp > 0 rotate left
        # if theta_v - theta_cp < 0 rotate right
        delta_theta = self.vel_ang - self.cp_ang
        self.k1 = .5
        ang = math.ceil(delta_theta * self.k1)
        if ang > 18:
            ang = 18
        elif ang < 0:
            ang = 0

        if delta_theta > 0:
            x_rot = self.x_vel*math.cos(ang) - self.y_vel*math.sin(ang)
            y_rot = self.x_vel*math.sin(ang) + self.y_vel*math.cos(ang)
        elif delta_theta < 0:
            x_rot = self.x_vel*math.cos(-ang) - self.y_vel*math.sin(-ang)
            y_rot = self.x_vel*math.sin(-ang) + self.y_vel*math.cos(-ang)
        else:
            x_rot = 0
            y_rot = 0

        if self.turn_num > 1:
            target_x = math.ceil(self.x + x_rot)
            target_y = math.ceil(self.y + y_rot)
        else:
            target_x = self.x
            target_y = self.y
        print("delta_theta={},x_rot={},y_rot={},target_x={},target_y={}\n".format(delta_theta,x_rot,y_rot,target_x,target_y) ,file=sys.stderr)
        return target_x,target_y




pod1 = Pod()
pod2 = Pod()

# game loop
while True:
    #Sets the turn values of checkpoint postition and current attitude
    x, y, vx, vy, angle, next_check_point_id = [int(j) for j in input().split()]
    x2, y2, vx2, vy2, angle2, next_check_point_id2 = [int(j) for j in input().split()]
    o_x, o_y, o_vx, o_vy, o_angle, o_next_check_point_id = [int(j) for j in input().split()]
    o2_x, o2_y, o2_vx, o2_vy, o2_angle, o2_next_check_point_id = [int(j) for j in input().split()]

    pod1.prep(x,y,vx,vy,angle,next_check_point_id)
    pod1.count_turn()
    pod1.angles()
    x,y = pod1.control()

    #print("<< POD1 >>\ntx1 = {} ty1 = {} nextcpx = {} nextcpy = {} dist = {}\nang = {} vel = {}\n".format(tx1,ty1,pod1.next_cp_x,pod1.next_cp_y,pod1.dist,pod1.cp_ang,pod1.vel) ,file=sys.stderr)
    print(str(x),str(y),str(100))




    pod2.prep(x2,y2,vx2,vy2,angle2,next_check_point_id2)
    pod2.count_turn()
    pod2.angles()
    x2,y2 = pod2.control()
    #print("<< POD2 >>\ntx2 = {} ty2 = {} nextcpx = {} nextcpy = {} dist = {}\nang = {} vel = {}\n".format(tx2,ty2,pod2.next_cp_x,pod2.next_cp_y,pod2.dist,pod2.cp_ang,pod2.vel) ,file=sys.stderr)
    print(str(x2),str(y2),str(100))
