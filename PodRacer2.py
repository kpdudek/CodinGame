# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 00:28:38 2018

@author: kpdud
"""
import sys
import math
from math import sqrt
from math import acos


# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.
# Write an action using print
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

    def cp_ang(self):
        self.xerror = self.nextcpx - self.x
        self.yerror = self.nextcpy - self.y

        self.dist = sqrt(self.xerror**2 + self.yerror**2)
        self.velocity = sqrt(self.x_vel**2 + self.y_vel**2)

        dot_product = (self.xerror * self.x_vel) + (self.yerror * self.y_vel)
        magnitudes = self.dist * self.velocity

        self.cp_ang = acos(dot_product/magnitudes)

        
