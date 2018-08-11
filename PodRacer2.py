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
        # Store the turn by turn values into respective class properties
        self.x = xpose # X position of pod
        self.y = ypose # Y position of pod
        self.x_vel = x_v # Velocity in x direction
        self.y_vel = y_v # Velocity in y direction
        self.ang = ang # Global angle of pod
        self.cp_id = next_check_id # Index of current checkpoint

    def cp_params(self):
        # Store the current checkpoint coordinates by indexing
        # into the list of every checkpoint
        self.nextcpx,self.nextcpy = chk_pts[self.cp_id]

    def count_turn(self):
        # Count what turn it is
        self.turn_num += 1

    def angles(self):
        pi = math.pi
        po2 = math.pi/2
        tpo2 = (3*math.pi)/2
        tpi = 2*math.pi
        # Calculate the unit vectors of euclidian distance
        self.xerror = self.nextcpx - self.x
        self.yerror = self.nextcpy - self.y

        # Calculate euclidian distance and velocity
        self.dist = sqrt(self.xerror**2 + self.yerror**2)
        self.velocity = sqrt(self.x_vel**2 + self.y_vel**2)

                    ### Global Euclidian Distance Vector ###
        if self.xerror < 0 and self.yerror == 0:
            self.cp_ang = pi
        elif self.xerror > 0 and self.yerror == 0:
            self.cp_ang = 0
        elif self.xerror == 0 and self.yerror > 0:
            self.cp_ang = po2
        elif self.xerror == 0 and self.yerror < 0:
            self.cp_ang = tpo2
        elif self.xerror > 0 and self.yerror < 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.cp_ang = tpi - theta
        elif self.xerror > 0 and self.yerror > 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.cp_ang = theta
        elif self.xerror < 0 and self.yerror < 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.cp_ang = pi + theta
        elif self.xerror < 0 and self.yerror > 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.cp_ang = pi - theta
        else:
            print("You missed a condition in the Euclidian Distance, Pod.Angles()",file=sys.stderr)

                        ###   GLobal Velocity Angle   ###
        if self.x_vel < 0 and self.y_vel == 0:
            self.vel_ang = pi
        elif self.x_vel > 0 and self.y_vel == 0:
            self.vel_ang = 0
        elif self.x_vel == 0 and self.y_vel > 0:
            self.vel_ang = po2
        elif self.x_vel == 0 and self.y_vel < 0:
            self.vel_ang = tpo2
        elif self.x_vel > 0 and self.y_vel < 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = tpi - theta
        elif self.x_vel > 0 and self.y_vel > 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = theta
        elif self.x_vel < 0 and self.y_vel < 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = pi + theta
        elif self.x_vel < 0 and self.y_vel > 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = pi - theta
        else:
            print("You missed a condition in the Global Velocity, Pod.Angles()",file=sys.stderr)

        # Print the result of angles()
        print("vel_angle={},cp_ang={},ang={},x_vel={},y_vel={},xer={},yer={}".format(math.degrees(self.vel_ang),math.degrees(self.cp_ang),self.ang,self.x_vel,self.y_vel,self.xerror,self.yerror) ,file=sys.stderr)

    def adjust_cp(self):
        l = len(chk_pts)-1
        if (self.dist < 1500) and (abs(self.vel_ang) < 25) and (self.velocity > 225):
            if self.cp_id == l:
                self.prep(self.x,self.y,self.x_vel,self.y_vel,self.ang,0)
                self.cp_params()
                self.angles()
            else:
                self.cp_id += 1
                self.prep(self.x,self.y,self.x_vel,self.y_vel,self.ang,self.cp_id)
                self.cp_params()
                self.angles()
    def control(self):
        pi = math.pi
        po2 = math.pi/2
        tpo2 = (3*math.pi)/2
        tpi = 2*math.pi
        eightn = math.radians(18)
        # if theta_v - theta_cp > 0 rotate left
        # if theta_v - theta_cp < 0 rotate right
        self.delta_theta = (self.vel_ang - self.cp_ang)
        if self.delta_theta > pi:
            ang = -(tpi-self.delta_theta)
        elif self.delta_theta < -pi:
            ang = (tpi + self.delta_theta)
        else:
            ang = self.delta_theta

        if ang > eightn:
            ang = eightn
        elif ang < -eightn:
            ang = -eightn

        if self.turn_num > 2:
            # We need the unit vector of v to be multiplied by 400 that way the target x and y still provide ample room for
            # acceleration if the velocity is below
            e_x = self.xerror / self.dist
            e_y = self.yerror / self.dist
            Re_x = e_x #4000 * e_x
            Re_y = e_y #4000 * e_y

            x_rot = Re_x*math.cos(ang) + Re_y*math.sin(ang)
            y_rot = -Re_x*math.sin(ang) + Re_y*math.cos(ang)
            target_x = math.ceil(self.x + (4000*x_rot))
            target_y = math.ceil(self.y + (4000*y_rot))

        else:
            target_x = self.nextcpx
            target_y = self.nextcpy
            x_rot = 0
            y_rot = 0


        print("delta_theta={},x_rot={},y_rot={},target_x={},target_y={},ang={}\ndist={}".format(self.delta_theta,x_rot,y_rot,target_x,target_y,math.degrees(ang),self.dist) ,file=sys.stderr)
        return target_x,target_y

    def thrust(self):
        angle = math.degrees(abs(self.delta_theta))
        thrust = (.002623*(angle-180)**2+30)
        if thrust > 100:
            thrust = 100
        elif thrust < 0:
            thrust = 0

        # Specify a distance to start decelerating
        dist = 3500
        t1 = .7 # Tuning parameter for distance contribution to thrust
        alpha = (35 - 100) / (-(dist**2))
        if self.dist < dist:
            thrust_dist = (-alpha)*(self.dist-dist)**2 + 100
            thrust_dist = (t1*thrust_dist)/100
        else:
            thrust_dist = 1.6

        thrust = thrust * thrust_dist

        if thrust > 100:
            thrust = 100
        elif thrust < 0:
            thrust = 0
        #Encorporate this into the function
        if self.velocity < 10:
            thrust = 100

        self.t = str(math.ceil(thrust))
        print("thrust = {}".format(thrust),file=sys.stderr)
        #return thrust

    def boost(self):
        if self.turn_num < 2:
            self.t = "BOOST"



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
    pod1.cp_params()
    pod1.angles()
    pod1.adjust_cp()
    x,y = pod1.control()
    pod1.thrust()
    pod1.boost()
    pod1.count_turn()
    print(str(x),str(y),pod1.t)




    pod2.prep(x2,y2,vx2,vy2,angle2,next_check_point_id2)
    pod2.cp_params()
    pod2.angles()
    pod2.adjust_cp()
    x2,y2 = pod2.control()
    pod2.thrust()
    pod2.count_turn()
    print(str(x2),str(y2),pod2.t)
