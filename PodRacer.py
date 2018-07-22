# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 00:28:38 2018

@author: kpdud
"""
import sys
import math

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
        #Pod attributes
        self.x = 0
        self.y = 0
        self.x_vel = 0
        self.y_vel = 0
        self.vel = 0
        self.ang = 0
        self.cp_id = 0
        
        #Check Point Distance
        self.xerror = 0
        self.yerror = 0
        self.next_cp_x = 0
        self.next_cp_y = 0
        self.dist = 0
        self.cp_ang = 0
        self.vel_ang = 0
        self.rotate_ang = 0
        self.turn_number = 0
        
        #PD Control
        self.xtun = .2
        self.ytun = .2
        self.ka = .05
        self.kd = .03
        self.t_a = -.55
        self.ktarget = .1
    
        #Boosting
        self.allow_boost = False
        self.boost = False
        self.chkpnt_x = 0
        self.chkpnt_y = 0
            
    #Initializes the pods attributes
    def prep(self,xpose,ypose,x_v,y_v,ang,next_check_id):
        self.x = xpose
        self.y = ypose
        self.x_vel = x_v
        self.y_vel = y_v
        self.ang = ang
        self.cp_id = next_check_id
    
    #Calculates pod check point distances
    def cp_params(self):
        self.next_cp_x,self.next_cp_y = chk_pts[self.cp_id]
        self.xerror = (self.next_cp_x - self.x)
        self.yerror = (self.next_cp_y - self.y)
        self.dist = math.sqrt(((self.xerror)**2) + ((self.yerror)**2))
        self.vel = math.sqrt((self.x_vel**2) + self.y_vel**2)
        
        
    def checkpoint_angle(self):
        if (self.xerror > 0) and (self.yerror > 0):
            theta = abs(math.acos(self.xerror/self.dist))
            self.cp_ang = 180 + theta
            
        elif (self.xerror > 0) and (self.yerror < 0):
            theta = abs(math.acos(self.xerror/self.dist))
            self.cp_ang = 180 - theta
            
        elif (self.xerror < 0) and (self.yerror > 0):
            theta = abs(math.acos(self.xerror/self.dist))
            self.cp_ang = 360 - theta
            
        elif (self.xerror < 0) and (self.yerror < 0):
            theta = abs(math.acos(self.xerror/self.dist))
            self.cp_ang = 0 + theta
            
        elif (self.xerror == 0) and (self.yerror > 0):
            self.cp_ang = 270
            
        elif (self.xerror == 0) and (self.yerror < 0):
            self.cp_ang = 90
            
        elif (self.xerror > 0) and (self.yerror == 0):
            self.cp_ang = 0
            
        elif (self.xerror < 0) and (self.yerror == 0):
            self.cp_ang = 180
            
            
            
    def velocity_angle(self):
        if (self.x_vel > 0) and (self.y_vel > 0):
            theta_vel = abs(math.acos(self.x_vel/self.vel))
            self.vel_ang = 180 + theta_vel
            
        elif (self.x_vel > 0) and (self.y_vel < 0):
            theta_vel = abs(math.acos(self.x_vel/self.vel))
            self.vel_ang = 180 - theta_vel
            
        elif (self.x_vel < 0) and (self.y_vel > 0):
            theta_vel = abs(math.acos(self.x_vel/self.vel))
            self.vel_ang = 360 - theta_vel
            
        elif (self.x_vel < 0) and (self.y_vel < 0):
            theta_vel = abs(math.acos(self.x_vel/self.vel))
            self.vel_ang = 0 + theta_vel
            
        elif (self.x_vel == 0) and (self.y_vel > 0):
            self.vel_ang = 270
            
        elif (self.x_vel == 0) and (self.y_vel < 0):
            self.vel_ang = 90
            
        elif (self.x_vel > 0) and (self.y_vel == 0):
            self.vel_ang = 0
            
        elif(self.x_vel < 0) and (self.y_vel == 0):
            self.vel_ang = 180    
            
        else:
            self.vel_ang = 0

            
              
    def target(self):

        if self.turn_number < 4:
            target_x = self.next_cp_x
            target_y = self.next_cp_y
            
        else:
            self.roate_ang = self.cp_ang - self.vel_ang
            
            if self.rotate_ang > 0:
                if self.rotate_ang > 180:
                    theta = 360 - self.roate_ang
                elif self.rotate_ang < 180:
                    theta = self.roate_ang
                else:# self.rotate_ang == 180:
                    theta = self.rotate_ang
            elif self.rotate_ang < 0:
                if self.rotate_ang > -180:
                    theta = self.rotate_ang
                elif self.rotate_ang < -180:
                    theta = 360 + self.rotate_ang
                else:# self.rotate_ang == -180:
                    theta = self.rotate_ang
            elif self.rotate_ang == 0:
                theta = 0
                
            theta_rotate = theta * self.ktarget
            target_x = (self.x + math.ceil((self.x_vel*math.cos(math.radians(theta_rotate))) - (self.y_vel*math.sin(math.radians(theta_rotate)))))
            target_y = (self.y + math.ceil((self.x_vel*math.sin(math.radians(theta_rotate))) + (self.y_vel*math.cos(math.radians(theta_rotate)))))
                
            if self.vel == 0:
                target_x = self.next_cp_x
                target_y = self.next_cp_y
                
        return math.ceil(target_x*1.4),math.ceil(target_y*1.4)
                

        
    def check_boost(self):
        if int(self.cp_id)  > 2:
            self.allow_boost = True
        return
   

     
    def thrust(self):
        t_d = math.ceil(self.kd*self.dist)
        t = t_d
        flag = 'Accelerating'      
        #Limits
        if t > 100:
            t = 100
            flag = 'MAX'
        elif t < 10:
            t = 10
            flag = 'MIN'           
        #Boost
        if self.dist > 3000 and abs(self.cp_ang) < 20 and self.allow_boost == True and self.boost == False:
            t = 'BOOST'
            flag = 'BOOST'
            self.boost = True
        return t,flag
    
    def turn_counter(self):
        self.turn_number += 1
        return self.turn_number
    
    
    
pod1 = Pod()
pod2 = Pod()
pod1.coun = True
pod1.allow_boost = True
# game loop
while True:
    #Sets the turn values of checkpoint postition and current attitude
    x, y, vx, vy, angle, next_check_point_id = [int(j) for j in input().split()]
    x2, y2, vx2, vy2, angle2, next_check_point_id2 = [int(j) for j in input().split()]        
    o_x, o_y, o_vx, o_vy, o_angle, o_next_check_point_id = [int(j) for j in input().split()]
    o2_x, o2_y, o2_vx, o2_vy, o2_angle, o2_next_check_point_id = [int(j) for j in input().split()]
                    
    pod1.prep(x,y,vx,vy,angle,next_check_point_id)
    num = pod1.turn_counter()
    pod1.cp_params()
    pod1.checkpoint_angle()
    pod1.velocity_angle()
    thrust,flag = pod1.thrust()
    tx1,ty1 = pod1.target()
    print("<< POD1 >>\ntx1 = {} ty1 = {} nextcpx = {} nextcpy = {} dist = {}\nang = {} vel = {}\n".format(tx1,ty1,pod1.next_cp_x,pod1.next_cp_y,pod1.dist,pod1.cp_ang,pod1.vel) ,file=sys.stderr)
    print(str(tx1),str(ty1),str(thrust),['1',flag])
    
    pod2.prep(x2,y2,vx2,vy2,angle2,next_check_point_id2)
    num2 = pod2.turn_counter()
    pod2.cp_params()
    pod2.checkpoint_angle()
    pod2.velocity_angle()
    pod2.check_boost()
    thrust2,flag2 = pod2.thrust()
    tx2,ty2 = pod2.target()
    print("<< POD2 >>\ntx2 = {} ty2 = {} nextcpx = {} nextcpy = {} dist = {}\nang = {} vel = {}\n".format(tx2,ty2,pod2.next_cp_x,pod2.next_cp_y,pod2.dist,pod2.cp_ang,pod2.vel) ,file=sys.stderr)
    print(str(tx2),str(ty2),str(thrust2),['2',flag2])
    
