# -*- coding: utf-8 -*-
import sys
import math
from math import sqrt
from math import acos
# To debug: print("Debug messages...", file=sys.stderr)

# Store the number of laps in the race and the check point locations
laps = int(input())
num_cp = int(input())
chk_pts = []
for i in range(num_cp):
    cp = ([int(j) for j in input().split()])
    chk_pts.append(cp)

# Class for a Pod:
# contains all setup methods, low level control, and high level control
# LOW LEVEL: indicates a method called every game loop by the pod
# RACE: indicates a method called by a pod when in race mode
# BLOCK: indicates a method called by a pod when in block mode

class Pod(object):
    def __init__(self):
        #Parameters given on each turn
        self.x = 0 # X pose
        self.y = 0 # Y pose
        self.x_vel = 0 # Velocity in the x direction
        self.y_vel = 0 # Velocity in the y direction
        self.ang = 0 # Orientation of the pod
        self.cp_id = 0 # Current checkpoint index
        self.last_cp_id = 1 # Store the checkpoint id from the last turn

        # Race Standing Values
        self.turn_num = 0 # Current turn number
        self.chkpts_complete = 0
        self.position = 1

        #Checkpoint coordinates
        self.nextcpx = 0 # X coordinate of current checkpoint
        self.nextcpy = 0 # Y coordinate of current checkpoint

        # Goal position
        # Coordinates to calculate all pod related angles and distances off of
        self.goalx = 0
        self.goaly = 0

        #Angle between pods velocity and the euclidian distance to goal pose
        self.xerror = 0 # Goal pose x - current pose x
        self.yerror = 0 # Goal pose y - current pose y
        self.dist = 0 # Euclidian distance to goal
        self.velocity = 0 # Velocity magnitude

        ### THE FOLLOWING ANGLES TREAT THE POD AS THE ORIGIN ###
        self.goal_ang = 0 # Global angle of goal pose vector
        self.vel_ang = 0 # Global angle of velocity vector

        # SHIELD
        self.past_turns = ['','','','']

    '''
    LOW LEVEL :: METHODS
    '''
    ### LOW LEVEL ###
    def prep(self,xpose,ypose,x_v,y_v,ang,next_check_id):
        # Store the turn by turn values into respective class properties
        self.x = xpose # X position of pod
        self.y = ypose # Y position of pod
        self.x_vel = x_v # Velocity in x direction
        self.y_vel = y_v # Velocity in y direction
        self.ang = ang # Global angle of pod
        self.cp_id = next_check_id # Index of current checkpoint

    ### LOW LEVEL ###
    # Count the number of checkpoints completed, by checking when the id switches
    def completed_checkpoints(self):
        if self.cp_id != self.last_cp_id:
            self.chkpts_complete += 1
        self.last_cp_id = self.cp_id
        return self.chkpts_complete

    # Determine the race standing of the pod by comparing the number of
    # checkpoints it has completed versus the number of checkpoints completed
    # by the other
    def determine_position(self,pod2):
        if self.chkpts_complete > pod2.chkpts_complete:
            self.position = 1
        elif self.chkpts_complete < pod2.chkpts_complete:
            self.position = 2
        elif self.chkpts_complete == pod2.chkpts_complete:
            if self.dist > pod2.dist:
                self.position = 2
            elif self.dist < pod2.dist:
                self.position = 1
            elif self.dist == pod2.dist:
                self.position = 1

    ### LOW LEVEL ###
    # Based on the pods race position, set the race tactic
    # TACTICS: race
    #          block
    def set_role(self):
        if self.position == 1:
            self.role = "race"
        else:
            self.role = "block"
        return self.role

    ### LOW LEVEL ###
    # Get cp coordinates off of current cp_id
    def cp_params(self):
        # Store the current checkpoint coordinates by indexing
        # into the list of every checkpoint
        self.nextcpx,self.nextcpy = chk_pts[self.cp_id]
        return self.nextcpx, self.nextcpy

    ### LOW LEVEL ###
    # Sets goal coordinates, from values passed
    def update_goal(self,x,y):
        self.goalx = x
        self.goaly = y

    ### LOW LEVEL ###
    # Calculates values that are based on the goal pose:
    #   distance to goal
    #   vel angle off goal

    def angles(self):
        # Radian values for convenience
        pi = math.pi
        po2 = math.pi/2
        tpo2 = (3*math.pi)/2
        tpi = 2*math.pi

        # Calculate the unit vectors of euclidian distance to goal pose
        self.xerror = self.goalx - self.x
        self.yerror = self.goaly - self.y

        # Calculate euclidian distance and velocity
        self.dist = sqrt(self.xerror**2 + self.yerror**2)
        self.velocity = sqrt(self.x_vel**2 + self.y_vel**2)

        ######### Global Euclidian Distance Vector #########
        if self.xerror < 0 and self.yerror == 0:
            self.goal_ang = pi
        elif self.xerror > 0 and self.yerror == 0:
            self.goal_ang = 0
        elif self.xerror == 0 and self.yerror > 0:
            self.goal_ang = po2
        elif self.xerror == 0 and self.yerror < 0:
            self.goal_ang = tpo2
        elif self.xerror > 0 and self.yerror < 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            #print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.goal_ang = tpi - theta
        elif self.xerror > 0 and self.yerror > 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            #print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.goal_ang = theta
        elif self.xerror < 0 and self.yerror < 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            #print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.goal_ang = pi + theta
        elif self.xerror < 0 and self.yerror > 0:
            theta = math.atan(abs(self.yerror)/abs(self.xerror))
            #print("cp theta={}".format(math.degrees(theta)) ,file=sys.stderr)
            self.goal_ang = pi - theta
        else:
            print("You missed a condition in the Euclidian Distance, Pod.Angles()",file=sys.stderr)

        #########   GLobal Velocity Angle   #########
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
            #print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = tpi - theta
        elif self.x_vel > 0 and self.y_vel > 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            #print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = theta
        elif self.x_vel < 0 and self.y_vel < 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            #print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = pi + theta
        elif self.x_vel < 0 and self.y_vel > 0:
            theta = math.atan(abs(self.y_vel)/abs(self.x_vel))
            #print("vel theta = {}".format(math.degrees(theta)) ,file=sys.stderr)
            self.vel_ang = pi - theta
        else:
            print("You missed a condition in the Global Velocity, Pod.Angles()",file=sys.stderr)
            self.vel_ang = 0

        # Print the result of angles()
        #print("vel_angle={},goal_ang={},ang={},x_vel={},y_vel={},xer={},yer={}".format(math.degrees(self.vel_ang),math.degrees(self.goal_ang),self.ang,self.x_vel,self.y_vel,self.xerror,self.yerror) ,file=sys.stderr)

    ### LOW LEVEL ###
    def control(self):
        # Store radian values for convenience
        pi = math.pi
        po2 = math.pi/2
        tpo2 = (3*math.pi)/2
        tpi = 2*math.pi
        eightn = math.radians(18)

        # if theta_v - theta_cp > 0 rotate left
        # if theta_v - theta_cp < 0 rotate right
        # Difference between vecocity orientation and goal orientation
        self.delta_theta = (self.vel_ang - self.goal_ang)
        # If values are greater than 180, rotate the opposite direction
        if self.delta_theta > pi:
            ang = -(tpi-self.delta_theta)
        elif self.delta_theta < -pi:
            ang = (tpi + self.delta_theta)
        else:
            ang = self.delta_theta

        ang = ang * 1.3
        # Limit the rotation to 18 degrees
        if ang > eightn:
            ang = eightn
        elif ang < -eightn:
            ang = -eightn

        # Only compute the PID control for target pose if after the second turn
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

        # Target the goal coordinates exactly
        else:
            target_x = self.nextcpx
            target_y = self.nextcpy
            x_rot = 0 # needed to prevent error stream error
            y_rot = 0 # needed to prevent error stream error

        # Print turn values
        #print("delta_theta={},x_rot={},y_rot={},target_x={},target_y={},ang={}\ndist={}".format(self.delta_theta,x_rot,y_rot,target_x,target_y,math.degrees(ang),self.dist) ,file=sys.stderr)

        #Function return: Coordinates that the pod will navigate to
        return target_x,target_y

    ### LOW LEVEL ###
    def thrust(self):
        # Thrust is a function of the angle between velocity and goal xpose
        # Thrust value is weighted by the distance
        # Result is that the pod slows down approaching the goal, and speeds up
        # out of the corner beause it has room to accelerat and correct its path

        ###   THRUST(VEL_ANGLE)   ###
        angle = math.degrees(abs(self.delta_theta))
        min_thrust = 30 # minimum thrust: occurs when velocity is 180 degrees off the goal vector
        beta = (100 - min_thrust) / ((-180)**2)
        thrust = (beta * (angle-180)**2 + min_thrust)
        if thrust > 100:
            thrust = 100
        elif thrust < 0:
            thrust = 0

        ###   DISTANCE WEIGHTING   ###
        dist = 3500 # Specify a distance to start decelerating
        t1 = .9 # Tuning parameter for distance contribution to thrust
        alpha = (35 - 100) / (-(dist**2))
        if self.dist < dist: # Solve the inverse parabolic function for thrust as a function of distance to cp
            thrust_dist = (-alpha)*(self.dist-dist)**2 + 100
            # Condition the thrust value so it can weight the angle_thrust value
            thrust_dist = (t1*thrust_dist)/100
        else:
            thrust_dist = 1.6 # If you're not in the deccelerating range, increase the thrust

        # CALCULATE THE WEIGHTED THRUST VALUE
        # 0 < thrust_dist < 1
        thrust = thrust * thrust_dist
        # Limit the thrust value
        if thrust > 100:
            thrust = 100
        elif thrust < 0:
            thrust = 0

        # Set thrust attribute as the calculated result
        self.t = str(math.ceil(thrust))
        #print("thrust = {}".format(thrust),file=sys.stderr)

    ### LOW LEVEL ###
    def count_turn(self):
        # Count what turn it is
        self.turn_num += 1

    # Adjusts the cp_id based on inputed val
    # Val can be positive or negative
    # Function accounts for roll over
    def update_cp_id(self,val):
        l = len(chk_pts)-1
        newcp = (self.cp_id+val)
        if newcp > l:
            diff = newcp - l
            self.cp_id = 0 + diff
        elif newcp < 0:
            diff = abs(newcp)
            self.cp_id = l - diff
        else:
            self.cp_id = newcp

    # Function calls methods that get cp_params, update goal,
    # and calcualte angles
    def get_state(self):
        x,y = self.cp_params()
        self.update_goal(x,y)
        self.angles()
        return x,y

    '''
    ROLE --> RACE :: METHODS
    '''
    ### RACE ###
    # Decide to set the boost flag
    def boost(self):
        # Boost on first turn
        if self.turn_num < 2 :
            self.t = "BOOST"

    ### RACE ###
    def adjust_cp(self):
        # This function decides if the pod can start navigating to the next checkpoint
        # while still passing through the current assigned goal

        l = len(chk_pts)-1 # Number of checkpoints
        # If the pod will pass though the checkpoint, switch to the next checkpoint
        if (self.dist < 1750) and (abs(self.vel_ang) < 25) and (self.velocity > 200):
            #print("Switching to next CP...",file=sys.stderr)
            if self.cp_id == l: # if the current checkpoint is the last in the list, target index 0
                self.cp_id = 0
                self.prep(self.x,self.y,self.x_vel,self.y_vel,self.ang,self.cp_id)
            else: # Target index += 1
                self.cp_id += 1
                self.prep(self.x,self.y,self.x_vel,self.y_vel,self.ang,self.cp_id)

    def race_shield(self,op1,op2):
        op1.update_goal(self.x,self.y)
        op1.angles()
        op2.update_goal(self.x,self.y)
        op2.angles()

        pos_1 = check_race_standing(self,op1)
        pos_2 = check_race_standing(self,op2)

        if "SHIELD" in self.past_turns:
            del self.past_turns[-1]
            self.past_turns.insert(0,'')
        else:
            if ((abs(op1.vel_ang) < 25) and (op1.dist < 1500)) or ((abs(op2.vel_ang) < 25) and (op2.dist < 1500)):
                self.t = "SHIELD"
                print('----SHIELD----',file=sys.stderr)
                self.past_turns[-1]
                self.past_turns.insert(0,"SHIELD")


    '''
    ROLE --> BLOCK :: METHODS
    '''
    # This function takes the given opponent handles and determines their positions
    def determine_opponent_leader(self,op1,op2):
        # This function
        if op1.position ==1:
            self.block_id = op1.cp_id
            self.op_dist = op1.dist
            op_lead = 'op1'
            return op1
        elif op2.position == 1:
            self.block_id = op2.cp_id
            self.op_dist = op2.dist
            op_lead = 'op2'
            return op2
        else:
            self.block_id = op1.cp_id
            if op1.dist > op2.dist:
                self.op_dist = op2.dist
            elif op2.dist > op1.dist:
                self.op_dist = op1.dist
            else:
                self.op_dist = op1.dist
            op_lead = 'tie'
            return op1
        print('opponent_leader={}'.format(op_lead),file=sys.stderr)

    def loiter(self):
        x = 1
        # wait for race pod to be losing

    # If blocker pod will beat leading opponent to the checkpoint, do so
    # If opponenet is closer to cp than blocker, advance to next cp
    def intercept(self):
        self.cp_id = self.block_id
        self.get_state()

        if self.op_dist == self.dist:
            self.update_cp_id(1)
            self.get_state()
        elif self.op_dist < self.dist:
            self.update_cp_id(1)
            self.get_state()

    # To activate shield:
    #   - The op must be within a certain distance, so that the velocity along
    #       the position vector warrants a collision
    #   - The op's velocity vector must be pointing towards the pod to warrant
    #       a collision
    def shield(self,op1,op2):
        op1.update_goal(self.x,self.y)
        op1.angles()
        op2.update_goal(self.x,self.y)
        op2.angles()
        if "SHIELD" in self.past_turns:
            del self.past_turns[-1]
            self.past_turns.insert(0,'')
        else:
            if ((abs(op1.vel_ang) < 25) and (op1.dist < 1500)) or ((abs(op2.vel_ang) < 25) and (op2.dist < 1500)):
                self.t = "SHIELD"
                print('----SHIELD----',file=sys.stderr)
                self.past_turns[-1]
                self.past_turns.insert(0,"SHIELD")


######################   -  GAME LOOP FUNCTIONS   -  ######################

# Calls the functions that take current cp_id, get checkpoint coordinates,
# updates the goal pose with cp params and then calculates all related angles
def get_state(pod):
    x,y = pod.cp_params()
    pod.update_goal(x,y)
    pod.angles()
    return x,y

def block(pod,op1,op2):
    #This function is prefixed by the function calls:
    #   prep()
    #   completed_checkpoints()
    #   determine_position()
    #   set_role()
    op = pod.determine_opponent_leader(op1,op2)
    pod.intercept()
    print('op = {}'.format(op),file=sys.stderr)
    ### LOW LEVEL ###
    xf,yf = pod.control()
    pod.thrust()
    pod.count_turn()

    pod.shield(op1,op2)
    return xf,yf

def race(pod,op1,op2):
    #This function is prefixed by the function calls:
    #   prep()
    #   completed_checkpoints()
    #   determine_position()
    #   set_role()
    get_state(pod)
    pod.adjust_cp()
    get_state(pod)

    ### LOW LEVEL ###
    xf,yf = pod.control()
    pod.thrust()
    pod.count_turn()
    #pod.shield(op1,op2)
    return xf,yf


def check_race_standing(pod,op):
    if pod.chkpts_complete > op.chkpts_complete:
        return 1
    elif pod.chkpts_complete < op.chkpts_complete:
        return 2
    elif pod.chkpts_complete == op.chkpts_complete:
        if pod.dist > op.dist:
            return 2
        elif pod.dist < op.dist:
            return 1
        elif pod.dist == op.dist:
            return 1

######################   -  POD INITIALIZATION   -  ######################
pod1 = Pod()
pod2 = Pod()
op1 = Pod()
op2 = Pod()
######################   -  Game Loop   -  ######################
while True:
    #Sets the turn values of checkpoint postition and current attitude
    x, y, vx, vy, angle, next_check_point_id = [int(j) for j in input().split()]
    x2, y2, vx2, vy2, angle2, next_check_point_id2 = [int(j) for j in input().split()]
    o_x, o_y, o_vx, o_vy, o_angle, o_next_check_point_id = [int(j) for j in input().split()]
    o2_x, o2_y, o2_vx, o2_vy, o2_angle, o2_next_check_point_id = [int(j) for j in input().split()]

    # Initialize the opponents
    op1.prep(o_x,o_y,o_vx,o_vy,o_angle,o_next_check_point_id)
    op2.prep(o2_x,o2_y,o2_vx,o2_vy,o2_angle,o2_next_check_point_id)
    op1cp = op1.completed_checkpoints()
    op2cp = op2.completed_checkpoints()
    op1.determine_position(op2)
    op2.determine_position(op1)
    op1.get_state()
    op2.get_state()
    op1.count_turn()
    op2.count_turn()

    # Pass turn into to the pods and determine how many checkpoints have
    # been completed
    pod1.prep(x,y,vx,vy,angle,next_check_point_id)
    pod2.prep(x2,y2,vx2,vy2,angle2,next_check_point_id2)
    chkpts1 = pod1.completed_checkpoints()
    chkpts2 = pod2.completed_checkpoints()
    pod1.determine_position(pod2)
    pod2.determine_position(pod1)

    print("chkpts1 = {}, chkpts2 = {}".format(chkpts1,chkpts2), file=sys.stderr)

    # If both pods are on the first checkpoint, both race
    if (chkpts1 < 1) and (chkpts2 < 1):
        print("Both on chkpt 1...", file=sys.stderr)

        get_state(pod1)
        pod1.adjust_cp()
        get_state(pod1)
        x,y = pod1.control()
        pod1.thrust()
        pod1.boost()
        pod1.count_turn()

        get_state(pod2)
        x2,y2 = pod2.control()
        pod2.thrust()
        pod2.count_turn()

    # Now determine block or race: based on which pod is the leader
    # TODO(add some logic so that the pods dont get stuck flipping states)
    else:
        print("Deciding Race/Block...", file=sys.stderr)

        role1 = pod1.set_role()
        role2 = pod2.set_role()
        print("pod1 = {}\npod2 = {}".format(role1,role2), file=sys.stderr)

        if role1 == "race":
            x,y = race(pod1,op1,op2)
            x2,y2 = block(pod2,op1,op2)
        else:
            x,y = block(pod1,op1,op2)
            x2,y2 = race(pod2,op1,op2)

    # Target Coordinates and thrust values of the pods
    print(str(x),str(y),pod1.t)
    print(str(x2),str(y2),pod2.t)
