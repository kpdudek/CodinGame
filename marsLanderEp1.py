import sys
import math
import numpy as np

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.
surface_n = int(input())  # the number of points used to draw the surface of Mars.
land_x = []
land_y = []

for i in range(surface_n):
    # land_x: X coordinate of a surface point. (0 to 6999)
    # land_y: Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    Lx, Ly = [int(j) for j in input().split()]
    land_x.append(Lx)
    land_y.append(Ly)

# game loop
class Mars(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.h_speed = 0
        self.v_speed = 0
        self.fuel = 0
        self.rotate = 0
        self.power = 0
        self.Vdesired = 0
        self.Target = []
        self.dist = 0

    def update(self,x,y,Vx,Vy,fuel,rotate,power):
        self.x = x
        self.y = y
        self.h_speed = Vx
        self.v_speed = Vy
        self.fuel = fuel
        self.rotate = rotate
        self.power = power

    def FindLandingSite(self,N,xLand,yLand):
        if len(land_x) < 2:
            print("Defaulting to under the rover...", file=sys.stderr)
            self.Target = [self.x,self.y]
            return
        
        for iLand in range(0,N-2):
            width = xLand[iLand+1]-xLand[iLand]
            if yLand[iLand] == yLand[iLand+1] and width > 1000:
                self.Target = [xLand[iLand]+math.ceil(width/2),yLand[iLand]]
                return
    
    def EuclidianDistance(self,x,y):
        dist = math.sqrt(math.pow(x[0]-y[0],2) + math.pow(x[1]-y[1],2))
        self.dist = dist
        print("Dist: {}".format(dist), file=sys.stderr)


    def setDesiredVelocity(self,kVel):
        self.EuclidianDistance([self.x,self.y],self.Target)
        
        errorX = self.Target[0]-self.x
        if abs(self.dist) > 1500:
            Vdesired = -15
        else:
            Vdesired = -math.pow(self.dist*kVel,2) - 10
            # print("Vel desired: {}".format(Vdesired), file=sys.stderr)
            if Vdesired > -10:
                Vdesired = -10
            elif Vdesired < -50:
                Vdesired = -50 

        self.Vdesired = Vdesired

    def setPower(self,kp):
        print("Vel desired: {} V_speed: {}".format(self.Vdesired,self.v_speed), file=sys.stderr)
        error =self.Vdesired - self.v_speed
        power = (kp * error)

        power = round(power)
        if power > 4:
            power = 4
        elif power < 0:
            power = 3

        rover.power = power
    
    def setAngle(self,kt):
        maxRotate = 22
        absErrorY = abs(self.Target[1] - self.y)

        ### Position control
        errorX = self.Target[0] - self.x
        thetaX = -kt * errorX
        print(thetaX,file=sys.stderr)
        thetaX = round(thetaX)
        if thetaX > maxRotate:
            thetaX = maxRotate
        elif thetaX < -maxRotate:
            thetaX = -maxRotate
        if absErrorY < 300:
            thetaX = 0

        ### Velocity control
        maxVel = 18
        # Velocity is positive if you're on left
        #             negative if you're on right
        setVel = maxVel * np.sign(self.Target[0]-self.x)
        errorV = setVel - self.h_speed
        thetaV = -1 * errorV
        thetaV = round(thetaV)
        if thetaV > maxRotate:
            thetaV = maxRotate
        elif thetaV < -maxRotate:
            thetaV = -maxRotate
        if absErrorY < 300:
            thetaV = 0

        # Decide which angle to use
        if abs(self.h_speed) > maxVel:
            self.rotate = thetaV
        else:
            self.rotate = thetaX
        
        if self.v_speed > -40 and absErrorY < 300:
            self.rotate = 0

        # self.rotate = thetaX + thetaV
        print("Angle X: {} Angle V: {}".format(thetaX,thetaV), file=sys.stderr)
        print("Angle: {}".format(self.rotate), file=sys.stderr)
    

# INITIALIZE PARAMETERS
rover = Mars()
kp = 15
kVel = .002
kt = .3
rover.FindLandingSite(surface_n,land_x,land_y)
print("Target X: {} Target Y: {}".format(rover.Target[0],rover.Target[1]), file=sys.stderr)

# GAME LOOP
while True:
    # h_speed: the horizontal speed (in m/s), can be negative.
    # v_speed: the vertical speed (in m/s), can be negative.
    # fuel: the quantity of remaining fuel in liters.
    # rotate: the rotation angle in degrees (-90 to 90).
    # power: the thrust power (0 to 4).
    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr)
    x,y,h_speed,v_speed,fuel,rotate,power = [int(i) for i in input().split()]
    
    # Update class members
    rover.update(x,y,h_speed,v_speed,fuel,rotate,power)
    
    # Determine what velocity is required
    rover.setDesiredVelocity(kVel)

    # Calculate thrust
    rover.setPower(kp)

    # Calculate angle
    rover.setAngle(kt)

    # 2 integers: rotate power. rotate is the desired rotation angle (should be 0 for level 1), power is the desired thrust power (0 to 4).
    print(int(rover.rotate),rover.power)