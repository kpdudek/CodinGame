import sys
import math
import numpy

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
                self.Target = [math.ceil(width/2),yLand[iLand]]
                return
    
    def EuclidianDistance(self,x,y):
        # print(x, file=sys.stderr)
        # print(y, file=sys.stderr)
        dist = math.sqrt(math.pow(x[0]-y[0],2) + math.pow(x[1]-y[1],2))
        self.dist = dist
        print(dist, file=sys.stderr)


    def setDesiredVelocity(self,kVel):
        self.EuclidianDistance([self.x,self.y],self.Target)
        
        Vdesired = -math.pow(self.dist*.002,2) - 10
        print(Vdesired, file=sys.stderr)
        if Vdesired > -10:
            Vdesired = -10
        elif Vdesired < -50:
            Vdesired = -50 

        self.Vdesired = Vdesired

    def setPower(self,kp):
        error =self.Vdesired - self.v_speed
        power = (kp * error)

        power = round(power)
        if power > 4:
            power = 4
        elif power < 0:
            power = 3

        rover.power = power
    

# INITIALIZE PARAMETERS
rover = Mars()
kp = 1.6
kVel = -1
rover.FindLandingSite(surface_n,land_x,land_y)

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
    rover.update(x,y,h_speed,v_speed,fuel,rotate,power)
    
    rover.setDesiredVelocity(kVel)
    rover.setPower(kp)

    # 2 integers: rotate power. rotate is the desired rotation angle (should be 0 for level 1), power is the desired thrust power (0 to 4).
    print(0,rover.power)