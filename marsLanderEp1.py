import sys
import math

# Auto-generated code below aims at helping you parse
# the standard input according to the problem statement.

surface_n = int(input())  # the number of points used to draw the surface of Mars.
for i in range(surface_n):
    # land_x: X coordinate of a surface point. (0 to 6999)
    # land_y: Y coordinate of a surface point. By linking all the points together in a sequential fashion, you form the surface of Mars.
    land_x, land_y = [int(j) for j in input().split()]

# game loop
    
class Mars(object):
    def __init__(self,xpos,ypos,h_sp,v_sp,fue,rotat,powe):
        self.x = xpos
        self.y = ypos
        self.h_speed = h_sp
        self.v_speed = v_sp
        self.fuel = fue
        self.rotate = rotat
        self.power = powe
    def accelerate(self):
        return 4
    def decellerate(self):
        return 3
    def hor(self):
        return 0
    
while True:
    # h_speed: the horizontal speed (in m/s), can be negative.
    # v_speed: the vertical speed (in m/s), can be negative.
    # fuel: the quantity of remaining fuel in liters.
    # rotate: the rotation angle in degrees (-90 to 90).
    # power: the thrust power (0 to 4).
    # Write an action using print
    # To debug: print("Debug messages...", file=sys.stderr)
    
    x, y, h_speed, v_speed, fuel, rotate, power = [int(i) for i in input().split()]
    
    mar = Mars(x,y,h_speed,v_speed,fuel,rotate,power)
    
    if mar.v_speed < -35:
        thr = mar.accelerate()
    else:
        thr = mar.decellerate()
        
    the = mar.hor()

    # 2 integers: rotate power. rotate is the desired rotation angle (should be 0 for level 1), power is the desired thrust power (0 to 4).
    print(the, thr)