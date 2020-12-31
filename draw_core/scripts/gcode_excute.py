import numpy as np
import arm_controller

class gcode_excute():
    def __init__(self, xi, yi, x0, y0):
        self.x0, self.y0 = x0, y0 # coordinate offset
        self.setPosition(xi, yi) # initial position
        self.s = 0.003 # scale factor (mm -> m)

        self.amax = 0.3

        # instanlize controller
        self.controller = Arm_Contrl()

    def G0(self, x, y): # the "just go there" instruction
        xn, yn = self.tf(x, y)
        print "G0",x,y
        # self.path = line.Line(self.x, self.y, xn, yn, 0.5, self.amax)
        self.controller.draw_line(x, y)

    def G1(self, x, y, fr): # move to x, y in a line at a speed in mm/minute
        fr /= 1000 * 60 # mm/minute -> m/sec
        xn, yn = self.tf(x, y)
        print "G1",x,y
        # self.path = line.Line(self.x, self.y, xn, yn, fr, self.amax)
        self.controller.draw_line(x, y)

    def G2(self, x, y, xc, yc, fr): # move in a clockwise arc to an endpoint around a center
        fr /= 1000 * 60 # mm/minute -> m/sec
        xn, yn = self.tf(x, y)
        xcn, ycn = self.tf(xc, yc)
        print "G2",x,y,xc,yc
        # self.path = arc.Arc(self.x, self.y, xn, yn, xcn, ycn, fr, self.amax, cw=True)
        
    def G3(self, x, y, xc, yc, fr):
        fr /= 1000 * 60
        xn, yn = self.tf(x, y)
        xcn, ycn = self.tf(xc, yc)
        print "G3",x,y,xc,yc
        # self.path = arc.Arc(self.x, self.y, xn, yn, xcn, ycn, fr, self.amax, cw=False)

    def setPosition(self, x, y):
        self.x, self.y = x, y

    def tf(self, x, y):
        x = x * self.s + self.x0
        y = y * self.s + self.y0
        return (x, y)