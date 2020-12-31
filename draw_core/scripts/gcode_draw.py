from pygcode import *
import os
import gcode_excute
class Interpreter():
    def __init__(self):
        self.file = None
        self.mac = Machine()
        self.planner = None
        self.fr = 0
        self.EOF = False

    def setPlanner(self, planner):
        self.planner = planner

    def loadFile(self, fname):
        if not (self.file is None):
            self.file.close()
        self.file = open(fname, "r")

    def gcode_draw(self,fname):
        for line_txt in self.file.readline():
            line = Line(line_txt)
            block = self.mac.block_modal_gcodes(line.block)
            xb, yb, zb = self.mac.pos.vector
            self.mac.process_gcodes(*sorted(block))
            xa, ya, za = self.mac.pos.vector
            for b in block:
                if isinstance(b, GCodeLinearMove):
                    self.planner.G1(self.mac.pos.vector[0], self.mac.pos.vector[1], self.fr)
                    # return True
                elif isinstance(b, GCodeRapidMove):
                    self.planner.G0(self.mac.pos.vector[0], self.mac.pos.vector[1])
                    # return True
                elif isinstance(b, GCodeFeedRate):
                    self.fr = b.word.value
                elif isinstance(b, GCodeArcMoveCW):
                    # I don't know how any of this works, just copied from a pygcode arc linearizing example
                    arc_center_ijk = dict((l, 0.) for l in "IJK")
                    arc_center_ijk.update(b.get_param_dict("IJK"))
                    arc_center_coords = dict(({"I":"x", "J":"y", "K":"z"}[k], v) for (k, v) in arc_center_ijk.items())
                    # incremental position
                    xc = arc_center_coords['x'] + xb
                    yc = arc_center_coords['y'] + yb
                    self.planner.G2(self.mac.pos.vector[0], self.mac.pos.vector[1], xc ,yc, self.fr)
                    # return True
                elif isinstance(b, GCodeArcMoveCCW):
                    arc_center_ijk = dict((l, 0.) for l in "IJK")
                    arc_center_ijk.update(b.get_param_dict("IJK"))
                    arc_center_coords = dict(({"I":"x", "J":"y", "K":"z"}[k], v) for (k, v) in arc_center_ijk.items())
                    xc = arc_center_coords['x'] + xb
                    yc = arc_center_coords['y'] + yb
                    self.planner.G3(self.mac.pos.vector[0], self.mac.pos.vector[1], xc, yc, self.fr)
                    # return True
                else:
                    print("Unknown Instruction: ", b)
            # return False

if __name__=="__main__":
    g = Interpreter()
    manipulator = gcode_excute(0,0,0,0)
    g.setPlanner(manipulator)
    g.loadFile("/home/derek/project/draw_robot/src/drawing_manipulator/draw_core/scripts/img/gcode.gcode")
    g.gcode_draw()

