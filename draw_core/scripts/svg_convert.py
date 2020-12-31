# -*- coding: utf-8 -*-
import cv2
import sys
import os
import numpy as np

# sys.path.append(os.path.dirname(os.path.realpath(__file__)+"/py_svg2gcode"))
# sys.path.append(os.path.dirname(os.path.realpath(__file__)+ "/py_svg2gcode/lib")
import py_svg2gcode

py_svg2gcode.generate_gcode("/home/derek/project/draw_robot/src/drawing_manipulator/draw_core/scripts/img/neu.svg")