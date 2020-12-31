
import os
import sys
import xml.etree.ElementTree as ET
import shapes as shapes_pkg
from shapes import point_generator
from config import *

gcode_text =[]
def generate_gcode(fname):
    svg_shapes = set(['rect', 'circle', 'ellipse', 'line', 'polyline', 'polygon', 'path'])
    tree = ET.parse(fname)
    root = tree.getroot()
    
    width = root.get('width')
    height = root.get('height')
    if width == None or height == None:
        viewbox = root.get('viewBox')
        if viewbox:
            _, _, width, height = viewbox.split()                

    if width == None or height == None:
        print "Unable to get width and height for the svg"
        sys.exit(1)

    width = float(width)
    height = float(height)

    scale_x = bed_max_x / max(width, height)
    scale_y = bed_max_y / max(width, height)

    print preamble 
    gcode_text.append(preamble)
    for elem in root.iter():
        
        try:
            _, tag_suffix = elem.tag.split('}')
        except ValueError:
            continue

        if tag_suffix in svg_shapes:
            shape_class = getattr(shapes_pkg, tag_suffix)
            shape_obj = shape_class(elem)
            d = shape_obj.d_path()
            m = shape_obj.transformation_matrix()

            if d:
                gcode_text.append(shape_preamble)
                print shape_preamble 
                p = point_generator(d, m, smoothness)
                try:
                    for x,y in p:
                        # if x > 0 and x < bed_max_x and y > 0 and y < bed_max_y:  
                        string_g =  "G1 X%0.1f Y%0.1f" % (scale_x*x, scale_y*y) 
                        gcode_text.append(string_g)
                        print string_g
                except:
                    print "ERROR"
                    pass
                gcode_text.append(shape_postamble)
                print shape_postamble

    print postamble 
    gcode_text.append(postamble)

if __name__ == "__main__":
    generate_gcode("/home/derek/project/draw_robot/src/drawing_manipulator/draw_core/scripts/img/neu.svg")
    with open("result.gcode",'w') as gfile:
        for t in gcode_text:
            gfile.write(t+'\n')



