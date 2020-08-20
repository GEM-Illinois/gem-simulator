'''
    This script will take raw lane data perfrom some smoothing on it then create a SDF file
    containing lane markers (which can be put into gazebo)
'''

import math
import random
import numpy as np


data = np.loadtxt('whole_lane_data.dat')


def calc_lane_pts(x1,y1,x2,y2):
    angle = math.atan2(y2-y1, x2-x1)
    dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    inner_dist = dist - 1
    outer_dist = dist + 1

    inner_x_pt = inner_dist * math.cos(angle)
    inner_y_pt = inner_dist * math.sin(angle)

    outer_x_pt = outer_dist * math.cos(angle)
    outer_y_pt = outer_dist * math.sin(angle)


    return inner_x_pt, inner_y_pt, outer_x_pt, outer_y_pt

def clean_lane_data():
    clean_data = []
    avg = 30 #change this to modfiy how many points we average over
    for i in range(0, len(data)-avg-1):
        accum_x = 0
        accum_y = 0

        for j in range(0, avg):
            pt = data[i+j]
            accum_x = accum_x + pt[0]
            accum_y = accum_y + pt[1]

        avg_x = accum_x / avg
        avg_y = accum_y / avg
        clean_data.append((avg_x, avg_y))
    return clean_data






f = open("model.sdf", "w+")

f.write("<?xml version=\"1.0\"?>\n")
f.write("<sdf version=\"1.6\">\n")
f.write("\t<model name=\"highbay_loop1\">\n")
f.write("\t\t<static>true</static>\n")
f.write("\t\t<link name=\"inner_loop\">\n\n")

i = 0
new_data = clean_lane_data()
for point in new_data:
    x = point[0] - 7
    y = point[1] + 8

    inner_x, inner_y, outer_x, outer_y = calc_lane_pts(0, 0, x, y)

    f.write("\t\t<visual name=\"inner_point"+str(i)+"\">\n\t\t\t<pose>"+str(inner_x)+" "+str(inner_y)+" "+ "0.01 0 0 0</pose>\n\t\t\t<geometry>\n\t\t\t\t<cylinder>\n\t\t\t\t\t<radius>0.1</radius>\n\t\t\t\t\t<length>0.001</length>\n\t\t\t\t</cylinder>\n\t\t\t</geometry>\n\t\t\t<material>\n\t\t\t\t<script>\n\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n\t\t\t\t\t<name>Gazebo/Yellow</name>\n\t\t\t\t</script>\n\t\t\t</material>\n\t\t</visual>\n\n")
    f.write("\t\t<visual name=\"outer_point"+str(i)+"\">\n\t\t\t<pose>"+str(outer_x)+" "+str(outer_y)+" "+ "0.01 0 0 0</pose>\n\t\t\t<geometry>\n\t\t\t\t<cylinder>\n\t\t\t\t\t<radius>0.1</radius>\n\t\t\t\t\t<length>0.001</length>\n\t\t\t\t</cylinder>\n\t\t\t</geometry>\n\t\t\t<material>\n\t\t\t\t<script>\n\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n\t\t\t\t\t<name>Gazebo/Yellow</name>\n\t\t\t\t</script>\n\t\t\t</material>\n\t\t</visual>\n\n")

    i = i+1
    #print("<point>",x,y,0.001,"</point>")


#generate perfect circle
# for i in np.arange(0, 2*math.pi, .01):
#     x = 3 * math.cos(i)
#     y = 3 * math.sin(i)
#     j = 5 * math.cos(i)
#     k = 5 * math.sin(i)
#     f.write("\t\t<visual name=\"inner_point"+str(i)+"\">\n\t\t\t<pose>"+str(x)+" "+str(y)+" "+ "0 0.001 0 0 0</pose>\n\t\t\t<geometry>\n\t\t\t\t<cylinder>\n\t\t\t\t\t<radius>0.1</radius>\n\t\t\t\t\t<length>0.001</length>\n\t\t\t\t</cylinder>\n\t\t\t</geometry>\n\t\t\t<material>\n\t\t\t\t<script>\n\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n\t\t\t\t\t<name>Gazebo/Yellow</name>\n\t\t\t\t</script>\n\t\t\t</material>\n\t\t</visual>\n\n")
#     f.write("\t\t<visual name=\"outer_point"+str(i)+"\">\n\t\t\t<pose>"+str(j)+" "+str(k)+" "+ "0 0.001 0 0 0</pose>\n\t\t\t<geometry>\n\t\t\t\t<cylinder>\n\t\t\t\t\t<radius>0.1</radius>\n\t\t\t\t\t<length>0.001</length>\n\t\t\t\t</cylinder>\n\t\t\t</geometry>\n\t\t\t<material>\n\t\t\t\t<script>\n\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n\t\t\t\t\t<name>Gazebo/Yellow</name>\n\t\t\t\t</script>\n\t\t\t</material>\n\t\t</visual>\n\n")
#     print("<point>",x,y,0,"</point>")



f.write("\t\t</link>\n\t</model>\n</sdf>")

f.close()
