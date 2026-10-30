#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

rospy.init_node('debug_inflated_map', anonymous=True)
msg = rospy.wait_for_message('/grid_map/occupancy_inflate', PointCloud2, timeout=5)
pts = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
print('points', len(pts))
for z in (0.4, 0.8, 1.2, 1.6, 1.8, 2.0):
    cells = {(round(x/0.2), round(y/0.2)) for x,y,pz in pts if abs(pz-z)<0.12}
    print('\nz=',z,'cells=',len(cells))
    for iy in range(15,-16,-1):
        line = ''
        for ix in range(-5,31):
            x,y=ix*0.2,iy*0.2
            if abs(x)<0.11 and abs(y)<0.11: ch='H'
            elif abs(x-0.1)<0.11 and abs(y+1.1)<0.11: ch='X'
            else: ch='#' if (ix,iy) in cells else '.'
            line += ch
        print(line)
