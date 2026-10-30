#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

rospy.init_node('probe_purple_hover', anonymous=True)
msg = rospy.wait_for_message('/grid_map/occupancy_inflate', PointCloud2, timeout=5)
pts = list(point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True))
for x,y in [(3.2,-1.7),(3.5,-1.8),(3.8,-1.8),(4.0,-1.8),(4.2,-1.8),(4.5,-1.8)]:
    near = sorted((math.hypot(px-x,py-y),px,py,pz) for px,py,pz in pts
                  if 0.3 <= pz <= 1.1)
    print(f'candidate=({x:.2f},{y:.2f},0.60) nearest_inflated_xy={near[0][0]:.2f} '
          f'point=({near[0][1]:.2f},{near[0][2]:.2f},{near[0][3]:.2f})')
