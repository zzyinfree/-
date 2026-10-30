#!/usr/bin/env python3
import sys, rosbag
b, a, z = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
last = 0
for topic,msg,stamp in rosbag.Bag(b).read_messages(topics=['/rflysim/uav1/local/odom','/mavros/local_position/odom']):
 t=stamp.to_sec()
 if a<=t<=z and t-last>.14:
  p=msg.pose.pose.position
  print(round(t,2),topic.split('/')[1],round(p.x,2),round(p.y,2),round(p.z,2))
  last=t
