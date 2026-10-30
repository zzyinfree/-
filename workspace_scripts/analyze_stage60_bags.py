#!/usr/bin/env python3
import json
import math
import sys
import rosbag

for path in sys.argv[1:]:
    odom=[]
    states=[]
    with rosbag.Bag(path) as bag:
        for topic,msg,t in bag.read_messages(topics=['/rflysim/uav1/local/odom','/mavros/state']):
            if topic.endswith('/odom'):
                p=msg.pose.pose.position
                odom.append((t.to_sec(),p.x,p.y,p.z))
            else:
                states.append((t.to_sec(),msg.mode,msg.armed))
    armed=[s for s in states if s[2]]
    land_modes=[s for s in states if s[1]=='AUTO.LAND' and s[2]]
    first_land=land_modes[0] if land_modes else None
    land_pose=min(odom,key=lambda p: abs(p[0]-first_land[0])) if first_land and odom else None
    result={
      'bag':path,
      'odom_samples':len(odom),
      'state_samples':len(states),
      'takeoff_pose':odom[0][1:] if odom else None,
      'first_auto_land_pose':land_pose[1:] if land_pose else None,
      'last_pose':odom[-1][1:] if odom else None,
      'last_xy_from_origin_m':round(math.hypot(odom[-1][1],odom[-1][2]),3) if odom else None,
      'first_auto_land_xy_from_origin_m':round(math.hypot(land_pose[1],land_pose[2]),3) if land_pose else None,
      'armed_seen':bool(armed),
      'disarmed_final':not states[-1][2] if states else None,
    }
    print(json.dumps(result,ensure_ascii=False))
