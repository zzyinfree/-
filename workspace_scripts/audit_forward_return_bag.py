#!/usr/bin/env python3
"""Audit whether horizontal return motion is nose-first in the control odometry."""
import math
import sys

import rosbag

bag_path = sys.argv[1]
start = float(sys.argv[2])
end = float(sys.argv[3])
samples = []
positions = []
last = None
with rosbag.Bag(bag_path) as bag:
    for _, msg, stamp in bag.read_messages(topics=["/rflysim/uav1/local/odom"]):
        t = stamp.to_sec()
        if t < start or t > end:
            continue
        p = msg.pose.pose.position
        positions.append((t, p.x, p.y, p.z))
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if last is not None:
            dt = t-last[0]
            if 0.04 < dt < 0.5:
                vx = (p.x-last[1])/dt
                vy = (p.y-last[2])/dt
                speed = math.hypot(vx,vy)
                if speed >= 0.12:
                    alignment = (vx*math.cos(yaw)+vy*math.sin(yaw))/speed
                    samples.append((t, speed, alignment, p.x, p.y, p.z))
        last = (t,p.x,p.y,p.z)

if not samples:
    raise SystemExit("no horizontal-motion samples")
n = len(samples)
forward = sum(a > math.cos(math.radians(35)) for _,_,a,*_ in samples)
backward = sum(a < 0 for _,_,a,*_ in samples)
worst = sorted(samples, key=lambda x: x[2])[:5]
print(f"samples={n} forward_within_35deg={forward/n:.1%} backward={backward/n:.1%}")
print("bounds:", {"x":(round(min(p[1] for p in positions),2),round(max(p[1] for p in positions),2)),
                  "y":(round(min(p[2] for p in positions),2),round(max(p[2] for p in positions),2)),
                  "z":(round(min(p[3] for p in positions),2),round(max(p[3] for p in positions),2))})
print("worst:", [(round(t,2),round(v,2),round(a,2),round(x,2),round(y,2),round(z,2))
                 for t,v,a,x,y,z in worst])
