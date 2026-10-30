#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

rospy.init_node('probe_purple_route', anonymous=True)
msg = rospy.wait_for_message('/grid_map/occupancy_inflate', PointCloud2, timeout=8)
pts = [(x, y, z) for x, y, z in point_cloud2.read_points(
    msg, field_names=('x', 'y', 'z'), skip_nans=True) if 0.48 <= z <= 1.25]
print('points_at_flight_height', len(pts), flush=True)
print('bbox', min(x for x,_,_ in pts), max(x for x,_,_ in pts), min(y for _,y,_ in pts), max(y for _,y,_ in pts), flush=True)
grid = {}
for p in pts:
    grid.setdefault((int(math.floor(p[0] / 0.5)), int(math.floor(p[1] / 0.5))), []).append(p)

start = (2.55, -1.15)
end = (4.20, -1.80)
for x0 in (2.20, 2.35, 2.45, 2.55):
    for yy in (-2.15, -2.30, -2.45):
        for x1 in (3.85, 4.05, 4.20):
            route = [start, (x0, yy), (x1, yy), end]
            best = (float('inf'), None, None)
            for a, b in zip(route, route[1:]):
                length = math.dist(a, b)
                for i in range(max(1, int(length / 0.05)) + 1):
                    t = i / max(1, int(length / 0.05))
                    x = a[0] + t * (b[0] - a[0])
                    y = a[1] + t * (b[1] - a[1])
                    ix, iy = int(math.floor(x / 0.5)), int(math.floor(y / 0.5))
                    for dx in range(-2, 3):
                        for dy in range(-2, 3):
                            for p in grid.get((ix + dx, iy + dy), ()): 
                                d = math.hypot(p[0] - x, p[1] - y)
                                if d < best[0]:
                                    best = (d, (x, y), p)
            print('route', x0, yy, x1, 'min_inflated_xy', round(best[0], 3),
                  'at', best[1], 'obstacle', best[2], flush=True)
