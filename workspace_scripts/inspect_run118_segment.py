import math
import rosbag
from tf.transformations import euler_from_quaternion

p='/root/gpufree-data/cicc_workspace/recordings/run_118_red_centerland_20261026/flight.bag'
samples=[]
with rosbag.Bag(p) as bag:
    for _,msg,t in bag.read_messages(topics=['/rflysim/uav1/local/odom']):
        stamp=t.to_sec()
        if 1793025735 <= stamp <= 1793025779:
            v=msg.pose.pose.position
            q=msg.pose.pose.orientation
            yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2]
            samples.append((stamp,v.x,v.y,v.z,yaw))
for tick in range(1793025737,1793025779,2):
    t,x,y,z,yaw=min(samples,key=lambda s: abs(s[0]-tick))
    print(f'{tick}: x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.2f}')
