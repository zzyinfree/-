#!/usr/bin/env python3
"""Verify invalid command is rejected before any mission execution."""
import threading
import time
import rospy
from std_msgs.msg import String

rospy.init_node('cicc_invalid_command_guard_test', anonymous=True, disable_signals=True)
seen=threading.Event()
result=[]
def cb(msg):
    result.append(msg.data)
    seen.set()
rospy.Subscriber('/mission_result',String,cb,queue_size=5)
pub=rospy.Publisher('/mission_command',String,queue_size=1,latch=True)
deadline=time.monotonic()+10
while pub.get_num_connections()<1 and time.monotonic()<deadline:time.sleep(.1)
if pub.get_num_connections()<1:raise RuntimeError('mission not connected')
pub.publish(String(data='{"commands":["takeoff","hit_balloon:green","land"]}'))
if not seen.wait(8):raise TimeoutError('no mission_result')
print('result=',result[0],flush=True)
if result[0]!='ERROR:INVALID_COMMAND':raise AssertionError(result)
