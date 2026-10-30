#!/usr/bin/env python3
"""Simulator mission watchdog: abort and request AUTO.LAND on boundary/timeout.

Uses the same /rflysim/uav1/local/odom frame as mission_pkg safety checks.
It never enlarges/disables the configured safety boundary.
"""
import json
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Supervisor:
    def __init__(self):
        rospy.init_node('cicc_mission_safety_supervisor', anonymous=False)
        self.state = None
        self.odom = None
        self.mission_started = None
        self.violating_since = None
        self.recovery_sent = False
        self.landing_started = None
        self.grace = float(rospy.get_param('~boundary_grace_sec', 0.25))
        self.max_mission_sec = float(rospy.get_param('~max_mission_sec', 210.0))
        self.landing_grace_sec = float(rospy.get_param('~landing_grace_sec', 20.0))
        self.limits = {key:float(rospy.get_param('safety/'+key,default))
                       for key,default in [('x_min',-1),('x_max',6),
                                           ('y_min',-3),('y_max',3),
                                           ('z_min',-0.1),('z_max',2.5)]}
        self.abort_pub = rospy.Publisher('/mission_abort',String,queue_size=1)
        rospy.Subscriber('/mavros/state',State,self.on_state,queue_size=1)
        rospy.Subscriber('/rflysim/uav1/local/odom',Odometry,self.on_odom,queue_size=1)
        rospy.Subscriber('/mission_command',String,self.on_command,queue_size=1)
        rospy.Subscriber('/mission_result',String,self.on_result,queue_size=1)
        self.land=rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)
        rospy.loginfo('Supervisor boundary=%s timeout=%.1fs',json.dumps(self.limits),self.max_mission_sec)

    def on_state(self,msg):
        self.state=msg
        if msg.armed and msg.mode == 'AUTO.LAND':
            if self.landing_started is None:
                self.landing_started = time.monotonic()
        else:
            self.landing_started = None
        if not msg.armed and self.recovery_sent:
            rospy.loginfo('Recovery verified: disarmed')
            self.mission_started=None
            self.recovery_sent=False

    def on_odom(self,msg):
        self.odom=msg

    def on_command(self,_msg):
        self.mission_started=time.monotonic()
        self.landing_started=None
        self.recovery_sent=False
        self.violating_since=None

    def on_result(self,msg):
        failed = msg.data in ('FAILURE','ABORTED') or msg.data.startswith('ERROR:')
        if failed and self.state and self.state.armed:
            self.recover('mission_'+msg.data[:60])
        if msg.data in ('SUCCESS','FAILURE','ABORTED') or msg.data.startswith('ERROR:'):
            self.mission_started=None

    def out_of_bounds(self):
        if self.odom is None:return False
        p=self.odom.pose.pose.position
        l=self.limits
        return not(l['x_min']<=p.x<=l['x_max'] and
                   l['y_min']<=p.y<=l['y_max'] and
                   l['z_min']<=p.z<=l['z_max'])

    def recover(self,reason):
        if self.recovery_sent:return
        self.recovery_sent=True
        p=self.odom.pose.pose.position if self.odom else None
        rospy.logerr('SAFETY RECOVERY: %s; sim_odom=%s',reason,
                     (p.x,p.y,p.z) if p else None)
        self.abort_pub.publish(String(data=reason))
        try:
            rospy.wait_for_service('/mavros/cmd/land',timeout=4)
            result=self.land(0.0,0.0,0.0,0.0,0.0)
            rospy.logerr('AUTO.LAND service success=%s result=%s',result.success,result.result)
        except Exception as exc:
            rospy.logerr('AUTO.LAND service failed: %s',exc)

    def run(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            now=time.monotonic()
            if self.state and self.state.armed and not self.recovery_sent:
                if self.out_of_bounds():
                    if self.violating_since is None:self.violating_since=now
                    if now-self.violating_since>=self.grace:
                        self.recover('out_of_bounds')
                else:
                    self.violating_since=None
                if self.mission_started and now-self.mission_started>self.max_mission_sec:
                    # Let an already initiated, near-home landing finish. The
                    # arena boundary checks above remain active throughout.
                    p = self.odom.pose.pose.position if self.odom else None
                    near_home = p and abs(p.x)<0.35 and abs(p.y)<0.35 and p.z<1.8
                    landing = (self.landing_started is not None and
                               now-self.landing_started<self.landing_grace_sec)
                    if not (near_home and landing):
                        self.recover('mission_timeout')
            rate.sleep()


if __name__=='__main__':Supervisor().run()
