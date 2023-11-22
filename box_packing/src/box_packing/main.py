#!/usr/bin/env python
import typing
from typing import Callable, List

import ed
import rospy
import numpy
import json
import tf2_ros
import tf_conversions.posemath as pm
from PyKDL import Frame, Vector, Rotation, dot
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Twist

from box_packing.skills.motion_skills import MoveToRegion, Middle, Over, InRegion, MoveInto, Contact
from box_packing.skills.region import BoxRegion

from plan_executive import PlanExecutive

# Global variables
wm = None
tfBuffer = None
measured_vel = None
counter = 0

class GuardedMotion:
    def __init__(self, motion, guard):
        self.motion_func = motion
        self.guard_func = guard

    def motion(self):
        if not self.motion_func:
            return None
        return self.motion_func()

    def guard(self):
        if not self.guard_func:
            rospy.logerr("No guard function provided to guarded motion")
            return False
        return self.guard_func()

class Plan:
    def __init__(self):
        self.guarded_motions = {"over": GuardedMotion(lambda: MoveToRegion(lambda:getEEPose(),
                                                                           lambda: Middle(lambda: Over(lambda: getContainer())),
                                                                           vmax=0.5),
                                                      lambda: InRegion(lambda:getEEPose(),
                                                                       lambda: Over(lambda: getContainer()))
                                                      ),
                                "into": GuardedMotion(lambda: MoveInto(),
                                                      lambda: Contact(lambda: getVelocity())),
                                "against": GuardedMotion(None,
                                                         lambda: Contact(lambda: getVelocity())),
                                "align": GuardedMotion(None,
                                                       lambda: False)}
        self.transitions = {"over" : "into",
                            "into" : "against",
                            "against": "DONE"}
        self.starting_motions = ["over", "align"]

def franka_callback(msg):
    global measured_vel
    measured_vel = msg

'''
Box packing demo.

goal: Define a plan as guarded motions. then configure the motions

'''
def main():
    rospy.init_node("goddammit")
    rospy.loginfo("starting thing")
    r = rospy.Rate(10)

    topic = 'visualization_marker_array'
    marker_publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

    vel_publisher = rospy.Publisher('my_controller/velocity_reference', Twist, queue_size=1)

    plan_executive = PlanExecutive()

    # load plan
    f = open("skills/pickup.json")
    plan = json.load(f)

    # print plan
    print(plan)

    plan_executive.set_plan(plan)

    rospy.loginfo("Goodbye")

if __name__ == '__main__':
    main()