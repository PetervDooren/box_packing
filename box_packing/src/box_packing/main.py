#!/usr/bin/env python
import typing
from typing import Callable, List

import ed
import rospy
import numpy
import tf2_ros
import tf_conversions.posemath as pm
from PyKDL import Frame, Vector, Rotation, dot
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Twist

from box_packing.skills.motion_skills import MoveToRegion, Middle, Over, InRegion, MoveInto, Contact
from box_packing.skills.region import BoxRegion

# Global variables
wm = None
tfBuffer = None
measured_vel = None
counter = 0

"""
translate a tf transform into a pyKDL frame representing the same transform
"""
def tfToKDLFrame(transform):
    position = Vector(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
    rotation = Rotation.Quaternion(transform.transform.rotation.x,
                                    transform.transform.rotation.y,
                                    transform.transform.rotation.z,
                                    transform.transform.rotation.w)
    return Frame(rotation, position)

def createMarker(id, origin, vector, constraint_met=False):
    marker = Marker()
    marker.id = id
    marker.lifetime = rospy.Duration()
    marker.header.frame_id = "map"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.02
    marker.scale.y = 0.04
    marker.color.a = 1.0
    if constraint_met:
        marker.color.g = 1.0
    else:
        marker.color.r = 1.0
    marker.pose.orientation.w=1.0
    marker.pose.position.x = origin.p.x()
    marker.pose.position.y = origin.p.y()
    marker.pose.position.z = origin.p.z()
    marker.points=[]
    # start point
    p1 = Point()
    p1.x = 0
    p1.y = 0
    p1.z = 0
    marker.points.append(p1)
    # direction
    p2 = Point()
    p2.x = vector.x()
    p2.y = vector.y()
    p2.z = vector.z()
    marker.points.append(p2)
    return marker

def getEEPose():
    # get ee pose
    try:
        trans = tfBuffer.lookup_transform("map", 'panda_EE', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None
    ee_pose = tfToKDLFrame(trans)
    rospy.loginfo(f"ee_pose {ee_pose.p}")
    return ee_pose

def getContainer():
    container = wm.get_entity("cardboard_box")

    container_length = 0.22 # x
    container_width = 0.235 # y
    container_height = 0.11 # z

    container.pose.frame.p.z(container.pose.frame.p.z() + container_height/2)

    return BoxRegion(container.pose.frame, container_length, container_width, container_height)

def getVelocity():
    global measured_vel
    return measured_vel

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
    marker_publisher = rospy.Publisher(topic, MarkerArray)

    vel_publisher = rospy.Publisher('my_controller/velocity_reference', Twist)

    global wm
    wm = ed.world_model.WM()
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    velocity_sub = rospy.Subscriber("my_controller/measured_velocity", Twist, franka_callback)

    # define plan
    plan = Plan()
    active_motions = plan.starting_motions

    plan_done = False

    while not rospy.is_shutdown() and not plan_done:
        v = [0,0,0]

        rospy.loginfo(f"loop start, active motions {active_motions}")

        # execute motion
        for m in active_motions:
            guarded_motion = plan.guarded_motions[m]
            velocity = guarded_motion.motion()
            if velocity:
                v = velocity
                rospy.loginfo(f"control velocity: {velocity}, determined by motion {m}")
                break # we cannot yet combine motions

        # update guards
        activated_guards = []
        for m in active_motions:
            guarded_motion = plan.guarded_motions[m]
            if guarded_motion.guard():
                activated_guards.append(m)

        # apply transitions
        for g in activated_guards:
            active_motions.remove(g) # remove the original guarded motion.
            new_gm = plan.transitions[g]
            if new_gm == "DONE":
                plan_done = True
                break
            if new_gm:
                active_motions.append(new_gm)
                rospy.loginfo(f"guard of guarded_motion {g} triggered. making transition to {new_gm}")
            else:
                rospy.loginfo(f"guard of guarded_motion {g} triggered. No further transition available")

        # communicate step
        # send command velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = v[0]
        cmd_vel.linear.y = v[1]
        cmd_vel.linear.z = v[2]

        vel_publisher.publish(cmd_vel)

        # display velocity

        ee_pose = getEEPose()
        if ee_pose:
            velocity_vector = Vector(v[0], v[1], v[2])
            marker = createMarker(0, ee_pose, velocity_vector, True)
            markerArray = MarkerArray()
            markerArray.markers.append(marker)
            marker_publisher.publish(markerArray)

        r.sleep()

    rospy.loginfo("Goodbye")

if __name__ == '__main__':
    main()