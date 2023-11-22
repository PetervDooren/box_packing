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


class PlanExecutive():
    def __init__(self) -> None:
        self.solvers = []
        self.skills = []
        self.plan = None

        self._register_skills

    def _register_solvers(self) -> None:
        pass

    def _register_skills(self) -> None:
        #TODO hardcoded skills
        self.skills = ["move_to_cartesian", "control_to_pose", "close_gripper", "move_out_of_contact"]
        pass

    def check_plan(self, plan) -> bool:
        error_msg = []
        plan_valid = True
        for subtask in plan["subtasks"]:
            subtask_skill = plan["subtasks"][subtask]["type"]
            if not subtask_skill in self.skills:
                plan_valid = False
                error_msg.append(f"subtask: {subtask} is of unknown type {subtask_skill}")

        print(error_msg)
        return plan_valid



    def set_plan(self, plan) -> bool:
        #check plan
        self.check_plan(plan)

        self.plan = plan
        return True

    def run(self) -> None:
        if not self.plan:
            rospy.logerror()
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
