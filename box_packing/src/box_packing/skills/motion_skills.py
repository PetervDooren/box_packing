import typing
from typing import Callable, List


import rospy
from PyKDL import Frame, Vector, Rotation, dot

"""
MoveToRegion: calculate a velocity setpoint to move the end effector in the direction of a region.
implements the concept MoveTo( ?region )

@return float[3] containing [velocity_x, velocity_y, velocity_z]
"""
def MoveToRegion(regionfunc: Callable[[], List[float]], vmax: float) -> List[float]:
    ee_pose = getBoxPose()
    if not ee_pose:
        rospy.logwarn(f"MoveToRegion: could not get ee_pose, instead got {ee_pose}")
        return [0, 0, 0]

    region = regionfunc()
    if not region:
        rospy.logwarn(f"MoveToRegion: could not find region, instead got {region}")
    region_v = Vector(region[0], region[1], region[2])
    dpos = region_v - ee_pose.p
    # ignore height if high enough #hack assumes region_z is lower bound
    dpos.z( max(dpos.z(), 0) )

    vel = dpos * 5.0
    if vel.Norm() > vmax:
        vel = vel*vmax/vel.Norm()
    return vel

"""
MiddleOver: calculate a line segment in the center of the region directly over the box.
implements the concept Middle ( Over(CONTAINER))

@return float[3] containing [x, y, z_min], or None
"""
def MiddleOver() -> List[float]:
    container = getContainer()
    if not container:
        rospy.logwarn(f"could not get object pose: container: {container}")
        return None

    # hardcoded dimensions for now
    container_height = 0.11 # z

    # hack, uses the fact that the pose is represented with respect the pose of the center of the container
    return [container.pose.frame.p.x(), container.pose.frame.p.y(), container.pose.frame.p.z() + container_height]

"""
MoveInto: give a velocity into the box. In this usecase, always down.
implements the concept MoveInto(CONTAINER)

@return double[3] containing [velocity_x, velocity_y, velocity_z]
"""
def MoveInto():
    return [0, 0, -0.2]


"""
MoveInto: give a velocity into the side of the box. In this usecase, we always move in the y direction of the box.
implements the concept MoveAgainst( Side( CONTAINER))

@return double[3] containing [velocity_x, velocity_y, velocity_z]
"""
def MoveAgainst():
    container = getContainer()
    if not container:
        rospy.logwarn(f"could not get object poses: container: {container}")
        return False
    # get the coordinates of dpos in the frame of the container
    y_container_map = container.pose.frame.M * Vector(0, 1, 0)

    velocity = 0.2*[y_container_map.x, y_container_map.y, y_container_map.z]
    return velocity

"""
InRegionOver: calculate whether the box is fully within the over region of the container
implements the concept of InRegion(BOX, Over(CONTAINER))

@return true: box is within the region
"""
def InRegionOver():
    container = getContainer()
    ee_pose = getBoxPose()
    if not container or not ee_pose:
        rospy.logwarn(f"could not get object poses: container: {container}, ee_pose {ee_pose}")
        return False

    # hardcoded dimensions for now
    container_length = 0.22 # x
    container_width = 0.235 # y
    container_height = 0.11 # z
    buffer = 0.05

    # pose of the container with respect to the end effector
    dpos = container.pose.frame.p - ee_pose.p

    # get the coordinates of dpos in the frame of the container
    x_container_map = container.pose.frame.M * Vector(1, 0, 0)
    y_container_map = container.pose.frame.M * Vector(0, 1, 0)
    z_container_map = container.pose.frame.M * Vector(0, 0, 1)

    dpos_x = dot(dpos, x_container_map)
    dpos_y = dot(dpos, y_container_map)
    dpos_z = dot(dpos, z_container_map)
    if abs(dpos_x) > container_length/2:
        return False
    if abs(dpos_y) > container_width/2:
        return False
    if dpos_z > -container_height:
        return False
    return True

"""
Contact: determine whether the robot has made contact with the environment.

@return True: if contact is thought to have been made.
"""
def Contact():
    global counter
    global measured_vel

    if not measured_vel:
        return False
    
    if measured_vel.linear.z > -0.01:
        counter += 1
    else:
        counter = 0
    
    if counter > 10:
        counter = 0
        return True
    return False