import typing
from typing import Callable, List


import rospy
from PyKDL import Frame, Vector, Rotation, dot
from geometry_msgs.msg import Twist

from .region import BoxRegion

counter = 0

"""
MoveToRegion: calculate a velocity setpoint to move the end effector in the direction of a region.
implements the concept MoveTo( ?region )

@param eeposefunc: callable function giving the pose of the end effector
@param regionfunc: function returning the boxregion to move to
@return float[3] containing [velocity_x, velocity_y, velocity_z]
"""
def MoveToRegion(eeposefunc: Callable[[], Frame], regionfunc: Callable[[], BoxRegion], vmax: float) -> List[float]:
    ee_pose = eeposefunc()
    if not ee_pose:
        rospy.logwarn(f"MoveToRegion: could not get ee_pose, instead got {ee_pose}")
        return [0, 0, 0]

    region = regionfunc()
    if not region:
        rospy.logwarn(f"MoveToRegion: could not find region, instead got {region}")
    dpos = region.getCenterPoint() - ee_pose.p

    # ignore height if high enough
    if ee_pose.p.z() > region.getMin().z():
        dpos.z(0)

    #TODO hardcoded gain
    vel = dpos * 5.0
    if vel.Norm() > vmax:
        vel = vel*vmax/vel.Norm()
    return vel

"""
MiddleOver: calculate a line segment in the center of the region directly over the box.
implements the concept Middle ( Over(CONTAINER))

@param entityfunc: resolves to the entity with respect to which the Over region is defined.
@return BoxRegion, or None
"""
def Middle(entityfunc: Callable[[], BoxRegion]) -> List[float]:
    over = entityfunc()
    if not over:
        rospy.logwarn(f"MiddleOver: could not get 'over' region, instead got {over}")
        return None

    middleLine = BoxRegion(over.getCenterFrame(),
                            0,
                            0,
                            over.dz)
    return middleLine

"""
MiddleOver: calculate a line segment in the center of the region directly over the box.
implements the concept Over(?)

@param entityfunc: resolves to the entity with respect to which the Over region is defined.
@return BoxRegion, or None
"""
def Over(entityfunc: Callable[[], BoxRegion]) -> List[float]:
    region = entityfunc()
    if not region:
        rospy.logwarn(f"Over: could not get region, instead got {region}")
        return None

    # hardcoded dimensions for now
    region_height = 1.0 # z

    middleframe = region.getCenterFrame()
    middleframe.p.z(region.getMax().z() + region_height/2)
    over_region = BoxRegion(middleframe,
                            region.dx,
                            region.dy,
                            region_height)
    return over_region

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
def MoveAgainst(entityfunc: Callable[[], Frame]):
    container = entityfunc()
    if not container:
        rospy.logwarn(f"could not get object poses: container: {container}")
        return False
    # get the coordinates of dpos in the frame of the container
    y_container_map = container.getRotation() * Vector(0, 1, 0)

    velocity = 0.2*[y_container_map.x, y_container_map.y, y_container_map.z]
    return velocity

"""
InRegionOver: calculate whether the box is fully within the over region of the container
implements the concept of InRegion(BOX, Over(CONTAINER))

@return true: box is within the region
"""
def InRegion(eeposefunc: Callable[[], Frame], regionfunc: Callable[[], BoxRegion]):
    region = regionfunc()
    ee_pose = eeposefunc()
    if not region or not ee_pose:
        rospy.logwarn(f"could not get entities: region: {region}, ee_pose {ee_pose}")
        return False

    # hardcoded dimensions for now
    buffer = 0.05

    # pose of the container with respect to the end effector in box frame
    dpos = region.getRotation().Inverse() * (ee_pose.p - region.getCenterPoint()) 

    if abs(dpos.x()) > region.dx/2 - buffer:
        return False
    if abs(dpos.x()) > region.dy/2 - buffer:
        return False
    if abs(dpos.z()) > region.dz/2 - buffer:
        return False
    return True

"""
Contact: determine whether the robot has made contact with the environment.

@return True: if contact is thought to have been made.
"""
def Contact(velfunc: Callable[[], Twist]):
    global counter
    measured_vel = velfunc()

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