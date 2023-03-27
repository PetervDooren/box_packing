from dataclasses import dataclass
from enum import Enum

import rospy


# feature functions
class Feature_function(Enum):
    DISTANCE = 1

@dataclass
class Constraint:
    F1: int # ED reference uuid
    F2: int
    Ffunc: Enum
    value_min: float
    value_max: float

@dataclass
class Position:
    x: float
    y: float
    z: float
        
@dataclass
class Box:
    width: float # x
    length: float # y
    height: float # z

'''
Box packing demo.

goal: move an end effector using constraints.

simplifying assumptions:
- only distance constraints
- hardcode worldmodel

'''
def main():

    # Define set of constraints
    above_constraint = Constraint(1, 2, Feature_function.DISTANCE, 0, 100.0)

    while not rospy.is_shutdown:
        # filter active constraints

        # create interaction matrix M

        # check M for conflicts

        # calculate inverse

        # robot control law

        # send joint velocities
