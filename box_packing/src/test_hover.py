import ed
import rospy
import numpy
import tf2_ros
import tf_conversions.posemath as pm
from PyKDL import Frame, Vector, Rotation, dot
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Twist

"""
get the value of a relative position constraint and its direction in world frame.

:param ref_pose: pose expressed in world frame
:param pose2: pose that should be compared to ref_pose, expressed in world frame
:param direction: direction along which the poses should be compared. expressed relative to ref_pose

returns Tuple of the relative distance along the direction and the direction in world frame.
"""
def getRelPosition(ref_pose, pose2, direction):

    # difference Vector
    dpos = ref_pose.p - pose2.p
    #rospy.loginfo(f"relative position {dpos}")
    
    # project distance on relevant axis (y axis of container in this case) #Note. cannot get an orientation from an ed.volume 
    normal = ref_pose.M * -direction # normal = dC/dpee
    c = dot(dpos, normal)

    return c, normal

"""
Controller with dead zone to control a given constraint

:param c: current value of the constraint
:param range: desired range of the constraint in format [c_min, c_max]
:param K: controller gain
:dy_max: control saturation
"""
def getConstraintVel(c, range, K, dy_max):
    if c < range[0]:
        dy = K*(c - range[0])
    elif c > range[1]:
        dy = K*(c - range[1])
    else: # c between range[0] and range[1]
        dy = 0
        return dy
    return min(max(dy, -dy_max), dy_max)

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

def main():
    rospy.init_node("goddammit")
    rospy.loginfo("starting thing")
    wm = ed.world_model.WM()
    r = rospy.Rate(10)

    topic = 'visualization_marker_array'
    marker_publisher = rospy.Publisher(topic, MarkerArray)

    vel_publisher = rospy.Publisher('my_controller/velocity_reference', Twist)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # define constraints
    constraint_directions = [Vector(1, 0, 0),
                                Vector(0, 1, 0),
                                Vector(0, 0, 1),
                                Vector(0, 0, 1)]

    # ranges
    box_length = 0.22 # x
    box_width = 0.235 # y
    box_height = 0.11 # z
    buffer = 0.05
    constraint_ranges = [[-box_length/2 + buffer, box_length/2 - buffer], # x
                            [-box_width/2 + buffer, box_width/2 - buffer], # y
                            [box_height, 1.0], # z
                            [0.0, box_height]] # z
    n_constraints = len(constraint_directions)

    plan = [[0, 1, 2],
            [0, 1, 3]]
    current_step = 0

    while not rospy.is_shutdown():
        container = wm.get_entity("cardboard_box")
        product = wm.get_entity("rice1")

        rospy.loginfo(f"container {container}, product {product}")

        # get ee pose
        try:
            trans = tfBuffer.lookup_transform("map", 'panda_EE', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            r.sleep()
            continue
        rospy.loginfo(f"tf {trans}")
        ee_pose = tfToKDLFrame(trans)

        rospy.loginfo(f"EE pose {ee_pose}")

        dy = []
        M = []

        markerArray = MarkerArray()
        # evaluate constraints
        for i in plan[current_step]:
            if i >= n_constraints:
                rospy.logerr(f"constraint {i} requested but only {n_constraints} defined")
                return
            rospy.loginfo(f"evaluating constaint {i}")
            c, normal = getRelPosition(container.pose.frame, ee_pose, constraint_directions[i])
            #rospy.loginfo(f"normal {normal}, c {c}")
            
            # visualization
            constraint_met = c > constraint_ranges[i][0] and c < constraint_ranges[i][1]
            marker = createMarker(i, ee_pose, c*normal, constraint_met)
            markerArray.markers.append(marker)

            if constraint_met:
                continue
            K = 5.0 # treating all constraints equal for now
            dy_max = 0.5
            dyi = getConstraintVel(c, constraint_ranges[i], K, dy_max)
            
            dy.append(dyi)
            M.append([normal.x(), normal.y(), normal.z()])

        #rospy.loginfo(f"constraint velocities: {dy}")        
        #rospy.loginfo(f"interaction matrix: {M}")

        if M:
            Minv = numpy.linalg.pinv(M)
            #rospy.loginfo(f"pseudo inverse interaction matrix: {Minv}")
            dp = numpy.matmul(Minv, dy)
        else:
            # all constraints are met. continue to next step
            if current_step <= len(plan):
                current_step += 1
                rospy.loginfo(f"All constraints met, continuing to step: {current_step}")
                continue
            else:
                rospy.loginfo(f"Final step completed, maintaining constraints of step: {current_step}")
                dp = [0, 0, 0]
        rospy.loginfo(f"control velocity: {dp}")
        
        cmd_vel = Twist()
        cmd_vel.linear.x = dp[0]
        cmd_vel.linear.y = dp[1]
        cmd_vel.linear.z = dp[2]

        vel_publisher.publish(cmd_vel)

        # display constraint:
        
        

        marker_publisher.publish(markerArray)
          
        r.sleep()



if __name__ == '__main__':
    main()