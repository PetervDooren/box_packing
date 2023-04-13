import ed
import rospy
import tf2_ros
import tf_conversions.posemath as pm
from PyKDL import Frame, Vector, dot
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Twist


def main():
    rospy.init_node("goddammit")
    rospy.loginfo("starting thing")
    wm = ed.world_model.WM()
    r = rospy.Rate(1)

    topic = 'visualization_marker_array'
    marker_publisher = rospy.Publisher(topic, MarkerArray)

    vel_publisher = rospy.Publisher('my_controller/velocity_reference', Twist)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

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
        ee_pose = Vector(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        rospy.loginfo(f"EE pose {ee_pose}")

        # difference Vector
        dpos = container.pose.frame.p - ee_pose
        rospy.loginfo(f"relative position {dpos}")

        # project distance on relevant axis (y axis of container in this case) #Note. cannot get an orientation from an ed.volume 
        normal = container.pose.frame.M * Vector(0, 1, 0)
        c = dot(dpos, normal)

        rospy.loginfo(f"normal {normal}, c {c}")

        # display relative position:
        markerArray = MarkerArray()
        
        marker2 = Marker() 
        marker2.id = 1 
        marker2.lifetime = rospy.Duration()
        marker2.header.frame_id = "map"
        marker2.type = Marker.ARROW 
        marker2.action = Marker.ADD 
        marker2.scale.x = 0.02
        marker2.scale.y = 0.04
        marker2.color.a = 1.0
        marker2.color.b = 1.0
        marker2.pose.orientation.w=1.0 
        marker2.pose.position.x = ee_pose.x() 
        marker2.pose.position.y = ee_pose.y()
        marker2.pose.position.z = ee_pose.z()
        marker2.points=[]
        # start point
        p1 = Point() 
        p1.x = 0 
        p1.y = 0 
        p1.z = 0 
        marker2.points.append(p1)
        # direction
        p2 = Point() 
        p2.x = c* normal.x() 
        p2.y = c* normal.y()
        p2.z = c* normal.z()
        marker2.points.append(p2) 
        markerArray.markers.append(marker2) #add linestrip to markerArray

        marker_publisher.publish(markerArray)

        # command velocity:
        # determine control velocity
        box_width = 0.235 - 0.05 # buffer for good measure
        K = 1.0
        v_max = 1.0
        if c > box_width/2:
            a = min(v_max, K*(c-box_width/2))
        elif c < - box_width/2:
            a = max(-v_max, K*(c+box_width/2))
        else:
            a = 0

        rospy.loginfo(f"control velocity: {a} m/s")
        
        cmd_vel = Twist()
        cmd_vel.linear.x = a*normal.x()
        cmd_vel.linear.y = a*normal.y()
        cmd_vel.linear.z = a*normal.z()

        vel_publisher.publish(cmd_vel)
          
        r.sleep()



if __name__ == '__main__':
    main()