import ed
import rospy
from PyKDL import Vector, dot
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

def main():
    rospy.init_node("goddammit")
    rospy.loginfo("starting thing")
    wm = ed.world_model.WM()
    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        container = wm.get_entity("cardboard_box")
        product = wm.get_entity("rice1")

        rospy.loginfo(f"container {container}, product {product}")

        # get volume poses
        container_wall_v = container.volumes["wall1"].center_point
        product_wall_v = product.volumes["wall1"].center_point

        rospy.loginfo(f"container wall {container_wall_v}, product wall {product_wall_v}")

        cwall_map = container.pose.frame * container_wall_v
        pwall_map = product.pose.frame * product_wall_v

        rospy.loginfo(f"in map frame: container wall {cwall_map}, product wall {pwall_map}")
        # get distance between things.
        dpos = pwall_map - cwall_map
        rospy.loginfo(f"relative pose {dpos}")

        # project distance on relevant axis (y axis of container in this case) #Note. cannot get an orientation from an ed.volume 
        normal = container.pose.frame.M * Vector(0, 1, 0)
        c = dot(dpos, normal)

        rospy.loginfo(f"normal {normal}, c {c}")

        # display relative position:
        topic = 'visualization_marker_array'
        publisher = rospy.Publisher(topic, MarkerArray)
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
        marker2.pose.position.x = cwall_map.x() 
        marker2.pose.position.y = cwall_map.y()
        marker2.pose.position.z = cwall_map.z()
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

        publisher.publish(markerArray)
        
        r.sleep()



if __name__ == '__main__':
    main()