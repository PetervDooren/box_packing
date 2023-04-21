import ed
import rospy
from PyKDL import Rotation
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   7    8    9
   4    5    6
   1    2    3
/ 	: rotate counterclockwise
* 	: rotate clockwise
anything else : stop
CTRL-C to quit
"""

moveBindings = {
		'1':(-0.7,0.7,0),
		'2':(-1,0,0),
		'3':(-0.7,-0.7,0),
		'4':(0,1,0),
		'6':(0,-1,0),
		'7':(0.7,0.7,0),
		'8':(1,0,0),
		'9':(0.7,-0.7,0),
		'/':(0,0,1),
		'*':(0,0,-1)
	       }


def getKey(key_timeout):
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    dist = 0.01 # m
    drad = 0.1 # rad
    turn = rospy.get_param("~turn", 1.0)
    key_timeout = None

    wm = ed.world_model.WM()

    frame_id = "map"
    entity_id = "cardboard_box"

    # pause so tf buffer can fill
    rospy.sleep(1)
    entity = wm.get_entity(entity_id)
    posestamped = entity.pose
    _, _, yaw = posestamped.frame.M.GetRPY()
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey(key_timeout)
            if key in moveBindings.keys():
                dx = dist* moveBindings[key][0]
                dy = dist* moveBindings[key][1]
                dth = drad * moveBindings[key][2]

                posestamped.frame.p.x(posestamped.frame.p.x() + dx)
                posestamped.frame.p.y(posestamped.frame.p.y() + dy)
                yaw += dth
                posestamped.frame.M = Rotation.RPY(0,0,yaw)
                posestamped.header.stamp = rospy.Time(0)
                rospy.loginfo(f"new pose: {posestamped}")
                wm.update_entity(entity_id, frame_stamped=posestamped)

            else:
                rospy.loginfo(f"Key {key} not in movebindings, available movebindings are {moveBindings.keys()}")
                break

    except Exception as e:
        print(e)
