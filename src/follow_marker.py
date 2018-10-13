import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

my_x = 0.0
my_y = 0.0
my_theta = 0.0
goal_x = 0.0
goal_y = 0.0


def newOdom(o):
	global my_x
	global my_y
	global my_theta

	my_x = o.pose.pose.position.x
	my_y = o.pose.pose.position.y

	rot_q = o.pose.pose.orientation
	(_, _, my_theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
	return


def newMarker(m):
	global goal_x
	global goal_y

	goal_x = m.pose.pose.position.x
	goal_y = m.pose.pose.position.y
	return


rospy.init_node("speed_controller")
odomSub = rospy.Subscriber("/odom", Odometry, newOdom)
markerSub = rospy.Subscriber("/marker", Marker, newMarker)
twistPub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(4)

while not rospy.is_shutdown():
	remaining_x = goal_x - my_x
	remaining_y = goal_y - my_y

	if my_x - 1 < goal_x and goal_x < my_x + 1 and my_y - 1 < goal_y and goal_y < my_y + 1:
		speed.linear.x = 1.0
		speed.angular.z = 0.0
	else:
		angle_to_goal = atan2(remaining_y, remaining_x)

		if abs(angle_to_goal - my_theta) > 0.1:
			speed.linear.x = 0.0
			speed.angular.z = 0.1
		else:
			speed.linear.x = 0.5
			speed.angular.z = 0.0

	twistPub.publish(speed)
	r.sleep()