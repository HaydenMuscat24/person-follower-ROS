import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from math import atan2

my_x = 0.0
my_y = 0.0
my_theta = 0.0
goal_x = 0.0
goal_y = 0.0
paused = True
drive = 1


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

	goal_x = m.pose.position.x
	goal_y = m.pose.position.y
	return


def newCommand(c):
	global paused

	if c == "start":
		paused = False
	elif c == "stop":
		paused = True;

	return


def newScan(scan):
	global drive

	SLOWDOWN_DIST     = 2.0
	MIN_APPROACH_DIST = 0.5

	distance = 1000000
	laser_angle = scan.angle_min
	n = 0

	while n < len(scan.points):
		if (-2 < laser_angle % 360 < 2):
			distance = scan.ranges[n]

		laser_angle += scan.angle_increment
		n += 1

	if distance < SLOWDOWN_DIST:
		drive = (distance - MIN_APPROACH_DIST) / (SLOWDOWN_DIST - MIN_APPROACH_DIST)

		if drive < -0.1:
			drive = -0.1
	else:
		drive = 1

	return


rospy.init_node("speed_controller")
odomSub 	= rospy.Subscriber("/odom", Odometry, newOdom)
markerSub 	= rospy.Subscriber("/marker", Marker, newMarker, queue_size=1)
commandSub 	= rospy.Subscriber("/cmd", String, newCommand, queue_size=1)
scanSub 	= rospy.Subscriber("/scan", LaserScan, newScan, queue_size=1)
twistPub 	= rospy.Publisher('cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(4)

while not rospy.is_shutdown():
	if paused:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		twistPub.publish(speed)
		r.sleep()
		continue

	remaining_x = goal_x - my_x
	remaining_y = goal_y - my_y

	if my_x - 0.2 < goal_x and goal_x < my_x + 0.2 and my_y - 0.2 < goal_y and goal_y < my_y + 0.2:
		speed.linear.x = 0.0
		speed.angular.z = 0.5
	else:
		angle_to_goal = atan2(remaining_y, remaining_x)

		if abs(angle_to_goal - my_theta) > 0.1:
			speed.linear.x = 0.0
			speed.angular.z = 0.1
		else:
			speed.linear.x = 0.5 * drive
			speed.angular.z = 0.0

	twistPub.publish(speed)
	r.sleep()