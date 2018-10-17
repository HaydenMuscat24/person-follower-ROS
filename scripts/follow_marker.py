import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
import math
from math import atan2

my_x = 0.0
my_y = 0.0
my_theta = 0.0
goal_x = 0.0
goal_y = 0.0
paused = True
drive = 1
turn_left = True


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

	print "New marker received"

	goal_x = m.pose.position.x
	goal_y = m.pose.position.y
	return


def newCommand(c):
	global paused
	print(c)
	if c.data == "start":
		paused = False
	elif c.data == "stop":
		paused = True;

	return


def newScan(scan):
	global drive

	ROBOT_RADIUS      = 0.20
	SLOWDOWN_DIST     = 2.0
	MAX_SPEED         = 0.5
	MAX_TURN          = 0.5
	MIN_APPROACH_DIST = 1.0

	distance = 1000000
	laser_angle = scan.angle_min
	n = 0

	while (n < len(scan.ranges)):
		if (-2 < laser_angle % 360 < 2):
			distance = scan.ranges[n]
			break

		laser_angle += scan.angle_increment
		n += 1



	if 0.01 < distance < SLOWDOWN_DIST:
		drive = (distance - MIN_APPROACH_DIST) / (SLOWDOWN_DIST - MIN_APPROACH_DIST)

		if drive < -0.1:
			drive = -0.1
	else:
		drive = 1

	return


def newAngle(a):
	global turn_left

	if a.data < 0:
		turn_left = True
	else:
		turn_left = False

	return


rospy.init_node("speed_controller")
odomSub 	= rospy.Subscriber("/odom", Odometry, newOdom)
markerSub 	= rospy.Subscriber("/marker", Marker, newMarker, queue_size=1)
commandSub 	= rospy.Subscriber("cmd", String, newCommand, queue_size=1)
scanSub 	= rospy.Subscriber("/scan", LaserScan, newScan, queue_size=1)
angleSub	= rospy.Subscriber("/angle", Float32, newAngle, queue_size=1)
twistPub 	= rospy.Publisher('cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(10)

print "Starting main loop"

while not rospy.is_shutdown():
	if paused:
		speed.linear.x = 0.0
		speed.angular.z = 0.0
		twistPub.publish(speed)
		r.sleep()
		continue

	remaining_x = goal_x - my_x
	remaining_y = goal_y - my_y

	if sqrt(remaining_x * remaining_x + remaining_y * remaining_y) < 0.4:
		if turn_left:
			speed.linear.x = 0.0
			speed.angular.z = 0.1
		else:
			speed.linear.x = 0.0
			speed.angular.z = -0.1
	else:
		angle_to_goal = atan2(remaining_y, remaining_x)

		if abs(angle_to_goal - my_theta) > 0.2:

            #clips angle to between -1 and 1
            angle = angle_to_goal - my_theta
            angle /= 2
            if (abs(angle) > 1):
                angle /= abs(angle)

			speed.linear.x = 0.0
			speed.angular.z = angle
		else:
			print "Driving forwards"
			speed.linear.x = 0.5 * drive
			speed.angular.z = 0.0

	twistPub.publish(speed)
	r.sleep()
