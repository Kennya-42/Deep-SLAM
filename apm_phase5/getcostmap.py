import rospy, numpy
numpy.set_printoptions(threshold='nan')
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import cv2


def og_callback(data):
	global full_costmap
	global stop_zone
	global front_zone
	global left_zone
	global right_zone

	rospy.loginfo("OccupancyGrid received")
	# rospy.loginfo("width %s",data.info.width)
	# rospy.loginfo("height %s",data.info.height)

	stop_zone = numpy.zeros((data.info.height, data.info.width))
	front_zone = numpy.zeros((data.info.height, data.info.width))
	left_zone = numpy.zeros((data.info.height, data.info.width))
	right_zone = numpy.zeros((data.info.height, data.info.width))

	stop_zone[26:30, 30:39] = 1		# 1m to sides, just under 2m in front
	front_zone[24:32, 39:57] = 1 	# Continued from stopzone to before end of o.g.
	left_zone[30:36, 30:40] = 1
	right_zone[20:26, 30:40] = 1
	# rospy.loginfo("stop_zone %s", stop_zone)
	full_costmap = numpy.reshape(numpy.array(data.data), (data.info.height, data.info.width))
	# rospy.loginfo("full_costmap %s", full_costmap * 2.5)


def og_update_callback(data):
	global velocity_publisher
	vel_msg = Twist()

	# rospy.loginfo("I heard %s",data.data)
	# rospy.loginfo("New map received")
	# rospy.loginfo("width %s",data.width)
	# rospy.loginfo("height %s",data.height)
	# rospy.loginfo("x %s",data.x)
	# rospy.loginfo("y %s",data.y)

	partial_costmap = numpy.reshape(numpy.array(data.data), (data.height, data.width))
	full_costmap[data.y:data.y + data.height, data.x:data.x + data.width] = partial_costmap

	stop_zone_occupied = True
	front_zone_occupied = True
	left_zone_occupied = True
	right_zone_occupied = True

	if sum(sum(numpy.multiply(stop_zone, full_costmap))) == 0:
		stop_zone_occupied = False
		rospy.loginfo("stop_zone_clear")
	if sum(sum(numpy.multiply(front_zone, full_costmap))) == 0:
		rospy.loginfo("front_zone_clear")
		front_zone_occupied = False
	if sum(sum(numpy.multiply(left_zone, full_costmap))) == 0:
		rospy.loginfo("left_zone_clear")
		left_zone_occupied = False
	if sum(sum(numpy.multiply(right_zone, full_costmap))) == 0:
		rospy.loginfo("right_zone_clear")
		right_zone_occupied = False

	full_costmap[26:30, 26:34] = 255
	full_costmap[23:33, 30:39] = 50
	# cv2.imwrite("costmap.png", numpy.flip(numpy.rot90(full_costmap, 1), 1))

	if(not stop_zone_occupied):
		vel_msg.linear.x = 1.7
		vel_msg.angular.z = 0
		if(not front_zone_occupied):
			vel_msg.linear.x = 1.9
			vel_msg.angular.z = 0
		elif(not left_zone_occupied):
			vel_msg.linear.x = 1.0
			vel_msg.angular.z = 0.3
		elif(not right_zone_occupied):
			vel_msg.linear.x = 1.0
			vel_msg.angular.z = -0.3
	else:
		vel_msg.linear.x = 0.0

	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	

	velocity_publisher.publish(vel_msg)


    
if __name__ == '__main__': 
  try:
    rospy.init_node('node_name')
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, og_callback)
    rospy.Subscriber("/costmap_node/costmap/costmap_updates", OccupancyGridUpdate, og_update_callback)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    full_costmap = []
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

  except rospy.ROSInterruptException:
    pass

