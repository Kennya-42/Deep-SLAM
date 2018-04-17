import rospy, numpy, math
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
	global front_zone_max
	global left_zone_max
	global right_zone_max

	rospy.loginfo("OccupancyGrid received")

	stop_zone = numpy.zeros((data.info.height, data.info.width))
	front_zone = numpy.zeros((data.info.height, data.info.width))
	left_zone = numpy.zeros((data.info.height, data.info.width))
	right_zone = numpy.zeros((data.info.height, data.info.width))

	stop_zone[26:30, 30:39] = 1
	# front_zone[24:32, 39:57] = 1
	# left_zone[32:56, 30:57] = 1
	# right_zone[0:24, 30:57] = 1

	for x in range(0,57):
		for y in range(30,57):
			if x < 24:
				right_zone[x, y] = (x + 1)*(57 - y)
			elif x < 32:
				front_zone[x, y] = (57 - y) # *(57 - y)
			else:
				left_zone[x, y] = (57 - x)*(57 - y)

	front_zone_max = sum(sum(front_zone))
	left_zone_max = sum(sum(left_zone))
	right_zone_max = sum(sum(right_zone))

	rospy.loginfo("front max %s", front_zone_max)
	rospy.loginfo("left max %s", left_zone_max)
	rospy.loginfo("right max %s", right_zone_max)
	# rospy.loginfo("front zone %s", front_zone)
	# rospy.loginfo("left zone %s", left_zone)
	# rospy.loginfo("right zone %s", right_zone)
	full_costmap = numpy.reshape(numpy.array(data.data), (data.info.height, data.info.width))


def og_update_callback(data):
	global velocity_publisher
	vel_msg = Twist()

	partial_costmap = numpy.reshape(numpy.array(data.data), (data.height, data.width))
	full_costmap[data.y:data.y + data.height, data.x:data.x + data.width] = partial_costmap

	front_weight = 0
	left_weight = 0
	right_weight = 0

	stop_zone_occupied = True

	if sum(sum(numpy.multiply(stop_zone, full_costmap))) == 0:
		stop_zone_occupied = False
		rospy.loginfo("stop_zone_clear")

	front_weight = 2*sum(sum(numpy.multiply(front_zone, full_costmap)))/front_zone_max
	left_weight = sum(sum(numpy.multiply(left_zone, full_costmap)))/left_zone_max
	right_weight = sum(sum(numpy.multiply(right_zone, full_costmap)))/right_zone_max
	rospy.loginfo("front weight %s", front_weight)
	rospy.loginfo("left weight %s", left_weight)
	rospy.loginfo("right weight %s", right_weight)

	# full_costmap[26:30, 26:34] = 255		# cart area
	# full_costmap[23:33, 30:39] = 50
	# cv2.imwrite("costmap.png", numpy.flip(numpy.rot90(full_costmap, 1), 1))

	if(stop_zone_occupied):
		vel_msg.linear.x = 0.0
		vel_msg.angular.z = 0.0
	else:
		vel_msg.linear.x = 1.7
		vel_msg.angular.z = 0.3*front_weight*(right_weight-left_weight)/100
		rospy.loginfo("steering angle: %s", vel_msg.angular.z)

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

