import rospy, numpy, math
numpy.set_printoptions(threshold='nan')
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import time
import cv2

class pid:
	def __init__(self):
		# Past readings
		self.last_time = time.time()
		self.last_error = 0.0

		# Values
		self.p = 0.0
		self.i = 0.0
		self.d = 0.0

		# Gains
		self.kp = 0.7
		self.ki = 0.0
		self.kd = 0.0

		# I clamping
		self.max_i = 5.0
		self.min_i = -5.0

	def update(self, error):
		self.delta_time = time.time() - self.last_time
		self.last_time = time.time()

		# Update p
		self.p = error

		# Update i
		self.i += error * self.delta_time
		if self.i > self.max_i:
			self.i = self.max_i
		elif self.i < self.min_i:
			self.i = self.min_i

		self.d = (error - self.last_error) / self.delta_time

		return (self.p * self.kp) + (self.i * self.ki) + (self.d * self.kd)

	def clear(self):
		self.last_time = time.time()
		self.last_error = 0.0

		# Values
		self.p = 0.0
		self.i = 0.0
		self.d = 0.0

	def printPID(self):
		rospy.loginfo("P: %s  I: %s  D: %s", self.p, self.i, self.d)

		# print("P:\t" + str(self.p) + "\tI:\t" str(self.i) + "\tD\t" + str(self.d))

def og_callback(data):
	global full_costmap
	global stop_zone
	global costmap_width
	global costmap_height
	global x_goal
	global y_goal
	global x_cart
	global y_cart

	x_cart = 28
	y_cart = 37
	x_goal = 28
	y_goal = 56

	rospy.loginfo("OccupancyGrid received")
	costmap_width = data.info.width
	costmap_height = data.info.height
	# rospy.loginfo("width %s",costmap_width)
	# rospy.loginfo("height %s",costmap_height)

	stop_zone = numpy.zeros((data.info.height, data.info.width))
	stop_zone[26:30, 30:39] = 1		# just under 2m in front
	full_costmap = numpy.reshape(numpy.array(data.data), (data.info.height, data.info.width))


def og_update_callback(data):
	global velocity_publisher
	global steering_pid
	vel_msg = Twist()
	obstacle_weight = 0.02
	goal_weight = 2
	influence_radius = costmap_height / 2

	partial_costmap = numpy.reshape(numpy.array(data.data), (data.height, data.width))
	full_costmap[data.y:data.y + data.height, data.x:data.x + data.width] = partial_costmap

	stop_zone_occupied = True

	if sum(sum(numpy.multiply(stop_zone, full_costmap))) == 0:
		stop_zone_occupied = False
		rospy.loginfo("stop_zone_clear")

	# full_costmap[26:30, 26:34] = 255
	# full_costmap[23:33, 30:39] = 50
	# cv2.imwrite("costmap.png", numpy.flip(numpy.rot90(full_costmap, 1), 1))

	dx = 0
	dy = 0

	if (not stop_zone_occupied):
		for x in range(costmap_width):
			for y in range(34, costmap_height):
				if full_costmap[x, y] != 0:
					distance = math.sqrt(math.pow(x-x_cart, 2) + math.pow(y-y_cart, 2))
					if distance < influence_radius:
						angle = math.atan2(y-y_cart, x_cart-x)
						dx += -obstacle_weight*(influence_radius-distance)*math.cos(angle)
						dy += -obstacle_weight*(influence_radius-distance)*math.sin(angle)
		distance = math.sqrt(math.pow(x_goal-x_cart, 2) + math.pow(y_goal-y_cart, 2))
		dy += goal_weight*distance 			# assuming goal directly in front of cart

		# v = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))	# velocity
		steering_angle = math.fabs(math.atan2(dy, dx))

		vel_msg.linear.x = 1.5
		new_angular = (math.degrees(steering_angle) - 90) / 10. / 4.
		vel_msg.angular.z = steering_pid.update(new_angular)

		# rospy.loginfo("steering angle: %s", math.degrees(steering_angle) - 90)
		rospy.loginfo("steering angle: %s", vel_msg.angular.z)
		steering_pid.printPID()



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

    steering_pid = pid()

    full_costmap = []
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

  except rospy.ROSInterruptException:
    pass

