#setup to be used to test how much turning is needed by the turtle bot because weight changes with
#payload etc, can be used to test maximum allowed rotation and motor speed as well

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
import time

class Degree(Node):
	
	def __init__(self):
		super().__init__('degree')
		self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
		
	def move(self):
		a = True
		twist = Twist()
		#when running code in conseq times it works every other time sometimes
		while(a == True):
			#twist.linear.x = 0.05
			#maximum workable angular twist is 1.8
			twist.angular.z = 1.7
			#required if not the turtlebot does not pick up published twist obj
			time.sleep(0.1)
			self.publisher_.publish(twist)
			#most prob does not move for whole timesleep seconds
			time.sleep(0.82)
			a = False
		#the sleeping time is abit messed up because of delays etc need to optimise
		twist.angular.z = 0.0
		time.sleep(0.1)
		self.publisher_.publish(twist)
		twist.linear.x = 0.05
		time.sleep(0.5)
		self.publisher_.publish(twist)
		time.sleep(2)
		twist.linear.x = 0.0
		time.sleep(0.5)
		self.publisher_.publish(twist)

def main(args=None):
	rclpy.init(args=args)


	degree = Degree()
	#rclpy.spin(degree)
	degree.move()

	degree.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
