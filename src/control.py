#!/usr/bin/python3

import rospy
import tf
from first.srv import GetNextDestination, GetNextDestinationResponse, GetNextDestinationRequest
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians
import math

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = .5
        self.angular_speed = .5
        self.goal_angle = 0
        self.stop_distance = .5
        self.epsilon = .1

        self.prev_x = 0
        self.prev_y = 0

        self.next_x = self.get_next_destination().next_x
        self.next_y = self.get_next_destination().next_y

        self.main_distance = math.sqrt(math.pow(self.prev_x - self.next_x, 2) + math.pow(self.prev_y - self.next_y, 2))
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.ROTATE
         
    def get_next_destination(self):
        
        rospy.wait_for_service('mammad')

        try:
            GetNext = rospy.ServiceProxy('mammad', GetNextDestination)
            a = int(self.prev_x)
            b = int(self.prev_y)
            next_dest = GetNext(a, b)
            return next_dest
        except rospy.ServiceException as e:
            self.get_next_destination()
    
    def goal_ajacancy(self):

        return math.sqrt(math.pow(self.prev_x - self.get_position().x, 2) + math.pow(self.prev_y - self.get_position().y, 2))
    
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw
    
    def get_destination_heading(self):

        deltay = self.next_y - self.get_position().y
        deltax = self.next_x - self.get_position().x
        
        return math.atan2(deltay, deltax)
    
    def get_position(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        p = msg.pose.pose.position
        
        # convert quaternion to odom
        
        
        return p

    def run(self):
        
        while not rospy.is_shutdown():

            if self.state == self.GO :

                twist = Twist()

                twist.linear.x = self.linear_speed

                self.cmd_publisher.publish(twist)
                print(self.goal_ajacancy())
                
                if abs(self.goal_ajacancy() - self.main_distance) < 1 :
                    print("Ok")
                    self.prev_x = self.get_position().x
                    self.prev_y = self.get_position().y

                    self.next_x = self.get_next_destination().next_x
                    self.next_y = self.get_next_destination().next_y

                    self.main_distance = math.sqrt(math.pow(self.prev_x - self.next_x, 2) + math.pow(self.prev_y - self.next_y, 2))

                    self.state = self.ROTATE

                    self.cmd_publisher.publish(Twist())

                    rospy.sleep(1)

                    continue
            
            if self.state == self.ROTATE :
            
                twist = Twist()

                twist.angular.z = self.angular_speed

                self.cmd_publisher.publish(twist)

                if abs(self.get_heading() - self.get_destination_heading()) < self.epsilon :

                    self.state = self.GO

                    self.cmd_publisher.publish(Twist())

                    rospy.sleep(2)

                    continue




           


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()