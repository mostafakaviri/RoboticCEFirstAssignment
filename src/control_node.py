#!/usr/bin/python3

import rospy, tf, math
from first.srv import GetNextDestination, GetNextDestinationResponse, GetNextDestinationRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from math import radians

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=True)
        
        # self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = .4 #rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = .1 #rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = 0 #radians(rospy.get_param("/controller/goal_angle")) # rad
        
        self.stop_distance = 5
        self.goal_distance = 0
        # self.epsilon = #rospy.get_param("/controller/epsilon")

        self.curent_x = 0
        self.curent_y = 0

        self.next_x = 0
        self.next_y = 0

        self.current_yaw = 0
        self.next_x = self.get_next_destination().next_x
        self.next_y = self.get_next_destination().next_y
        self.get_destination_heading()
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.ROTATE
         
    def get_destination_heading(self):

        deltay = self.next_y - self.curent_y
        deltax = self.next_x - self.curent_x
        
        self.goal_angle = math.atan2(deltay, deltax)
    
    def get_heading(self):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        return yaw
    
    def get_next_destination(self):
        rospy.wait_for_service('mammad')

        try:
            GetNext = rospy.ServiceProxy('mammad', GetNextDestination)
            next_dest = GetNext(self.curent_x, self.curent_y)
            return next_dest
        except rospy.ServiceException as e:
            print("anaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

    def goal_ajacency(self):

        return math.sqrt(math.pow(self.next_x - self.curent_x, 2) + math.pow(self.next_y - self.curent_y, 2))

    def status_update(self):

        self.current_yaw = self.get_heading()

        self.goal_distance = self.goal_ajacency()

        if self.goal_distance < self.stop_distance :

            self.cmd_publisher.publish(Twist())

            self.next_x = self.get_next_destination().next_x
            self.next_y = self.get_next_destination().next_y

            self.state = self.ROTATE

        self.get_destination_heading()

        if abs(self.goal_angle - self.current_yaw) < .005 :
            # pass
            self.state = self.GO

            self.cmd_publisher.publish(Twist())
        
    def run(self):
        
        while not rospy.is_shutdown():
            
            self.status_update()

            # check whether state is changed or not
            if self.state == self.GO:
                rospy.sleep(1)
                twist = Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = 0
                self.cmd_publisher.publish(twist)
                continue
            
            # self.cmd_publisher.publish(Twist())
            
            # rospy.sleep(1)
            
            
            twist = Twist()

            twist.angular.z = self.angular_speed
            twist.linear.x = 0

            self.cmd_publisher.publish(twist)
            
            # rotation loop
            
            # self.cmd_publisher.publish(Twist())

            # rospy.sleep(1)
            
            # self.state = self.GO


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()
    # while True:
    #     print(controller.get_next_destination().next_x)