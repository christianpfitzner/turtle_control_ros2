


import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill


import math


# use the name of the turtle
turtlename = "turtle1"

class TurtleGoal(Node):

    def __init__(self):
        super().__init__('turtle_circle')

        # create publisher for turtle_cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            turtlename + '/cmd_vel',
            10)
        
        # create subscriber for turtle_pose
        self.subscriber = self.create_subscription(
            Pose,
            turtlename + '/pose',
            self.turtle_pose_callback,
            10)
        
        self.subscriber = self.create_subscription(
            Pose,
            turtlename + '/goal',
            self.turtle_goal_callback,
            10)
        
        self.goal           = Pose()
        self.pose           = Pose()
        self.goal_received  = False
        self.heading_i      = 0.0
        

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        rclpy.spin(self)


        

    def turtle_pose_callback(self, msg):
        # self.get_logger().info(str(msg))
        self.pose = msg



    def turtle_goal_callback(self, msg):
        self.get_logger().info(str(msg))
        self.goal = msg

        self.goal_received = True





    def timer_callback(self):

        # check if the goal has been received otherwise abort   
        if not self.goal_received:
            return


        # calculate the angle between the goal and the pose
        delta_x          = self.pose.x - self.goal.x
        delta_y          = self.pose.y - self.goal.y

        # calculate the euclidean distance between the goal and the pose
        dist             = ((delta_x*delta_x)**2 + (delta_y*delta_y)**2)**0.5   


        delta_goal_angle = self.pose.theta - math.atan2(delta_y, delta_x) 


        # print the distance and the angle to the goal as debug information
        self.get_logger().info("Distance to goal: " + str(dist))
        self.get_logger().info("Angle to goal: "    + str(delta_goal_angle))


        if delta_goal_angle > math.pi:
            delta_goal_angle -= 2*math.pi
        if delta_goal_angle < -math.pi:
            delta_goal_angle += 2*math.pi

        # stop if close to the goal
        if dist > 0.1:
            dist = 1.0

        if dist < 0.1:
            dist             = 0.0
            delta_goal_angle = 0.0

        self.heading_i = self.heading_i + 1.5*delta_goal_angle*0.1



        # anti windup
        limit = 2.5 
        if self.heading_i  >  limit:
            self.heading_i =  limit
        if self.heading_i  < -limit:
            self.heading_i = -limit


        # generate a new twist message to send the turtle to the goal
        msg = Twist()
        msg.linear.x  = 1 * dist                                     # p controller for linear velocity
        msg.angular.z = 0.1*dist*delta_goal_angle + self.heading_i   # pi controller for angular velocity

        self.publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    turtle_goal = TurtleGoal()

    rclpy.spin(turtle_goal)

    turtle_goal.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


        