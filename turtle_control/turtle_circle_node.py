


import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill



# use the name of the turtle
turtlename = "my_turtle"

class TurtleCircle(Node):

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
        
        self.kill_client = self.create_client(
            Kill,
            'kill')
        
        # create service client to spawn a new turtle
        self.client = self.create_client(
            Spawn,
            'spawn')
        

        self.timer = self.create_timer(
            1.0,
            self.timer_callback)
        







        

    def turtle_pose_callback(self, msg):
        self.get_logger().info(str(msg))

        # change the color of the pen, based on the turtles pose


    def timer_callback(self):

        # create the code to publish the command to drive in circles
        # have a look at the following page for the Twist message
        # https://docs.ros.org/en/humble/p/geometry_msgs/interfaces/msg/Twist.html

        pass




def main(args=None):
    rclpy.init(args=args)

    turtle_circle = TurtleCircle()

    rclpy.spin(turtle_circle)

    turtle_circle.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


        