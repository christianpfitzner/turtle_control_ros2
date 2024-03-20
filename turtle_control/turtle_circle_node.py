


import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill



# use the name of the turtle
turtlename = "chris"

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
        
        # create request to spawn a new turtle
        self.request = Spawn.Request()
        self.request.name = turtlename
        self.request.x = 3.0
        self.request.y = 3.0
        self.request.theta = 0.0

        self.client.wait_for_service()
        # try:
        self.response = self.client.call_async(self.request)

        




        

        self.timer = self.create_timer(
            1.0,
            self.timer_callback)
        


        # # create a new turtle via service call
        self.client.wait_for_service()
        # try:
        self.response = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.response)


        

    def turtle_pose_callback(self, msg):
        self.get_logger().info(str(msg))

        # change the color of the pen, based on the turtles pose


    def timer_callback(self):

        # create the code to publish the command to drive in circles
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0

        self.publisher.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    turtle_circle = TurtleCircle()

    rclpy.spin(turtle_circle)

    turtle_circle.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


        