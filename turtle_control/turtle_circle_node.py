

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist


from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill


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
        
        self.request = Spawn.Request()
        self.request.x      = 5.0
        self.request.y      = 5.0
        self.request.theta  = 0.0
        self.request.name   = turtlename
        

        self.timer = self.create_timer(
            1.0,
            self.timer_callback)
        


        # # create a new turtle via service call
        self.client.wait_for_service()
        # try:
        self.response = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.response)
        # except Exception as e:
        #     self.get_logger().error(e)
        #     rclpy.shutdown()
        #     exit(1)

        

    def turtle_pose_callback(self, msg):
        self.get_logger().info(str(msg))


    def timer_callback(self):

        vel = Twist()
        vel.linear.x = 1.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 1.0
        self.publisher.publish(vel)




def main(args=None):
    rclpy.init(args=args)

    turtle_circle = TurtleCircle()

    rclpy.spin(turtle_circle)

    turtle_circle.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


        