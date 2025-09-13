import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0

    def create_twist(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    # Make a circle
    # Uses modular arithmetic to move forward on even times and rotate on odd times
    def get_twist_msg(self):
        if self.time >= 199:
            msg = self.create_twist(0.0, 0.0) # Stop after completing the circle
        elif self.time % 2 == 0:
            msg = self.create_twist(0.5, 0.0) # Move forward
        else:
            msg = self.create_twist(0.0, 0.2) # Rotate slightly
        return msg
    
    def timer_callback(self):
        msg = self.get_twist_msg()       
        self.publisher.publish(msg)
        self.time += 1
        print("time: {}".format(self.time))

def main(args=None):
    rclpy.init(args=args)

    turtle_controller = TurtleController()

    rclpy.spin(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
