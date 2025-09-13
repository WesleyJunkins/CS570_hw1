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

    # Make a diamond
    # All edges are the same length
    # Two rotations are 0.3 rads (0.6 for the opposite angle because we are not starting off on a horizontal plane)
    # Two other rotations are 1.25 radians
    def get_twist_msg(self):
        if self.time < 5:
            msg = self.create_twist(0.0, 0.3) # Rotate counterclockwise by 0.3 radians
        elif self.time >= 5 and self.time < 10:
            msg = self.create_twist(1.5, 0.0) # Move forward
        elif self.time >= 10 and self.time < 15:
            msg = self.create_twist(0.0, 1.25) # Rotate counterclockwise by 1.25 radians
        elif self.time >= 15 and self.time < 20:
            msg = self.create_twist(1.5, 0.0) # Move forward
        elif self.time >= 20 and self.time < 25:
            msg = self.create_twist(0.0, 0.6) # Rotate counterclockwise by 0.6 radians
        elif self.time >= 25 and self.time < 30:
            msg = self.create_twist(1.5, 0.0) # Move forward
        elif self.time >= 30 and self.time < 35:
            msg = self.create_twist(0.0, 1.25) # Rotate counterclockwise by 1.25 radians
        elif self.time >= 35 and self.time < 40:
            msg = self.create_twist(1.5, 0.0) # Move forward
        else:
            msg = self.create_twist(0.0, 0.0) # Stop
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
