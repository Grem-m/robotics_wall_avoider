import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from webots_ros2_msgs.msg import FloatStamped

class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')

        # Declare parameters
        self.declare_parameter('avoidance_threshold', 0.3)
        self.declare_parameter('maximum_speed', 6.28)
        self.declare_parameter('turning_speed', 0.5)
        self.get_logger().debug(f"parameters retrieved from file (I hope): \n{self.get_parameter('avoidance_threshold').value} \n{self.get_parameter('maximum_speed').value} \n{self.get_parameter("turning_speed").value}")

        # Get parameters
        self.avoidance_threshold = self.get_parameter('avoidance_threshold').value
        self.maximum_speed = self.get_parameter('maximum_speed').value
        self.turning_speed = self.get_parameter('turning_speed').value
        self.get_logger().debug(f"parameters retrieved from file (I hope): \n{self.avoidance_threshold} \n{self.maximum_speed} \n{self.turning_speed}")

        # declare sensor value array
        self.sensor_array = [0.0, 0.0, 0.0]

        # publisher (we publish globally on /cmd_vel so it reaches the diffdrive controller)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscription to sensor data
        self.sensor0_subscription = self.create_subscription(
            FloatStamped,
            '/ps7',
            self.side_sensor_0_callback,
            10
        )
        self.sensor0_subscription

        self.sensor1_subscription = self.create_subscription(
            FloatStamped,
            '/ps6',
            self.side_sensor_1_callback,
            10
        )
        self.sensor1_subscription

        #TODO: Create timers
        timer_period = 0.5
        self.control_timer = self.create_timer(timer_period, self.control_callback)

        self.get_logger().info('Wall avoider node started')

    def side_sensor_0_callback(self, msg):
        # TODO: check which sensor this is
        self.sensor_array[0] = msg.data
        self.get_logger().info(f"sensor_1 received {msg.data}")

    def side_sensor_1_callback(self, msg):
        # TODO: check which sensor this is
        self.sensor_array[1] = msg.data
        self.get_logger().info(f"sensor_2 received {msg.data}")

    def control_callback(self):
        # TODO: implement wall avoidance logic using sensor array
        # test message for now

        # initialise message
        twist_msg = TwistStamped()

        # do robot logic (check which one is left and right)
        # if both sensors too close reverse
        if self.sensor_array[0] and self.sensor_array[1] < self.avoidance_threshold:
            twist_msg.twist.linear.x = -self.maximum_speed

        # if one sensor too close turn a mystery direction
        elif self.sensor_array[0] < self.avoidance_threshold:
            twist_msg.twist.linear.x = self.maximum_speed/2
            twist_msg.twist.angular.z = self.turning_speed

        # if other sensor too close turn a second mystery direction
        elif self.sensor_array[1] :
            twist_msg.twist.linear.x = self.maximum_speed/2
            twist_msg.twist.angular.z = -self.turning_speed

        # otherwise go in a straight line full speed
        else:
            twist_msg.twist.linear = self.maximum_speed

        # publish message for robot
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(f'Publishing: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}')

def main():
    rclpy.init()
    node = WallAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()