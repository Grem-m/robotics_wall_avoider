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
            self.side_sensor_1_callback,
            10
        )

        self.sensor1_subscription = self.create_subscription(
            FloatStamped,
            '/ps0',
            self.side_sensor_2_callback,
            10
        )
        
        # Publisher for movement commands already created above as self.cmd_pub

        #TODO: Create timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.control_callback)

        self.get_logger().info('Wall avoider node started')

    def side_sensor_1_callback(self, msg):
        # TODO: check which sensor this is
        self.sensor_array[0] = msg.data
        pass

    def side_sensor_2_callback(self, msg):
        # TODO: check which sensor this is
        self.sensor_array[1] = msg.data
        pass

    def control_callback(self):
        # TODO: implement wall avoidance logic using sensor array
        # test message for now
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = self.maximum_speed
        twist_msg.twist.angular.z = self.turning_speed
        self.cmd_pub.publish(twist_msg)
        self.get_logger().info(f'Publishing: linear.x={twist_msg.twist.linear.x}, angular.z={twist_msg.twist.angular.z}')
        pass

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