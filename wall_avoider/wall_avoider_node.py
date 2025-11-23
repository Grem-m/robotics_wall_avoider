import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from webots_ros2_msgs.msg import FloatStamped

class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')

        # Declare parameters
        self.declare_parameter('avoidance_threshold', 0.3)
        self.declare_parameter('maximum_speed', 6.28)
        self.declare_parameter('turning_speed', 0.5)

        # Get parameters
        avoidance_threshold = self.get_parameter('avoidance_threshold').value
        maximum_speed = self.get_parameter('maximum_speed').value
        turning_speed = self.get_parameter('turning_speed').value

        # Subscription to sensor data
        self.sensor0_subscription = self.create_subscription(
            FloatStamped,
            '/distance_sensor_0',
            self.front_sensor_callback,
            10
        )

        self.sensor1_subscription = self.create_subscription(
            FloatStamped,
            '/distance_sensor_1',
            self.front_sensor_callback,
            10
        )

        self.sensor2_subscription = self.create_subscription(
            FloatStamped,
            '/distance_sensor_2',
            self.front_sensor_callback,
            10
        )
        
        # Publisher for movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Wall avoider node started')
    
    def front_sensor_callback(self, msg):
        # Simple wall avoidance logic
        cmd_vel = Twist()
        
        if min(msg.FloatStamped) < self.avoidance_threshold:  # 30cm threshold

            cmd_vel.angular.z = self.turning_speed  # Turn right
        else:
            cmd_vel.linear.x = self.maximum_speed   # Move forward
        
        self.cmd_pub.publish(cmd_vel)

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