import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            'ldlidar_node/scan',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(String, 'object_detected', 10)

    def listener_callback(self, msg):
        # Define the range in which you want to detect objects
        min_distance = 0.01  # meters
        max_distance = 0.45 # meters

        # Check if any of the range values are within the specified distance
        for range_value in msg.ranges:
            if min_distance < range_value < max_distance:
                # If an object is detected within the specified range, publish a message
                print("y a un truc")
                self.publisher.publish(String(data='Object detected'))
                break

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()