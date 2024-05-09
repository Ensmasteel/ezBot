## things to do:
# - listen to the (ros2) topic tirette (bool) status, once it goes from zero to 1 make the robot move with cmd_vel (0.2m.s-1) for 6 seconds (cumulated)
# - listen to the topic detected object (string), if there is anything and we are moving send cmd_vel (0.0m.s-1) to avoid it 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import time

class SequenceurHomologation(Node):
    def __init__(self):
        super().__init__('sequenceur_homologation')
        self.subscription_tirette = self.create_subscription(Bool, 'tirette_status', self.tirette_callback, 10)
        self.subscription_detected_object = self.create_subscription(String, 'object_detected', self.detected_object_callback, 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/omnidirectional_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tirette = False
        self.detected_object = False
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.3
        self.cmd_vel.linear.y = 0.3
        self.cmd_vel.angular.z = 0.0
        self.start_time = 0
        self.cumulated_time = 0
        self.start_moving = False
        self.time_since_detection = 0
        self.movement_time = 11.2


    def tirette_callback(self, msg):
        self.tirette = msg.data

    def detected_object_callback(self, msg):
        self.time_since_detection = 0

    def timer_callback(self):

        if self.tirette:
            self.time_since_detection = self.time_since_detection + 0.1
            
            if not self.start_moving:
                self.start_time = time.time()
                self.start_moving = True
            self.cumulated_time = time.time() - self.start_time
            if self.cumulated_time < 100 and self.time_since_detection > 0.5 and self.movement_time > 0:
                self.publisher_cmd_vel.publish(self.cmd_vel)
                self.movement_time = self.movement_time - 0.1
            else:
                self.publisher_cmd_vel.publish(Twist())
        else:
            self.cumulated_time = 0
            self.movement_time = 11.2
            self.start_moving = False
        if self.time_since_detection<0.5 and self.start_moving:
            self.publisher_cmd_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)

    sequenceur = SequenceurHomologation()

    rclpy.spin(sequenceur)

    sequenceur.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()