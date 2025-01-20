import rclpy
from rclpy.node import Node

from grob.helpers import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from grob_interfaces.msg import RobotStates

import message_filters

class localizer(Node):
    def __init__(self):
        super().__init__("localizer")

        # Input measurement data from imu, odom and #TODO: gps
        self.odom_subscriber = message_filters.Subscriber(self, Odometry, "/odom", qos_profile = 10)
        self.imu_subscriber = message_filters.Subscriber(self, Imu, "/imu", qos_profile = 10)

        # sync odom and imu messages together
        message_syncer = message_filters.ApproximateTimeSynchronizer([self.odom_subscriber, self.imu_subscriber], queue_size=10, slop=0.1)
        message_syncer.registerCallback(self.odomIMUFusedCallback)

        # publisher to send robot kinematic states
        self.publisher = self.create_publisher(RobotStates, "/robot_states", qos_profile = 10)

    
    def odomIMUFusedCallback(self, odom_msg: Odometry, imu_msg: Imu):
        states = RobotStates()

        # TODO: Implement EKF, for now just return raw sensor data
        states.x = odom_msg.pose.pose.position.x
        states.y = odom_msg.pose.pose.position.y
        states.theta = euler_from_quaternion(odom_msg.pose.pose.orientation)

        states.v = odom_msg.twist.twist.linear.x
        states.w = odom_msg.twist.twist.angular.z

        states.vdot = imu_msg.linear_acceleration.x

        states.stamp = odom_msg.header.stamp
        
        self.publisher.publish(states)
        

def main(args=None):
    rclpy.init()

    localization = localizer()

    rclpy.spin(localization)


if __name__ == "__main__":
    main()