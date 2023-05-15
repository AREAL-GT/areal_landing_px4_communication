
'''
Written by Adam Garlow

This node follows a straightforward sub/pub framework to take mocap messages
and publish them on the desired px4 microRTPS bridge topic

MoCap messages received on topic: '/vicon/X500_v2_IRcam/X500_v2_IRcam'

PX4 messages published on topic: '/fmu/vehicle_visual_odometry/in'

'''

# Standard library imports



# Third-party imports
import rclpy
from rclpy.node import Node

from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleOdometry

from vicon_receiver.msg import Position
from rclpy.qos import qos_profile_sensor_data

from numpy import NaN

# Pubsub node class definition
class MoCapPubSub(Node):

    # Pubsub constructor
    def __init__(self):
        super().__init__('px4_mocap_pubsub') # Initialize node

        # Initialize subscriber to mocap(VICON) topic
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Brian_X500/Brian_X500', self.mocap_callback, 10)

        # Initialize subscriber to PX4 timesync topic
        self.timesync_sub = self.create_subscription(TimesyncStatus, 
            "/fmu/out/timesync_status", self.timesync_callback, 10)
        self.timesync = 0

        # Initialize publisher to PX4 vehicle_visual_odometry topic
        self.mocap_pub = self.create_publisher(VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', 10)

        self.get_logger().info("PX4 mocap pub-sub node initialized")


    # Callback for when new mocap message recieved to publish to PX4
    def mocap_callback(self, msg):

        msg_px4 = VehicleOdometry() # Message to be sent to PX4

        msg_px4.timestamp = self.timesync # Set timestamp
        msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample

        # Transfer data from mocap message to PX4 message
        # VICON Front, Left, Up to PX4 Front, Right, Down
        # Position/orientation components
        msg_px4.pose_frame = 2 # FRD from px4 message
        msg_px4.position = [msg.x_trans/1000.0, -msg.y_trans/1000.0, -msg.z_trans/1000.0]
        msg_px4.q = [msg.w, msg.x_rot, -msg.y_rot, -msg.z_rot]
        
        # Velocity components (unknown)
        msg_px4.velocity_frame = 2 # FRD from px4 message
        msg_px4.velocity = [NaN, NaN, NaN]
        msg_px4.angular_velocity = [NaN, NaN, NaN]

        # Variances
        msg_px4.position_variance = [0.0, 0.0, 0.0]
        msg_px4.orientation_variance = [0.0, 0.0, 0.0]
        msg_px4.velocity_variance = [0.0, 0.0, 0.0]

        self.mocap_pub.publish(msg_px4) # Publish to PX4

    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):

        # self.get_logger().info('CC Timesync')
        # self.get_logger().info(str(msg.timestamp))
        self.timesync = msg.timestamp

def main(args=None):
    rclpy.init(args=args)

    px4_mocap_pubsub = MoCapPubSub()

    rclpy.spin(px4_mocap_pubsub)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
