
from areal_landing_uav_interfaces.srv import PX4OffboardControlMode
from areal_landing_uav_interfaces.srv import PX4TrajectorySetpoint
from areal_landing_uav_interfaces.srv import PX4VehicleCommand
from areal_landing_uav_interfaces.srv import PX4ArmingControl

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TimesyncStatus

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class PX4ControlCommand(Node):

    def __init__(self):
        super().__init__('service_px4_control_command') # Initialize node

        # Initialize PX4 command and control services
        self.control_mode_srv = self.create_service(PX4OffboardControlMode, 
            'px4_offboard_control_mode', self.offboard_control_mode_callback)

        self.trajectory_setpoint_srv= self.create_service(PX4TrajectorySetpoint, 
            'px4_trajectory_setpoint', self.trajectory_setpoint_callback)

        self.vehicle_command_srv = self.create_service(PX4VehicleCommand, 
            'px4_vehicle_command', self.vehicle_command_callback)

        self.arming_control_srv = self.create_service(PX4ArmingControl ,
            'px4_arming_control', self.arming_control_callback)
        



        # Initialize publishers for PX4 command and control messages

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.control_mode_pub = self.create_publisher(OffboardControlMode,
            "/fmu/in/offboard_control_mode", qos_profile)

        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint", qos_profile)

        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
            "/fmu/in/vehicle_command", qos_profile)

        # Initialize subscriber to PX4 timesync topic
        self.timesync_sub = self.create_subscription(TimesyncStatus, 
            "/fmu/out/timesync_status", self.timesync_callback, 10)
        self.timesync = 0

        self.get_logger().info("PX4 command and control service servers "
            "initialized on command and control server")

    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):

        # self.get_logger().info('CC Timesync')
        # self.get_logger().info(str(msg.timestamp))
        self.timesync = msg.timestamp

    # Callback function to publish to offboard control mode upon service call
    def offboard_control_mode_callback(self, request, response):

        msg = OffboardControlMode() # Message to be published to PX4

        msg.timestamp = self.timesync # Set timestamp

        # Transfer data from request to publishing message
        msg.position = request.position
        msg.velocity = request.velocity
        msg.acceleration = request.acceleration
        msg.attitude = request.attitude
        msg.body_rate = request.body_rate
        msg.actuator = request.actuator

        self.control_mode_pub.publish(msg) # Publish message

        # self.get_logger().info("Mode message published")

        response.response = "mode pub"
        return response

    # Callback function to publish to trajectory setpoint upon service call
    def trajectory_setpoint_callback(self, request, response):

        # self.get_logger().info("Trajectory setpoint service start")

        msg = TrajectorySetpoint() # Message to be published to PX4

        msg.timestamp = self.timesync # Set timestamp

        # Transfer data from request to publishing message
        msg.position = request.position
        msg.velocity = request.velocity
        msg.acceleration = request.acceleration
        msg.jerk = request.jerk
        msg.yaw = request.yaw
        msg.yawspeed = request.yawspeed

        self.trajectory_setpoint_pub.publish(msg) # Publish message

        # self.get_logger().info("Trajectory message published from service")

        response.response = "trajectory pub"
        return response

    # Callback function to publish vehicle commands upon service call
    def vehicle_command_callback(self, request, response):

        msg = VehicleCommand()

        msg.timestamp = self.timesync # Set timestamp

        # Transfer data from request to publishing message
        msg.param1 = request.param1
        msg.param2 = request.param2
        msg.param3 = request.param3
        msg.param4 = request.param4
        msg.param5 = request.param5
        msg.param6 = request.param6
        msg.param7 = request.param7
        msg.command = request.command
        msg.target_system = request.target_system
        msg.target_component = request.target_component
        msg.source_system = request.source_system
        msg.source_component = request.source_component
        msg.confirmation = request.confirmation
        msg.from_external = request.from_external

        self.vehicle_command_pub.publish(msg) # Publish message

        self.get_logger().info("Vehicle command message published from service")

        response.response = "command pub"
        return response

    # Callback function to control arming and disarming of the quadcopter
    def arming_control_callback(self, request, response):
        # Not currently confirming arming status, but could implement in future

        if request.command == 'arm': # If commanded to arm quadcopter

            msg = VehicleCommand()
            msg.timestamp = self.timesync # Set timestamp
            msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = 1.0
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True

            self.vehicle_command_pub.publish(msg) # Publish message

            response.response = 'armed'

            self.get_logger().info("Arm command sent from service")

        elif request.command == 'disarm': # If commanded to disarm quadcopter 

            msg = VehicleCommand()
            msg.timestamp = self.timesync # Set timestamp
            msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            msg.param1 = 0.0
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True

            self.vehicle_command_pub.publish(msg) # Publish message

            response.response = 'disarmed'

            self.get_logger().info("Disarm command sent from service")

        else: # Some other invalid request

            response.response = 'invalid'

        return response

def main(args=None):
    rclpy.init(args=args)

    px4_control_command = PX4ControlCommand()

    rclpy.spin(px4_control_command)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

