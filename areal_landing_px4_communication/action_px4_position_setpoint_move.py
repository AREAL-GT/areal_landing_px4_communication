
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from areal_landing_uav_interfaces.srv import PX4OffboardControlMode
from areal_landing_uav_interfaces.srv import PX4TrajectorySetpoint
from areal_landing_uav_interfaces.srv import PX4VehicleCommand
from areal_landing_uav_interfaces.srv import PX4ArmingControl

from areal_landing_uav_interfaces.action import PX4PosSetMove

from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleCommand

from timeit import default_timer as timer # Time in seconds

import numpy as np
from numpy import linalg as LA
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class PX4PosSetMoveActionServer(Node):

    """
    A ROS2 action server that can be used to move the PX4 quadcopter to a
    desired position setpoint with its internal control algorithms. Also,
    handles setting the PX4 into system wide offboard control mode after 
    establishing a stream of trajectory setpoints.

    Feedback is not currently implemented on this action server (4/25/22) but 
    the code for it will be left in place and commented out for future use
    """

    def __init__(self):
        super().__init__('px4_position_setpoint_move_action_server')

        # Create action server to move quadcopter
        self.action_server = ActionServer(self, PX4PosSetMove, 
            'px4_pos_set_move', self.action_execute_callback)

        # Create PX4 command and control service clients
        self.px4_mode_client = self.create_client(PX4OffboardControlMode, 
            'px4_offboard_control_mode')
        self.px4_trajectory_client = self.create_client(PX4TrajectorySetpoint, 
            'px4_trajectory_setpoint')
        self.px4_command_client = self.create_client(PX4VehicleCommand, 
            'px4_vehicle_command')
        self.px4_armming_client = self.create_client(PX4ArmingControl, 
            'px4_arming_control')

        # Wait for PX4 command and control service clients to be ready
        while not self.px4_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "PX4 mode service not avaliable, waiting again...")
        while not self.px4_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "PX4 command service not avaliable, waiting again...")
        while not self.px4_trajectory_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "PX4 trajectory service not avaliable, waiting again...")
        while not self.px4_armming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "PX4 arming service not avaliable, waiting again...")

        self.get_logger().info("PX4 command and control service clients "
            "initialized on position setpoint move action server")

        # Create PX4 command and control service request variables
        self.mode_request = PX4OffboardControlMode.Request()
        self.traj_request = PX4TrajectorySetpoint.Request()
        self.command_request = PX4VehicleCommand.Request()
        self.armming_request = PX4ArmingControl.Request()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # Initialize subscriber to PX4 timesync topic
        self.timesync_sub = self.create_subscription(TimesyncStatus, 
            "/fmu/out/timesync_status", self.timesync_callback, qos_profile)
        self.timesync = 0

        # Initialize subscriber to PX4 quadcopter position topic
        self.timesync_sub = self.create_subscription(VehicleLocalPosition, 
            "/fmu/out/vehicle_local_position", self.position_callback, qos_profile)
        self.quad_position = np.array([0.0, 0.0, 0.0])
        # self.quad_x = 0.0
        # self.quad_y = 0.0
        # self.quad_z = 0.0

        # Initialize subscriber to vehicle control mode topic
        self.control_mode_sub = self.create_subscription(VehicleControlMode,
            "/fmu/out/vehicle_control_mode", self.control_mode_callback, 10)
        self.sys_offboard_mode = False # Track of px4 offboard control status
        self.sys_armed = False # Track px4 armed status

        self.get_logger().info("Subscribers created on px4 move action server "
            "for the relevant topics")

    # Callback function for overall ROS2 action call
    def action_execute_callback(self, goal_handle):

        # Save tolerance information from request message
        self.tolerance = goal_handle.request.tolerance

        # Save position setpoint to a vector
        self.setpoint_vec = np.array(goal_handle.request.position_set)

        # Construct initial goal execution summary message
        self.execute_start_msg = "Timestamp: %i | " \
            "Executing PX4 positon move callback | " \
            "Current position: [%f, %f, %f] Goal Position: [%f, %f, %f] " \
            "Goal Tolerance: %f" % (self.timesync, self.quad_position[0], \
            self.quad_position[1], self.quad_position[2], \
            self.setpoint_vec[0], self.setpoint_vec[1], self.setpoint_vec[2], \
            self.tolerance)

        # Print initial action execute goal message
        self.get_logger().info(self.execute_start_msg)

        # If px4 uav not currently armed send arming command
        if not self.sys_armed: 

            self.get_logger().info("Sending PX4 arm command in px4 setpoint " 
                "action")
            self.armming_request.command = 'arm'
            self.arm_req_future = self.px4_armming_client.call_async(
                self.armming_request)

        # Set up feedback message - not used (4/25/22)
        feedback_msg = PX4PosSetMove.Feedback()

        # Save current position to a vector
        # self.current_pos_vec = np.array([self.quad_x, self.quad_y, self.quad_z])

        # Calculate initial distance from setpoint
        self.setpoint_displacement = self.setpoint_vec - self.quad_position
        self.setpoint_distance = LA.norm(self.setpoint_displacement)

        # Create timer for sending position setpoint to PX4
        self.sp_call_count = 0 # Initialize count of timer calls
        self.setpoint_timer = self.create_timer(0.1, self.sp_timer_callback)

        # Create timer for sending feedback for the action - not used (4/25/22)
        self.fb_timer_call_count = 0 # Initialize count of timer calls
        self.feedback_timer = self.create_timer(0.1, self.fb_timer_callback)

        self.get_logger().info("Setpoint and feedback timers created on " 
            "px4 move action server")

        # Save start time for feedback and time elapsed
        self.time_start = timer()
        self.time_mark = timer()

        self.get_logger().info("Entering holding loop in PX4 move action call")

        # Hold until done method for this class completed
        while not self.done():

            pass
        
        #kill timers
        self.setpoint_timer.cancel()
        self.feedback_timer.cancel()

        # Fill out result message
        result = PX4PosSetMove.Result()
        result.result = 'Succeed'
        # result.position_reached = [self.quad_x, self.quad_y, self.quad_z]
        result.position_reached = [float(self.quad_position[0]), \
            float(self.quad_position[1]), float(self.quad_position[2])]
        result.time_elapsed_final = timer() - self.time_start

        self.get_logger().info("px4 move action complete")
        self.get_logger().info(str(result.position_reached))

        # Mark goal as having suceeded
        goal_handle.succeed()

        return result # Return result and complete action callback

    # Method to determine if move action should be concluded
    def done(self):

        # Recalculate distance to setpoint position every cycle
        # self.current_pos_vec = np.array([self.quad_x, self.quad_y, 
        #     self.quad_z])
        self.setpoint_displacement = (self.setpoint_vec - 
            self.quad_position)
        # self.get_logger().info("current position is: " + str(self.quad_position))
        self.setpoint_distance = LA.norm(self.setpoint_displacement)
        # self.get_logger().info("displacement is: " + str(self.setpoint_distance))
        # self.get_logger().info("tolerance is: " + str(self.tolerance))
        if self.setpoint_distance <= self.tolerance:

            return True

        else:

            return False

    # Callback method for the timer sending position setpoints to the PX4
    def sp_timer_callback(self):

        # self.get_logger().info("sp callback")
        # self.get_logger().info(str(self.setpoint_distance))

        # Divide callback operations depending on PX4 system offboard mode
        if self.sys_offboard_mode: # If offboard mode enabled
            # self.get_logger().info("test")
            self.send_pos_set(self.setpoint_vec) # Send setpoint to PX4

            self.sp_call_count += 1 # Increment timer call count
        
        else: # If offboard mode not enabled

            self.send_pos_set(self.setpoint_vec) # Send setpoint to PX4

            # Need to send stream of setpoints before offboard control switch
            if self.sp_call_count < 10: # If less than 10 calls

                self.sp_call_count += 1 # Increment timer call count

                # self.get_logger().info("Count: %i" % self.sp_call_count)

            else: # If more than 10 setpoint calls sent

                # Set offboard control mode PX4 command message
                self.command_request.param1 = 1.0
                self.command_request.param2 = 6.0
                self.command_request.command = (
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE)

                # Send service call and set its future
                self.offboard_mode_future = self.px4_command_client.call_async(
                    self.command_request)

                self.get_logger().info("PX4 system offboard control mode " 
                    "command sent")

                self.sp_call_count += 1 # Increment timer call count

    def fb_timer_callback(self):

        # Not currently implemented - 4/25/22

        # self.get_logger().info("fb callback")

        pass

        #    if not self.fb_timer_activate: # If timer not set to send feedback

        #         self.get_logger().info("fb callback not active")

        #     else: # If timer set to send feedback messages

        #         self.fb_timer_call_count += 1

        #         self.get_logger().info("check_9")

        # # Set feedback message fields
        # feedback_msg.time_elapsed_in_progress = time_elapsed
        # feedback_msg._distance_to_setpoint = self.setpoint_distance
        # feedback_msg._position_current = current_pos_vec

        # goal_handle.publish_feedback(feedback_msg) # Publish feedback

    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):
        self.timesync = msg.timestamp

    # Callback on vehicle position for action feedback purposes
    def position_callback(self, msg):
        # self.quad_x = msg.x
        # self.quad_y = msg.y
        # self.quad_z = msg.z
        self.quad_position = np.array([msg.x, msg.y, msg.z])

    # Callback to track PX4 system offboard control state
    def control_mode_callback(self, msg):

        # Update armed and offboard control from px4 message
        self.sys_armed = msg.flag_armed
        self.sys_offboard_mode = msg.flag_control_offboard_enabled

        # self.get_logger().info("PX4 offboard status: %s" % 
        #     self.sys_offboard_mode)

    # Method to use communication services to send position to PX4 FMU
    def send_pos_set(self, setpoint_vec):

        # Assign fields for control mode message (all false by default)
        self.mode_request.timestamp = self.timesync
        self.mode_request.position = True

        # Assign fields for trajectory setpoint message
        self.traj_request.timestamp = self.timesync
        self.traj_request.position = [float(self.setpoint_vec[0]), \
            float(self.setpoint_vec[1]), float(self.setpoint_vec[2])]
        # self.traj_request.y = float(setpoint_vec[1])
        # self.traj_request.z = float(setpoint_vec[2])

        # self.get_logger().info("Calling PX4 Positon setpoint service")

        # Send both service calls and set their futures
        self.mode_req_future = self.px4_mode_client.call_async(
            self.mode_request)
        self.traj_req_future = self.px4_trajectory_client.call_async(
            self.traj_request)

def main(args=None):
    rclpy.init(args=args)

    # Use MultiThreadedExecutor to allow other operations to persist
    executor = MultiThreadedExecutor()

    px4_pos_move_action_server = PX4PosSetMoveActionServer()

    rclpy.spin(px4_pos_move_action_server, executor=executor)

if __name__ == '__main__':
    main()
