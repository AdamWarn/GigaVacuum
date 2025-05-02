import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
import re
import math

class MotorController(Node):
    def __init__(self):
        # Initialize the node with the name 'motor_controller'
        super().__init__('motor_controller')

        # Log to confirm initialization
        self.get_logger().info("MotorController node initialized")

        # Initialize serial communication with Arduino
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update with the correct port if needed
            self.serial_port.reset_input_buffer()  # Clear any previous data in the buffer
            self.get_logger().info("Serial communication initialized with Arduino")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to initialize serial port: {e}")
            raise

        # Declare parameters
        self.declare_parameter('distance_per_pulse', 0.00046)  # Distance moved per encoder pulse (meters)
        self.declare_parameter('wheel_base', 0.35)              # Distance between wheels (meters)
        self.distance_per_pulse = self.get_parameter('distance_per_pulse').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.get_logger().info(f"Parameters loaded: distance_per_pulse={self.distance_per_pulse}, wheel_base={self.wheel_base}")

        # Define wheel radius (half of wheel diameter; here assumed to be 25cm diameter → 0.125 m)
        self.wheel_radius = 0.125

        # Initialize odometry variables and cumulative encoder distances
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_prev = 0.0
        self.right_wheel_prev = 0.0

        # Store cumulative distance for each wheel (from encoder counts)
        self.left_distance = 0.0
        self.right_distance = 0.0

        # TF broadcaster for odometry transform
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TF broadcaster initialized")

        # Create odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.get_logger().info("Odometry publisher created")

        # Create joint state publisher for the wheel joints
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.get_logger().info("Joint state publisher created")

        # Create subscription for cmd_vel messages
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.get_logger().info("cmd_vel subscriber created")

        # Create a timer for processing encoder data every 100ms
        self.timer = self.create_timer(0.1, self.process_encoder_data)
        self.get_logger().info("Timer for encoder data processing created")

        # State to handle initial serial data (skipping a few messages to prevent startup noise)
        self.initial_messages_skipped = 0
        self.get_logger().info("MotorController fully initialized")

    def listener_callback(self, msg):
        # Extract linear and angular velocities
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Format data for Arduino
        command = f"{linear_speed},{angular_speed}\n"
        self.get_logger().info(f"Received /cmd_vel: linear={linear_speed}, angular={angular_speed}")
        self.get_logger().info(f"Sending to Arduino: {command.strip()}")

        # Send command via serial
        self.serial_port.write(command.encode('utf-8'))

    def process_encoder_data(self):
        # Skip the first few messages to handle incomplete startup data
        if self.initial_messages_skipped < 5:
            self.get_logger().info(f"Skipping initial message {self.initial_messages_skipped}")
            self.serial_port.readline()  # Read and discard data
            self.initial_messages_skipped += 1
            return

        # Check if data is available on the serial port
        if self.serial_port.in_waiting > 0:
            try:
                raw_data = self.serial_port.readline().decode('utf-8').strip()
                # Validate correct encoder data format e.g., "left_pulses,right_pulses"
                if re.match(r'^-?\d+,-?\d+$', raw_data):
                    left_pulses, right_pulses = map(int, raw_data.split(','))
                    # Compute odometry using the encoder data
                    self.compute_odometry(left_pulses, right_pulses)
                    # Publish joint state for the wheels
                    self.publish_joint_states()
                else:
                    self.get_logger().error(f"Invalid data format: {raw_data}")
            except ValueError:
                self.get_logger().error(f"Failed to parse encoder data: {raw_data}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error while processing encoder data: {e}")
        # Else: No data available; you could log this if needed

    def compute_odometry(self, left_pulses, right_pulses):
        # Compute cumulative distances for each wheel (in meters)
        left_distance = left_pulses * self.distance_per_pulse
        right_distance = right_pulses * self.distance_per_pulse

        # Update cumulative values and compute changes since last measurement
        delta_left = left_distance - self.left_wheel_prev
        delta_right = right_distance - self.right_wheel_prev
        self.left_wheel_prev = left_distance
        self.right_wheel_prev = right_distance

        # Store cumulative distances for joint state publication
        self.left_distance = left_distance
        self.right_distance = right_distance

        # Compute change in orientation
        delta_theta = (delta_right - delta_left) / self.wheel_base
        # Compute average linear movement
        delta_x = (delta_left + delta_right) / 2.0

        # Update robot pose based on current orientation
        self.x += delta_x * math.cos(self.theta)
        self.y += delta_x * math.sin(self.theta)
        self.theta += delta_theta

        # Publish odometry and TF transform
        self.publish_odometry(delta_x, delta_theta)
        self.broadcast_tf()

    def publish_odometry(self, linear_velocity, angular_velocity):
        # Create and populate an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set robot pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Set velocities
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

    def broadcast_tf(self):
        # Create and populate a TransformStamped message for the odom->base_link transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def publish_joint_states(self):
        # Create and populate a JointState message for the wheel joints
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        # Names must match those defined in your URDF (e.g., "left_wheel_joint" and "right_wheel_joint")
        joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']

        # Calculate joint angles from cumulative wheel distances.
        # For a wheel, angle = (distance traveled) / (wheel radius)
        left_wheel_angle = self.left_distance / self.wheel_radius
        right_wheel_angle = self.right_distance / self.wheel_radius

        joint_state_msg.position = [left_wheel_angle, right_wheel_angle]

        # Publish the JointState message
        self.joint_state_publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()

    try:
        while rclpy.ok():
            motor_controller.process_encoder_data()
            rclpy.spin_once(motor_controller)
    except KeyboardInterrupt:
        pass

    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
