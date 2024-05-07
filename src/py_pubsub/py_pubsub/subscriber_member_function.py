import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.time import Time

class VisualInertialOdometryPublisher(Node):
    def __init__(self):
        super().__init__('VIO_pub')
        self.odometry_publisher_ = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.odometry_subscription_ = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.odometry_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.timer = self.create_timer(10, self.timer_callback)  # Timer callback every 10 seconds
        
    def odometry_callback(self, msg):
        vehicle_odometry_msg = VehicleOdometry()
        vehicle_odometry_msg.timestamp = Time.from_msg(msg.header.stamp).nanoseconds

        vehicle_odometry_msg.pose_frame = 2
        vehicle_odometry_msg.velocity_frame = 2
        # Set the position information
        position = msg.pose.pose.position
        # Rotate the position vector by 180 degrees around the x-axis
        rotated_position = Vector3()
        rotated_position.x = position.x
        rotated_position.y = -position.y
        rotated_position.z = -position.z

        vehicle_odometry_msg.position[0] = rotated_position.x
        vehicle_odometry_msg.position[1] = rotated_position.y
        vehicle_odometry_msg.position[2] = rotated_position.z

        # Set the linear velocity information
        velocity = msg.twist.twist.linear
        
        # Rotate the velocity vector by 180 degrees around the x-axis
        rotated_velocity = Vector3()
        rotated_velocity.x = velocity.x
        rotated_velocity.y = -velocity.y
        rotated_velocity.z = -velocity.z

        vehicle_odometry_msg.velocity[0] = rotated_velocity.x
        vehicle_odometry_msg.velocity[1] = rotated_velocity.y
        vehicle_odometry_msg.velocity[2] = rotated_velocity.z

        # Set the angular velocity information
        angular_velocity = msg.twist.twist.angular
        
        # Rotate the velocity vector by 180 degrees around the x-axis
        rotated_angular_velocity = Vector3()
        rotated_angular_velocity.x = angular_velocity.x
        rotated_angular_velocity.y = -angular_velocity.y
        rotated_angular_velocity.z = -angular_velocity.z

        vehicle_odometry_msg.angular_velocity[0] = rotated_angular_velocity.x
        vehicle_odometry_msg.angular_velocity[1] = rotated_angular_velocity.y
        vehicle_odometry_msg.angular_velocity[2] = rotated_angular_velocity.z

        # Set the orientation information
        orientation = msg.pose.pose.orientation
        
        # Rotate the quaternion by 180 degrees around the X-axis
        euler_angles = self.quaternion_to_euler(orientation)
        euler_angles[0] += math.pi
        rotated_quaternion = self.euler_to_quaternion(euler_angles)
        vehicle_odometry_msg.q[0] = rotated_quaternion[0]
        vehicle_odometry_msg.q[1] = rotated_quaternion[1]
        vehicle_odometry_msg.q[2] = rotated_quaternion[2]
        vehicle_odometry_msg.q[3] = rotated_quaternion[3]

        # Publish the vehicle odometry message
        self.odometry_publisher_.publish(vehicle_odometry_msg)

    def timer_callback(self):
        self.get_logger().info('Publishing visual-inertial odometry')

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return [roll, pitch, yaw]

    def euler_to_quaternion(self, euler_angles):
        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll = euler_angles[0]
        pitch = euler_angles[1]
        yaw = euler_angles[2]

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)
    publisher = VisualInertialOdometryPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
