import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import glob
import time

DEBUG = True

class WT901Node(Node):
    def __init__(self):
        super().__init__('wt901_node')

        self.declare_parameter("orientation_covariance", [-1.0] * 9)
        self.declare_parameter("linear_acceleration_covariance", [-1.0] * 9)
        self.declare_parameter("angular_velocity_covariance", [-1.0] * 9)

        self.orientation_covariance = self.get_parameter("orientation_covariance").value
        self.linear_acceleration_covariance = self.get_parameter("linear_acceleration_covariance").value
        self.angular_velocity_covariance = self.get_parameter("angular_velocity_covariance").value

        ports = glob.glob('/dev/serial/by-id/usb-1a86*')
        if not ports:
            self.get_logger().error("IMU device not found! Check connections.")
            exit(1)
        self.device = ports[0]

        self.g = 9.80665
        self.PI = 3.14159

        self.ser = serial.Serial(self.device, 115200, timeout=0.5)
        if self.ser:
            self.get_logger().info(f"Connected to {self.device}")
        else:
            self.get_logger().error("Failed to connect to IMU!")
            exit(1)

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now().to_msg().sec
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(0.02, self.update)

    def update(self):
        imu = Imu()

        imu.angular_velocity_covariance = self.angular_velocity_covariance
        imu.linear_acceleration_covariance = self.linear_acceleration_covariance
        imu.orientation_covariance = self.orientation_covariance

        data = self.ser.read(2)
        if len(data) < 2 or data[1] != 0x61:
            return 

        data = self.ser.read(18)
        if len(data) != 18:
            self.get_logger().warn("Received incomplete data!")
            return

        if DEBUG:
            self.get_logger().info(f"Received data: {' '.join(format(x, '02X') for x in data)}")

        axL, axH, ayL, ayH, azL, azH, wxL, wxH, wyL, wyH, wzL, wzH, RollL, RollH, PitchL, PitchH, YawL, YawH = data

        ax = int.from_bytes([axH, axL], byteorder="big", signed=True) / 32768 * 16 * self.g
        ay = int.from_bytes([ayH, ayL], byteorder="big", signed=True) / 32768 * 16 * self.g
        az = int.from_bytes([azH, azL], byteorder="big", signed=True) / 32768 * 16 * self.g

        wx = int.from_bytes([wxH, wxL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
        wy = int.from_bytes([wyH, wyL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
        wz = int.from_bytes([wzH, wzL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI

        Roll = int.from_bytes([RollH, RollL], byteorder="big", signed=True) / 32768 * self.PI
        Pitch = int.from_bytes([PitchH, PitchL], byteorder="big", signed=True) / 32768 * self.PI
        Yaw = int.from_bytes([YawH, YawL], byteorder="big", signed=True) / 32768 * self.PI

        current_time = self.get_clock().now().to_msg().sec
        dt = 0.02
        self.last_time = current_time

        self.roll = -Roll
        self.pitch = -Pitch
        self.yaw += wz * dt

        if DEBUG:
            self.get_logger().info(f"Acceleration: {ax:.2f}, {ay:.2f}, {az:.2f}")
            self.get_logger().info(f"Gyro: {wx:.2f}, {wy:.2f}, {wz:.2f}")
            self.get_logger().info(f"Angles (Roll, Pitch, Yaw): {self.roll:.2f}, {self.pitch:.2f}, {self.yaw:.2f}")

        imu.angular_velocity = Vector3(x=wx, y=-wy, z=-wz)
        imu.linear_acceleration = Vector3(x=ax,y=-ay, z=-az - 2 * self.g)

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        imu.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu"

        self.pub_imu.publish(imu)

        t = TransformStamped()  
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "imuwt"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = imu.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WT901Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()