# Estimator Node
# Written by Malcolm Benedict, mmbenedi@mtu.edu
# Last Rev. 1/31/26
# Set up a ROS2 node to calculate robot position via dead reckoning

import rclpy
import math as m
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Imu
from rclpy.clock import Clock

class DeadReckoner(Node):
    """
    Calculate a robot position via two dead reckoning methods: integrating incoming /cmd_vel messages 
    and integrating IMU sensor data.
    
    """
    def __init__(self):
        super().__init__('estimator_node')

        #define publishers
        publisher_qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dead_reckoning_path_publisher = self.create_publisher(Path, '/dead_reckoning/path', publisher_qos_profile)
        self.dead_reckoning_odom_publisher = self.create_publisher(Odometry, '/dead_reckoning/odom', publisher_qos_profile)
        self.imu_odom_publisher = self.create_publisher(Odometry, '/imu_integration/odom', publisher_qos_profile)
        self.imu_path_publisher = self.create_publisher(Path, '/imu_integration/path', publisher_qos_profile)

        #initialize cmd_vel_callback states to zero
        self.cmd_vel_x = 0.0
        self.cmd_vel_y = 0.0
        self.cmd_vel_t = 0.0
        self.cmd_vel_v = 0.0
        self.cmd_vel_w = 0.0
        self.cmd_vel_deltaT = 0.0
        self.cmd_vel_last_timestamp = 0.0
        self.cmd_vel_path_arr = []

        #initialize imu_callback states to zero
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.imu_t = 0.0
        self.imu_vx = 0.0
        self.imu_vy = 0.0
        self.imu_w = 0.0
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_world_ax = 0.0
        self.imu_world_ay = 0.0
        self.imu_deltaT = 0.0
        self.imu_last_timestamp = 0.0
        self.imu_path_arr = []

        #define subscribers
        subscription_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_subscription = self.create_subscription(TwistStamped,'/cmd_vel', self.cmd_vel_callback, subscription_qos_profile)
        self.imu_subscription = self.create_subscription(Imu,'/imu', self.imu_callback, subscription_qos_profile)

    def cmd_vel_callback(self,msg):
        """
        Handle incoming /cmd_vel messages by calculating current pose data and publishing it to /dead_reckoning/path 
        and /dead_reckoning/odom as Path and Odometry messages respectively.
        
        :param self: self, used to keep it current to the specific node.
        :param msg: the incoming TwistStamped /cmd_vel message.
        """
        #calculate current state est.
        self.cmd_vel_deltaT = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec) - self.cmd_vel_last_timestamp
        self.cmd_vel_x = self.cmd_vel_x + (self.cmd_vel_v * m.cos(self.cmd_vel_t) * self.cmd_vel_deltaT)
        self.cmd_vel_y = self.cmd_vel_y + (self.cmd_vel_v * m.sin(self.cmd_vel_t) * self.cmd_vel_deltaT)
        self.cmd_vel_t = self.cmd_vel_t + (self.cmd_vel_w * self.cmd_vel_deltaT)

        #update values for next timestamp
        self.cmd_vel_v = msg.twist.linear.x
        self.cmd_vel_w = msg.twist.angular.z
        self.cmd_vel_last_timestamp = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec)
        
        #generate pose message
        current_pose = PoseStamped()
        current_pose.header.stamp = Clock().now().to_msg()
        current_pose.header.frame_id = 'odom'
        current_pose.pose.position.x = self.cmd_vel_x
        current_pose.pose.position.y = self.cmd_vel_y

        #get orientation quaternion
        w,x,y,z = convertEulerToQuaternion(0,0,self.cmd_vel_t)
        current_pose.pose.orientation.w = w
        current_pose.pose.orientation.x = x
        current_pose.pose.orientation.y = y
        current_pose.pose.orientation.z = z

        #publish odom message
        odom = Odometry()
        odom.header.stamp = Clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose = current_pose.pose
        self.dead_reckoning_odom_publisher.publish(odom)

        #publish path values
        path = Path()
        path.header.stamp = Clock().now().to_msg()
        path.header.frame_id = 'odom'
        self.cmd_vel_path_arr.append(current_pose)
        path.poses = self.cmd_vel_path_arr
        self.dead_reckoning_path_publisher.publish(path)

    def imu_callback(self,msg):
        """
        Handle incoming IMU data by integrating to get the current pose data and publishing it to /imu_integration/path 
        and /imu_integration/odom as Path and Odometry messages respectively.
        
        :param self: self, used to keep it current to the specific node.
        :param msg: the incoming Imu /imu message.
        """
        #calculate current state est.
        self.imu_world_ax = (self.imu_ax * m.cos(self.imu_t)) - (self.imu_ay * m.sin(self.imu_t))
        self.imu_world_ay = (self.imu_ax * m.sin(self.imu_t)) + (self.imu_ay * m.cos(self.imu_t))  
        self.imu_deltaT = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec) - self.imu_last_timestamp
        self.imu_x = self.imu_x + (self.imu_vx * self.imu_deltaT) 
        self.imu_y = self.imu_y + (self.imu_vy * self.imu_deltaT) 
        self.imu_vy = self.imu_vy + (self.imu_world_ax * self.imu_deltaT)
        self.imu_vx = self.imu_vx + (self.imu_world_ax * self.imu_deltaT)
        
        #update values for next timestamp
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_t = self.imu_t + (self.imu_w * self.imu_deltaT)
        self.imu_w = msg.angular_velocity.z
        self.imu_last_timestamp = msg.header.stamp.sec + (0.000000001 * msg.header.stamp.nanosec)

        #generate pose message
        current_pose = PoseStamped()
        current_pose.header.stamp = Clock().now().to_msg()
        current_pose.header.frame_id = 'odom'
        current_pose.pose.position.x = self.imu_x
        current_pose.pose.position.y = self.imu_y

        #get orientation quaternion
        w,x,y,z = convertEulerToQuaternion(0,0,self.imu_t)
        current_pose.pose.orientation.w = w
        current_pose.pose.orientation.x = x
        current_pose.pose.orientation.y = y
        current_pose.pose.orientation.z = z

        #publish odom message
        odom = Odometry()
        odom.header.stamp = Clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose = current_pose.pose
        self.imu_odom_publisher.publish(odom)

        #publish path values
        path = Path()
        path.header.stamp = Clock().now().to_msg()
        path.header.frame_id = 'odom'
        self.imu_path_arr.append(current_pose)
        path.poses = self.imu_path_arr
        self.imu_path_publisher.publish(path)


def convertEulerToQuaternion(roll,pitch,yaw)-> tuple[float,float,float,float]:
    """
    Helper function to convert Euler angles to a Quaternion
    based on code from
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_(in_3-2-1_sequence)_to_quaternion_conversion
    This could probably be done more elegantly with a service call, but the instructions state only one python file

    :param roll: Rotation about the X axis
    :param pitch: Rotation about the Y axis
    :param yaw: Rotation about the Z axis
    """
    cr = m.cos(roll * 0.5)
    sr = m.sin(roll * 0.5)
    cp = m.cos(pitch * 0.5)
    sp = m.sin(pitch * 0.5)
    cy = m.cos(yaw * 0.5)
    sy = m.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w,x,y,z


def main(args=None):
    rclpy.init(args=args)
    estimator_node = DeadReckoner()
    rclpy.spin(estimator_node)
    estimator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()