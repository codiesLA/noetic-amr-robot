#!/usr/bin/env python3

import sys
sys.path.append('/home/codybui/dev_robot/src/robot_control/include')
from dataclasses import dataclass
from RS485 import rs_485
from AZDKD import AZDKD as motor
from DIFFROBOT import ROBOT as robot
import rospy
import math
import time
from tf2_ros import TransformBroadcaster
from std_msgs.msg import _String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry

def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    val = int(val, 2)
    
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return float(val/100) 

def con_dps_mps(a: float):
    vel = a*2*math.pi*RobotParameters.ROBOT_WHEEL_RADIUS/360
    return float(vel)

def conv_msg(a: int, v: int):
    val = a+65535*(a - v > 1000)*-1*(a!=0)
    # if val < 0: 
    #     val = val*(-1)
    return val

@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_speed: float
    right_speed: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float

@dataclass
class RobotParameters:
    """Class for Robot Chassis Parameters"""
    ROBOT_WHEEL_RADIUS = 0.1
    ROBOT_WHEEL_SEPARATION = 0.5
    ROBOT_MAX_LINEAR_M_S = 0.1  # m/s
    ROBOT_MIN_LINEAR_M_S = -0.1 # m/s
    ROBOT_MAX_ANGULAR_R_S = 0.1   # rad/s
    ROBOT_MIN_ANGULAR_R_S = -0.1  # rad/s

global velocity_l
global velocity_r
velocity_l = 0
velocity_r = 0 

class RobotControlNode():
    def __init__(self):
        rospy.init_node("robot_control_node", anonymous=True)

        self.serial_ = rs_485("/dev/ttyUSB0", 1, 8, "E", 115200, 0.2)
        self.client_ = self.serial_.connect_()

        self.robot = robot(RobotParameters.ROBOT_WHEEL_SEPARATION, RobotParameters.ROBOT_WHEEL_RADIUS,
                           RobotParameters.ROBOT_MAX_LINEAR_M_S, RobotParameters.ROBOT_MIN_LINEAR_M_S, 
                           RobotParameters.ROBOT_MAX_ANGULAR_R_S, RobotParameters.ROBOT_MIN_ANGULAR_R_S)

        self.az = motor("ContinusOperationWithSpeed")

        self.twist_subscription = rospy.Subscriber(
            "/cmd_vel",
            Twist,
            self.kine_callback,
        )

        self.twist_subscription
        self.odom_publisher = rospy.Publisher(
            "/odom",
            Odometry,
            queue_size=10,
        )
        rospy.sleep(0.2)
        rospy.loginfo("Infor serial port %s", self.serial_)
        self.twist = Twist()

        rospy.Timer(rospy.Duration(1.0/50.0), self.timer_callback) #publish rate
        # rospy.Timer(rospy.Duration(1.0/50.0), self.pub_callback)

        
    def timer_callback(self, event):
        tf_broadcaster = TransformBroadcaster()        
        robot_state = self.feedback_()
        if robot_state is None: 
            return
        
        robot_orientation = quaternion_from_euler(0,0,robot_state.theta)

        timestamp = rospy.Time.now()

        # transforms
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = robot_state.x_pos
        t.transform.translation.y = robot_state.y_pos
        t.transform.translation.z = 0 #not sure, measure again
        t.transform.rotation = robot_orientation

        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = robot_state.x_pos
        odom_msg.pose.pose.position.y = robot_state.y_pos
        odom_msg.pose.pose.position.z = 0 #not sure, measure again
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = robot_state.v
        odom_msg.twist.twist.angular.z = robot_state.w

        # broadcast and publish
        # self.tf_broadcaster.se
        tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)

    def kine_callback(self, twist: Twist):
        global velocity_l
        global velocity_r
        twist = twist
        velocity_l, velocity_r = self.robot.kinematic_(float(twist.linear.x), float(twist.angular.z))
        vl_ = self.az.directComand_(velocity_l)
        vr_ = self.az.directComand_(velocity_r)
        self.client_.write_registers(vl_[0], vl_[1], slave = 0x03, skip_encode = True)
        # rospy.sleep(0.03)
        self.client_.write_registers(vr_[0], vr_[1], slave = 0x02, skip_encode = True)
    
    def feedback_(self)->SerialStatus:
        global velocity_l
        global velocity_r
        registerSpeedHZ = 0xD0
        vl_feedback = self.client_.read_holding_registers(address=registerSpeedHZ, count = 2, slave =  0x03)
        speedL = conv_msg(vl_feedback.registers[1], velocity_l)
        time.sleep(0.03)
        vr_feedback = self.client_.read_holding_registers(address=registerSpeedHZ, count = 2, slave = 0x02)
        speedR = conv_msg(vr_feedback.registers[1], velocity_r)
        time.sleep(0.03)
        vl_speed = con_dps_mps(-speedL/100)
        vr_speed = con_dps_mps(speedR/100)
        if abs(vl_speed) > 1:
            vl_speed = 0
        if abs(vr_speed) > 1:
            vr_speed = 0
        
        x, y , theta, v, w = self.robot.updateOdometry_(vl_speed, vr_speed, 0.23)
        # rospy.loginfo("x_pos: %f y_pos: %f theta: %f v: %f w: %f vl: %f vr: %f", x, y, theta, v, w, -vl_speed, vr_speed)
        return SerialStatus(-vl_speed, vr_speed, x, y, theta, v, w)
        # x, y , theta, v, w = robot.updateOdometry_(con_dps_mps(-speedL/100), con_dps_mps(speedR/100), 0.17)
        # rospy.loginfo("x_pos: %f y_pos: %f theta: %f v: %f w: %f vl: %f vr: %f", x, y, theta, v, w, con_dps_mps(speedL/100), con_dps_mps(speedR/100))

    
def main(args=None):
    robot_control_node = RobotControlNode()
    rospy.spin()

if __name__=="__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass