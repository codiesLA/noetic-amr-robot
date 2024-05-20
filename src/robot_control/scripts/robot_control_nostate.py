import sys
sys.path.append('/home/codybui/dev_robot/src/robot_control/include')
from RS485 import rs_485
from AZDKD import AZDKD as motor
from DIFFROBOT import ROBOT as robot
import rospy
import math
import time
from geometry_msgs.msg import Twist

global velocity_l
global velocity_r
global ROBOT_WHEEL_SEPARATION
global ROBOT_ROBOT_WHEEL_RADIUS
global ROBOT_MAX_LINEAR_M_S
global ROBOT_MIN_LINEAR_M_S
global ROBOT_MAX_ANGULAR_R_S
global ROBOT_MIN_ANGULAR_R_S


ROBOT_WHEEL_RADIUS = 0.1
ROBOT_WHEEL_SEPARATION = 0.5
ROBOT_MAX_LINEAR_M_S = 40  # m/s
ROBOT_MIN_LINEAR_M_S = -40 # m/s
ROBOT_MAX_ANGULAR_R_S = 0.1   # rad/s
ROBOT_MIN_ANGULAR_R_S = -0.1  # rad/s
velocity_l = 0
velocity_r = 0

robot = robot(ROBOT_WHEEL_SEPARATION, ROBOT_WHEEL_RADIUS,
              ROBOT_MAX_LINEAR_M_S, ROBOT_MIN_LINEAR_M_S,
              ROBOT_MAX_ANGULAR_R_S, ROBOT_MIN_ANGULAR_R_S)

az = motor("ContinusOperationWithSpeed")

serial_ = rs_485("/dev/ttyUSB0", 1, 8, "E", 115200, 0.5)
client = serial_.connect_()
rospy.loginfo(client)

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    val = int(val, 2)
    
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return float(val/100) 

def con_dps_mps(a: float):
    vel = a*2*math.pi*ROBOT_WHEEL_RADIUS/360
    return vel

def conv_msg(a: int, v: int):
    val = a+65535*(a - v > 1000)*-1*(a!=0)
    # if val < 0: 
    #     val = val*(-1)
    return val

def kine_callback(twist: Twist):
    global velocity_l
    global velocity_r
    twist = twist
    velocity_l, velocity_r = robot.kinematic_(float(twist.linear.x), float(twist.angular.z))
    vl_ = az.directComand_(velocity_l)
    vr_ = az.directComand_(velocity_r)
    # serial_.write_value_(vl_[0], vl_[1], 0x03)
    client.write_registers(vl_[0], vl_[1], slave = 0x03, skip_encode = True)
    time.sleep(0.05)
    client.write_registers(vr_[0], vr_[1], slave = 0x02, skip_encode = True)

def feedback_(event):
    global velocity_r
    global velocity_l
    registerSpeedHZ = 0xD0
    vl_feedback = client.read_holding_registers(address=registerSpeedHZ, count = 2, slave =  0x03)
    # rospy.loginfo(vl_feedback.registers)
    speedL = conv_msg(vl_feedback.registers[1], velocity_l)
    time.sleep(0.03)
    vr_feedback = client.read_holding_registers(address=registerSpeedHZ, count = 2, slave = 0x02)
    # rospy.loginfo(vr_feedback.registers)
    speedR = conv_msg(vr_feedback.registers[1], velocity_r)
    time.sleep(0.03)
    if velocity_l == 0:
        speedL = 0
    if velocity_r == 0:
        speedR = 0
    # rospy.loginfo("%f %f %f %f", velocity_l,velocity_r, sp)
    # rospy.loginfo(velocity_r)
    x, y , theta, v, w = robot.updateOdometry_(con_dps_mps(-speedL/100), con_dps_mps(speedR/100), 0.17)
    rospy.loginfo("x_pos: %f y_pos: %f theta: %f v: %f w: %f vl: %f vr: %f", x, y, theta, v, w, con_dps_mps(speedL/100), con_dps_mps(speedR/100))

def main():
    rospy.init_node("robot_control_nostate_node", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, kine_callback)
    rospy.Timer(rospy.Duration(1.0/10.0), feedback_)
    rospy.spin()

if __name__=="__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass