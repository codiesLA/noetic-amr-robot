import math

global x_pos
global y_pos
global theta 
global v
global w 

x_pos = 0
y_pos = 0
theta = 0
v = 0
w = 0

class ROBOT:
    def __init__(self, wheel_separation: float, wheel_radius: float, 
                 max_linear: float, min_linear: float,
                 max_angular: float, min_angular: float,
                ):
        self.ROBOT_WHEEL_SEPARATION = wheel_separation
        self.ROBOT_WHEEL_RADIUS = wheel_radius
        self.ROBOT_MAX_LINEAR_M_S = max_linear
        self.ROBOT_MIN_LINEAR_M_S = min_linear
        self.ROBOT_MAX_ANGULAR_R_S = max_angular
        self.ROBOT_MIN_ANGULAR_R_S = min_angular
    
    def kinematic_(self, linear_x: float, angular_z: float):
        if linear_x > self.ROBOT_MAX_LINEAR_M_S:
            linear_x = self.ROBOT_MAX_LINEAR_M_S
        if linear_x < self.ROBOT_MIN_LINEAR_M_S:
            linear_x = self.ROBOT_MIN_LINEAR_M_S
        if angular_z > self.ROBOT_MAX_ANGULAR_R_S:
            angular_z = self.ROBOT_MAX_ANGULAR_R_S
        if angular_z < self.ROBOT_MIN_ANGULAR_R_S:
            angular_z = self.ROBOT_MIN_ANGULAR_R_S

        if angular_z == 0: 
            self.vel_l = linear_x
            self.vel_r = linear_x
        else:
            self.vel_l = linear_x - angular_z * self.ROBOT_WHEEL_SEPARATION / 2
            self.vel_r = linear_x + angular_z * self.ROBOT_WHEEL_SEPARATION / 2
        
        if self.vel_l > 0 and self.vel_r > 0:
            self.vel_l = self.vel_l*(-1)
            self.vel_r = self.vel_r
        if self.vel_l > 0 and self.vel_r < 0:
            self.vel_l = self.vel_l*(-1)
            self.vel_r = self.vel_r*(-1)
        if self.vel_l < 0 and self.vel_r > 0:
            self.vel_l = self.vel_l
            self.vel_r = self.vel_r
        if self.vel_l < 0 and self.vel_r < 0:
            self.vel_l = self.vel_l
            self.vel_r = self.vel_r*(-1)

        #------------rad/s---------------#
        self.vel_l = self.vel_l/self.ROBOT_WHEEL_RADIUS 
        self.vel_r = self.vel_r/self.ROBOT_WHEEL_RADIUS
        self.v_l = self.vel_l
        self.v_r = self.vel_r
        #------------deg/s---------------#
        self.vel_l = self.vel_l*360/(2*math.pi) 
        self.vel_r = self.vel_r*360/(2*math.pi)
        #------------hertz---------------#
        self.vel_l = self.vel_l*100 
        self.vel_r = self.vel_r*100

        return int(self.vel_l),  int(self.vel_r)
    
    def updateOdometry_(self, vl_, vr_, time_):
        global x_pos
        global y_pos
        global theta 
        global v
        global w

        delta_l = vl_ * time_ * self.ROBOT_WHEEL_RADIUS #*
        delta_r = vr_ * time_ * self.ROBOT_WHEEL_RADIUS #*
        delta_center = (delta_l + delta_r) / 2

        x_pos += delta_center * math.cos(theta)
        y_pos += delta_center * math.sin(theta)
        theta += (delta_r - delta_l) / self.ROBOT_WHEEL_SEPARATION

        v = (1 / 2)*(vr_ + vl_) # https://en.wikipedia.org/wiki/Differential_wheeled_robot
        w = (1 / self.ROBOT_WHEEL_SEPARATION) * (vr_ - vl_) # https://en.wikipedia.org/wiki/Differential_wheeled_robot

        return [float(x_pos), float(y_pos), float(theta), float(v), float(w)]
        