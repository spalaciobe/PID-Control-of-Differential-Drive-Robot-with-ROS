#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import numpy as np
from simple_pid import PID

kp_vel = -0.1
ki_vel = 0
kd_vel = 0

kp_angular_vel = 0.2
ki_angular_vel = 0
kd_angular_vel = 0

x = 0.0
y = 0.0
theta = 0.0

pid_angular_vel = PID(kp_angular_vel, ki_angular_vel, kd_angular_vel, setpoint=0)
# pid_angular_vel.output_limits = (-3.0, 3.0)
pid_angular_vel.set_auto_mode(enabled=True)


pid_vel = PID(kp_vel, ki_vel, kd_vel, setpoint=0)
# pid_vel.output_limits = (-0.4, 0.4)
pid_vel.set_auto_mode(enabled=True)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("pid_ddr_controller_node")

sub = rospy.Subscriber("/odom", Odometry, newOdom)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(20)

linear_velocity=0
angle_control=0

x_goal = float(input("Enter the desired x coordinate: "))
y_goal = float(input("Enter the desired y coordinate: "))

x_error = x_goal - x
y_error = y_goal - y
theta_goal = np.arctan2(y_error, x_error)
euc_distance = np.sqrt(y_error**2+x_error**2)

while not rospy.is_shutdown() and euc_distance>0.05:
    x_error = x_goal - x
    y_error = y_goal - y

    theta_goal = np.arctan2(y_error, x_error)  
    if theta < 0:
        theta += 2*np.pi 

    # Implementing additional logic
    if theta_goal < -np.pi/4 or theta_goal > np.pi/4:
        if y_goal < 0 and y < y_goal:
            theta_goal = -2*np.pi + theta_goal
        elif y_goal >= 0 and y > y_goal:    
            theta_goal = 2*np.pi + theta_goal

    # Handle theta wrapping around
    if theta > np.pi - 0.1 and theta_goal <= 0:
        theta_goal = 2*np.pi + theta_goal
    elif theta < -np.pi + 0.1 and theta_goal > 0:
        theta_goal = -2*np.pi + theta_goal    

    pid_angular_vel.setpoint = theta_goal
    angular_vel = pid_angular_vel(theta)

    euc_distance = np.sqrt(y_error**2+x_error**2)
    linear_velocity=pid_vel(euc_distance)    

    speed.linear.x = linear_velocity
    speed.angular.z = angular_vel

    pub.publish(speed)

print("Goal reached")
speed.linear.x = 0
speed.angular.z = 0
pub.publish(speed)

r.sleep()
