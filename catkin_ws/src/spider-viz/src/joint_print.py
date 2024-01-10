#!/usr/bin/env python
import serial 
import rospy
import numpy as np
import time

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray, WrenchStamped
from moveit_commander import MoveGroupCommander
from std_msgs.msg import Float32MultiArray

pub = None

def publisher():
    global pub
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Read Data
        if ESP_port.isOpen():
            read_val = str(ESP_port.readline().decode('utf-8'))
            joint_angles = parse_input(read_val)
            
            print(joint_angles)
            conv_angles = convert_states(joint_angles)
            push_states(conv_angles)

def convert_states(joint_angles):
    conv_angles = np.array(joint_angles)
    
    conv_angles = conv_angles - np.array([0.17, 1.1, 0.17, 1.1, 0.17, 1.1, 0.17, 1.1])
    conv_angles = conv_angles * np.array([-1, 1, 1, 1, 1, 1, -1, 1])
    
    
    
    return conv_angles

def push_states(angles):
    js = JointState()
    js.header.stamp = rospy.Time.now()
    js.name = ["Upper0", "Lower0", "Upper1", "Lower1", "Upper2", "Lower2", "Upper3", "Lower3"]
    js.position = angles
    pub.publish(js)


def parse_input(read_val):
    if read_val != "":
        try:
            angles = [float(i) * (np.pi/180) for i in read_val.split(" ")]
            return angles
        except:
            print("failed split")
    return []
    
    

if __name__ == '__main__':
    rospy.init_node('ESP_Interface') 
    
    ESP_port = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

    try:
        publisher()
    except rospy.ROSInterruptException: pass
    