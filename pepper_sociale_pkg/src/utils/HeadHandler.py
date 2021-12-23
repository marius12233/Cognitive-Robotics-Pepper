#!/usr/bin/env python
import rospy
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed


class HeadHandler:
    '''
    Classe responsabile del movimento dei giunti della testa di pepper.
    '''
    def __init__(self):
        '''init del publisher e dei parametri per il movimento dei giunti '''
        self._pub_joint = rospy.Publisher(
            '/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0,latch=True)
        self._s = JointAnglesWithSpeed()
        self._init_joint()
       
       
        
    def _init_joint(self):
        '''set dei parametri per il movimento della sola testa'''
        self._s.joint_names = ['HeadPitch', 'HeadYaw']
        self._s.relative = 0
        self._s.speed = 0.2

    def set_joint_angles(self,angles):
        '''set dell'angolo di rotazione della testa di pepper'''
        self._s.joint_angles = angles
        
    def publish_joint(self):
        '''publish dei parametri per il movimento dei giunti'''
        self._pub_joint.publish(self._s)
        
    
    
    