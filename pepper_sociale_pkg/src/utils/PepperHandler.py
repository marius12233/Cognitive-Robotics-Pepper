#!/usr/bin/env python
import rospy
from utils.HeadHandler import HeadHandler
from utils.DetectionInfoHandler import DetectionInfoHandler

class PepperHandler:
    '''Classe che implementa il design pattern Facade per la gestione delle pubblicazioni relative sia alla movimento che alle detection di pepper'''
    def __init__(self):
        '''init degli handler e di un dizionario per la posizione da far assumere a pepper'''
        self._head_handler = HeadHandler()
        self._detection_info_handler=DetectionInfoHandler()
        self._head_position={'left':1,'center':0,'right':-1,'home':0,'start':0}

    def turn_head(self,pos):
        '''ruota la testa nella direzione indicata del parametro pos.
           Posizioni consentite:
                -left: ruota la testa di 90 gradi verso sinistra
                -center: ruota la testa in direzione centrale
                -right: ruota la testa di 90 gradi verso destra
                -start: ruota la testa in direzione centrale (posizione iniziale scelta)
                -home: ruota la testa in direzione centrale (posizione finale scelta)
        '''
        #set dei parametri dell'HeadHandler        
        self._head_handler.set_joint_angles([0.2, self._head_position[pos]])
        #publish dei parametri sul topic designato da HeadHandler
        self._head_handler.publish_joint()
        rospy.loginfo("I'm going to "+pos+ ' position')
    
    def publish_detection(self,detections,head_position):
        '''pubblica una lista delle detection individuate e la relativa posizione nel campo visivo di pepper attraverso il DetectionInfoHandler.'''
        self._detection_info_handler.publish_detection_message(detections,head_position)

    
    
    