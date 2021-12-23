from pepper_sociale_pkg.msg import DetectionInfo
import rospy

class DetectionInfoHandler:
    '''
    Classe responsabile delle pubblicazioni delle detection e della relativa posizione nel campo visivo di pepper
    '''
    def __init__(self):
        '''init del messaggio e del publisher'''
        self._message=DetectionInfo()
        self._pub_detection = rospy.Publisher('detection_and_head_position', DetectionInfo, queue_size=0)

    def publish_detection_message(self,detections,head_position):
        '''costruzione del messaggio e publish sul topic'''
        self._message.detections=detections
        self._message.head_position=head_position
        self._pub_detection.publish(self._message)
        

