import tensorflow as tf
assert(int(tf.__version__.split('.')[0]) >= 2)
import numpy as np

class Detector:
    '''
    Classe Detector che istanzia un detector preaddestrato con la libreria tensorflow
    ''' 
    def __init__(self,model_path):
        '''load del modello da file'''
        self.detect_fn = tf.saved_model.load(model_path)

    def __call__(self, img, threshold=0.5):
        '''chiamata al detector che restituisce le detection sull'immagine passata, fissato il parametro threshold'''
        img = img[:,:,::-1]
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        detections = self.detect_fn(input_tensor)
        num_above_thresh = np.sum( detections['detection_scores'] > threshold )
        detections.pop('num_detections')
        detections = {key: value[0, :num_above_thresh].numpy() for key, value in detections.items()}
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
        return detections