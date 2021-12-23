# Documentazione Pepper Sociale

### Directory Tree
![DirectoryTree](https://user-images.githubusercontent.com/58516353/105385229-65fac780-5c13-11eb-8ccf-01d6796f8a02.PNG)

### pepper_sociale.launch
Il launchfile permette di eseguire tutti i nodi con il solo comando:
```sh
roslaunch pepper_sociale_pkg pepper_sociale_pkg.launch
```
##### Il codice
Segue il contenuto del file
```html
<launch>
  <node name="speech_server" pkg="pepper_sociale_pkg" type="speech_server" output="screen"/>
  <node name="pepper_speech" pkg="pepper_sociale_pkg" type="pepper_speech" output="screen"/>
  <node name="detector_server" pkg="pepper_sociale_pkg" type="detector_server" output="screen"/>
  <node name="pepper_object_recognition" pkg="pepper_sociale_pkg" type="pepper_object_recognition" output="screen"/>
</launch>
```
### detector_server
Il file fornisce un nodo chiamato detector_server che mette a disposizione un servizio chiamato detect_service.
Utilizzando il servizio detect_service viene chiamata la funzione detect_image che si occupa di 
utilizzare il detector, istanziato appena viene lanciato il file, per fare la detection degli oggetti
nell'immagine.
#### Il codice
Segue una spiegazione delle principali fasi del programma
```python
if __name__=="__main__":
    detection_server()
```
#### Funzioni
```python
    def detect_image(msg):
        """
        La funzione è una callback che viene chiamata quando si invoca il servizio detect_service.
        Essa riceve in ingresso un messagio di tipo DetectRequest, dal quale preleva un oggetto Image
        che passa a un Detector. Il detector restituisce un istanza di tipo Detection2DArray, che questa
        funzione restituisce in output.
        """
```
```python
    def detection_server():
        """
        Questa funzione inizializza il nodo detector_server, per poi mettere a disposizione il servizio detect_service.
        """
        rospy.init_node('detector_server') #dichiarazione del nodo
        s = rospy.Service('detect_service', Detect, detect_image) #dichiarazione del servizio detect_service.
        rospy.spin()
```
#### Detect
Detect è un servizio così composto
```python
sensor_msgs/Image img
---
vision_msgs/Detection2DArray det
```

### pepper_object_recognition

Questo file contiene il nodo, chiamato pepper_object_recognition, che ha il compito di coordinare le azione degli altri nodi.
Rappresenta il core del progetto, infatti permette al robot di girare la testa nell'ordine e nelle posizioni
prestabilite (left, center, right). In ognuna di queste posizioni ottiene un'immagine richiamando il metodo __rospy.wait_for_message()__ sul topic relativo alla camera frontale del robot.
Richiama quindi il servizio per effettuare la detection e salva lo stato della testa e i risultati della detection in un unico messaggio che pubblica
sul topic /detection_and_head_position.

#### Il codice
Segue una spiegazione delle principali fasi del programma

Inizializza i nodi e registra i topic dei Publisher al nodo master.
```python
rospy.init_node('pepper_object_recognition')
pub_detection = rospy.Publisher('detection_and_head_position', DetectionInfo, queue_size=0)
pepper_handler=PepperHandler()
```
pubblica un messaggio di tipo DetectionInfo, utilizzando una lista vuota e lo stato di start per indicare che è nella posizione di start e sta quindi per iniziare a girare la testa
```python
rospy.sleep(2.0) # attende 2 secondi all'inizio
pepper_handler.turn_head('start') # gira la testa al centro
pepper_handler.publish_detection([],'start') 
```
utilizza il detector su un'immagine prima di iniziare a far girare la testa al robot per ultimare l'inizializzazione della rete e consentire un tempo costante per le successive detection 
```python
apply_detection() 
rospy.loginfo("Detection service ready")
```
gira la testa, in ordine, prima a sinistra, poi al centro e poi a destra
```python
for pos in ['left','center','right']:  
    pepper_handler.turn_head(pos) #gira la testa nella posizione corrente
    rospy.sleep(2.0) #attendi che la testa si fermi e l'immagine sia stabile e non in movimento
    detection_msg=apply_detection() #chiamata al servizio per la detection
    pepper_handler.publish_detection(detection_msg.detections,pos) #pubblica il risultato della detection e l'informazione
```
torna nella posizione di home (mette la testa in posizione centrale) e pubblica lo stato home per indicare che ha terminato le operazioni
```python
pepper_handler.turn_head('home') 
rospy.sleep(1.0) 
pepper_handler.publish_detection([],'home')
```
#### Funzioni
```python
def detect_image_client(img):
    """
    funzione che chiama il servizio detect_service, fungendone da Client.
    Riceve un messaggio di tipo Image e restituisce un oggetto del tipo Detection2DArray
    """
```
```python
def apply_detection():
    current_img_msg = rospy.wait_for_message("/pepper_robot/camera/front/camera/image_raw", Image) #si iscrive al topic messo a disposizione da Pepper, prende l'immagine corrente e annulla l'iscrizione 
    detection_msg=detect_image_client(current_img_msg)
    return detection_msg
```

### pepper_speech
Questo file inizializza un nodo, chiamato pepper_speech,il quale si occupa di tenere traccia dello stato in cui si trova il robot e degli oggetti individuati dal detector in ognuno di questi stati. Utilizza un Subscriber che si iscrive al topic detection_and_head_position sul
quale pepper_object_recognition pubblica le informazioni appunto sullo stato e sugli oggetti.
Quando pepper_object_recognition indica la fine delle sue operazioni, il nodo costruisce la stringa da inviare
al servizio animated_speech, che permette di interagire con le API di NAOqi e consente al robot di parlare.


#### Il codice
Segue una spiegazione delle principali fasi del programma
```python
# initializza il nodo
rospy.init_node('pepper_speech')
# dizionario che traccia degli oggetti identificati nelle posizioni assunte dalla testa di Pepper
pos_obj = {"left": {}, "center": {}, "right": {}}

shd = rospy.Subscriber("detection_and_head_position",
                       DetectionInfo, rcv_detection_and_head_pos)
...
#in attesa dell'attivazione della callback
rospy.spin()
...
```
#### Funzioni
```python
def call_srv(text):
    """
    Funzione che chiama il servizio animated_say, che permette a Pepper di parlare.
    """
```
```python
def rcv_detection_and_head_pos(msg):
    """
    Funzione di callback invocata quando viene ricevuto un messaggio DetectInfo.
    Essa permette a Pepper di:
    - inviare un messaggio di saluto prima che inizi a girare la testa
    - tenere traccia degli oggetti in ogni posizione in cui si trova il robot
    - quando il robot arriva nella posizione di home, costruisce la frase che Pepper dovrà pronunciare.
    """
    global pos_obj
    current_pos = msg.head_position
```
La callback differisce il suo comportamento in tre fasi:
- Prima fase
    ```python
    if current_pos == 'start':
        call_srv("Hello to everyone. I'm going to inspect the room.")
    ```
- Seconda fase
    ```python
    elif current_pos != 'home': # se non sono nella posizione di home riempie il dizionario con gli oggetti in ogni posizione
        detected_objs = {}
        for d in msg.detections:
            c = d.results[0].id
            detected_objs[classmap[c]] = 1
        pos_obj[current_pos] = detected_objs
    ```
- Terza fase
    ```python
    else: # costruisce la stringa da inviare al service per permettere a Pepper di parlare
        content = "I can see "
        for position in ['left', 'center', 'right']:
            content += " at "+position+' : '
            classes = pos_obj[position].keys()
            if len(classes) <= 0:
                content += "nothing, "
            for _cls in classes:
                content += _cls+", "
        content+=" goodbye."
        call_srv(content)
    ```





### speech_server
Questo file sefinisce la classe AnimatedSpeech.
Questa classe fornisce un proxy verso AlAnimatedSpeech che utilizza le API di NAOqi
per permettere al robot di parlare. Inoltre fornisce un Service per dare la possibilità all'utente finale
di utilizzare tali API.
#### Il codice
Segue una spiegazione delle principali fasi del programma
```python
if __name__=="__main__":
    pub = AnimatedSay()
    rospy.spin()
```
#### Metodi
```python
class AnimatedSay(NaoqiNode):
    def __init__(self):
        """
        Inizializza il nodo animated_speech
        """
        NaoqiNode.__init__(self,'animated_speech')
        #set pepper ip
        self.pip="10.0.1.230"
        self.connectNaoQi()    
        pass
```
```python
    def say(self,data):
        """
        Utilizza il metodo say dell'oggetto proxy ALAnimatedSpeech
        """
        rospy.loginfo("SPEECH SERVICE CALL: %s" % data.message)
        self.speech.say(data.message)
        #rospy.loginfo("END: %s" % data.message)
        return True
```
```python
    def connectNaoQi(self):
        """
        Istanzia il proxy verso le API di NAOqi e mette a disposizione un servizio per utilizzarle.
        """
        self.speech=self.get_proxy("ALAnimatedSpeech")
        self.s = rospy.Service('animated_say', Say, self.say)
```
#### Say
Say è un servizio così composto
```python
string message
---
bool result
```

### UML Class Diagram
![cogrob](https://user-images.githubusercontent.com/58516353/105393241-677cbd80-5c1c-11eb-90eb-a41f74d3f2eb.png)

###  DetectionInfoHandler

Classe responsabile delle pubblicazioni delle detection e della relativa posizione nel campo visivo di pepper 
#### Metodi
```python
class DetectionInfoHandler:
    def __init__(self):
        #init del messaggio DetectionInfo e del relativo publisher
        self._message=DetectionInfo()
        self._pub_detection = rospy.Publisher('detection_and_head_position', DetectionInfo, queue_size=0)
```
```python
    def publish_detection_message(self,detections,head_position):
        #costruzione del messaggio e publish sul topic
```
 
#### DetectionInfo
DetectionInfo è un messaggio così composto
```python
Header header
vision_msgs/Detection2D[] detections
string head_position
```



###  HeadHandler

Classe responsabile del movimento dei giunti della testa di pepper
#### Metodi
```python
class HeadHandler:
    def __init__(self):
        #init del publisher e dei parametri per il movimento dei giunti 
        self._pub_joint = rospy.Publisher(
            '/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0,latch=True)
        self._s = JointAnglesWithSpeed()
        self._init_joint()
```      
       
```python      
    def _init_joint(self):
        #set dei parametri per il movimento della sola testa
```
```python
    def set_joint_angles(self,angles):
        #set dell'angolo di rotazione della testa di pepper
```
```python
    def publish_joint(self):
        #publish dei parametri per il movimento dei giunti
```
###  PepperHandler

Classe che implementa il design pattern Facade per la gestione delle pubblicazioni relative sia al movimento che alle detection di pepper
#### Metodi
```python
class PepperHandler:
    def __init__(self):
        #init degli handler e di un dizionario per la posizione da far assumere a pepper
        self._head_handler = HeadHandler()
        self._detection_info_handler=DetectionInfoHandler()
        self._head_position={'left':1,'center':0,'right':-1,'home':0,'start':0}
```      
       
```python     
    def turn_head(self,pos):
        '''ruota la testa nella direzione indicata del parametro pos.
           Posizioni consentite:
                -left: ruota la testa di 90° verso sinistra
                -center: ruota la testa in direzione centrale
                -right: ruota la testa di 90° verso destra
                -start: ruota la testa in direzione centrale (posizione iniziale scelta)
                -home: ruota la testa in direzione centrale (posizione finale scelta)
        '''
```
```python
    def set_joint_angles(self,angles):
        #set dell angolo di rotazione della testa di pepper
```
```python
    def publish_detection(self,detections,head_position):
        '''pubblica una lista delle detection individuate e la relativa posizione nel campo visivo di pepper 
            attraverso il DetectionInfoHandler.'''
```

###  Detector
Classe Detector che istanzia un detector preaddestrato con la libreria tensorflow
#### Metodi
```python
class Detector:
    def __init__(self,model_path):
        #load del modello da file
```
```python
    def __call__(self, img, threshold=0.5):
        #chiamata al detector che restituisce le detection sull'immagine passata, fissato il parametro threshold
```
###  Classmap
Dizionario contenente l associazione tra tutte le classi individuabili col detector ed il relativo id
#### Il codice
```python
category_map = {
    1: 'person',
    ...,
    90: 'toothbrush'
}
category_index = {
    1: {'id': 1, 'name': 'person'},
    ...
    90: {'id': 90, 'name': 'toothbrush'}
}
```
