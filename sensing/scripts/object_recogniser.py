#!/usr/bin/python3
import rospy, cv2, cv_bridge, roboflow, os
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ObjectRecognition():
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()
        return
        
    def initParameters(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.do_topic = rospy.get_param("~do_recognition_topic", "/do_recognition")
        self.roboflow_info = rospy.get_param("~roboflow_info", {})
        self.image_file = rospy.get_param("~image_file", {})
        self.verbose = rospy.get_param("~verbose", False)
        self.labels = rospy.get_param("~labels", {})
        self.delete_file = rospy.get_param("~delete_file", False)
        self.wait_for_request = rospy.get_param("~wait_for_request", False)
        return
        
    def initSubscribers(self):
        rospy.Subscriber(self.image_topic, Image, self.callbackImage)
        rospy.Subscriber(self.do_topic, Bool, self.callbackDoRecognition)
        return
        
    def initPublishers(self):
        self.pub_predict = rospy.Publisher("/predicting", Bool, queue_size = 10)
        return
        
    def initVariables(self):
        self.image = Image()
        self.bridge = cv_bridge.CvBridge()
        if self.wait_for_request == False:
            self.do_recognition = True
        else: 
            self.do_recognition = False
        self.rate = rospy.Rate(0.3)
        # Roboflow stuff
        if self.roboflow_info != {}:
            rospy.loginfo("[%s] Loading roboflow info", self.name)
            # Extract parameters from the dict loaded from the .yaml file
            api = self.roboflow_info["api_key"]
            workspace_name = self.roboflow_info["workspace"]
            project_name = self.roboflow_info["project"]
            version = self.roboflow_info["version"]
            # Create roboflow objects and get model
            rf = roboflow.Roboflow(api_key=api)
            project = rf.workspace(workspace_name).project(project_name)
            self.model = project.version(version).model
        else:
            rospy.logerr("[%s] Failed to load roboflow info", self.name)
            self.model = None
        return
        
    def callbackImage(self, msg):
        if self.get_new_image == True:
            try:
                img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite('object.jpg', img)
                self.get_new_image = False
            except:
                print("Error getting image")
        return

    def callbackDoRecognition(self, msg):
        self.do_recognition = msg.data
        return
        
        
    def predict(self):
        # Create filename to store the image
        file = self.image_file["path"] + self.image_file["filename"]
        
        # Get prediction from roboflow
        prediction = self.model.predict("object.jpg") 
        
        # Convert predictions to JSON
        prediction.json()
        
        #display predictions
        if self.verbose:
            print(prediction.json()) 
        
        vals = dict(prediction.json())
        
        #number of objects detected
        noOfItems = len(vals['predictions'])  
        
        if(noOfItems == 0): #no object in the frame
          print("No items")
        else:
          for i in range(noOfItems):
            objects = dict((vals['predictions'][i]) )
            objectName = objects['class']
            if self.labels != {}:
                objectID = self.labels[objectName]
            else:
                ropsy.logwarn("[%s] Unknown object, using default id 0", self.name)
                objectID = 0
            print(objectID, " ", objectName)
          
        #delete the captured image after inference
        if os.path.exists(file) and self.delete_file:
            os.remove(file)   
           
        if self.wait_for_request == True:
            self.do_recognition = False
        return
        
    def main(self):
        rospy.loginfo("[%s] Configuration done", self.name)
        while not rospy.is_shutdown():
            if self.do_recognition == True:
                self.predict()
            self.rate.sleep()
        return
        
if __name__ == "__main__":
    try:
        node = ObjectRecognition('ObjectRecognizer')
    except rospy.ROSInterruptException:
        pass
        




#uncomment this to show bounding boxes
#prediction.plot() 





