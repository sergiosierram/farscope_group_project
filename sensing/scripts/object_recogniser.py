#!/usr/bin/python3
import rospy, cv2, cv_bridge, roboflow, os
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from sensing_msgs.msg import Object, ObjectArray

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
        self.object_topic = rospy.get_param("~detected_object_topic", "/detected_objects")
        self.bounding_topic = rospy.get_param("~bounding_box_topic", "/bounding_box")
        self.roboflow_info = rospy.get_param("~roboflow_info", {})
        self.image_file = rospy.get_param("~image_file", {})
        self.verbose = rospy.get_param("~verbose", False)
        self.labels = rospy.get_param("~labels", {})
        self.delete_file = rospy.get_param("~delete_file", False)
        self.wait_for_request = rospy.get_param("~wait_for_request", False)
        self.node_rate = rospy.get_param("~rate", 1)
        return
        
    def initSubscribers(self):
        rospy.Subscriber(self.image_topic, Image, self.callbackImage)
        rospy.Subscriber(self.do_topic, Bool, self.callbackDoRecognition)
        return
        
    def initPublishers(self):
        self.pub_objects = rospy.Publisher(self.object_topic, ObjectArray, queue_size = 10)
        self.pub_bounding_box = rospy.Publisher(self.bounding_topic, Image, queue_size = 10)
        return
        
    def initVariables(self):
        self.image = Image()
        self.bridge = cv_bridge.CvBridge()
        self.img = None
        if self.wait_for_request == False:
            self.do_recognition = True
            print("______________________________SELFREQUEST_TRUE")
        else: 
            self.do_recognition = False
            print("______________________________SELFREQUEST_FALSE")
        self.image_is_ready = False
        self.rate = rospy.Rate(self.node_rate)
        
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
        if self.image_is_ready == False:
            try:
                # Create filename to get the image
                file = self.image_file["path"] + self.image_file["filename"]
                self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(file, self.img)
                self.image_is_ready = True
            except:
                print("Error getting image")
        return

    def callbackDoRecognition(self, msg):
        if msg.data:
            rospy.loginfo("[%s] Recognition request received", self.name)
        self.do_recognition = msg.data
        self.image_is_ready = False    
        return
        
        
    def predict(self):
        # Create filename to get the image
        file = self.image_file["path"] + self.image_file["filename"]
        
        # Get prediction from roboflow
        prediction = self.model.predict(file)
        
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
            self.pub_objects.publish([])
        else:
            if self.verbose:
                print("-"*30)
            msg_list = []
            for i in range(noOfItems):
                objects = dict((vals['predictions'][i]) )
                objectName = objects['class']
                if self.labels != {}:
                    objectID = self.labels[objectName]
                else:
                    rospy.logwarn("[%s] Unknown object, using default id 0", self.name)
                    objectID = 0
                if self.verbose:
                    print(objectID, " ", objectName)
                msg = Object() 
   
                msg.class_name = objectName
                msg.x = objects['x']
                msg.y = objects['y']
                msg.width = objects['width']
                msg.height = objects['height']
                msg.confidence = objects['confidence']
                msg_list.append(msg)
                #cv2.rectangle(self.img, (int(msg.x), int(msg.y)), (int(msg.x + msg.width), int(msg.y + msg.height)), (255,0,0), 4)
                cv2.rectangle(self.img, (int(msg.x - msg.width/2), int(msg.y - msg.height/2)), (int(msg.x + msg.width/2), int(msg.y + msg.height/2)), (255,0,0), 4)
            msg_img = self.bridge.cv2_to_imgmsg(self.img, "passthrough")
            self.pub_bounding_box.publish(msg_img)
            self.pub_objects.publish(msg_list)                
          
        #delete the captured image after inference
        if os.path.exists(file) and self.delete_file:
            os.remove(file)
            
        self.image_is_ready = False   
           
        if self.wait_for_request == True:
            self.do_recognition = False
        return
        
    def main(self):
        rospy.loginfo("[%s] Configuration done", self.name)
        while not rospy.is_shutdown():
            
            if self.do_recognition == True and self.image_is_ready == True:
                print("recognise")
                self.predict()
            else:
                print("non image")
            self.rate.sleep()
        return
        
if __name__ == "__main__":
    try:
        print("initial")
        node = ObjectRecognition('ObjectRecogniser')
        print(node)
        print("success")
        
    except rospy.ROSInterruptException:
        print("failed")
        pass
        




#uncomment this to show bounding boxes
#prediction.plot() 





