#!/usr/bin/python3
import rospy, cv2, cv_bridge, roboflow, os
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ObjectRecognition():
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name, anonymous = True)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()
        return
        
    def initParameters(self):
        return
        
    def initSubscribers(self):
        rospy.Subscriber("/camera/color/image_raw", Image, self.callbackImage)
        rospy.Subscriber("/force_new_image", Bool, self.callbackForceNewImage)
        return
        
    def initPublishers(self):
        self.pub_predict = rospy.Publisher("/predicting", Bool, queue_size = 10)
        return
        
    def initVariables(self):
        self.image = Image()
        self.bridge = cv_bridge.CvBridge()
        self.get_new_image = True
        rf = roboflow.Roboflow(api_key="jdOkPzYHJ2d3fxl3sO2H")
        project = rf.workspace("manny").project("amazon_picking_challenge")
        self.model = project.version("3").model
        self.rate = rospy.Rate(0.3)
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
        
    def callbackForceNewImage(self, msg):
        if msg.data == True:
            self.get_new_image = True
        return
        
    def predict(self):
        prediction = self.model.predict("object.jpg") 
        
        # Convert predictions to JSON
        prediction.json()
        
        print(prediction.json()) #display predictions
        
        vals = dict(prediction.json())
        
        labels = {"Bin": 1, "Crackers": 2,"Crayons": 3, "Cups": 4, "Dovesoap": 5, "Duck": 6, "GreenToy": 7,
        "JokesBook": 8, "MarkTwainBook": 9, "Markers": 10, "MetalCup": 11, "Netting": 12, "OutletPlugs": 13,
        "Pencils": 14, "Plug": 15, "QuickNotes": 16, "Sketchers": 17, "Soap": 18, "SpareParts": 19,
        "Squarenotes": 20, "StickyNotes": 21, "Straw": 22, "Tools": 23, "Treats": 24, "YellowToy": 25, "Brush":26}

        noOfItems = len(vals['predictions'])  #number of objects detected
        
        if(noOfItems == 0): #no object in the frame
          print("No items")
        else:
          for i in range(noOfItems):
            objects = dict((vals['predictions'][i]) )
            objectName = objects['class']
            objectID = labels[objectName]
            print(objectID, " ", objectName)
          
        #delete the captured image after inference
        if os.path.exists("object.jpg"):
            #os.remove("object.jpg") 
            pass   
            
        self.get_new_image = True  
        return
        
    def main(self):
        print("Here")
        while not rospy.is_shutdown():
            if self.get_new_image == False:
                self.predict()
            self.rate.sleep()
        return
        
if __name__ == "__main__":
    try:
        node = ObjectRecognition('ImageTaker')
    except rospy.ROSInterruptException:
        pass
        




#uncomment this to show bounding boxes
#prediction.plot() 





