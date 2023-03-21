#!usr/bin/python3
import rospy
from sensing_msgs.msg import Object, ObjectArray
from sensor_msgs.msg import Image as rosImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import pyrealsense2 as rs


class PoseEstimation():
    def __init__(self, depth_image_topic, camera_info_topic, bounding_box_topic, position_topic):
        self.bridge = CvBridge()
        rospy.Subscriber(bounding_box_topic, ObjectArray, self.getLocationCallback)
        # rospy.Subscriber(depth_image_topic, rosImage, self.getLocationCallback)

        rospy.Subscriber(depth_image_topic, rosImage, self.depthImageCallback)
        rospy.Subscriber(camera_info_topic, CameraInfo, self.cameraInfoCallback )
        self.pub = rospy.Publisher(position_topic, ObjectArray, queue_size=10)
        self.intrinsics = None
        self.objPositions = []

    def test(self, data):
        print("recieved callback")

    def depthImageCallback(self, data):
        
        try:
            # convert image message to cv image and np array for yolo
            cv_image = self.bridge.imgmsg_to_cv2(data, )
            # depth_image = np.asanyarray(cv_image)
            # self.depth_image = depth_image

            self.depth_image = cv_image
            
        except CvBridgeError as e:
            print(e)
            return

    
    def cameraInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except:
            print("Camera_info error")
            return


    def getLocationCallback(self, obj):
        print("got locations")
        self.objPositions=[]
        #get bounding box center
        for object in obj.objects:
            # bbx = min(640,int(object.x +(object.width/2)))
            # bby = min(479,int(object.y+((object.height)/2)))
            bbx = int(object.x)
            bby = int(object.y)
        
                    
                #get depth at center of bounding box
            centre_z = int(self.depth_image[bby, bbx])

            #get x y positions of center of bounding box

            centre_x = int(centre_z *(bbx-self.intrinsics.ppx)/self.intrinsics.fx)
            centre_y = int(centre_z *(bby-self.intrinsics.ppy)/self.intrinsics.fy)
            print("bbx="+ str(object.x)+" "+"bby="+str(object.y)+" "+"z="+str(centre_z))
            if centre_x != 0 and centre_y != 0 and centre_z != 0 or True:
                self.objPositions.append({'class':object.class_name, 'xPos':centre_x, 'yPos': centre_y, 'zPos':centre_z})
        self.publishPositions()


    def publishPositions(self):
        msgArray = []
        for object in self.objPositions:

            msg = Object()
        
            msg.class_name = object['class']
            msg.x = object['xPos']
            msg.y = object['yPos']
            msg.z = object['zPos']
        
            msgArray.append(msg)
        print(self.objPositions)
        
        self.pub.publish(msgArray)


def main():

    depth_image_topic = '/camera/depth/image_rect_raw'
    #depth_image_topic = '/camera/aligned_depth_to_color/image_raw'

    camera_info_topic = '/camera/depth/camera_info'
    bounding_box_info_topic = '/detected_objects'
    positions_topic= '/object_postitons'
    locator = PoseEstimation(depth_image_topic,camera_info_topic,bounding_box_info_topic,positions_topic)
    print
    rospy.spin()



if __name__ == "__main__":

    try:
        rospy.init_node("getObjectPositions", anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
