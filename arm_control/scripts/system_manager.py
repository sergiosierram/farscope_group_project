#!/usr/bin/python3
#ToDo
#Base off of confience aswell as class matching
#Add In checks to move only after a vacuum/grip has been achieved
#Add In check to only move from bin when vaccuum has been lost
#Add in positional determining and movement with reece
#setup launch file to launch all the needed scripts
#create list of picked objects
# add positions to be done from end effector for picking

import rospy, time
from geometry_msgs.msg import Pose
import json
from std_msgs.msg import Bool, String
from sys import stdin
from sensing_msgs.msg import ObjectArray, Object

class SystemManager(object):
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.init_positions()
        self.main()

    def init_positions(self):
        self.home_pose = Pose()
        self.home_pose.position.x = 0
        self.home_pose.position.y = 350
        self.home_pose.position.z = 450
        self.home_pose.orientation.x = 1.57
        self.home_pose.orientation.y = 0
        self.home_pose.orientation.z = 3.14
        self.home_pose.orientation.w = 0

        self.bin_pose = Pose()
        self.bin_pose.position.x = 0
        self.bin_pose.position.y = 425
        self.bin_pose.position.z = -50
        self.bin_pose.orientation.x = 1.57
        self.bin_pose.orientation.y = 0
        self.bin_pose.orientation.z = 3.14
        self.bin_pose.orientation.w = 0

    def initParameters(self):
        self.machine_vision_topic = rospy.get_param("~machine_vision_topic", "/machine_vision")
        self.output_topic = rospy.get_param("~output_topic", "/output")
        self.home_topic = rospy.get_param("~home_topic", "/home_pos")
        self.pause_topic = rospy.get_param("~pause_topic", "/pause")
        self.emergency_topic = rospy.get_param("~emergency_topic", "/emergency")
        self.mux_rate = rospy.get_param("~rate", 10)
        return

    def initSubscribers(self):
        rospy.Subscriber(self.machine_vision_topic, Pose, self.callbackMachineVision)
        rospy.Subscriber("/detected_objects", ObjectArray, self.callbackObjectDetect)
        rospy.Subscriber("robot/isMoving", String, self.callbackMoving)
        rospy.Subscriber("robot/positions", Pose, self.callbackPosition)
        rospy.Subscriber("/object_postitons", ObjectArray, self.callbackObjectPosition)
        return

    def initPublishers(self):
        self.pub_home = rospy.Publisher(self.home_topic, Pose, queue_size = 10)
        self.pub_output = rospy.Publisher(self.output_topic, Pose, queue_size = 10)
        self.pub_pause = rospy.Publisher(self.pause_topic, Bool, queue_size = 10)
        self.pub_emergency = rospy.Publisher(self.emergency_topic, Bool, queue_size = 10)
        self.pub_shelf = rospy.Publisher('shelf/selector', String, queue_size = 10)
        self.pub_pose  = rospy.Publisher('robot/positions', Pose, queue_size = 10)
        self.pub_vacuum = rospy.Publisher('/vacuum_power', Bool, queue_size=10)
        
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        self.isMoving = False
        self.currentPose = Pose()
        self.shelf_list = []
        self.workOrder = []
        self.max_attempts = 2
        self.hoover_pause = 2
        self.objectPositions = ObjectArray()
        self.object_pos = Object()
        self.adjustedPos = Object()
        return


    def callbackMachineVision(self, msg):
        return

    def callbackObjectDetect(self, msg):
        self.objectmsg = msg
        return

    def callbackMoving(self, msg):
        print(msg.data)
        if(str(msg.data )== "False"):
            self.isMoving = False
            print("Done")
        else:
            print("isMoving")
            self.isMoving = True
        return

    def callbackPosition(self, pose):
        self.currentPose = pose
        return

    def callbackObjectPosition(self, msg):
        self.objectPositions = msg.objects


    def shelfTalker(self, shelf_num):
        msg = String()
        msg.data = str(shelf_num)

        rospy.loginfo(msg)
        self.pub_shelf.publish(msg)
        return

    def positionTalker(self, Pose):
        self.pub_shelf.publish(Pose)
        return
    
    def readWorkOrder(self, workOrder):
        path = '/home/farscoperoom/catkin_ws/src/farscope_group_project/arm_control/scripts/work_order/' + workOrder
        f = open(path)
        data = json.load(f)
        binsID = [] #bin_a bin_b
        outputList = []

        ##create list of bin IDs - bin_A ...
        ids_list =   list ((data["bin_contents"]).keys()  ) 
        valsID = []
        shelfID = 1
        for i in range(len(data["work_order"])):
            vals = list( data["work_order"][i].values())
            binsID.append(vals[0])
            
            if vals[0] in ids_list:
                #print(ids_list.index(vals[0]) + 1)
                self.shelf_list.append(ids_list.index(vals[0]) + 1)
                shelfID = ids_list.index(vals[0]) + 1
            valsID.append(vals[1])
            outputList.append([shelfID, vals[1]])

        # print(outputList.index[0])
        return outputList
    def convertMsgToDict(self, msg):
        val_list = msg.split("\n")
        val_dict = []
        for j in val_list:
            j = j.replace('"','')
            x = j.split(":")
            val_dict.append(x)
        val_dict = dict(val_dict)
        return val_dict
    
    def posePublisher(self, pose):
        self.pub_pose.publish(pose)
        time.sleep(0.1)
        while self.isMoving == True:
            pass
    
    def offsetPosition(self, x_off = 0, y_off = 0, z_off = 0, orientx = 0, orienty = 0, orientz = 0):
        pose = self.currentPose
        pose.position.y += x_off
        pose.position.y += y_off
        pose.position.z += z_off
        pose.orientation.x += orientx
        pose.orientation.y += orienty
        pose.orientation.z += orientz
        self.posePublisher(pose)
        while self.isMoving == True:
            pass

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        time.sleep(1)
        self.workOrder = self.readWorkOrder("test_pick_2.json")
        print(self.workOrder)
        print(self.shelf_list)

        self.posePublisher(self.home_pose)
        input("Ready to Start")
        while not rospy.is_shutdown():
            n = len(self.shelf_list)
            print("N: ", n)
            idx = 0
            while idx < n and not rospy.is_shutdown(): 

                if (self.isMoving == False):
                    
                    
                    self.shelfTalker(self.shelf_list[idx])
                    time.sleep(0.1)
                    
                    while self.isMoving == True:
                        pass

                    tik = time.time()
                    recognition_attempts = 0
                    is_correct_object = False

                    while recognition_attempts < self.max_attempts and is_correct_object == False and not rospy.is_shutdown():
                        timeout = time.time() + 3
                        if recognition_attempts == 1:
                            self.offsetPosition(0,100,130,0,0,-0.3)
                        while time.time() < timeout and is_correct_object == False and not rospy.is_shutdown(): 
                            for i in range(len(self.objectmsg.objects)):
                                if not self.objectmsg.objects == None:
                                    val_dict = self.convertMsgToDict(str(self.objectmsg.objects[i]))
                                    #print(val_dict["class_name"].strip())
                                    #print(self.workOrder[idx][1])

                                    if val_dict["class_name"].strip() == self.workOrder[idx][1]:
                                        is_correct_object = True
                            time.sleep(0.01)
                        
                        recognition_attempts += 1        
                    
                    tok = time.time()
                    if(is_correct_object == True):
                        print("Object Found \nAttempting Pick")
                        time.sleep(0.2)
                        # if recognition_attempts < 2:
                        #     self.offsetPosition(0,100,100,0,0,0)
                        time.sleep(0.5)
                        #Will Need To Check Grip
                        #Move to Position
                        for object in self.objectPositions:
                            if object.class_name ==  self.workOrder[idx][1]:
                                self.object_pos = object
                                print("Class name: "+self.object_pos .class_name+"\n"+"x: "+ str(self.object_pos .x)+"\n"+"y: "+ str(self.object_pos .y)+"\n"+"z: "+ str(self.object_pos.z))
                                print("Adjusted positions")
                                self.adjustedPos.x = self.object_pos.x -40
                                self.adjustedPos.y = 115 - self.object_pos.y
                                self.adjustedPos.z = self.object_pos.z -110
                                print(str(self.adjustedPos.x))
                                print(str(self.adjustedPos.y))
                                print(str(self.adjustedPos.z))
                                time.sleep(0.5)
                                input("Next/Positions")
                                self.offsetPosition(self.adjustedPos.x, self.adjustedPos.z-60, self.adjustedPos.y)
                                self.pub_vacuum.publish(True)
                                time.sleep(2)
                                self.offsetPosition(0,60)
                                time.sleep(self.hoover_pause)
                                self.offsetPosition(100,-self.adjustedPos.z-150, 20)


                        input("Are These There?")


                        self.posePublisher(self.bin_pose)
                        time.sleep(1)
                        self.pub_vacuum.publish(False)
                        time.sleep(self.hoover_pause)

                    else:
                        print("object_not_found")
                    
                    print(tok-tik)
                    input("Next")
                    
                        
                    
                    #print( ( self.objectmsg ).to_list() )
                    #print( ("vals: ", vals[0]) )
                    #for i in ()
                    time.sleep(0.5)

                    # //raw_input("Next?")
                    # time.sleep(0.01)
                    idx += 1
                    
                    #time.sleep(3)
                
                

                    
                

            self.posePublisher(self.home_pose)
            user = input("Finished Shelf List. Want to go again?:").strip()
            if (user == "n" or user == "N"):
                break
            self.rate.sleep()
        return

if __name__ == '__main__':
    try:
        ac = SystemManager("SystemManager")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
