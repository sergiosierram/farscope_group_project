#!/usr/bin/python3
import rospy, time
from geometry_msgs.msg import Pose
import json
from std_msgs.msg import Bool, String
from sys import stdin

class SystemManager(object):
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

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
        rospy.Subscriber("robot/isMoving", String, self.callbackMoving)
        rospy.Subscriber("robot/positions", Pose, self.callbackPosition)
        return

    def initPublishers(self):
        self.pub_home = rospy.Publisher(self.home_topic, Pose, queue_size = 10)
        self.pub_output = rospy.Publisher(self.output_topic, Pose, queue_size = 10)
        self.pub_pause = rospy.Publisher(self.pause_topic, Bool, queue_size = 10)
        self.pub_emergency = rospy.Publisher(self.emergency_topic, Bool, queue_size = 10)
        self.pub_shelf = rospy.Publisher('shelf/selector', String, queue_size = 10)
        self.pub_pose  = rospy.Publisher('robot/positions', Pose, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        self.isMoving = False
        self.currentPose = Pose()
        self.shelf_list = []
        self.workOrder = []
        return


    def callbackMachineVision(self, msg):
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
        path = 'work_order/' + workOrder
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


    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)

        self.workOrder = self.readWorkOrder("test_pick_2.json")
        print(self.workOrder)
        print(self.shelf_list)

        while not rospy.is_shutdown():
            n = len(self.shelf_list)
            print("N: ", n)
            idx = 0
            while idx < n and not rospy.is_shutdown():

                if (self.isMoving == False):
                    
                    time.sleep(0.1)
                    self.shelfTalker(self.shelf_list[idx])
                    # //raw_input("Next?")
                    # time.sleep(0.01)
                    idx += 1
                    time.sleep(1)
                    
                    print("IDX: ", idx)

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
