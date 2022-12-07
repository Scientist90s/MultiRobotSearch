#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import json
import pandas
import pandas as pd
# import time


class PublishList():
    def __init__(self):
        self.publishFlag = True
        self.nofagents = 5
        self.Q = 10
        self.setpointPub = []
        self.drone_pose_subscriber = []
        self.thresh = 0.5
        self.checkSetpointFlag = False
        
        for agent in range(self.nofagents):
            self.setpointPub.append(rospy.Publisher('/uav'+str(agent)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
            
            self.drone_pose_subscriber.append(rospy.Subscriber('/uav'+str(agent)+'/mavros/local_position/pose', PoseStamped, self.drone_pose_cb, agent))

    def drone_pose_cb(self, msg, agent):
        if self.checkSetpointFlag:
            pose = msg.pose.position
            if pose.x < self.currSetpoint[agent][0] + self.thresh and pose.x > self.currSetpoint[agent][0] - self.thresh and pose.y < self.currSetpoint[agent][1] + self.thresh and pose.y > self.currSetpoint[agent][1] - self.thresh:
                self.setpointReached[agent] = True
            
    # Load data from file
    def loadData(self, fname):
        with open(fname) as json_file:
            data = json.load(json_file)
            rospy.loginfo("Data loaded")
        return data
    
    # Extract setpoints from data
    def extractSetpoints(self, data):
        agent = [[]]*self.nofagents
        merged = []
        for cycle in range(len(data)):
            for iteration in range(len(data[cycle])):
                merged.append(data[cycle][iteration])
       
        merged_pd = pandas.read_json(json.dumps(merged))
        
        def work(data):
            if data.Px:
                ll = []
                for x,y in zip(data.Px, data.Py):
                    ll.append((x,y))
                return ll
            
        df = pd.DataFrame()
        df['col'] = merged_pd[['Px', 'Py']].apply(work, axis=1)
        df = df.dropna()
        
        listData = df['col'].values.tolist()
        rospy.loginfo("Setpoints extracted")
        # print(ll[5][3])
        return listData
        
    # Publish setpoints to the topic
    def PublishSetpoint(self, setpoint):
        rospy.loginfo("Publishing setpoints")
        for item in setpoint:
            self.currSetpoint = item
            self.setpointReached = [False]*self.nofagents
            self.checkSetpointFlag = True
            while not all(self.setpointReached):
                for agent, cordinates in enumerate(item):
                    pose = PoseStamped()
                    pose.pose.position.x = cordinates[0]
                    pose.pose.position.y = cordinates[1]
                    pose.pose.position.z = 2
                    pose.pose.orientation.x = 0
                    pose.pose.orientation.y = 0
                    pose.pose.orientation.z = 0
                    pose.pose.orientation.w = 1
                    self.setpointPub[agent].publish(pose)
                    # self.rate.sleep()
            self.checkSetpointFlag = False
            rospy.loginfo("Setpoint reached")
            
    # Main function
    def main(self):
        rospy.init_node('publishList', anonymous=True)
        # self.rate = rospy.Rate(10000)
        
        file = "/home/yash/catkin_ws/src/MultiRobotSearch/data/Params_"+str(self.nofagents)+"_"+str(self.Q)+"/agentSetpoint_"+str(self.nofagents)+"_"+str(self.Q)+".json"
        Data = self.loadData(file)
        setPoints = self.extractSetpoints(Data)
        if self.publishFlag:
            self.PublishSetpoint(setPoints)
            
        rospy.spin()
        
if __name__ == '__main__':
    try:
        pub = PublishList()
        pub.main()
    except Exception as e:
        print(e)

# # Convert json data to dictionary 
# def loadData(fname):
#     with open(fname) as json_file:
#         data = json.load(json_file)
#         print("Data loaded")
#     return data

# # extracting setpoints from json file
# def extractSetpoints(data):
#     agent = [[]]*nofagents
#     merged = []
#     for cycle in range(len(data)):
#         for iteration in range(len(data[cycle])):
#             merged.append(data[cycle][iteration])
   
#     merged_pd = pandas.read_json(json.dumps(merged))
    
#     def work(data):
#         if data.Px:
#             ll = []
#             for x,y in zip(data.Px, data.Py):
#                 ll.append((x,y))
#             return ll
        
#     df = pd.DataFrame()
#     df['col'] = merged_pd[['Px', 'Py']].apply(work, axis=1)
#     df = df.dropna()
    
    
    
#     ll = df['col'].values.tolist()
#     print("Setpoints extracted")
#     # print(ll[5][3])
#     return ll
    

# # publish the list to the topic
# def publishList(ll):
#     pub = []
#     for agent in range(nofagents):
#         pub.append(rospy.Publisher('/uav'+str(agent)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
        
        
#     rospy.init_node('publishList', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     rospy.loginfo("Publishing setpoints")
#     while not rospy.is_shutdown():
#         for item in ll:
#             for agent, cordinates in enumerate(item):
#                 pose = PoseStamped()
#                 pose.pose.position.x = cordinates[0]
#                 pose.pose.position.y = cordinates[1]
#                 pose.pose.position.z = 2
#                 pose.pose.orientation.x = 0
#                 pose.pose.orientation.y = 0
#                 pose.pose.orientation.z = 0
#                 pose.pose.orientation.w = 1
#                 pub[agent].publish(pose)
#                 rate.sleep()
#                 time.sleep(0.5)
    
        
# # main function
# if __name__ == '__main__':
#     try:
#         file = "/home/yash/catkin_ws/src/MultiRobotSearch/data/Params_"+str(nofagents)+"_"+str(Q)+"/agentSetpoint_"+str(nofagents)+"_"+str(Q)+".json"
#         dd = loadData(file)
#         setPoints = extractSetpoints(dd)
#         if publishFlag:
#             publishList(setPoints)
#     except rospy.ROSInterruptException:
#         pass

