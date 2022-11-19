#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import json

publishFlag = False
nofagents = 5

# Convert json data to dictionary 
def loadData():
    with open('/home/yash/catkin_ws/src/MultiRobotSearch/agentSetpoint1.json') as json_file:
        data = json.load(json_file)
        print(data[0][0]["Px"])
    return data

def extractSetpoints(data):
    Px = []
    Py = []
    for cycle in range(len(data)):
        for iteration in range(len(data[cycle])):
            for agent in range(len(data[cycle][iteration]["Px"])): 
                Px.append(data[cycle][iteration]["Px"][agent])
                Py.append(data[cycle][iteration]["Py"][agent])
                
    print(Px, Py)
    return Px, Py
                

# publish the list to the topic
def publishList(list):
    pub = []
    for agent in range(nofagents):
        pub[agent].append(rospy.Publisher('/uav'+str(agent)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
        #pub[i] = rospy.Publisher('/uav'+str(i)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # pub_0 = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # pub_1 = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # pub_2 = rospy.Publisher('/uav2/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # pub_3 = rospy.Publisher('/uav3/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node('publishList', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        for i in range(len(list)):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = list[i][0]
            pose.pose.position.y = list[i][1]
            pose.pose.position.z = 1
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            if i == 0:
                pub_0.publish(pose)
            elif i == 1:
                pub_1.publish(pose)
            elif i == 2:
                pub_2.publish(pose)
            elif i == 3:
                pub_3.publish(pose)
            rate.sleep()
            

# main function
if __name__ == '__main__':
    try:
        dd = loadData()
        setPoints = extractSetpoints(dd)
        if publishFlag:
            publishList(setPoints)
    except rospy.ROSInterruptException:
        pass

