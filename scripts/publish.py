#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import json
import pandas
import pandas as pd

publishFlag = True
nofagents = 5

# Convert json data to dictionary 
def loadData():
    with open('/home/yash/catkin_ws/src/MultiRobotSearch/scripts/agentSetpoint.json') as json_file:
        data = json.load(json_file)
    return data

def extractSetpoints(data):
    agent = [[]]*nofagents
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
    
    
    
    ll = df['col'].values.tolist()
    return ll
    
    # agent = [[]]*nofagents
    # for i in range(len(df.iloc[0][0])):
    #     agent[i] = []
    #     for j in range(len(df)):
    #         agent[i].append(df.iloc[j][0][i])
            
    # return agent
    

# publish the list to the topic
def publishList(ll):
    pub = []
    for agent in range(nofagents):
        pub.append(rospy.Publisher('/uav'+str(agent)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
        
    rospy.init_node('publishList', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        
        for item in ll:
            for agent, cordinates in enumerate(item):
                
                pose = PoseStamped()
                pose.pose.position.x = cordinates[0]
                pose.pose.position.y = cordinates[1]
                pose.pose.position.z = 2
                pose.pose.orientation.x = 0
                pose.pose.orientation.y = 0
                pose.pose.orientation.z = 0
                pose.pose.orientation.w = 1
                pub[agent].publish(pose)
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

