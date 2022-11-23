#! /usr/bin/env python3
"""
 * File: offb_node.py
 * Stack and tested in Gazebo 9 SITL
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
brkFlag = False
def state_cb(msg):
    global current_state
    current_state = msg

def drone_pose_cb(msg):
    if msg.pose.position.z < 1.99 and msg.pose.position.z > 1.89:
        brkFlag = True    


if __name__ == "__main__":
    rospy.init_node("offb_uav4_node")
    
    pose_sub = rospy.Subscriber('/uav4/mavros/local_position/pose', PoseStamped, callback = drone_pose_cb)
    state_sub = rospy.Subscriber("/uav4/mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("/uav4/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/uav4/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/uav4/mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/uav4/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/uav4/mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()
                
        if brkFlag:
            break
        
        local_pos_pub.publish(pose)

        rate.sleep()
        
    while True:
        rospy.spin()
