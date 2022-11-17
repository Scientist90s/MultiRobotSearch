#!/usr/bin/env python3
# Off Board Control multi robot simulation
# Yash Shethwala

# Importing libraries
import math
import rospy
import numpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from time import sleep

# Offboard control class
class OffbPosCtl():
    def __init__(self):
        # defined counters
        self.distThreshold = 0.2
        self.angThreshold = 0.1
        self.nofagents = 1
        # some more definitions
        self.pose_pub = []
        self.drone_pose_subscriber = []
        self.state_sub = []
        self.armService = []
        self.flightModeService = []
        self.curr_drone_pose = []
        self.curr_drone_state = []
        self.des_pose = []
        self.waypointIndex = []
        self.isReadyToFly = []
        self.arm = []
        self.mode = "TAKEOFF"
        
        self.orientation = quaternion_from_euler(0, 0, 0)
        self.locations = numpy.matrix([[0, 0, 10, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]]])
        self.shape = self.locations.shape
        
        # rospy nodes, publishers, subscribers and services defined
        # node initialization
        rospy.init_node('offboard_test', anonymous=True)
        
        for i in range(self.nofagents):
            # Publishers
            self.pose_pub.append(rospy.Publisher('/uav'+str(i)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
            # Subscribers
            self.drone_pose_subscriber.append(rospy.Subscriber('/uav'+str(i)+'/mavros/local_position/pose', PoseStamped, self.drone_pose_cb, i))
            self.state_sub.append(rospy.Subscriber('/uav'+str(i)+'/mavros/state', State, self.drone_state_cb, i))
            # Services
            self.armService.append(rospy.ServiceProxy('/uav'+str(i)+'/mavros/cmd/arming', CommandBool))
            self.flightModeService.append(rospy.ServiceProxy('/uav'+str(i)+'/mavros/set_mode', SetMode))
            # pose variables
            self.curr_drone_pose.append(PoseStamped())
            self.curr_drone_state.append("")
            self.des_pose.append(PoseStamped())
            self.waypointIndex.append(0)
            self.isReadyToFly.append(False)
            self.arm.append(False)
            
        # setting ros rate
        self.rate = rospy.Rate(20)  # Hz
        self.rate.sleep()
        rospy.loginfo("-----Starting control-----")
        self.controller()
        
    # callback function to store drone pose
    def drone_pose_cb(self, msg, agent):
        self.curr_drone_pose[agent] = msg
        
    # callback function to store drone state
    def drone_state_cb(self, msg, agent):
        self.curr_drone_state[agent] = msg
        
    # Function to set offboard mode
    def set_offboard_mode(self, agent):
        rospy.wait_for_service('/uav'+str(agent)+'/mavros/set_mode')
        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'
        try:
            if(self.flightModeService[agent].call(offb_set_mode).mode_sent == True):
                rospy.loginfo(f"{agent} agent's Offboard enabled")
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)
            
    def set_hold_mode(self, agent):
        rospy.wait_for_service('/uav'+str(agent)+'/mavros/set_mode')
        hold_set_mode = SetModeRequest()
        hold_set_mode.custom_mode = 'HOLD'
        try:
            if(self.flightModeService[agent].call(hold_set_mode).mode_sent == True):
                rospy.loginfo(f"{agent} agent's hold enabled")
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. HOLD Mode could not be set. Check that GPS is enabled" % e)

    # Function to arm the drone
    def set_arm(self, agent):
        rospy.wait_for_service('/uav'+str(agent)+'/mavros/cmd/arming')
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        try:
            if(self.armService[agent].call(arm_cmd).success == True):
                rospy.loginfo(f"{agent} agent's Armed")
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)
            
    # Function to takeoff
    def takeoff(self):
        for i in range(self.nofagents):
            if self.curr_drone_state[i].mode == "OFFBOARD" and self.curr_drone_state[i].arm == True:
                self.isReadyToFly[i] = True
            else:
                if self.curr_drone_state[i].mode != "OFFBOARD":
                    self.set_offboard_mode(i)
                if self.curr_drone_state[i].armed != True:
                    self.set_arm(i)
            sleep(1)
        self.mode = "YASH"
        
    def ascend(self):
        for i in range(self.nofagents):
            self.des_pose[i].pose.position.z = 1
            while self.mode[i] == "ASCEND" and not rospy.is_shutdown():
                self.pose_pub[i].publish(self.des_pose[i])
                if self.curr_drone_pose[i].pose.position.z >= 0.95:
                    self.set_hold_mode(i)
        
    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
            # control your UAV states and functionalities here...
            if self.mode == "TAKEOFF":
                rospy.loginfo("-----Being Ready to TakeOff-----")
                self.takeoff()
            if self.mode == "ASCEND":
                rospy.loginfo("-----Ascending!-----")
                self.ascend()
            # if self.mode == "BELLY-FLOP":
            #     print("belly flop!")
            #     self.belly_flop()
            # if self.mode == "RETRO":
            #     print("Retro")
            #     self.retro()
            # if self.mode == "LAND":
            #     print("Landing!")
            #     self.land()
        
if __name__ == "__main__":
    OffbPosCtl()