#!/usr/bin/env python3
# Off Board Control multi robot simulation
# Yash Shethwala

# Importing libraries
import math
import rospy
import numpy
from mavros_msgs.msg import State
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Offboard control class
class OffbPosCtl():
    def __init__(self):
        # defined counters
        self.distThreshold = 0.2
        self.angThreshold = 0.1
        self.nofagents = 4
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
        self.rate = rospy.Rate(10)  # Hz
        self.rate.sleep()
        print("-----Starting control-----")
        self.controller()
        
    # callback function to store drone pose
    def drone_pose_cb(self, msg, agent):
        self.curr_drone_pose[agent] = msg
        
    # callback function to store drone state
    def drone_state_cb(self, msg, agent):
        self.curr_drone_state[agent] = msg.mode
        
    # Function to set offboard mode
    def set_offboard_mode(self, agent):
        rospy.wait_for_service('/uav'+str(agent)+'/mavros/set_mode')
        try:
            self.flightModeService[agent](custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    # Function to arm the drone
    def set_arm(self, agent):
        rospy.wait_for_service('/uav'+str(agent)+'/mavros/cmd/arming')
        try:
            self.armService[agent](True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)
            
    # Function to takeoff
    def takeoff(self):
        for i in range(self.nofagents):
            if self.curr_drone_state[i] == "OFFBOARD" and self.arm[i] == True:
                self.isReadyToFly[i] = True
            else:
                self.set_offboard_mode(i)
                self.set_arm(i)
        self.mode = "ASCEND"
        
    def controller(self):
        """ A state machine developed to have UAV states """
        while not rospy.is_shutdown():
            # control your UAV states and functionalities here...
            if self.mode == "TAKEOFF":
                print("-----Being Ready to TakeOff-----")
                self.takeoff()
            # if self.mode == "ASCEND":
            #     print("-----Ascending!-----")
            #     self.ascend()
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