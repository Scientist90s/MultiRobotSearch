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
# from gazebo_ros_link_attacher.srv import Attach, AttachRequest

# Offboard control class
class OffbPosCtl():
    def __init__(self):
                
        # defined counters
        self.distThreshold = 0.2
        self.angThreshold = 0.1
        self.sim_ctr = 1
        self.nofagents = 4
        # some more definitions
        self.pose_pub = []
        self.drone_pose_subscriber = []
        self.state_sub = []
        self.armService = []
        self.flightModeService = []
        self.curr_drone_pose = []
        self.des_pose = []
        self.waypointIndex = []
        self.isReadyToFly = False
        self.arm = False
        self.curr_mode = "HOLD"

        # location and orientation
        self.probe = [40.368056, 3.947377, 11.208788]
        self.washer = [84.789988, -54.251096, 17.836362]
        self.rock = [60.208121, -12.502033, 18.775096]
        self.rover = [12.621400, -65.749400, -3.502830]
        self.landing = [self.rover[0], self.rover[1]+0.55, self.rover[2]+1]

        self.orientation = quaternion_from_euler(0, 0, 3.14 / 2 + 3.14 / 8)
        self.locations = numpy.matrix([[0, 0, 10, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.probe[0], self.probe[1], self.probe[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.probe[0], self.probe[1], self.probe[2]+0.1, self.orientation[0],self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.probe[0], self.probe[1], self.probe[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.washer[0], self.washer[1], self.washer[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.washer[0], self.washer[1], self.washer[2]+0.50, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.washer[0], self.washer[1], self.washer[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.rock[0], self.rock[1]-4, self.rock[2]+1, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.rover[0], self.rover[1], self.rover[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.landing[0], self.landing[1], self.landing[2]+5, self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       [self.landing[0], self.landing[1], self.landing[2], self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]],
                                       ])
        self.shape = self.locations.shape
        
        # rospy nodes, publishers, subscribers and services defined
        # node initialization
        rospy.init_node('offboard_test', anonymous=True)
        
        for i in range(self.nofagents):
            # Publishers
            self.pose_pub.append(rospy.Publisher('/uav'+str(i)+'/mavros/setpoint_position/local', PoseStamped, queue_size=10))
            # Subscribers
            self.drone_pose_subscriber.append(rospy.Subscriber('/uav'+str(i)+'/mavros/local_position/pose', PoseStamped, self.drone_pose_cb, i))
            self.state_sub.append(rospy.Subscriber('/uav'+str(i)+'/mavros/state', State, self.drone_state_cb))
            # Services
            self.armService.append(rospy.ServiceProxy('/uav'+str(i)+'/mavros/cmd/arming', CommandBool))
            self.flightModeService.append(rospy.ServiceProxy('/uav'+str(i)+'/mavros/set_mode', SetMode))
            # pose variables
            self.curr_drone_pose.append(PoseStamped())
            self.des_pose.append(PoseStamped())
            self.waypointIndex.append(0)
        
        
        # setting ros rate
        self.rate = rospy.Rate(10)  # Hz
        self.rate.sleep()
        print("starting control")
        self.controller()
    
    # function to control drone position and orientation
    def dronePosOriControl(self, pose):
        des_posi = pose.position
        des_ori = pose.orientation
        curr_posi = self.curr_drone_pose.pose.position
        curr_ori = self.curr_drone_pose.pose.orientation
        dist = math.sqrt((curr_posi.x - des_posi.x) * (curr_posi.x - des_posi.x) + 
                        (curr_posi.y - des_posi.y) * (curr_posi.y - des_posi.y) + 
                        (curr_posi.z - des_posi.z) * (curr_posi.z - des_posi.z))
        ang = euler_from_quaternion([des_ori.x,des_ori.y, des_ori.z, des_ori.w])[2] - euler_from_quaternion([curr_ori.x,curr_ori.y, curr_ori.z, curr_ori.w])[2]
        if dist < self.distThreshold and ang < self.angThreshold:
            self.waypointIndex +=1 
    
    # function to control the motion of drone
    def controller(self):
        print("inside controller")
        # starting continuous simulation loop
        while not rospy.is_shutdown():
            print(self.sim_ctr, self.shape[0], self.waypointIndex)
            # condition for endpoint
            if self.waypointIndex is self.shape[0]:
                self.land()
                break
            
            # condition for movement control according to waypoint
            if self.isReadyToFly:
                req_pose = self.set_desired_pose()
                self.dronePosOriControl(req_pose)
            
            self.pose_pub.publish(self.des_pose)
            self.rate.sleep()

    # callback function to store drone pose
    def drone_pose_cb(self, msg, i):
        self.curr_drone_pose[i] = msg

    # callback function to store drone state
    def drone_state_cb(self, msg):
        print(msg.mode)
        if msg.mode == 'OFFBOARD' and self.arm == True:
            self.isReadyToFly = True
        else:   
            self.take_off()
    
    # function ot set offboard mode
    def set_offboard_mode(self):
        print("Setting Offboard mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)
            
    # function ot set land mode
    def set_land_mode(self):
        print("Setting land mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            self.flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print("service set_mode call failed: %s. OFFBOARD Mode could not be set. Check that GPS is enabled" % e)

    # function to arm drone
    def set_arm(self):
        print("arming drone")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.armService(True)
            self.arm = True
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)
    
    # function to take off
    def take_off(self):
        print("Taking off")
        self.set_offboard_mode()
        self.set_arm()
    
    # function to land
    def land(self):
        print("Landing")
        self.set_land_mode()
        self.set_disarm()
            
    # function to disarm drone
    def set_disarm(self):
        print("disarming drone")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.armService(False)
            self.arm = False
        except rospy.ServiceException as e:
            print("Service arm call failed: %s" % e)


    # function to set desired pose from locations using waypoints
    def set_desired_pose(self):
        for i in range(self.nofagents):
            self.des_pose[i].pose.position.x = self.locations[i][self.waypointIndex, 0]
            self.des_pose[i].pose.position.y = self.locations[i][self.waypointIndex, 1]
            self.des_pose[i].pose.position.z = self.locations[i][self.waypointIndex, 2]
            
            des_azimuth = math.atan2(self.des_pose[i].pose.position.y - self.curr_drone_pose[i].pose.position.y,
                                    self.des_pose[i].pose.position.x - self.curr_drone_pose[i].pose.position.x)
            self.des_az_quat = quaternion_from_euler(0, 0, des_azimuth)
            
            self.des_pose[i].pose.orientation.x = self.des_az_quat[0]
            self.des_pose[i].pose.orientation.y = self.des_az_quat[1]
            self.des_pose[i].pose.orientation.z = self.des_az_quat[2]
            self.des_pose[i].pose.orientation.w = self.des_az_quat[3]
        return self.des_pose


if __name__ == "__main__":
    OffbPosCtl()
