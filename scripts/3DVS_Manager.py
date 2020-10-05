#!/usr/bin/python
import rospy
import roslib
import numpy as np
import geometry_msgs.msg as geo_msg
import std_msgs.msg as std_msg
import time
from std_msgs.msg import Bool
from Mission_Management.msg import my_msg
from std_srvs.srv import SetBool

class MissionManagement:
    def __init__(self):
        self.ros_node = rospy.init_node('3DVS_Manager', anonymous=True)

        #define ROS Publisher and Subscriber
        self.pose_ur_pub = rospy.Publisher('/ur_cmd_pose', geo_msg.Pose, queue_size=1 )
        self.adjust_pose_ur_pub = rospy.Publisher('/ur_cmd_adjust_pose', geo_msg.Pose, queue_size=1 )
        self.vel_ur_pub  = rospy.Publisher('/ur_cmd_vel', geo_msg.Twist, queue_size=1 )
        self.Hole_Subscriber = rospy.Subscriber("/hole_pos", my_msg, self.hole_pose_callback)
        self.Plane_Orien_Subscriber = rospy.Subscriber("/plane_oreintation", geo_msg.Quaternion, self.Plane_Orien_callback)
        self.detection_mode_pub = rospy.Publisher('/detection_mode', Bool, queue_size=1)
        self.tactile_mode_pub = rospy.Publisher('/tactile_control_mode', Bool, queue_size=1)

        self.rate = rospy.Rate(100)

        #define ROS service
        self.MM_service_start = rospy.Service('startMM', SetBool, self.start_MM)
        self.startEMVScall = rospy.ServiceProxy('startEMVS', SetBool)
        
        self.hole_pose = []
        self.plane_orientation = np.empty((4,1))

        self.tactile_adjusted_pose = geo_msg.Pose()
        
        rospy.spin()

    def start_MM(self, mess):
        if mess.data == True:
            print("MissionManagment Started")
                    
            #Intial Pose
            camera_initial_pose = geo_msg.Pose()
            camera_initial_pose.position.x = 0.2
            camera_initial_pose.position.y = -0.4
            camera_initial_pose.position.z = 0.4
            camera_initial_pose.orientation.x = 0
            camera_initial_pose.orientation.y = 1
            camera_initial_pose.orientation.z = 0
            camera_initial_pose.orientation.w = 0
            self.pose_ur_pub.publish(camera_initial_pose)
            time.sleep(5)
            
            startEMVS = self.startEMVScall(True)
            time.sleep(1)
            
            #Velocity Command
            camera_vel_cmd = geo_msg.Twist()
            camera_vel_cmd.linear.x = 0.03
            camera_vel_cmd.linear.y = -0.006
            camera_vel_cmd.linear.z = 0
            camera_vel_cmd.angular.x = 0
            camera_vel_cmd.angular.y = 0
            camera_vel_cmd.angular.z = 0
            self.vel_ur_pub.publish(camera_vel_cmd)

    def hole_pose_callback(self, data): #TODO: make a list of hole poses (now its only of size 3x1      
        for i in range(len(data.points)):
            self.hole_pose.append(data.points[i])
        
        print(self.hole_pose[1].x)
        #Velocity Command
        camera_vel_cmd = geo_msg.Twist()
        camera_vel_cmd.linear.x = 0
        camera_vel_cmd.linear.y = 0
        camera_vel_cmd.linear.z = 0
        camera_vel_cmd.angular.x = 0
        camera_vel_cmd.angular.y = 0
        camera_vel_cmd.angular.z = 0
        self.vel_ur_pub.publish(camera_vel_cmd)
        print("subscribed to holes position")

    def Plane_Orien_callback(self, data):
        self.plane_orientation[0] = data.x
        self.plane_orientation[1] = data.y
        self.plane_orientation[2] = data.z
        self.plane_orientation[3] = data.w

         
        self.NavigateToFirstHole() 
        print("subscribed to plane orientation")

    def NavigateToFirstHole(self):
        stopEMVS = self.startEMVScall(False)
        Hole_pose = geo_msg.Pose()
        Hole_pose.position.x = self.hole_pose[0].x
        Hole_pose.position.y = self.hole_pose[0].y
        Hole_pose.position.z = self.hole_pose[0].z
        Hole_pose.orientation.x = self.plane_orientation[0]
        Hole_pose.orientation.y = self.plane_orientation[1]
        Hole_pose.orientation.z = self.plane_orientation[2]
        Hole_pose.orientation.w = self.plane_orientation[3]
        self.pose_ur_pub.publish(Hole_pose)
        time.sleep(5)
        self.achieveTactileContact()

        self.NavigateToHole() 

    def NavigateToHole(self):
        
        print("Navigate to Hole Started")
        #Hole Pose
        for i in range(len(self.hole_pose)):
            print("hole pose %d is: %d",i, self.hole_pose[i])
            Hole_pose = geo_msg.Pose()
            Hole_pose.position.x = self.hole_pose[i].x
            Hole_pose.position.y = self.hole_pose[i].y
            Hole_pose.position.z = self.hole_pose[i].z
            Hole_pose.orientation.x = self.tactile_adjusted_pose.orientation.x
            Hole_pose.orientation.y = self.tactile_adjusted_pose.orientation.y
            Hole_pose.orientation.z = self.tactile_adjusted_pose.orientation.z
            Hole_pose.orientation.w = self.tactile_adjusted_pose.orientation.w
            time.sleep(5)
            self.pose_ur_pub.publish(Hole_pose)
            time.sleep(5)
            self.Start_2DVS()
            time.sleep(1)
            self.Adjust_tool_position()
            time.sleep(3)
            self.achieveTactileContact()
        
    def Start_2DVS(self):
        start_VS = Bool()
        start_VS.data = True 
        self.detection_mode_pub.publish(start_VS)
        try:
            Servoing_complete = rospy.wait_for_message('ur_detection_status', Bool, timeout = 10.0)
        except:
            start_VS = Bool()
            start_VS.data = False 
            self.detection_mode_pub.publish(start_VS)
            
        start_VS = Bool()
        start_VS.data = False 
        self.detection_mode_pub.publish(start_VS)
        #print("Servoing Complete: %d", Servoing_complete)

    def Adjust_tool_position(self):
        x_correction = -0.0043
        y_correction = -0.0521
        z_correction = 0 
        Adjust_pose = geo_msg.Pose()
        Adjust_pose.position.x = x_correction
        Adjust_pose.position.y = y_correction
        Adjust_pose.position.z = z_correction
        Adjust_pose.orientation.x = 0
        Adjust_pose.orientation.y = 0
        Adjust_pose.orientation.z = 0
        Adjust_pose.orientation.w = 1
        self.adjust_pose_ur_pub.publish(Adjust_pose)

    def achieveTactileContact(self):
        start_tactile = Bool()
        start_tactile.data = True
        self.tactile_mode_pub.publish(start_tactile)
        
        self.tactile_adjusted_pose = rospy.wait_for_message('tactile_pose', geo_msg.Pose, timeout = 30)


if __name__ == '__main__':
    MissionManagement()
    exit()  


