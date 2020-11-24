#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State,PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu, NavSatFix
import time
import math
import os
import socket
import threading
import struct

import subprocess


class PX4MavCtrler:

    def __init__(self, ip, port):
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if ip == '255.255.255.255':
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.ip = ip
        self.port = port
        self.child = None        
        
        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_enu_position = None
        self.local_vel = None
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.state = None
        self.command = TwistStamped()
        self.offCmd = PositionTarget()
        
        self.isInOffboard = False
        
        self.uavAngEular = [0, 0, 0]
        self.uavAngRate = [0, 0, 0]
        self.uavPosNED = [0, 0, 0]
        self.uavVelNED = [0, 0, 0]
        
        
        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.local_vel_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        '''
        ros publishers
        '''
        #self.vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.vel_raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")


    # def start(self):
    #     rospy.init_node("offboard_node")
    #     rate = rospy.Rate(20)

    #     for i in range(10):
    #         self.vel_pub.publish(self.command)
    #         self.arm_state = self.arm()
    #         self.offboard_state = self.offboard()
    #         rate.sleep()

    #     start_time = rospy.Time.now()
    #     '''
    #     main ROS thread
    #     '''
    #     while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
    #         if rospy.Time.now() - start_time < rospy.Duration(5):
    #             self.command.twist.linear.x = 0
    #             self.command.twist.linear.z = 2
    #             self.command.twist.angular.z = 0
    #         elif rospy.Time.now() - start_time < rospy.Duration(20):
    #             self.command.twist.linear.x = 2
    #             self.command.twist.linear.z = 0
    #             self.command.twist.angular.z = 0.1
    #         else:
    #             self.command.twist.linear.x = 0
    #             self.command.twist.linear.z = -1
    #             self.command.twist.angular.z = 0

    #         self.vel_pub.publish(self.command)
    #         rate.sleep()

    def InitMavLoop(self):
        cmdStr = 'roslaunch mavros px4.launch fcu_url:="udp://:'+str(int(self.port)+1)+'@'+self.ip+':'+self.port+'"'
        print(cmdStr)
        #os.system("gnome-terminal -e 'bash -c \""+cmdStr+"\"'")
        #os.system(cmdStr)
        self.child = subprocess.Popen(cmdStr,
                         shell=True,
                         stdout=subprocess.PIPE)
        
        time.sleep(10)
        print(self.child.poll())

    def SendMavArm(self, isArm):
        if self.armService(isArm):
            return True
        else:
            if isArm:
                print("Vehicle disarming failed!")
            else:
                print("Vehicle arming failed!")
            return False        

    def initOffboard(self):
        self.t2 = threading.Thread(target=self.OffboardLoop, args=())
        self.isInOffboard = True        
        print("Offboard Started.")
        rospy.init_node("offboard_node")
        rate = rospy.Rate(20)
        self.SendVelNED(0, 0, 0, 0) 
        for i in range(10):
            self.vel_raw_pub.publish(self.offCmd)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rate.sleep()
        self.t2.start()

    def OffboardLoop(self):
        rate = rospy.Rate(30)
        while True:
            if not self.isInOffboard:
                break
            if self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
                self.vel_raw_pub.publish(self.offCmd)
                #print('Send offboard msg.')
                #print(self.offCmd)
                rate.sleep()
            else:
                break
        print("Offboard Stoped.")

    def endOffboard(self):
        self.isInOffboard = False
        self.t2.join()

    def stopRun(self):
        self.child.kill()
        print('Please close all Terminal windows to close')
        
    def calcTypeMask(self,EnList):
        enPos = EnList[0]
        enVel = EnList[1]
        enAcc = EnList[2]
        enForce = EnList[3]
        enYaw = EnList[4]
        EnYawrate= EnList[5]
        y=int(0)
        if not enPos:
            y = y | 7

        if not enVel:
            y = y | (7<<3)

        if not enAcc:
            y = y | (7<<6)

        if not enForce:
            y = y | (1<<9)

        if not enYaw:
            y = y | (1<<10)

        if not EnYawrate:
            y = y|(1<<11)
        return y

    def SendVelNED(self,vx,vy,vz,yawrate):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = vx
        self.offCmd.velocity.y = vy
        self.offCmd.velocity.z = vz
        self.offCmd.yaw_rate = yawrate


    def SendVelFRD(self,vx,vy,vz,yawrate):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([0,1,0,0,0,1])
        self.offCmd.velocity.x = vx
        self.offCmd.velocity.y = vy
        self.offCmd.velocity.z = vz
        self.offCmd.yaw_rate = yawrate

    def SendPosNED(self,x,y,z,yaw):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_LOCAL_NED
        self.offCmd.type_mask = self.calcTypeMask([1,0,0,0,1,0])
        self.offCmd.position.x = x
        self.offCmd.position.y = y
        self.offCmd.position.z = z
        self.offCmd.yaw = yaw

    def SendPosFRD(self,x,y,z,yaw):
        self.offCmd.coordinate_frame = self.offCmd.FRAME_BODY_NED
        self.offCmd.type_mask = self.calcTypeMask([1,0,0,0,1,0])
        self.offCmd.position.x = x
        self.offCmd.position.y = y
        self.offCmd.position.z = z
        self.offCmd.yaw = yaw

    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        self.uavAngEular = self.q2Euler(self.local_pose.pose.orientation)
        self.uavPosNED[0]=self.local_pose.pose.position.x
        self.uavPosNED[1]=self.local_pose.pose.position.y
        self.uavPosNED[2]=self.local_pose.pose.position.z
        
    def local_vel_callback(self, msg):    
        self.local_vel = msg
        self.uavVelNED[0] = self.local_vel.twist.linear.x
        self.uavVelNED[1] = self.local_vel.twist.linear.y
        self.uavVelNED[2] = self.local_vel.twist.linear.z
        self.uavAngRate[0] = self.local_vel.twist.angular.x
        self.uavAngRate[1] = self.local_vel.twist.angular.y
        self.uavAngRate[2] = self.local_vel.twist.angular.z

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def q2yaw(self, q):
        q0, q1, q2, q3 = q.w, q.x, q.y, q.z
        math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    def q2Euler(self,q):
        w,x,y,z= q.w, q.x, q.y, q.z
        roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        pitch = math.asin(2*(w*y-z*x))
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        return [roll,pitch,yaw]

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False
        
        
    def sendUE4Cmd(self,cmd,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("i52s",1234567890,cmd)
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("i52s",1234567890,cmd)
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID))

    def sendUE4Pos(self,copterID,vehicleType,MotorRPMSMean,PosE,AngEuler,windowID=-1):
        if windowID<0:
            for i in range(5):
                buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
                self.udp_socket.sendto(buf, (self.ip, 20010+i))
        else:    
            buf = struct.pack("3i7f",1234567890,copterID,vehicleType,MotorRPMSMean,PosE[0],PosE[1],PosE[2],AngEuler[0],AngEuler[1],AngEuler[2])
            self.udp_socket.sendto(buf, (self.ip, 20010+windowID)) 