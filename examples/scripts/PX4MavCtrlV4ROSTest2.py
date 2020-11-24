import time
import math
import sys

import PX4MavCtrlV4ROS as PX4MavCtrl


mav = PX4MavCtrl.PX4MavCtrler('192.168.1.167','20100')


#Turn on MAVLink to monitor CopterSim data and update it in real time. 
mav.InitMavLoop()
time.sleep(0.5)


#Display Position information received from CopterSim
print(mav.uavPosNED)


#Turn on Offboard mode
mav.initOffboard()
# Send the desired position signal, fly to the target point 0,0, -1.7 position, the yaw angle is 0
mav.SendPosNED(0, 0, -1.7, 0) 
print("Send target Pos")
