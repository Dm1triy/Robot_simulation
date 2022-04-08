#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from robot_sim.msg import Encoder


#W1 - left, W2 - right (рад/с)
#V(m/s)
#против часовой - положительное направление

WheelsEncoder = []            ## e1, e2 (impulse_num)
Moments = []                  ## time
MessageNums = []              ## index
WheelsVel = []                ## w1, w2 (rad/s)
RobotVel = []                 ## V (m/s)
RobotAngVel = []              ## omega (rad/s)
RobotAng = [0]                ## theta (rad)
Coordinates = [[0,0]]         ## x, y (m)
IdealCoordinates = [[0,0]]    ## x_ideal, y_ideal (m)



def drawIdealTrajectory():
    ideal_V = RobotVel[-1]
    ideal_omega = RobotAngVel[-1]
    x_ideal = [0]
    y_ideal = [0]
    dt = 0.1
    for i in range(len(Moments)-1):
        ideal_theta = ideal_omega * dt * i 
        x = ideal_V * np.cos(ideal_theta) * dt
        x_ideal.append(x + x_ideal[-1])
        y = ideal_V * np.sin(ideal_theta) * dt
        y_ideal.append(y + y_ideal[-1])
        
    plt.plot(x_ideal, y_ideal)
    plt.title("ideal")
    plt.show()
        
        
        
def publishCoordinates(x, y, theta):
    pub = rospy.Publisher('Debugging', Pose2D, queue_size = 10)
    
    pose = Pose2D()
    pose.x = x
    pose.y = y
    pose.theta = theta
    
    pub.publish(pose)
    
    rospy.loginfo(f' Send to debugging coordinates x = {x}(m), y = {y}(m), \
theta = {theta}(rad).\n Sending time: {rospy.Time.now()}') 



def coordinates ():				# координаты 
    dt = 0.1
    temp = (RobotVel[-1]*np.cos(RobotAng[-1]) + RobotVel[-2]*np.cos(RobotAng[-2]))*dt/2
    x = temp + Coordinates[-1][0]
	
    temp = (RobotVel[-1]*np.sin(RobotAng[-1]) + RobotVel[-2]*np.sin(RobotAng[-2]))*dt/2
    y = temp + Coordinates[-1][1]
    
    theta = RobotAng[-1]
    
    publishCoordinates(x, y, theta)
    	
    Coordinates.append([x,y])


def robotAng ():				# угол, на который повернут робот
    dt = 0.1
    temp = (RobotAngVel[-1] + RobotAngVel[-2])*dt/2
    RobotAng.append(temp + RobotAng[-1])


def robotAngVel ():				# реальная угловая скорость робота 
    w1 = WheelsVel[-1][0]
    w2 = WheelsVel[-1][1]
    r = 0.033
    L = 0.287
    omega = r/L * (w2 - w1)
    RobotAngVel.append(omega)


def robotVel():    			    # реальная скорость робота с помощью угловых скоростей колес
    w1 = WheelsVel[-1][0]
    w2 = WheelsVel[-1][1]
    r = 0.033	# meters
    V = r/2 * (w1 + w2) #(m/s)
    RobotVel.append(V)


def encoderToAngVel(): 	# значение энкодера в угловую скорость колес
## e = w*0.1*4096/6.28 => w = e * 6.28/(4096*0.1) 
    e1 = WheelsEncoder[-1][0]
    e2 = WheelsEncoder[-1][1]
    dt = 0.1
    w1 = e1 * 6.28/(4096 * dt)
    w2 = e2 * 6.28/(4096 * dt)
    WheelsVel.append([w1,w2])
	
	
	
def servCallback(data):
    e1 = data.leftWheel
    e2 = data.rightWheel
    sending_time = data.header.stamp
    message_index = data.header.seq
    message_id = data.header.frame_id

    rospy.loginfo(rospy.get_caller_id() + 
                  f' {message_id}: Get from robot left:{e1}, right:{e2} Encoders.\n \
Sending time:{sending_time}. Receiving time: {rospy.Time.now()}') 
 	 			  
    WheelsEncoder.append([e1,e2])
    Moments.append(sending_time)
    MessageNums.append (message_index)
	
    encoderToAngVel()
    robotVel()
    robotAngVel()
    if (len(Moments)>1):
        robotAng()
        coordinates()
		
		
	
def listenRobot():
    rospy.Subscriber('RobotToServer', Encoder, servCallback)
    rospy.spin()



def sendToRobot(V, omega):
    pub = rospy.Publisher('ServerToRobot', Twist, queue_size = 10)
	
    vel = Twist()
    vel.linear.x = V
    vel.angular.z = omega
    
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while rospy.Time.now() < now + rospy.Duration.from_sec(0.2):
        pub.publish(vel)
        rate.sleep()
        
    rospy.loginfo(f"Send to robot V: {V}(m/s), omega: {omega}(rad/s). Sending time: %s" % rospy.Time.now())
    
    
    
if __name__=="__main__":
    rospy.init_node('server', anonymous=True)
	
    sendToRobot (0.5, 0.5)  ## Таргетные значения линейной и угловой скоростей робота
    listenRobot()
    
    x = [i[0] for i in Coordinates]
    y = [i[1] for i in Coordinates]
    
    plt.plot(x, y)
    plt.title("real")
    plt.show()
    drawIdealTrajectory()
	
	
