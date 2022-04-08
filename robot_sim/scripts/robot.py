#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from robot_sim.msg import Encoder



def getNew(W1prev, W2prev, W1tar, W2tar):
    t = 0.1
    T = 0.01    ## чем больше Т тем быстрее система переходит в устойчивый режим
    betta = t/(t+T)
    W1n = betta * W1prev + (1-betta)*W1tar
    W2n = betta * W2prev + (1-betta)*W2tar
    return (W1n, W2n)
    
def wheelVel(V, omega):
    L = 0.287
    r = 0.033
    R = V/omega
    l = R-L/2
    Wleft = omega * l/r
    l = R+L/2
    Wright = omega * l/r
    return Wleft, Wright
	
def velToEncoder (W1, W2):
# кол-во оборотов = W(рад/с)*deltaТ(с)/2*pi, 
# где deltaT - время между отправкой сообщений
# если частота отправки сообщений 10Гц, то deltaT = 0.1(сек) 
    n1 = W1*0.1/6.283
    n2 = W2*0.1/6.283
# кол-во оборотов * разрешение энкодеров на 1 оборот = кол-во импульсов энкодера
    e1 = int(n1 * 4096)
    e2 = int(n2 * 4096)
    return (e1,e2)



def sendToServ(w1_tar, w2_tar):
    pub = rospy.Publisher('RobotToServer', Encoder, queue_size=10)

    w1_prev = w2_prev = 0
    message_id = 0
    
    now = rospy.Time.now()
    rate = rospy.Rate(10)
    while rospy.Time.now() < now + rospy.Duration.from_sec(20): ## 20 сек
        w1_new, w2_new = getNew (w1_prev, w2_prev, w1_tar, w2_tar)
        e1, e2 = velToEncoder (w1_new, w2_new)
		
        msg = Encoder()
        msg.leftWheel = e1
        msg.rightWheel = e2
        
        frame_id = f"Mesurment number {message_id}"

        msg.header.stamp = rospy.Time.now()	
        msg.header.seq = message_id	
        msg.header.frame_id = frame_id
        
        try:		 
            pub.publish(msg)
        except rospy.ROSInterruptException:
            continue
            
        rospy.loginfo(f"{frame_id}: Send to serv left: {e1} and right: {e2} Encoders.\n \
Sending time: {msg.header.stamp}")
	
        message_id += 1
        w1_prev, w2_prev = w1_new, w2_new
        rate.sleep()

	

def robotCallback (data):
    V = data.linear.x
    omega = data.angular.z
	
    rospy.loginfo(rospy.get_caller_id() + 
                  f' Get from serv V: {V}(m/s) and omega {omega}(rad/s). \
Getting time: %s' % rospy.Time.now())
				  
    w1_tar, w2_tar = wheelVel(V, omega)
    
    
    sendToServ (w1_tar, w2_tar)
    
   
    
def listenServ ():
    rospy.init_node('robot', anonymous = True)
    rospy.Subscriber('ServerToRobot', Twist, robotCallback)
    rospy.spin()


if __name__=="__main__":
    listenServ()
	
	
	
