#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from yellowarm.msg import Angles
from yellowarm.msg import Goal_pose
from numpy import pi as PI

class calcIK():
    def __init__(self):
        #l have length of each links.
        #theta have angles of each joints.
        '''
            |
            | |  l4
             T 
             O   theta4
             |
             |   l3
             O   theta3
             |
             |   l2
             O   theta2
             |   l1
            <->  theta1
             |   l0
            777  theta0 -- it's fixed
        '''
        self.l = []
        self.theta = [0,0,0,0,0]
        #Acthal motors' angle
        self.a_theta = [0,0,0,0,0]
        #Limits of motors' angle
        self.theta_min = []
        self.theta_max = []
        #Threashold for comparing between ideal goal position and 
        #calculated goal position.
        is_small = 0.002
        #Threashold for comparing between ideal arm pose and
        #actual arm pose.
        is_large = 0.002

        pub = rospy.Publisher('pub/goal/angles', Angles, queue_size=10)
        sub = rospy.Subscriber('sub/goal/pose', Goal_pose, self.main_callback)
        a_theta_sub = rospy.Subscriber('sub/actual/angles', String, self.a_thetaCB)

        self.arm_init()

    def arm_init(self):
        self.l.append(rospy.get_param('~l_0', 0.0410))
        self.l.append(rospy.get_param('~l_1', 0.0405))
        self.l.append(rospy.get_param('~l_2', 0.1499))
        self.l.append(rospy.get_param('~l_3', 0.1500))
        self.l.append(rospy.get_param('~l_4', 0.1500))
        self.theta_min.append(rospy.get_param('~theta_0', 0))
        self.theta_max.append(rospy.get_param('~theta_0', 0))
        self.theta_min.append(rospy.get_param('~theta_1',-3.1415926535897930))
        self.theta_max.append(rospy.get_param('~theta_1', 3.1400586728019073))
        self.theta_min.append(rospy.get_param('~theta_2',-1.8070293681292853))
        self.theta_max.append(rospy.get_param('~theta_2', 2.0739420252213870))
        self.theta_min.append(rospy.get_param('~theta_3',-1.7456701366138596))
        self.theta_max.append(rospy.get_param('~theta_3', 1.4143302864305611))
        self.theta_min.append(rospy.get_param('~theta_4',-1.7303303287350031))
        self.theta_max.append(rospy.get_param('~theta_4', 1.8898643306751100))

    def main_callback(data, self):
        self.calc_ik(data,self)
        self.check_limits()
        if self.error_between(self.calculated_goal_position) < self.is_small:
            self.move_arm()
        else:
            rospy.loginfo("failed inverse kinematic.")

    def a_thetaCB(actual_pose, self):
        for i in range(len(theta)):
            self.a_theta[i] = actual_pose.theta[i]

    def calc_ik(Angles,self):
        self.theta[1] = math.atan2(data.y, data.x)
        self.A = math.sqrt(math.pow(math.sqrt(math.pow(x,2)+math.pow(y,2))-self.l[4]*math.sin(data.theta),2)
                +math.pow(data.z+self.l[4]*math.cos(data.theta)-self.l[0]-self.l[1],2))
        self.Ca = (math.pow(self.l[2],2)+math.pow(self.A,2)-math.pow(self.l[3],2))/(2*self.l[2]*A)
        self.theta[2] = (PI/2-math.atan2(data.z+self.l[4]*math.cos(data.theta)-self.l[0]-self.l[1],
                math.sqrt(math.pow(data.x,2)+math.pow(data.y,2))-self.l[4]*math.sin(theta))
                -math.atan2(math.sqrt(1-math.pow(self.Ca,2)),self.Ca))
        self.Cb = (math.pow(self.l[1],2)+math.pow(self.l[2],2)-math.pow(self.A,2))/(2*self.l[1]*self.l[2])
        self.theta[3] = PI+math.atan2(math.sqrt(1-math.pow(self.Cb,2)),self.Cb)
        self.theta[4] = PI-self.theta[2]-self.theta[3]-data.theta


    def check_limits(self):
        for i in range(len(self.theta)):
            if self.theta[i] < self.theta_min:
                self.theta[i] = self.theta_min
                rospy.loginfo("replaced id: %d angle for minimum angle ",i)
            elif self.theta[i] > self.theta_max:
                self.theta[i] = self.theta_max
                rospy.loginfo("replaced id: %d angle for maximum angle ",i)

    def error_between(data):
        return math.sqrt(pow(data[0],2) + pow(data[1],2) + pow(data[2],2))

    def calclated_goal_position(self):
        #Calculate forward kinematics
        #x = ( l1 sin(theta2) + l2 sin(theta2+theta3) + l3 sin(theta2+theta3+theta4) ) sin(theta1)
        x = (self.l[1]*math.sin(self.theta[2])+self.l[2]*math.sin(self.theta[2]+self.theta[3])+self.l[3]*math.sin(self.theta[2]+self.theta[3]+self.theta[4]))*math.sin(self.theta[1])
        #y = ( l1 sin(theta2) + l2 sin(theta2+theta3) + l3 sin(theta2+theta3+theta4) ) cos(theta1)
        y = (self.l[1]*math.sin(self.theta[2])+self.l[2]*math.sin(self.theta[2]+self.theta[3])+self.l[3]*math.sin(self.theta[2]+self.theta[3]+self.theta[4]))*math.cos(self.theta[1])
        #z = l0 + l1 cos(theta1) + l2 cos(theta1+theta2) + l3 cos(theta1+theta2+theta3)
        z = self.l[0] + self.l[1]*math.cos(self.theta[1]) + self.l[2]*math.cos(self.theta[1]+self.theta[2]) + self.l[3]*math.cos(self.theta[1]+self.theta[2]+self.theta[3])
        

    def actual_position(self):
        #Calculate forward kinematics
        #x = ( l1 sin(theta2) + l2 sin(theta2+theta3) + l3 sin(theta2+theta3+theta4) ) sin(theta1)
        x = (self.l[1]*math.sin(self.a_theta[2])+self.l[2]*math.sin(self.a_theta[2]+self.a_theta[3])+self.l[3]*math.sin(self.a_theta[2]+self.a_theta[3]+self.a_theta[4]))*math.sin(self.a_theta[1])
        #y = ( l1 sin(theta2) + l2 sin(theta2+theta3) + l3 sin(theta2+theta3+theta4) ) cos(theta1)
        y = (self.l[1]*math.sin(self.a_theta[2])+self.l[2]*math.sin(self.a_theta[2]+self.a_theta[3])+self.l[3]*math.sin(self.a_theta[2]+self.a_theta[3]+self.a_theta[4]))*math.cos(self.a_theta[1])
        #z = l0 + l1 cos(theta1) + l2 cos(theta1+theta2) + l3 cos(theta1+theta2+theta3)
        z = self.l[0] + self.l[1]*math.cos(self.a_theta[1]) + self.l[2]*math.cos(self.a_theta[1]+self.a_theta[2]) + self.l[3]*math.cos(self.a_theta[1]+self.a_theta[2]+self.a_theta[3])

    def move_arm():
        self.r = rospy.Rate(10)
        while self.error_between(self.actual_position) > self.is_large and not rospy.is_shutdown():
            pub.publish(self.thetax)
            r.sleep()

#---- main ----
rospy.init_node('yellowarm')

#Create an instance of yellowarm
yp = calcIK()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
