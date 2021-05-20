#!/usr/bin/env python

import rospy
import math
from math import atan2
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import sensor_msgs.msg
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, PoseStamped

class FollowtheGap():
	def __init__(self):
		rospy.init_node('followthegap')
		self.sub=rospy.Subscriber('/scan',LaserScan,self.clbk_laser)
		self.x=0.0
		self.y=0.0
		self.total_angle=0
		self.max_gap_length=0
		self.theta=0.0
		self.publisher=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.odom_sub=rospy.Subscriber('/odometry/filtered',Odometry,self.newOdom)
		self.twist=Twist()
		self.rate=rospy.Rate(10)
		self.goal=Point()
		self.move()

	def clbk_laser(self,msg):
		self.thislist=list(msg.ranges)
		self.thislist.insert(0,10)
		self.thislist.append(10)
		for x in range(len(self.thislist)):
			if math.isnan(self.thislist[x]):
				self.thislist[x]=10
		Threshold=20
		lookforsec = False
		index=[]
		obj_list=[]
		gap_list=[]
		angles=[]
		gaps=[]
		for x in range(len(self.thislist)):
			if not lookforsec:
				if self.thislist[x] != 10:
					index.append(x)
					lookforsec = True
			else:
				if self.thislist[x] == 10:
					index.append(x)
					lookforsec = False
		lookforsec=False
		if  index:
			obj_list.append(index[0])
			obj_list.append(index[-1])
		for x in range(1,len(index)-2):
			if (index[x]+Threshold)<index[x+1]:
				obj_list.append(index[x])
				obj_list.append(index[x+1])
		inflaction_radius=30
		print(obj_list)
		for x in range(0,len(obj_list)):
			if x%2==0:
				if obj_list[x]-inflaction_radius>0:
					obj_list[x]-=inflaction_radius
				else:
					obj_list[x]=0
			else:
				if obj_list[x]+inflaction_radius>640:
					obj_list[x]=640
				else:
					obj_list[x]=obj_list[x]+inflaction_radius
		obj_list.sort()
		gap_list.extend(obj_list)
		gap_list.insert(0,0)
		gap_list.insert(-1,len(self.thislist))
		gap_list.sort()
		for x in range(0,len(gap_list),2):
			angle=(gap_list[x+1]-gap_list[x])*75/640
			angles.append(angle)
		self.max_gap=angles.index(max(angles))
		self.max_gap_rays=[gap_list[2*self.max_gap],gap_list[2*self.max_gap+1]]
		self.max_gap_length=0
		self.center_of_gap=0
		self.center_of_gap_ray=0
		self.total_angle=0
		if index:
			self.max_gap_length=math.sqrt(pow(self.thislist[self.max_gap_rays[0]],2)+pow(self.thislist[self.max_gap_rays[1]-1],2)-2*self.thislist[self.max_gap_rays[0]]*self.thislist[self.max_gap_rays[1]-1]*math.cos(max(angles)*math.pi/180))
			self.center_of_gap_ray=int((self.max_gap_rays[0]+self.max_gap_rays[1])/2)
			self.total_angle=52.5+self.center_of_gap_ray*75/640
		else:
			self.total_angle=0
			self.center_of_gap_ray=320
			self.max_gap_length=math.sqrt(200-200*math.cos(75*math.pi/180))
		self.distance=self.thislist[self.center_of_gap_ray]
		if index:
			self.goal.x=self.x+self.distance*math.sin(self.total_angle*math.pi/180)
			self.goal.y=self.y-self.distance*math.cos(self.total_angle*math.pi/180)
		else:
			self.goal.x=self.x+self.distance
			self.goal.y=self.y
	def newOdom(self,msg):
		self.x=msg.pose.pose.position.x
		self.y=msg.pose.pose.position.y
		self.q_rotation=msg.pose.pose.orientation
		(self.roll,self.pitch,self.theta)=euler_from_quaternion([self.q_rotation.x,self.q_rotation.y,self.q_rotation.z,self.q_rotation.w])
	def move(self):
		while not rospy.is_shutdown():

			if  self.total_angle !=0:
				self.dist_x = self.goal.x- self.x
				self.dist_y = self.goal.y- self.y
				self.measured_angle=atan2(self.dist_y,self.dist_x)
				if self.max_gap_length<5:
					self.twist.linear.x=0
					self.twist.angular.z=0.6
					print("all-left")
				else:
					if self.total_angle<90:
						if(abs(self.measured_angle-self.theta)>0.05):
							self.twist.linear.x=1
							self.twist.angular.z=-0.6
							print("right")

					if self.total_angle>90:
						if(abs(self.measured_angle-self.theta)>0.05):
							self.twist.linear.x=1
							self.twist.angular.z=0.6
							print("left")
			else:
				self.twist.linear.x=1
				self.twist.angular.z=0.0
				print("moving")

			self.publisher.publish(self.twist)
			self.rate.sleep()

if __name__ == '__main__':
	kt=FollowtheGap()
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
			print(e)
