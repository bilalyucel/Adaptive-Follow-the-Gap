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
from collections import OrderedDict

class FollowtheGap():
	def __init__(self):
		rospy.init_node('followthegap')
		
		self.goal=Point()
		self.xgoal=input("Enter goal for x: ")
		self.ygoal=input("Enter goal for y: ")
		
		
		self.sub=rospy.Subscriber('/scan',LaserScan,self.clbk_laser)
		self.publisher=rospy.Publisher("/cmd_vel",Twist,queue_size=10)
		self.odom_sub=rospy.Subscriber('/odometry/filtered',Odometry,self.newOdom)

		self.obj_list=[]
		self.thislist=[]
		self.obj_dict={}
		self.gap_list=[]
		self.old_list=[]

		self.x=0.0	# Initial position of the robot
		self.y=0.0
		self.poseCorrect=False
		self.avoidObs=False
		self.avoidingPoint=Point()
		self.max_gap=0
		self.max_gap_length=0
		self.center_of_gap_ray=0
		self.theta=0.0
		self.dist_x=0
		self.dist_y=0
		self.previous_x=0
		self.previous_y=0
		self.first_run=True
		self.total_distance=0

		self.state=1
		
		self.twist=Twist()
		self.rate=rospy.Rate(10)
		self.move()

	def newOdom(self,msg):
		if self.first_run:
			self.previous_x=msg.pose.pose.position.x
			self.previous_y=msg.pose.pose.position.y
		x=msg.pose.pose.position.x
		y=msg.pose.pose.position.y
		self.total_distance=self.total_distance+math.sqrt((x-self.previous_x)*(x-self.previous_x)+(y-self.previous_y)*(y-self.previous_y))
		self.previous_x=x
		self.previous_y=y
		self.first_run=False
		self.x=msg.pose.pose.position.x
		self.y=msg.pose.pose.position.y
		self.q_rotation=msg.pose.pose.orientation
		(self.roll,self.pitch,self.theta)=euler_from_quaternion([self.q_rotation.x,self.q_rotation.y,self.q_rotation.z,self.q_rotation.w])

	def clbk_laser(self,msg):

		self.thislist=list(msg.ranges)
		'''Distance information is collected'''
		for x in range(len(self.thislist)):
			if math.isnan(self.thislist[x]):
				self.thislist[x]=10
		self.thislist.insert(0,10)
		self.thislist.append(10)

		Threshold=5
		inflactionRadius=20
		lookforsec=False
		self.gap_list=[]
		self.obj_list=[]
		self.obj_dict={}
		self.gap_list=[]
		self.inf_radius=0.3
		self.angles=[]
		'''Object_List is created'''
		for x in range(len(self.thislist)):
			if not lookforsec:
				if self.thislist[x] != 10:
					check=True
					if  x<len(self.thislist)-Threshold:
						for j in range(1,Threshold):
							if self.thislist[x+j] !=10:
								continue
							else:
								check=False
								break
					if check:
						self.obj_list.append(x)
						lookforsec = True
			else:
				if self.thislist[x] == 10:
					check=True
					if  x<len(self.thislist)-Threshold:
						for i in range(1,Threshold):
							if self.thislist[x+i]==10:
								continue
							else:
								check=False
								break
					if check:
						self.obj_list.append(x-1)
						lookforsec = False
		'''Inflastion_Radius=20 is added to values and converted to a dictionary where the value holds the distance information'''
		for x in range(0,len(self.obj_list)):
			if x%2==0:
				if self.obj_list[x]-inflactionRadius<0:
					self.obj_dict.update({0:self.thislist[self.obj_list[x]]})
				else:
					self.obj_dict.update({self.obj_list[x]-inflactionRadius:self.thislist[self.obj_list[x]]})
			else:
				if self.obj_list[x]+inflactionRadius>640:
					self.obj_dict.update({640:self.thislist[self.obj_list[x]]})
				else:
					self.obj_dict.update({self.obj_list[x]+inflactionRadius:self.thislist[self.obj_list[x]]})
		self.obj_dict=OrderedDict(sorted(self.obj_dict.items()))
		self.gap_list.append(0)
		self.gap_list.extend(self.obj_dict.keys())
		self.gap_list.append(640)

	
	def move(self):
		while not rospy.is_shutdown():
			angles=[]
			px=[]
			py=[]
			myKeys=self.obj_dict.keys()		
			if self.state==1:
				if self.avoidObs==False:
					xgoal=self.xgoal
					ygoal=self.ygoal
				else:
					xgoal=self.avoidingPoint.x
					ygoal=self.avoidingPoint.y
				self.dist_x=xgoal-self.x
				self.dist_y=ygoal-self.y
				self.heading_angle=atan2(self.dist_y,self.dist_x)
				if abs(self.heading_angle-self.theta)>0.05:
					self.poseCorrect=False
					if self.avoidObs:
						print("Pose is wrong for obstacle avoidance!")
					else:
						print("Pose is wrong for goal point!")
					self.state=2
				else:
					self.poseCorrect=True
					if self.avoidObs:
						print("Pose is correct for obstacle avoidance!")
					else:
						print("Pose is correct for goal point!")
					self.state=3
			if self.state==2:
				if (self.heading_angle-self.theta)>0.05:
					if self.avoidObs:
						print("left for obstacle avoidance!")
					else:
						print("left for goal point!")
					self.twist.linear.x=0.5
					self.twist.angular.z=0.2
				elif (self.heading_angle-self.theta)<-0.05:
					if self.avoidObs:
						print("Right for obstacle avoidance!")
					else:
						print("Right for goal point!")
					self.twist.linear.x=0.5
					self.twist.angular.z=-0.2
				self.state=3
			if self.state==3:
				need=False
				if self.thislist:
					for x in range(30,610,1):
						if self.thislist[x]!=10:
							need=True
							break
				if need:
					print("New obstacle Detected!")
					self.avoidObs=True
					for x in range(len(self.obj_list)):
						angle=(52.5+self.obj_list[x]*75/640)*math.pi/180
						if self.obj_list[x]<320:
							x1=self.thislist[self.obj_list[x]]*math.sin(angle)
							y1=self.thislist[self.obj_list[x]]*-math.cos(angle)
						else:
							x1=self.thislist[self.obj_list[x]]*math.sin(math.pi-angle)
							y1=self.thislist[self.obj_list[x]]*math.cos(math.pi-angle)	
						px.append(math.cos(self.theta)*x1-math.sin(self.theta)*y1+self.x)
						py.append(math.sin(self.theta)*x1+math.cos(self.theta)*y1+self.y)
					for x in range(0,len(self.gap_list),2):
						angle=(self.gap_list[x+1]-self.gap_list[x])*75/640
						angles.append(angle)
						
					if angles:
						self.max_gap=angles.index(max(angles))
					else:
						self.max_gap=0
					'''Heading Point Calculation'''
					if len(self.gap_list)>=2*self.max_gap+1:
						if not (self.gap_list[2*self.max_gap]==0 or self.gap_list[2*self.max_gap+1]==640):
							yh=(py[2*self.max_gap-1]+py[2*self.max_gap])/2
							xh=(px[2*self.max_gap-1]+px[2*self.max_gap])/2
							self.max_gap_length=math.sqrt(abs(px[2*self.max_gap-1]-px[2*self.max_gap])*abs(px[2*self.max_gap-1]-px[2*self.max_gap])+abs(py[2*self.max_gap-1]-py[2*self.max_gap])*abs(py[2*self.max_gap-1]-py[2*self.max_gap]))
							if self.theta>0 and self.theta<math.pi/2:
								print("PF-Centered")
							elif self.theta>-math.pi/2 and self.theta<0:
								print("NF-Centered")
							elif self.theta>math.pi/2 and self.theta<math.pi:
								print("PB-Centered")
							elif self.theta<-math.pi/2 and self.theta>-math.pi:
								print("NB-Centered")
						elif self.gap_list[2*self.max_gap]==0 and self.gap_list[2*self.max_gap+1]!=640:
							d1=self.obj_dict.get(self.gap_list[2*self.max_gap+1])
							d2=d1*math.sin((127.5-max(angles))*math.pi/180)/math.sin(52.5*math.pi/180)
							angle=(52.5+max(angles)*75/640)*math.pi/180
							x2=math.cos(self.theta)*(d2*math.sin(angle))-math.sin(self.theta)*(d2*-math.cos(angle))+self.x
							y2=math.sin(self.theta)*(d2*math.sin(angle))+math.cos(self.theta)*(d2*-math.cos(angle))+self.y
							xh=(x2+px[0])/2
							yh=(y2+py[0])/2
							self.max_gap_length=math.sqrt(abs(x2-px[0])*abs(x2-px[0])+abs(y2-py[0])*abs(y2-py[0]))
							if self.theta>0 and self.theta<math.pi/2:
								print("PF-Right")
							elif self.theta>-math.pi/2 and self.theta<0:
								print("NF-Right")
							elif self.theta>math.pi/2 and self.theta<math.pi:
								print("PB-Right")
							elif self.theta<-math.pi/2 and self.theta>-math.pi:
								print("NB-Right")
						elif self.gap_list[2*self.max_gap]!=0 and self.gap_list[2*self.max_gap+1]==640:
							d1=self.obj_dict.get(self.gap_list[2*self.max_gap])
							d2=d1*math.sin((127.5-max(angles))*math.pi/180)/math.sin(52.5*math.pi/180)
							angle=(52.5+max(angles)*75/640)*math.pi/180
							x2=math.cos(self.theta)*(d2*math.sin(math.pi-angle))-math.sin(self.theta)*(d2*-math.cos(math.pi-angle))+self.x
							y2=math.sin(self.theta)*(d2*math.sin(math.pi-angle))+math.cos(self.theta)*(d2*-math.cos(math.pi-angle))+self.y
							xh=(x2+px[2*self.max_gap-1])/2
							yh=(y2+py[2*self.max_gap-1])/2
							self.max_gap_length=math.sqrt(abs(x2-px[2*self.max_gap-1])*abs(x2-px[2*self.max_gap-1])+abs(y2-py[2*self.max_gap-1])*abs(y2-py[2*self.max_gap-1]))
							if self.theta>0 and self.theta<math.pi/2:
								print("PF-Left")
							elif self.theta>-math.pi/2 and self.theta<0:
								print("NF-Left")
							elif self.theta>math.pi/2 and self.theta<math.pi:
								print("PB-Left")
							elif self.theta<-math.pi/2 and self.theta>-math.pi:
								print("NB-Left")
						self.avoidingPoint.x=xh
						self.avoidingPoint.y=yh
						self.state=1	
				else:
					if self.poseCorrect:
						self.state=4		
					else:
						self.state=1	
			if self.state==4:
				self.twist.linear.x=0.5
				self.twist.angular.z=0
				if self.avoidObs:
					print("Moving towards to avoiding point")
					if (abs(self.x-xh)<0.3 and abs(self.y-yh)<0.3) or (math.sqrt(self.x*self.x+self.y*self.y)>math.sqrt(xgoal*xgoal+ygoal*ygoal)):
						print("Avoiding point has reached!")
						self.avoidObs=False
					self.state=1
				else:
					print("Moving towards to goal point!")
					if abs(self.x-xh)<0.5 and abs(self.y-yh)<0.5:
						print("Goal point has reached!")
						self.twist.linear.x=0
						self.twist.angular.z=0
					else:
						self.state=1
					

			print("Total Distance Travelled:"+str(self.total_distance))
			print(xgoal,ygoal)
			self.publisher.publish(self.twist)
			self.rate.sleep()

if __name__ == '__main__':
	kt=FollowtheGap()
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
			print(e)
