#! /usr/bin/env python
import rospy
import pygame 
import numpy
from std_msgs.msg import Int32MultiArray

'''
Map for XBox One S controller is as follows:

	# hat controls: possible values = -1, 0, 1
		tuple where: -left/down = (-1,-1), right/up = (1,1), no input = (0,0)
		
	# buttons: bool values = 1, 0
		A=0, B=1, Y=4, X=3, l bumper=6, r bumper=7, start=11, left push=13, right push=14
		
	# ball axes & triggers: possible values =-1, 0, 1 
		lrLeft=0,  udLeft=1, lrRight=2, udRight=3, rTrigger=4, lTrigger=5 
'''
		
class TeleOp:
	
	def __init__(self):
			 
		self.velPub=rospy.Publisher("/wheel_velocity/commanded", Int32MultiArray, queue_size=30)
		self.bytePub=rospy.Publisher("/bytes", Int32MultiArray, queue_size=10)
		
		pygame.init() # initializes pygame modules
		self.getID()  # gets name on specific controller (only really useful for setup)

		self.hat=0 	  # d-pad (or joystick hat)
		self.B=1	  # buttons of interest 
		self.lBumper=6
		self.rBumper=7
		
	
	def getID(self):
	
		joystick_count=pygame.joystick.get_count()
		if joystick_count<1: 			    # check if a joystick is available
			print('Joystick not found') 
			pygame.quit()

		for i in range(joystick_count):     # list IDs before running and get current controller name 
			joy=pygame.joystick.Joystick(i)
			joy.init()
			self.name=joy.get_name() 
			self.joy_id=joy.get_id()
			print('Name: %s, ID: %s\n' %(self.name,self.joy_id))
			
	
	def joyPub(self):
	
		vl, vr = 200, 200 		   	# define teleop velocity
		v, bytes=[0]*2, [0]*2 	   	# allocate space for arrays that will be published
		velocity=Int32MultiArray() 	# objects for ros variables
		ros_bytes=Int32MultiArray()

		if self.name=='Xbox Wireless Controller':	
			joy=pygame.joystick.Joystick(self.joy_id)
			joy.init()	# initialize object created above

			if joy.get_button(self.B)==1:		# If B is pressed, start sending commands				
				hat=joy.get_hat(self.hat)		# check hat controls for input	
				if hat[0]==-1:					# store desired velocity per input
					v=[-vl,vr]
				elif hat[0]==1:
					v=[vl,-vr]
				elif hat[1]==-1:
					v=[-vl,-vr]
				elif hat[1]==1:
					v=[vl, vr]	
				velocity.data=v
				self.velPub.publish(velocity)		
				
				if joy.get_button(self.rBumper)==1:	# turn on vacuum and all brushes if right trigger is active
					bytes=[138, 7] 					# motor byte determined from bits available given in create specs
					ros_bytes.data=bytes
					self.bytePub.publish(ros_bytes) # publish bytes for writing to create
					
				if joy.get_button(self.lBumper)==1: # turn off vacuum if left trigger is activated
					bytes=[138, 0]		
					ros_bytes.data=bytes
					self.bytePub.publish(ros_bytes)
				
if __name__=='__main__':

	rospy.init_node("teleoperation")
	rate=rospy.Rate(15) 
	op=TeleOp()
	
	while not rospy.is_shutdown():
		pygame.event.get()	# get all event messages from queue
		op.joyPub()		
		rate.sleep()
	
	pygame.quit()
		
