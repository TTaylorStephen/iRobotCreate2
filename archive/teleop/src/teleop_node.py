#! /usr/bin/env python
import rospy
import pygame 
import numpy
import os
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

'''
Map for XBox One S controller is as follows:

	# hat controls: possible values = -1, 0, 1
		tuple where: -left/down = (-1,-1), right/up = (1,1), no input = (0,0)
		
	# buttons: bool values = 1, 0
		A=0, B=1, Y=3, X=2, l bumper=4, r bumper=5, select=6, start=7, left push=8, right push=9
		
	# ball axes & triggers: possible values =-1, 0, 1 
		lrLeft=0,  udLeft=1, lrRight=3, udRight=4, rTrigger=2, lTrigger=5 
'''
		
class TeleOp:
	
	def __init__(self):
	
		os.system("rosnode kill /move_base_node") #kill autonmous commands during teleop
		
		self.velPub=rospy.Publisher("/wheel_velocity/commanded", Float32MultiArray, queue_size=30) #initialize publishers
		self.bytePub=rospy.Publisher("/bytes", Int32MultiArray, queue_size=10)
		
		pygame.init() # initializes pygame modules
		self.getID()  # gets name on specific controller (only really useful for setup)

		self.hat=0 	  # d-pad (or joystick hat)
		self.B=1	  # buttons of interest 
		self.lBumper=4
		self.rBumper=5
		
	
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
	
		vl, vr = 200.0, 200.0 		   	# define teleop velocity
		v, bytes=[0.0]*2, [0]*2 	   	# allocate space for arrays that will be published
		velocity=Float32MultiArray() 	# objects for ros variables
		ros_bytes=Int32MultiArray()

		if self.name=='Xbox Wireless Controller':	
			joy=pygame.joystick.Joystick(self.joy_id)
			joy.init()	# initialize object created above

			for i in range(joy.get_numaxes()):
				axis=joy.get_axis(i)
				print('buttons active at %d are %s\n' %(i, axis))
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
	
	#pygame.quit()
		
