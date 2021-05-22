#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

'''
Team Id: 6768
Author List: Arya Das, Sumit Sourabh, Jay Shankar Pandit, Abhipray Singh
Filename: 6768_pos_hold.py
Theme: Hungry Bird
'''

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class Edrone():
	"""docstring for Edrone"""

	'''
	Function Name: __init__
	Input: self
	Output: none   
	Logic: Initializes the drone
  	'''

	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]#,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [0,0,20] #,0.01] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.taskdone = 0

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [10,10,80]
		self.Ki = [0.5,0.5,0]
		self.Kd = [10,10,10]


		self.alts = [30]*10


		#-----------------------Add other required variables for pid here ----------------------------------------------




		self.prev_error = [0,0,0]#,0]
		self.error_int = [0,0,0]#,0]
		self.integral_max = 3000

		self.max_values = [1800,1800,1800]#,1800]
		self.min_values = [1200,1200,1200]#,1200]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.025 # in seconds
		self.sample_freq = 1/self.sample_time
		self.ros_rate = rospy.Rate(self.sample_freq)
		self.thres_duration = rospy.Duration(secs=2)
		self.last_updated = rospy.Time.now()

		self.Ki[0] = self.Ki[0]*self.sample_time
		self.Ki[1] = self.Ki[1]*self.sample_time
		self.Ki[2] = self.Ki[2]*self.sample_time

		self.Kd[0] = self.Kd[0]/self.sample_time
		self.Kd[1] = self.Kd[1]/self.sample_time
		self.Kd[2] = self.Kd[2]/self.sample_time



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.error_alt = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.error_pitch = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.error_roll = rospy.Publisher('/roll_error', Float64, queue_size=1)
		#self.error_yaw = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line = rospy.Publisher('/zero_line', Float64, queue_size=1)
		self.plustwo = rospy.Publisher('/plustwo', Float64, queue_size=1)
		self.minustwo = rospy.Publisher('/minustwo', Float64, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		#rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		#rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------
		#self.disarm()
		#self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	'''
	Function Name: disarm
	Input: self
	Output: none   
	Logic: Disarms the drone
  	'''

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	'''
	Function Name: arm
	Input: self
	Output: none   
	Logic: Arms the drone
  	'''

	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 

	'''
	Function Name: whycon_callback
	Input: self,msg
	Output: none   
	Logic: Sets position of drone
  	'''

	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y

		for i in range(len(self.alts)-1):
			self.alts[i] = self.alts[i+1]

		self.alts[len(self.alts)-1] = msg.poses[0].position.z

		sum = 0
		for i in range(len(self.alts)):
			sum += self.alts[i]

		self.drone_position[2] = sum/len(self.alts)

		self.last_updated = rospy.Time.now()

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3
		print(self.Kp,self.Ki,self.Kd)

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	def pitch_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[0] = alt.Ki * 0.008
		self.Kd[0] = alt.Kd * 0.3
		print(self.Kp,self.Ki,self.Kd)

	def roll_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[1] = alt.Ki * 0.008
		self.Kd[1] = alt.Kd * 0.3
		print(self.Kp,self.Ki,self.Kd)

	#def yaw_set_pid(self,alt):
	#	self.Kp[3] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
	#	self.Ki[3] = alt.Ki * 0.008
	#	self.Kd[3] = alt.Kd * 0.3
	#	print(self.Kp,self.Ki,self.Kd)

	#def yaw_callback(self,data):
	#	self.drone_position[3] = data.data








	'''
	Function Name: clip
	Input: self,n,m,M
	Output: 
	Logic: Initializes the drone
  	'''

	def clip(self,n,m,M):
		n = min(n,M)
		n = max(n,m)
		return n
	#----------------------------------------------------------------------------------------------------------------------



	'''
	Function Name: pid
	Input: self
	Output: none   
	Logic: Main PID script of the drone
  	'''

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

	#------------------------------------------------------------------------------------------------------------------------

		error = [0,0,0]#,0]
		error[0] = self.drone_position[0] - self.setpoint[0]
		error[1] = self.drone_position[1] - self.setpoint[1]
		error[2] = self.drone_position[2] - self.setpoint[2]
		#error[3] = self.drone_position[3] - self.setpoint[3]
		#error[3] *= -1
		error[0] *= -1    # axis is not inverted

		# task is done if drone is within threshold
		if ((abs(error[0])<0.8 and abs(error[1])<0.8 and abs(error[2])<0.8) or (rospy.Time.now()-self.last_updated > self.thres_duration)):
			self.taskdone = 1
			self.last_updated = rospy.Time.now()
		else:
			self.taskdone = 0

		control_sig = [1500,1500,1500]#,1500]

		# proportional
		control_sig[0] += self.Kp[0]*error[0]
		control_sig[1] += self.Kp[1]*error[1]
		control_sig[2] += self.Kp[2]*error[2]
		#control_sig[3] += self.Kp[3]*error[3]

		#integral
		self.error_int[0] += error[0]
		self.error_int[1] += error[1]
		self.error_int[2] += error[2]
		#self.error_int[3] += error[3]*self.sample_time

		self.error_int[0] = self.clip(self.error_int[0],-self.integral_max, self.integral_max)
		self.error_int[1] = self.clip(self.error_int[1],-self.integral_max, self.integral_max)
		self.error_int[2] = self.clip(self.error_int[2],-self.integral_max, self.integral_max)
		#self.error_int[3] = self.clip(self.error_int[3],-self.integral_max, self.integral_max)

		control_sig[0] += self.Ki[0]*self.error_int[0]
		control_sig[1] += self.Ki[1]*self.error_int[1]
		control_sig[2] += self.Ki[2]*self.error_int[2]
		#control_sig[3] += self.Ki[3]*self.error_int[3]

		# derivative
		control_sig[0] += self.Kd[0]*(error[0]-self.prev_error[0])
		control_sig[1] += self.Kd[1]*(error[1]-self.prev_error[1])
		control_sig[2] += self.Kd[2]*(error[2]-self.prev_error[2])
		#control_sig[3] += self.Kd[3]*(error[3]-self.prev_error[3])/self.sample_time

		self.prev_error[0] = error[0]
		self.prev_error[1] = error[1]
		self.prev_error[2] = error[2]
		#self.prev_error[3] = error[3]

		# assignment
		self.cmd.rcRoll = control_sig[0]
		self.cmd.rcPitch = control_sig[1]
		self.cmd.rcThrottle = control_sig[2]
		self.cmd.rcYaw = 1500

		self.cmd.rcPitch = self.clip(self.cmd.rcPitch,self.min_values[0],self.max_values[0])
		self.cmd.rcRoll = self.clip(self.cmd.rcRoll,self.min_values[1],self.max_values[1])
		self.cmd.rcThrottle = self.clip(self.cmd.rcThrottle,self.min_values[2],self.max_values[2])
		#self.cmd.rcYaw = self.clip(self.cmd.rcYaw,self.min_values[3],self.max_values[3])

		self.error_roll.publish(error[0])
		self.error_pitch.publish(error[1])
		self.error_alt.publish(error[2])
		#self.error_yaw.publish(error[3])
		self.zero_line.publish(0)
		self.plustwo.publish(2)
		self.minustwo.publish(-2)
		self.command_pub.publish(self.cmd)
		#print(control_sig)

		self.ros_rate.sleep()


class progress_task():

	'''
	Function Name: __init__
	Input: self
	Output: none   
	Logic: Initializes the task
  	'''

	def __init__(self):
		''' state codes:
		0 - on ground
		1 - at start
		2 - at hoop_front
		3 - at hoop back
		4 - at start
		5 - on ground
		'''

		self.state = 0
		self.waypoints = []
		self.waypointsUpdated = True
		self.firstsetpoint = False

		rospy.Subscriber('/vrep/waypoints',PoseArray,self.update_waypoints)
		
		self.psstate = rospy.Publisher('/psstate', Float64, queue_size=1)
		self.e_drone = Edrone()

		while(not self.firstsetpoint):
			pass

		self.setsetpoint(self.waypoints[0])
		self.e_drone.arm()

		print("PID controller started")

	'''
	Function Name: process
	Input: self
	Output: none   
	Logic: main process loop
  	'''

	def process(self):

		self.e_drone.pid()
		
		if (self.waypointsUpdated == False):
			return
		
		if(self.state == 0 and self.e_drone.taskdone==1):
			self.state = 1
			self.waypointsUpdated = False
			self.psstate.publish(self.state)

		elif(self.state == 1):
			if (self.e_drone.taskdone == 1):
				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 2
					self.psstate.publish(self.state)
					self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

				self.currentWaypoint+=1

		elif(self.state == 2):
			if (self.e_drone.taskdone == 1):
				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 3
					self.psstate.publish(self.state)
					self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

				self.currentWaypoint+=1


		elif(self.state == 3):
			if (self.e_drone.taskdone == 1):
				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 4
					#self.psstate.publish(self.state)
					#self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

				self.currentWaypoint+=1

		elif(self.state == 4):
			self.e_drone.disarm()
			self.state = 5


	'''
	Function Name: setsetpoint
	Input: self,waypoint
	Output: none   
	Logic: Updates the drone waypoint
  	'''

	def setsetpoint(self,waypoint):
		self.e_drone.setpoint = [waypoint.position.x, waypoint.position.y, waypoint.position.z]
		print(self.e_drone.setpoint)


	'''
	Function Name: update_waypoint
	Input: self,poses
	Output: none
	Logic: Fetch new path from V-REP
  	'''

	def update_waypoints(self,poses):
		self.firstsetpoint = True
		self.waypoints = poses.poses
		self.noOfwaypoints = len(self.waypoints)
		self.currentWaypoint = 0
		self.waypointsUpdated = True

if __name__ == '__main__':

	rospy.sleep(2) # let everything else startup properly

	task = progress_task()

	while (not rospy.is_shutdown()) and (task.state < 5):
		task.process()