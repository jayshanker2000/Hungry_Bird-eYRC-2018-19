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
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		#self.setpoint = [-8.39,4.98,27.92,0.01] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [0,0,0]
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
		self.Kp = [12.18,12.18,18.6,3.84]
		self.Ki = [0,0,0.168,0.856]
		self.Kd = [14.4,14.4,48,6.3]


		#-----------------------Add other required variables for pid here ----------------------------------------------





		self.prev_error = [0,0,0,0]
		self.error_int = [0,0,0,0]
		self.integral_max = 10

		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.120 # in seconds







		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.error_alt = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.error_pitch = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.error_roll = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.error_yaw = rospy.Publisher('/yaw_error', Float64, queue_size=1)

		#------------------------Add other ROS Publishers here-----------------------------------------------------







		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/drone_yaw', Float64, self.yaw_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------






		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
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
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------





		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	def pitch_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[0] = alt.Ki * 0.008
		self.Kd[0] = alt.Kd * 0.3

	def roll_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[1] = alt.Ki * 0.008
		self.Kd[1] = alt.Kd * 0.3

	def yaw_set_pid(self,alt):
		self.Kp[3] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[3] = alt.Ki * 0.008
		self.Kd[3] = alt.Kd * 0.3

	def yaw_callback(self,data):
		self.drone_position[3] = data.data









	def clip(self,n,m,M):
		n = min(n,M)
		n = max(n,m)
		return n
	#----------------------------------------------------------------------------------------------------------------------


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

		error = [0,0,0,0]
		error[0] = self.drone_position[0] - self.setpoint[0]
		error[1] = self.drone_position[1] - self.setpoint[1]
		error[2] = self.drone_position[2] - self.setpoint[2]
		error[3] = self.drone_position[3] - self.setpoint[3]
		error[3] *= -1

		if (abs(error[0])<0.5 and abs(error[1])<0.5 and abs(error[2])<0.5 and abs(error[3])<0.5):
			self.taskdone = 1
		else:
			self.taskdone = 0

		control_sig = [1500,1500,1500,1500]

		# proportional
		control_sig[0] += self.Kp[0]*error[0]
		control_sig[1] += self.Kp[1]*error[1]
		control_sig[2] += self.Kp[2]*error[2]
		control_sig[3] += self.Kp[3]*error[3]

		#integral
		self.error_int[0] += error[0]*self.sample_time
		self.error_int[1] += error[1]*self.sample_time
		self.error_int[2] += error[2]*self.sample_time
		self.error_int[3] += error[3]*self.sample_time

		self.error_int[0] = self.clip(self.error_int[0],-self.integral_max, self.integral_max)
		self.error_int[1] = self.clip(self.error_int[1],-self.integral_max, self.integral_max)
		self.error_int[2] = self.clip(self.error_int[2],-self.integral_max, self.integral_max)
		self.error_int[3] = self.clip(self.error_int[3],-self.integral_max, self.integral_max)

		control_sig[0] += self.Ki[0]*self.error_int[0]
		control_sig[1] += self.Ki[1]*self.error_int[1]
		control_sig[2] += self.Ki[2]*self.error_int[2]
		control_sig[3] += self.Ki[3]*self.error_int[3]

		# derivative
		control_sig[0] += self.Kd[0]*(error[0]-self.prev_error[0])/self.sample_time
		control_sig[1] += self.Kd[1]*(error[1]-self.prev_error[1])/self.sample_time
		control_sig[2] += self.Kd[2]*(error[2]-self.prev_error[2])/self.sample_time
		control_sig[3] += self.Kd[3]*(error[3]-self.prev_error[3])/self.sample_time

		self.prev_error[0] = error[0]
		self.prev_error[1] = error[1]
		self.prev_error[2] = error[2]
		self.prev_error[3] = error[3]

		# assignment
		self.cmd.rcPitch = control_sig[0]
		self.cmd.rcRoll = control_sig[1]
		self.cmd.rcThrottle = control_sig[2]
		self.cmd.rcYaw = control_sig[3]

		self.cmd.rcPitch = self.clip(self.cmd.rcPitch,self.min_values[0],self.max_values[0])
		self.cmd.rcRoll = self.clip(self.cmd.rcRoll,self.min_values[1],self.max_values[1])
		self.cmd.rcThrottle = self.clip(self.cmd.rcThrottle,self.min_values[2],self.max_values[2])
		self.cmd.rcYaw = self.clip(self.cmd.rcYaw,self.min_values[3],self.max_values[3])

		self.error_pitch.publish(error[0])
		self.error_roll.publish(error[1])
		self.error_alt.publish(error[2])
		self.error_yaw.publish(error[3])
		self.command_pub.publish(self.cmd)

		time.sleep(self.sample_time)


class Task1dot2():

	def __init__(self):
		''' state codes:
		0 - on ground
		1 - at initial waypoint
		2 - at goal 1
		3 - at goal 2
		4 - at initial waypoint
		5 - on ground
		'''

		self.state = 0
		self.waypoints = []
		self.waypointsUpdated = True

		rospy.Subscriber('/vrep/waypoints',PoseArray,self.update_waypoints)
		
		self.psstate = rospy.Publisher('/psstate', Float64, queue_size=1)

		self.e_drone = Edrone()
		self.e_drone.setpoint = [5.68,-1.91,33.40,0]

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
				self.currentWaypoint+=1

				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 2
					self.psstate.publish(self.state)
					self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

		elif(self.state == 2):
			if (self.e_drone.taskdone == 1):
				self.currentWaypoint+=1

				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 3
					self.psstate.publish(self.state)
					self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

		elif(self.state == 3):
			if (self.e_drone.taskdone == 1):
				self.currentWaypoint+=1

				if (self.currentWaypoint == self.noOfwaypoints):
					self.state = 4
					self.psstate.publish(self.state)
					self.waypointsUpdated = False
				else:
					self.setsetpoint(self.waypoints[self.currentWaypoint])
					print(self.currentWaypoint)

		elif(self.state == 4):
			self.e_drone.disarm()
			self.state = 5



	def setsetpoint(self,waypoint):
		self.e_drone.setpoint = [waypoint.position.x, waypoint.position.y, waypoint.position.z, 0]

	def update_waypoints(self,poses):
		self.waypoints = poses.poses
		self.noOfwaypoints = len(self.waypoints)
		self.currentWaypoint = 0
		self.waypointsUpdated = True

if __name__ == '__main__':

	task = Task1dot2()

	while not rospy.is_shutdown():
		task.process()