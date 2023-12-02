import time
import matplotlib.pyplot as plt
import random
import math
import csv
import pandas as pd

def compute(pid):
	global integral
	global previous_error
	global current_angle
	error = setpoint - current#error angle
	if pid:#PID system
			# Proportional term
		P = kp * error

			# Integral term
		integral += error
		I = ki * integral

			# Derivative term
		derivative = error - previous_error
		D = kd * derivative

			# PID output
		output = P + I + D

			# Save the current error for the next iteration
		previous_error = error

		if output > servomax:
			output = servomax
		elif output < -servomax:
			output = -servomax
		
	else:#current system
		if error < 0:
			if error > -servomax:
				current_angle-=0.1
			output = current_angle
		else:
			if error < servomax:
				current_angle+=0.1
			output = current_angle
	return output
# Example usage
if __name__ == "__main__":

	df = pd.read_csv('thrust.csv')#header=None)
	df = df.T#transpose
	thrust,time= df.values.tolist()
	servomax = 7#servo max angle
	canard_impact = 0.1 #impact of canards in the equation NEWANGLE = SERVOANGLE * SPEED * DELTAT * CANARDIMPACT 

	for j in range(1):#scientific testing purposes

		#CONTROL
		kp = 0.1#present
		ki = 0.05#past
		kd = 0.1#future

		previous_error = 0
		integral = 0
		setpoint = 0.0#desired angle
		current_angle = 0.0
		#SIMULATION
		speed = 0.0 #m/s
		b = 0.01#drag coef
		w = 0.65#0.65kg rocket
		altitude = 0.0#meters
		current = 0.0#current angle
		ts = 0#time miliseconds

		#DATALOGGING
		currents = []
		setpoints= []
		altitudes = []
		ceiling = []
		servo_measurements = []
		t = []

		for i in range(len(time)):#it's simulating time
			#SIMULATION
			deltaT = time[i]
			ts += deltaT
			
			speed += (thrust[i]/w)*deltaT#thrust from F__ rocket
			speed -= (speed*deltaT*b)#drag
			#current = 0
			yspeed = math.cos(math.radians(current))*speed#/1000# m/deltaT
			yspeed -= 9.81*ts#/1000#*deltaT
			altitude += yspeed * deltaT#/1000
			
			if altitude < 0:#simulation portion
				altitude = 0

			

			
			if altitude < 100:
				setpoint = 0
			elif altitude > 250:
				setpoint = 90
			else:
				setpoint = 100/(1+pow(2.7,-(altitude-200)/25))-1.8

			#CONTROL
			output = compute(True)

			current += output * speed * deltaT * canard_impact#we can assume the output is the servo angle

			#DATA LOGGING
			setpoints.append(setpoint)
			currents.append(current)
			servo_measurements.append(output)
			altitudes.append(altitude)
			t.append(ts*1000)
			ceiling.append(250)

			
		print ('apogee', max(altitudes))
		#plt.plot(t, currents, label='Angle')
		plt.plot(t, altitudes, label='Altitude')
		plt.plot(t, setpoints, label='Desired Angle')
		plt.plot(t, currents, label='Current Angle')
		plt.plot(t, ceiling,label='goodalt')
		plt.plot(t, servo_measurements,label='Servo')

		#plt.plot(t, thrust,label='thrust')
		plt.legend()
		plt.xlabel('Time (miliseconds)')
		plt.ylabel('Data')
		plt.title('SIMULATED Flight Data')
		plt.show()
		