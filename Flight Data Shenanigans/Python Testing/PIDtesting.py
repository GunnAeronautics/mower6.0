import time
import matplotlib.pyplot as plt
import random
import math

def compute(pid):
	global integral
	global previous_error
	global current_angle
	error = setpoint - current
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

		if output > 7:
			output = 7
		elif output < -7:
			output = -7
		
	else:#current system
		if error < 0:
			current_angle-=0.1
			output = current_angle
		else:
			current_angle+=0.1
			output = current_angle
	return output
# Example usage
if __name__ == "__main__":
	kp = 0.2#present
	ki = 0.1#past
	kd = 0.2#future

	speed = 300.0 #m/s
	b = 0.1
	previous_error = 0
	integral = 0
	altitude = 0.0
	setpoint = 0.0  # Desired setpoint
	current = 0.0	# Current state
	currents = []
	setpoints= []
	altitudes = []
	ceiling = []
	current_angle = 0.0
	t = []
	deltaT = 0.03 #1 milisecond
	#yspeed = 300
	for i in range(0,1000):#30 miliseconds
		#gravityspeed += 9.8
		
		# Compute PID output
		speed = speed-(speed*deltaT*b)#drag
		yspeed = math.cos(math.radians(current))*(speed)#/1000# m/deltaT
		yspeed -= 9.8*deltaT*i#/1000#*deltaT
		altitude += yspeed * deltaT#/1000
		
		if altitude < 0:#simulation portion
			altitude = 0
		t.append(i*30)
		altitudes.append(altitude)
		
		if altitude < 100:
			setpoint = 0
		elif altitude > 250:
			setpoint = 90
		else:
			setpoint = 100/(1+pow(2.7,-(altitude-200)/25))-1.8
		setpoints.append(setpoint)
		
		output = compute(False)
		current += output#we can assume the output is the servo angle
		print(current, output * speed)
		currents.append(current)
		ceiling.append(250)
		# Simulate a delay (for demonstration purposes)
		#time.sleep(0.1)
		
	print (altitude)
	#plt.plot(t, currents, label='Angle')
	plt.plot(t, altitudes, label='Altitude')
	plt.plot(t, setpoints, label='Desired Angle')
	plt.plot(t, currents, label='Current Angle')
	plt.plot(t, ceiling,label='goodalt')
	plt.legend()
	plt.xlabel('Time (miliseconds)')
	plt.ylabel('Data')
	plt.title('Flight Data')
	plt.show()