import csv
import pandas as pd
import matplotlib.pyplot as plt
import math
from ursina import *
import time
REF_GROUND_PRESSURE =	101185.29#//in pascals CHANGE BEFORE FLIGHT
REF_GROUND_ALTITUDE =	0#//in meters CHANGE BEFORE FLIGHT
REF_GROUND_TEMPERATURE = 1
def pressureToAlt(pressure): #returns alt (m) from pressure in h pascals
	return (REF_GROUND_ALTITUDE+((273+REF_GROUND_TEMPERATURE)/(-.0065))*((pow((pressure*100/REF_GROUND_PRESSURE),((8.314*.0065)/(9.807*.02896))))-1)) #https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
def radiansToDegrees(radians):
	return (radians*180/3.1415926) 
def integration(times,values):#right hand side integral with variable data points
	lastT = times[0] #right side reimenn sum 
	Valuelist = [0]
	for i in range(1,len(times)):
		deltaT = (times[i]-lastT)/1000
		Valuelist.append(Valuelist[i-1] + values[i]*deltaT)
		lastT = times[i]
	return Valuelist
def derrivative(times,values):#calculate velocity from displacement and time delta d/ delta t
	lastV = values[0]#reverse right side reimenn sum (not sure what it's called I didn't take calculus)
	lastT = times[0]
	Valuelist = [0]
	for i in range(1,len(times)):
		deltaV= (values[i] - lastV)
		deltaT= (times[i] - lastT)
		Valuelist.append(deltaV / (deltaT / 1000))
		lastV = values[i]
		lastT = times[i]
	return Valuelist
def current_milli_time():
	return round(time.time() * 1000)



if __name__ == "__main__":
	df = pd.read_csv('datalog0.csv')#header=None)
	df = df.T#transpose
	df = df.values.tolist()
	t = df[0]#time

	altitudes = []
	gyro_xyz = [[],[],[]]
	angle_xyz = [[],[],[]]
	vel_xyz=[[0],[0],[0]]
	displacement_xyz = [[0],[0],[0]]
	absolute_x = []
	#altitudes = []
	t = df[0]

	#lastT = t[0]
	for i in range(len(df[0])):
		altitudes.append(pressureToAlt(df[7][i]))
		#deltaT = (t[i]-lastT)/1000
		#angle_xyz[0],angle_xyz[1],angle_xyz[2] = MahonyFilterWithAltitude.update([df[1][i],df[2][i],df[3][i]],[df[4][i],df[5][i],df[6][i]],altitudes[i],deltaT)
		#lastT = t[i]
		gyro_xyz[0].append(radiansToDegrees(df[4][i]))
		gyro_xyz[1].append(radiansToDegrees(df[5][i]))
		gyro_xyz[2].append(radiansToDegrees(df[6][i]))


	angle_xyz[0] = integration(t,gyro_xyz[0])
	angle_xyz[1] = integration(t,gyro_xyz[1])
	angle_xyz[2] = integration(t,gyro_xyz[2])
	'''
	vel_xyz[0] = integration(t,df[1])
	vel_xyz[1] = integration(t,df[2])
	vel_xyz[2] = integration(t,df[3])

	displacement_xyz[0] = integration(t,vel_xyz[0])
	displacement_xyz[1] = integration(t,vel_xyz[1])
	displacement_xyz[2] = integration(t,vel_xyz[2])

	velocity_absolute_x = derrivative(t,altitudes)
	acceleration_absolute_x = derrivative(t,velocity_absolute_x)
	'''
	#for i in range(t):
	#	acceleration_absolute_x[i] += 9.81
	#comment out unnessecary data

	#plt.plot(t, df[1], label='Ax (m/s^2)')
	#plt.plot(t, df[2], label='Ay (m/s^2)')
	#plt.plot(t, df[3], label='Az (m/s^2)')

	#plt.plot(t, vel_xyz[0], label='Vx (m/s)')
	#plt.plot(t, vel_xyz[1], label='Vy (m/s)')
	#plt.plot(t, vel_xyz[2], label='Vz (m/s)')

	#plt.plot(t, displacement_xyz[0], label='Dx (m)')
	#plt.plot(t, displacement_xyz[1], label='Dy (m)')
	#plt.plot(t, displacement_xyz[2], label='Dz (m)')

	plt.plot(t, gyro_xyz[0], label='Gvx (°/s)')
	plt.plot(t, gyro_xyz[1], label='Gvy (°/s)')
	plt.plot(t, gyro_xyz[2], label='Gvz (°/s)')

	plt.plot(t, angle_xyz[0], label='Ax (°)')
	plt.plot(t, angle_xyz[1], label='Ay (°)')
	plt.plot(t, angle_xyz[2], label='Az (°)')

	#plt.plot(t, acceleration_x, label='Altitude V (m/s)')
	#plt.plot(t, altitudes, label= 'Altitude (m)')

	#
	plt.legend()
	plt.xlabel('Time')
	plt.ylabel('Data')
	plt.title('Flight Data')
	# vel_laying the plot
	plt.show()



app = Ursina()
Sky()
ground = Entity(model='plane',scale=(100,1,100),color = color.gray, texture = 'white_cube',texture_scale=(100,100), collider='box',position=(0,-10,0))

e = Entity(model='cube', color=color.orange, position=(0,0,0), scale=1.5, rotation=(0,0,0), texture='brick')

x = Entity(model='cube', color=color.rgb(255,0,0,50), scale=(0.1, 5, 5))# y=0.5)
y = Entity(model='cube', color=color.rgb(0,255,0,50), scale=(5, 0.1, 5))# y=0.5)
z = Entity(model='cube', color=color.rgb(0,0,255,50), scale=(5, 5, 0.1))# y=0.5)
#z = Entity(model='cube', color=color.rgb(0,255,0,128), scale=(5, 0.1, 5))# y=0.5)

lasttime = current_milli_time()
frame = 0
lastT = t[0]

def update():
	global lasttime
	global frame
	global lastT
	if current_milli_time()-lasttime >=(t[frame]-lastT)/2:#100 ms cooldown
		if held_keys['e']:
			if frame != len(t)-1:
				frame += 1
		if held_keys['q']:	
			if frame != 0:
				frame -= 1
		e.world_rotation_x = angle_xyz[1][frame]
		e.world_rotation_y = angle_xyz[2][frame]
		e.world_rotation_z = angle_xyz[0][frame]
		x.world_rotation_x = angle_xyz[1][frame]
		y.world_rotation_y = angle_xyz[2][frame]
		z.world_rotation_z = angle_xyz[0][frame]

		lastT = t[i]
		lasttime = current_milli_time()
		#self.x -= held_keys['a'] * time.dt * 10



EditorCamera()  # add camera controls for orbiting and moving the camera

app.run()