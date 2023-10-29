import csv
import pandas as pd
import matplotlib.pyplot as plt

REF_GROUND_PRESSURE =    101185.29#//in pascals CHANGE BEFORE FLIGHT
REF_GROUND_ALTITUDE =    0#//in meters CHANGE BEFORE FLIGHT
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

df = pd.read_csv('datalog2.csv')#header=None)
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

for i in range(len(df[0])):
	altitudes.append(pressureToAlt(df[7][i]))
	gyro_xyz[0].append(radiansToDegrees(df[4][i]))
	gyro_xyz[1].append(radiansToDegrees(df[5][i]))
	gyro_xyz[2].append(radiansToDegrees(df[6][i]))

plt.plot(t, gyro_xyz[0], label='Gx')
plt.plot(t, gyro_xyz[1], label='Gy')
plt.plot(t, gyro_xyz[2], label='Gz')
angle_xyz[0] = integration(t,gyro_xyz[0])
angle_xyz[1] = integration(t,gyro_xyz[1])
angle_xyz[2] = integration(t,gyro_xyz[2])

vel_xyz[0] = integration(t,df[1])
vel_xyz[1] = integration(t,df[2])
vel_xyz[2] = integration(t,df[3])

displacement_xyz[0] = integration(t,vel_xyz[0])
displacement_xyz[1] = integration(t,vel_xyz[1])
displacement_xyz[2] = integration(t,vel_xyz[2])

absolute_x = derrivative(t,altitudes)
#comment out unnessecary data
plt.plot(t, df[1], label='Ax (m/s^2)')
plt.plot(t, df[2], label='Ay (m/s^2)')
plt.plot(t, df[3], label='Az (m/s^2)')

plt.plot(t, vel_xyz[0], label='Vx (m/s)')
plt.plot(t, vel_xyz[1], label='Vy (m/s)')
plt.plot(t, vel_xyz[2], label='Vz (m/s)')

plt.plot(t, displacement_xyz[0], label='Dx (m)')
plt.plot(t, displacement_xyz[1], label='Dy (m)')
plt.plot(t, displacement_xyz[2], label='Dz (m)')

plt.plot(t, gyro_xyz[0], label='Gvx (°/s)')
plt.plot(t, gyro_xyz[1], label='Gvy (°/s)')
plt.plot(t, gyro_xyz[2], label='Gvz (°/s)')

plt.plot(t, angle_xyz[0], label='Ax (°)')
plt.plot(t, angle_xyz[1], label='Ay (°)')
plt.plot(t, angle_xyz[2], label='Az (°)')

plt.plot(t, absolute_x, label='Altitude V (m/s)')
plt.plot(t, altitudes, label= 'Altitude (m)')

#
plt.legend()
plt.xlabel('Time')
plt.ylabel('Data')
plt.title('Flight Data')
# vel_laying the plot
plt.show()