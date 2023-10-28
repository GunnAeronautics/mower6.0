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



df = pd.read_csv('datalog1.csv')#header=None)
df = df.T
df = df.values.tolist()
#print (df)
t = df[0]

altitudes = []
gyroxyz = [[],[],[]]
anglexyz = [[0],[0],[0]]
#altitudes = []
lastT = df[0][0]
for i in range(len(df[7])):
	altitudes.append(pressureToAlt(df[7][i]))
	gyroxyz[0].append(radiansToDegrees(df[4][i]))
	gyroxyz[1].append(radiansToDegrees(df[5][i]))
	gyroxyz[2].append(radiansToDegrees(df[6][i]))

for i in range(len(df[7])-1):#integration
	deltaT = (df[0][i]-lastT)/1000
	anglexyz[0].append(anglexyz[0][i-1] + gyroxyz[0][i]*deltaT)
	anglexyz[1].append(anglexyz[1][i-1] + gyroxyz[1][i]*deltaT)
	anglexyz[2].append(anglexyz[2][i-1] + gyroxyz[2][i]*deltaT)
	#anglexyz[0].append(gyroxyz[0][i]*deltaT)
	#anglexyz[1].append(gyroxyz[1][i]*deltaT)
	#anglexyz[2].append(gyroxyz[2][i]*deltaT)
	lastT = df[0][i]

#plt.plot(t, df[1], label='Ax')
#plt.plot(t, df[2], label='Ay')
#plt.plot(t, df[3], label='Az')
plt.plot(t, gyroxyz[0], label='Gx')
plt.plot(t, gyroxyz[1], label='Gy')
plt.plot(t, gyroxyz[2], label='Gz')
plt.plot(t, anglexyz[0], label='Ax')
plt.plot(t, anglexyz[1], label='Ay')
plt.plot(t, anglexyz[2], label='Az')
#plt.plot(t, altitudes, label= 'B')
plt.legend()
plt.xlabel('Time')
plt.ylabel('Data')
plt.title('Flight Data')
# Displaying the plot
plt.show()