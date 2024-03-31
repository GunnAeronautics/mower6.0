import csv
import pandas as pd
import matplotlib.pyplot as plt
import math

def pressureToAlt(pressure): #returns alt (m) from pressure in h pascals
	return (REF_GROUND_ALTITUDE+((273+REF_GROUND_TEMPERATURE)/(-.0065))*((pow((pressure*100/REF_GROUND_PRESSURE),((8.314*.0065)/(9.807*.02896))))-1)) #https://www.mide.com/air-pressure-at-altitude-calculator, https://en.wikipedia.org/wiki/Barometric_formula 
def radiansToDegrees(radians):
	return (radians*180/3.1415926) 
def integration(times,values):#right hand side integral with variable data points
	lastT = times[0] #right side reimenn sum 
	Valuelist = [0]
	timetable = []
	for i in range(1,len(times)):
		deltaT = (times[i]-lastT)/1000
		Valuelist.append(Valuelist[i-1] + values[i]*deltaT)
		lastT = times[i]
		timetable.append(deltaT*1000)
	print ('sum', sum(timetable)/len(times))
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
	df = pd.read_csv('DATALOG0.csv')#header=None)
	titles = df.columns
	#print (df.fieldnames)
	df = df.T#transpose

	df = df.values.tolist()
	t = df[0]#time

	#altitudes = []
	print (len(df))
	for i in range(1,len(df)):
		plt.plot(t, df[i], label=titles[i])

	

	#plt.plot(t, acceleration_x, label='Altitude V (m/s)')
	#plt.plot(t, altitudes, label= 'Altitude (m)')

	#
	plt.legend()
	plt.xlabel('Time')
	plt.ylabel('Data')
	plt.title('Flight Data')
	# vel_laying the plot
	plt.show()

