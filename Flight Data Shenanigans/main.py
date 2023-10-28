import csv
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('datalog0.csv',header=None)
df = df.T
df = df.values.tolist()
print (df)
t = df[0]

plt.plot(t, df[1], label='Ax')
plt.plot(t, df[2], label='Ay')
plt.plot(t, df[3], label='Az')
plt.plot(t, df[4], label='Gx')
plt.plot(t, df[5], label='Gy')
plt.plot(t, df[6], label='Gz')
#plt.plot(t, df[7], label= 'B')
plt.legend()
plt.xlabel('Time')
plt.ylabel('Data')
plt.title('Flight Data')
# Displaying the plot
plt.show()