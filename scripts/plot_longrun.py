''' 
Script to plot the longrun files of a longrun

Used for the graphs of section 4.8

usage
>> python plot_longrun.py (from parent folder of Longrun folder)
'''

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

degreesFile = './Longrun/degrees.csv'
rangesFile = './Longrun/ranges.csv'

degrees = np.loadtxt(degreesFile, delimiter=',')
deg_low = [row[0] for row in degrees]
deg_mid = [row[1] for row in degrees]
deg_high = [row[2] for row in degrees]
deg_avg = [row[3] for row in degrees]

ranges = np.loadtxt(rangesFile, delimiter=',')
range_low = [row[0] for row in ranges]
range_mid = [row[1] for row in ranges]
range_high = [row[2] for row in ranges]
range_avg = [row[3] for row in ranges]

x = range(0, len(range_avg))

df = pd.DataFrame({'deg_low':deg_low, 'x':x, 'deg_mid':deg_mid, 'deg_high':deg_high, 'deg_avg':deg_avg,
                    'range_low':range_low, 'range_mid':range_mid, 'range_high':range_high, 'range_avg':range_avg})

print('deg_low min', np.amin(deg_low))
print('deg_low max', np.amax(deg_low))
print('deg_low average', np.average(deg_low), np.std(deg_low))
print('deg_mid min', np.amin(deg_mid))
print('deg_mid max', np.amax(deg_mid))
print('deg_mid average', np.average(deg_mid), np.std(deg_mid))
print('deg_high min', np.amin(deg_high))
print('deg_high max', np.amax(deg_high))
print('deg_high average', np.average(deg_high), np.std(deg_high))
print('deg_avg min', np.amin(deg_avg))
print('deg_avg max', np.amax(deg_avg))
print('deg_avg average', np.average(deg_avg), np.std(deg_avg))

print

print('range_low min', np.amin(range_low))
print('range_low max', np.amax(range_low))
print('range_low average', np.average(range_low), np.std(range_low))
print('range_mid min', np.amin(range_mid))
print('range_mid max', np.amax(range_mid))
print('range_mid average', np.average(range_mid), np.std(range_mid))
print('range_high min', np.amin(range_high))
print('range_high max', np.amax(range_high))
print('range_high average', np.average(range_high), np.std(range_high))
print('range_avg min', np.amin(range_avg))
print('range_avg max', np.amax(range_avg))
print('range_avg average', np.average(range_avg), np.std(range_avg))


plt.figure('degrees')
plt.plot('x', 'deg_low', data=df, label='Low')
plt.plot('x', 'deg_mid', data=df, label='Median')
plt.plot('x', 'deg_high', data=df, label='High')

#coef = np.polyfit(x, deg_mid, 1)
#poly1d_fn = np.poly1d(coef)
#plt.plot(x, deg_mid, 'yo', x, poly1d_fn(x), '--k')

plt.plot('x', 'deg_avg', data=df, label='Mean')
plt.xlabel('Time step (t)')
plt.ylabel('Degree')
plt.legend()

plt.figure('ranges')
plt.plot('x', 'range_low', data=df, label='Low')
plt.plot('x', 'range_mid', data=df, label='Median')
plt.plot('x', 'range_high', data=df, label='High')
plt.plot('x', 'range_avg', data=df, label='Mean')
plt.xlabel('Time step (t)')
plt.ylabel('Range (m)')
plt.legend()

plt.tight_layout()
plt.show()

