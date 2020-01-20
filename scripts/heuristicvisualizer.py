'''
Script to plot the history data of the n_min, n_mid and n_max nodes as selected by the NNDH and PDH heuristics
Random selection heuristic is also supported

Used for section about heuristics 

usage
>> python heuristicvisualizer.py (from parent folder of HeuristicData/ folder)
'''

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

heuristics = ['extreme', 'random', 'postdegree']
heuristic = heuristics[0]
idxs = ['max', 'mid', 'min']
idx = idxs[1]
time = 100
seed = 10
swarmsize = 200
base = './HeuristicData/{}_{}_{}_'.format(seed, swarmsize, time)#, heuristic, datatype)
file = base + heuristic + '_{}'.format(idx)

print(file)

ranges = np.loadtxt(file + '_{}'.format('range'))
nnranges = np.loadtxt(file + '_{}'.format('nnrange'))
nndistances = np.loadtxt(file + '_{}'.format('nndist'))
localdegrees = np.loadtxt(file + '_{}'.format('localdegree'))
globaldegrees = np.loadtxt(file + '_{}'.format('degree'))
directiondecisions = np.loadtxt(file + '_{}'.format('directiondecision'))
breakdowns = np.loadtxt(file + '_{}'.format('breakdown'))
fractiondifferences = np.loadtxt(file + '_{}'.format('fractiondifference'))

x1 = range(ranges.size)
y1 = np.array(ranges)
y2 = np.array(nnranges)

df = pd.DataFrame({'nndistances':np.sqrt(nndistances), 'nnranges':nnranges, 'directiondecisions':directiondecisions, 'ranges':ranges,
                    'globaldegrees':globaldegrees, 'localdegrees':localdegrees, 'breakdowns':np.ma.masked_where(breakdowns < -0.01, breakdowns), 'fractiondifferences':np.ma.masked_where(fractiondifferences < -0.01, fractiondifferences), 'x1':x1})

plt.figure('local degree vs global degree')
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot('x1', 'localdegrees', data=df, label='Local', color='dodgerblue')
plt.plot('x1', 'globaldegrees', data=df, label='Global', color='darkorange')
plt.xlabel('Time step (t)')
plt.ylabel('Degree')
plt.legend()

plt.figure('range vs nn range vs nn distance')
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot('x1', 'ranges', data=df, label='Own range', color='dodgerblue')
plt.plot('x1', 'nnranges', data=df, label='NN range', color='darkorange')
plt.plot('x1', 'nndistances', data=df, label='NN distance', color='limegreen')
plt.xlabel('Time step (t)')
plt.ylabel('Range or distance (m)')
plt.legend()

'''plt.figure('breakdown')
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot('x1', 'breakdowns', data=df, label='Own range', color='dodgerblue')
plt.plot('x1', 'fractiondifferences', data=df, label='NN range', color='darkorange')
plt.plot('x1', 'directiondecisions', data=df, label='NN distance', color='limegreen')
plt.xlabel('Time step (t)')
#plt.ylabel('Range or distance (m)')
plt.legend()'''


'''plt.figure('range vs nn distance')
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot('x1', 'ranges', data=df, label='Own range', color='dodgerblue')
plt.plot('x1', 'nndistances', data=df, label='NN distance', color='darkorange')
plt.xlabel('Time step (t)')
plt.ylabel('Distance (m)')
plt.legend()


plt.figure('nn range vs nn distance')
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot('x1', 'nnranges', data=df, label='NN range', color='darkorange')
plt.plot('x1', 'nndistances', data=df, label='NN distance', color='dodgerblue')
plt.xlabel('Time step (t)')
plt.ylabel('Distance (m)')
plt.legend()'''


plt.figure('main')
plt.subplot(4, 1, 1)
plt.plot(x1, y1)
plt.title('Max plot')
plt.xlabel('Time step (t)')
plt.ylabel('Range (m)')
plot2 = plt.twinx()
plot2.set_ylabel('NN range (m)')
plot2.plot(x1, y2, color='tab:red') # 'o-', voor punten

y1 = np.array(localdegrees)
y2 = np.array(globaldegrees)

plt.subplot(4, 1, 2)
plt.plot(x1, y1)
plt.xlabel('time (s)')
plt.ylabel('Local degree')
plot2 = plt.twinx()
plot2.set_ylabel('Global degree')
plot2.plot(x1, y2, color='tab:red') # 'o-', voor punten

y1 = np.array(breakdowns)
y2 = np.array(directiondecisions)

plt.subplot(4, 1, 3)
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot(x1, np.ma.masked_where(y1 < -0.1, y1))
plt.ylabel('Breakdown')
offset = 0.05
#plt.ylim(-offset,1.0 + offset)
plot2 = plt.twinx()
plot2.set_ylabel('Direction decision')
plot2.plot(x1, y2, '.--', color='darkorange') # 'o-', voor punten

y1 = np.array(breakdowns)
y2 = np.array(fractiondifferences)

plt.subplot(4, 1, 4)
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5, linestyle='--')
plt.minorticks_on()
plt.plot(x1, np.ma.masked_where(y1 < -0.1, y1))
plt.ylabel('Breakdown')
#plt.ylim(-offset,1.0 + offset)
plot2 = plt.twinx()
plot2.set_ylabel('Fraction difference')
plot2.plot(x1, np.ma.masked_where(y2 < -0.1, y2), '.--', color='darkorange') # 'o-', voor punten


#y1 = np.array(globaldegrees)
#y2 = np.array(localdegrees)
#plt.figure('hehe xd')
#plt.plot(x1, y1)
#plt.ylabel('Global degree')
#plot2 = plt.twinx()
#plot2.set_ylim(-2, 43)
#plot2.set_ylabel('Local degree')
#plot2.plot(x1, y2, color='tab:red') # 'o-', voor punten

#plt.legend()

plt.tight_layout()
plt.show()