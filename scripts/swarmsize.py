''' 
Makes graphs based on data from the .csv files of the datasets/PLevaluator folder

Used to generate graphs from section 4.7

usage
>> swarmsize.py (inside datasets/PLevaluator folder
'''
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

sizes = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500]#, 750]#, 750]
seeds = 30

PLdata = []
AVGdata = []
for size in sizes:
    file = open('{}.csv'.format(size), 'r')
    read = file.readline()
    #print(read)
    #print('first')

    degDist = 0
    degDistTime = 0
    rangeDist = 0
    rangeDistTime = 0
    nnDist = 0
    nnDistTime = 0
    avgDegree = 0
    avgNNDist = 0
    avgRange = 0

    i = 0
    while read:
        if i > 0:
            line = read.split(',')   
            #print(line[2])
        
            degDist += float(line[2])
            degDistTime += float(line[3])
            rangeDist += float(line[4])
            rangeDistTime += float(line[5])
            nnDist += float(line[6])
            nnDistTime += float(line[7])
            avgDegree +=  float(line[8])
            avgNNDist += float(line[9])
            avgRange += float(line[10])

        i += 1
        read = file.readline()

    PLdata.append([degDist/seeds, degDistTime/seeds, rangeDist/seeds, rangeDistTime/seeds, nnDist/seeds, nnDistTime/seeds])
    AVGdata.append([avgDegree/seeds, avgNNDist/seeds, avgRange/seeds])

    file.close()

#print (PLdata)
#print
#print(AVGdata)


#x1 = sizes
#y1 = [PLdata[0][0]]
# deg
degs = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    degs[size_index] = PLdata[size_index][0]


#deg time
degsTime = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    degsTime[size_index] = PLdata[size_index][1]

# range
ranges = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    ranges[size_index] = PLdata[size_index][2]

# range time
rangeTime = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    rangeTime[size_index] = PLdata[size_index][3]

# nn 
nn = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    nn[size_index] = PLdata[size_index][4]

# nn time
nnTime = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    nnTime[size_index] = PLdata[size_index][5]

# avgDeg
avgDeg = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    avgDeg[size_index] = AVGdata[size_index][0]

# avgNNDist
avgNNDist = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    avgNNDist[size_index] = AVGdata[size_index][1]

# avgRange
avgRange = [0] * len(sizes)
for size in sizes:
    size_index = int((size/50) - 1)
    if size_index > 10:
        size_index = 10
    avgRange[size_index] = AVGdata[size_index][2]


df = pd.DataFrame({'x1':sizes, 'degs':degs, 'degsTime':degsTime, 'ranges':ranges, 'rangeTime':rangeTime, 'nn':nn, 'nnTime':nnTime,
                    'avgDeg':avgDeg, 'avgNNDist':avgNNDist, 'avgRange':avgRange})

plt.figure('PL data')
plt.grid(True)
plt.grid(which='minor', alpha=0.2, linestyle='--')
plt.grid(which='major', alpha=0.5)
plt.minorticks_on()
#plt.axis([0,550,-0.1,1.1])
#plt.subplot(2,2,1)
plt.plot('x1', 'degs', data=df, marker='.', color='deepskyblue', label='Degree (snapshot)')
plt.plot('x1', 'degsTime', data=df, marker='^', color='dodgerblue', label='Degree (over time)')

plt.plot('x1', 'ranges', data=df, marker='.', color='orangered', label='Range (snapshot)')
plt.plot('x1', 'rangeTime', data=df, marker='^', color='maroon', label='Range (over time)')

#plt.plot('x1', 'nn', data=df, marker='+', color='limegreen', label='nn')
#plt.plot('x1', 'nnTime', data=df, marker='+', color='darkgreen', label='nnTime')

plt.title('Probabilities for various swarm sizes')
plt.xlabel('Swarm size (N)')
plt.ylabel('Probability of power law distribution in data set (p(x))')

plt.legend()
plt.tight_layout()

plt.figure('AVG data') # Is avg data interesting? no over time, only snapshot at end
plt.grid(True)
plt.grid(which='minor', alpha=0.2)
plt.grid(which='major', alpha=0.5)
plt.minorticks_on()
plt.plot('x1', 'avgDeg', data=df, marker='.', color='deepskyblue', label='Average degree')
plt.plot('x1', 'avgNNDist', data=df, marker='.', color='orangered', label='Average nearest neighbour distance')
plt.plot('x1', 'avgRange', data=df, marker='.', color='limegreen', label='Average range')
plt.title('Averages at the end of simulation for various swarm sizes')

plt.xlabel('Swarm size (N)')
plt.ylabel('Averages')

plt.legend()
plt.tight_layout()
plt.show()