'''
Loads seed data (kappa or lambda), adds them together, then takes the average over the amount of seeds
Finally, output this file to a reusable format for the plevaluator

Used in conjunction with plot_powerlaw.py but acts as an intermediary to generate seed averaged distribution

Generates the files from the datasets/GeneralPowerLaw folder

Used in preparation for the graphs of section 4.3

usage
>> python prepperforplvisualizer.py (from 
'''

import numpy as np

seeds = range(1, 31)
size = 750 # any from 50 -> 750
time = 100

distributions = np.zeros((1, size))

for seed in seeds:
    print('Loading seed', seed)
    dist = np.loadtxt('./SeedData/{}_{}_{}_degTime'.format(int(seed), int(size), int(time)))
    print('Adding to distribution')
    distributions = np.add(distributions, dist)

print('Dividing distribution by seed count:', len(seeds))
distributions = np.divide(distributions, len(seeds))
print('Done. Saving...')

np.savetxt('./{}.prep'.format(size), np.transpose(distributions), fmt='%.5f') #, newline='\n')
print('Saved! Done')