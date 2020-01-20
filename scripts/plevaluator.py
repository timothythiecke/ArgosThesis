'''
Runs the PL checker script for each distribution of a certain population size
in order to create a file reminiscent of the october research.
Creates a .csv file with information and result of binomial test
Regexp is performed on the result of the plchecker script
0 in csv -> test failed, 1 in csv -> PL succesful
It is recommended you run this file in the parent folder of the SeedData folder
That same folder should contain the checkPL.sh as well

usage
>> python plevaluator.py
'''

import os
import re
import numpy as np

seeds = range(1, 31)
populations = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 750, 1000]
filenames = ['deg', 'degTime', 'range', 'rangeTime', 'NN', 'NNTime']

for population in populations:
    print('Processing for population {}...'.format(population))
    f = open('{}.csv'.format(population), "w+")
    f.write('seed,time,degDist,degDistTime,rangeDist,rangeDistTime,nnDist,nnDistTime,avgDeg,avgNNDistance,avgRange\n')

    for seed in seeds:
        f.write('{},'.format(seed))
        f.write('{},'.format(100))
        print('Processing seed {}...'.format(seed))
        for filename in filenames:
            file_proper = './SeedData/{}_{}_{}_{}'.format(seed, population, 100, filename)
            file_sanitized = file_proper + '_sanitized.dat'
            
            print ('Calling PLChecker for {}'.format(file_sanitized))
            
            data = np.loadtxt(file_proper)
            #np.savetxt(file_sanitized, np.trim_zeros(data), fmt='%.3f') # Data requires sanitization, as PL is not well defined for values of 0
            
            sanitized = open(file_sanitized, 'w+')
            #sanitized.write(np.trim_zeros(data))
            for x in np.nditer(np.trim_zeros(data)):
                sanitized.write('{}\n'.format(x))
            sanitized.close()

            
            cmd = os.popen('./CheckPL.sh {}'.format(file_proper)).read()
            # 6_50_100_deg
            print ('Result: ', cmd)
            result = re.search("NOT", cmd)
            if (result):
                f.write('0,')
            else:
                f.write('1,')

        meta = open('./SeedData/{}_{}_{}_{}'.format(seed, population, 100, 'meta'))
        contents = meta.read()
        f.write(contents)

        print ('Done with seed {}!'.format(seed))

    f.close()    
    print('Done with processing for population {}!'.format(population))

print ('======= DONE =======')