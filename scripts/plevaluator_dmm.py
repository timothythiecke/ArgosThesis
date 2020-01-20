'''
Runs the PL checker script for each file of a certain population size
in order to create a file reminiscent of the october research
Functionally the same as plevaluator.py but handles different file handles

Used to generate the files of dataset/DMManalysis folder

Used for dmm analysis section in 4.4

usage
>> python plevaluator_dmm.py  (from parent folder of DMMAnalysis folder (contains different dataset than SeedData!))
'''

import os
import re
import numpy as np

seeds = range(1, 31)
populations = [200]
cases = ['00', '01', '10', '11']

filenames = ['deg', 'degTime']

for case in cases:
    print('Processing for case {}...'.format(case))
    f = open('dmm_{}_{}.csv'.format(100, case), "w+")
    f.write('seed,time,degDist,degDistTime,avgDeg,avgNNDistance,avgRange\n')

    for seed in seeds:
        f.write('{},'.format(seed))
        f.write('{},'.format(100))
        print('Processing seed {}...'.format(seed))
        for filename in filenames:
            file_proper = './SeedData/{}_{}_{}_{}_{}'.format(seed, populations[0], 100, case, filename)
            file_sanitized = file_proper + '_sanitized.dat'
            
            print ('Calling PLChecker for {}'.format(file_sanitized))
            
            data = np.loadtxt(file_proper)
            #print(data)
            #print(np.trim_zeros(data))
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

        meta = open('./SeedData/{}_{}_{}_{}_{}'.format(seed, populations[0], 100, case, 'meta'))
        #contents = meta.read().splt(',')
        #print(contents)
        contents = meta.read()
        #print (contents)
        f.write(contents)

        print ('Done with seed {}, case {}!'.format(seed, case))

    f.close()    
    print('Done with processing for case {}!'.format(case))

print ('======= DONE =======')