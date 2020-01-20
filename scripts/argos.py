'''
This script will change the .argos file for you depending on what you need then call ARGoS with 
this new ARGoS file as its input. This saves having to pass other command line arguments to ARGoS.
Script terminated when all seeds, quantity and time combinations have been ran
It is recommended that you comment the <visualization> element of the original file to speed up the process

It should only be run if you want to automate the generation of kappa and lambda datasets (which are supplied in the repo already!!!)

usage
>> python argos.py
'''

import os
import xml.etree.ElementTree as ET

seeds = range(1, 31)
quantities = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500, 750, 1000]
times = [100]
folder = '/mnt/c/argos/argos3/argos3-examples/experiments/'
for quantity in quantities:
    for time in times:
        for seed in seeds:
            print ('Preparing .argos file for seed {}, quantity {}, maxtime {}...'.format(seed, quantity, time))
            # Prepare values in ARGoS file
            et = ET.parse(folder + 'marching_large.argos')
            root = et.getroot()
            
            experiment_node = root.findall('./framework/experiment')[0]
            experiment_node.set('random_seed', str(seed))
            experiment_node.set('length', str(time))
            
            entity_node = root.findall('./arena/distribute/entity')[0]
            entity_node.set('quantity', str(quantity))

            et.write(folder + 'marching_large_s.argos') 
            print ('Done preparing!')

            # Then call ARGoS with script argos file
            print ('Calling ARGoS...')
            cmd = 'cd /mnt/c/argos/argos3/argos3-examples/; argos3 -c experiments/marching_large_s.argos >> ./output.txt'
            os.system(cmd)
            print ('Done with ARGoS for seed {}, quantity {}, maxtime {}\n'.format(seed, quantity, time))

print('======== COMPLETED ========')
