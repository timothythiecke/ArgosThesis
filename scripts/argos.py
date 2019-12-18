'''
This script will change the .argos file then call ARGoS with this new file as its input.
This saves having to pass other command line arguments to ARGoS
'''

import os
import xml.etree.ElementTree as ET

seeds = range(1, 4) #range(1, 31)
quantities = [50, 100]#, 150]#, 200]
times = [100]

for seed in seeds:
    for quantity in quantities:
        for time in times:
            print ('Preparing .argos file for seed {}, quantity {}, maxtime {}...'.format(seed, quantity, time))
            # Prepare values in ARGoS file
            et = ET.parse('/mnt/c/argos/argos3/argos3-examples/experiments/marching_large.argos')
            root = et.getroot()
            
            experiment_node = root.findall('./framework/experiment')[0]
            experiment_node.set('random_seed', str(seed))
            experiment_node.set('length', str(time))
            
            entity_node = root.findall('./arena/distribute/entity')[0]
            entity_node.set('quantity', str(quantity))

            et.write('/mnt/c/argos/argos3/argos3-examples/experiments/marching_large_s.argos') 
            print ('Done preparing!')

            # Then call ARGoS with script argos file
            print ('Calling ARGoS...')
            cmd = 'cd /mnt/c/argos/argos3/argos3-examples/; argos3 -c experiments/marching_large_s.argos'
            os.system(cmd)
            print ('Done with ARGoS for seed {}, quantity {}, maxtime {}\n'.format(seed, quantity, time))

print('======== COMPLETED ========')
