'''
Script prepares data for DMM analysis
Funcitonally does the same as argos.py
'''

import os
import xml.etree.ElementTree as ET

seeds = range(1, 31) 
quantities = [100, 300] 
cases = ['00', '01', '10', '11']
folder = '/mnt/c/argos/argos3/argos3-examples/experiments/'
for quantity in quantities:
    for case in cases:
        for seed in seeds:
            print ('Preparing .argos file for seed {}, quantity {}, maxtime {}...'.format(seed, quantity, 100))
            # Prepare values in ARGoS file
            et = ET.parse(folder + 'marching_large.argos')
            root = et.getroot()
            
            experiment_node = root.findall('./framework/experiment')[0]
            experiment_node.set('random_seed', str(seed))
            
            entity_node = root.findall('./arena/distribute/entity')[0]
            entity_node.set('quantity', str(quantity))

            dmm_node = root.findall('./controllers/footbot_marching_controller/params/dmm')[0]
            if case == '00':
                dmm_node.set('breakdownEnabled', 'false')
                dmm_node.set('localNeighbourhoodCheck', 'false')

            if case == '01':
                dmm_node.set('breakdownEnabled', 'false')
                dmm_node.set('localNeighbourhoodCheck', 'true')

            if case == '10':
                dmm_node.set('breakdownEnabled', 'true')
                dmm_node.set('localNeighbourhoodCheck', 'false')
            
            if case == '11':
                dmm_node.set('breakdownEnabled', 'true')
                dmm_node.set('localNeighbourhoodCheck', 'true')

            et.write(folder + 'marching_large_s.argos') 
            print ('Done preparing!')

            # Then call ARGoS with script argos file
            print ('Calling ARGoS...')
            cmd = 'cd /mnt/c/argos/argos3/argos3-examples/; argos3 -c experiments/marching_large_s.argos >> ./output.txt'
            os.system(cmd)
            print ('Done with ARGoS for seed {}, quantity {}, maxtime {}\n'.format(seed, quantity, 100))

print('======== COMPLETED ========')
