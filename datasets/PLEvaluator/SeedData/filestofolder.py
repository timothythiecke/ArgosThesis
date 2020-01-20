from shutil import copyfile
import os
import re

#result = re.search("NOT", cmd)
#            if (result):
count = 0
for file in os.listdir('.'):
    match = re.search(".dat$", file)
    if match:
        count += 1
        #print (file)
        copyfile('./' + file, './Sanitized/' + file)

print ('Files: ', count)