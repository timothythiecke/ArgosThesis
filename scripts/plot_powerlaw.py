''' 
Script written and given by Ilja Rausch 
Will plot the data from a file on the logarithmic scales
with alpha and xmin as returned in the output of a succesful PLevaluator
You will have to edit the parameters yourself in this file after the checkPL.sh call

The script saves the plot to a .png file for you

Example: section 4.3 

usage
>> python plot_powerlaw.py

cf below: alpha, xmin, filename
'''



import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

# ========== FIT FUNCTION ==========

def power_law_cdf(x, xmin, alpha):
      # cdf = cumulative density function
      x = x/xmin
      alpha = -alpha+1
      return x**alpha

# ========== PARAMETERS ==========
# ========== You have to adjust these yourself ==========
      
alpha = 4.493616
xmin = 21.745700

filename = 'degOverTime.lr'

# ========== EMPIRICAL DATA ==========

imported_data = pd.read_csv(filename, header=None, usecols=[0])

data_unsorted = imported_data.values.flatten()
data = np.sort(data_unsorted)[::-1]
count_data_points = len(data)

y_data = []
 
for i in range(1,count_data_points+1):
      y_data.append(i/count_data_points)
      
# ========== FIT ==========
      
idx = (np.abs(data - xmin)).argmin()
nearest_to_xmin = data[idx]

y_fit = []
truncated_data = []

for i in data:
      if i >= xmin:
            y_fit.append(power_law_cdf(i, xmin, alpha)*(idx+1)/count_data_points)
            truncated_data.append(i)
      
# ========== PLOT ==========
    
plt.loglog(data, y_data, label='Data')
plt.loglog(truncated_data, y_fit, linestyle='--', label='Fit')

plt.xlabel('degree')
plt.ylabel('p(x)')

plt.legend()

plt.savefig('data_and_power_law_fit.png', bbox_inches='tight')