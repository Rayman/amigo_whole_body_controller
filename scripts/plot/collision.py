#!/usr/bin/env python

import os
from glob import glob

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

files = glob('/tmp/wbc_CollisionAvoidance_*.dat')

df = pd.read_csv(files[0], sep='\t', index_col='Time')

# remove all empty rows
df = df[df.index.values != 0]

# start of experiment = 0 secon
start_date = min(df.index.values)
df.set_index(df.index.values - start_date, inplace=True)

fig = plt.figure()

ax1 = df.min_distance.plot(label='Minimum distance')
ax1.set_xlabel('Time (s)')
ax1.set_ylim(0, 0.066*2)
ax1.set_ylabel('Distance (m)')
ax1.legend(loc=2)
ax1.axhline(y=0.066, color='k')

ax2 = df.amplitude.plot(secondary_y=True, style='g', label='Repulsive force')
ax2.set_ylabel('Force magnitude (N)')
ax2.legend(loc=1)

plt.show()