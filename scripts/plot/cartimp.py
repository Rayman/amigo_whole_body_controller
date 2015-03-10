#!/usr/bin/env python

import os
from glob import glob

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

files = glob('/tmp/wbc_CartImp_*.dat')

df = pd.read_csv(files[0], sep='\t')

rcolumns = ['rx','ry','rz']
poscolumns = ['x', 'y', 'z']

time = df.loc[:,'Time']
r0   = df.head(1).loc[:,rcolumns]
r    = df.loc[:,rcolumns]
pos  = df.loc[:,poscolumns]

ref = pd.DataFrame(r.values   - r0.values, columns=r.columns)
pos = pd.DataFrame(pos.values - r0.values, columns=pos.columns)

data = ref.join(time).join(pos)
data.plot(x='Time')
plt.legend(loc='best')
plt.show()