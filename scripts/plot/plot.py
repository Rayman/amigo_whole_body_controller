#!/usr/bin/env python

import os
from glob import glob

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

files = glob('/media/49A0-55C2/wbc/wbc_joints_*.dat')

df = pd.read_csv(files[0], sep='\t')

columns = [column for column in df.columns if column.endswith('_joint_left')]
tau_columns = [column for column in columns if column.startswith('tau_totalshoulder')]
tau_columns = ['Time'] + tau_columns

taus = df.loc[:,tau_columns]

taus.plot(x='Time')
plt.legend(loc='best')
plt.show()