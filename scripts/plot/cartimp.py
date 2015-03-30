#!/usr/bin/env python

"""Plot cartesian impedance goals

Usage:
  plot.py
  plot.py <folder>

Options:
  -h --help     Show this screen
"""

import os
from glob import glob
from os import path
from docopt import docopt

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot(df):
    rcolumns = ['rx','ry','rz']
    poscolumns = ['x', 'y', 'z']

    # start of experiment = 0 secon
    start_date = min(df.index.values)
    df.set_index(df.index.values - start_date, inplace=True)

    r0   = df.head(1).loc[:,rcolumns]
    r    = df.loc[:,rcolumns]
    pos  = df.loc[:,poscolumns]

    ref = pd.DataFrame(r.values   - r0.values, columns=r.columns, index=df.index.values)
    pos = pd.DataFrame(pos.values - r0.values, columns=pos.columns, index=df.index.values)

    data = ref.join(pos)
    data.plot(style='.')
    plt.legend(loc='best')
    plt.show()


if __name__ == '__main__':
    arguments = docopt(__doc__)

    folder = arguments['<folder>']
    if folder:
        files = glob(path.join(folder, 'wbc_CartImp_*.dat'))

        dfs = [pd.read_csv(f, sep='\t', index_col='Time') for f in files]
        df = pd.concat(dfs)

        # remove all empty rows
        df = df[df.index.values != 0]

        plot(df)
