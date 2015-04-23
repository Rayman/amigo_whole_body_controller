#!/usr/bin/env python

"""Plot Taus

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

    # start of experiment = 0 secon
    start_date = min(df.index.values)
    df.set_index(df.index.values - start_date, inplace=True)

    columns = [column for column in df.columns if column.endswith('_joint_left')]
    for c in columns:
        print c
    tau_columns = [column for column in columns if column.startswith('tau_totalshoulder')]

    taus = df.loc[:, tau_columns]

    taus.plot(style='.')
    plt.legend(loc='best')
    plt.show()

if __name__ == '__main__':
    arguments = docopt(__doc__)

    folder = arguments['<folder>']
    if folder:
        files = glob(path.join(folder, 'wbc_joints_*.dat'))

        dfs = [pd.read_csv(f, sep='\t', index_col='Time') for f in files]
        df = pd.concat(dfs)
        df.sort_index(inplace=True)

        # remove all empty rows
        df = df[df.index.values != 0]

        plot(df)
