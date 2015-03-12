#!/usr/bin/env python

"""Plot collision avoidance

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

if __name__ == '__main__':
    arguments = docopt(__doc__)

    folder = arguments['<folder>']
    if folder:
        files = glob(path.join(folder, 'wbc_CollisionAvoidance_*.dat'))

        dfs = [pd.read_csv(f, sep='\t', index_col='Time') for f in files]
        df = pd.concat(dfs)

        # remove all empty rows
        df = df[df.index.values != 0]

        plot(df)
