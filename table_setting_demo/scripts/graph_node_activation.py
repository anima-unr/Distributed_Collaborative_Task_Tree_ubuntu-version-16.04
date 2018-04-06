#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as colors
import matplotlib.cm as cmx
import argparse
import os
import logging
import fnmatch
import csv
import pickle
import pdb
from math import sqrt
import matplotlib
SPINE_COLOR = 'gray'


def latexify(fig_width=None, fig_height=None, columns=2):
    """Set up matplotlib's RC params for LaTeX plotting.
    Call this before plotting a figure.

    Parameters
    ----------
    fig_width : float, optional, inches
    fig_height : float,  optional, inches
    columns : {1, 2}
    """

    # code adapted from http://www.scipy.org/Cookbook/Matplotlib/LaTeX_Examples

    # Width and max height in inches for IEEE journals taken from
    # computer.org/cms/Computer.org/Journal%20templates/transactions_art_guide.pdf

    assert(columns in [1,2])

    if fig_width is None:
        fig_width = 3.39 if columns==1 else 6.9 # width in inches

    if fig_height is None:
        golden_mean = (sqrt(5)-1.0)/2.0    # Aesthetic ratio
        fig_height = fig_width*golden_mean # height in inches

    MAX_HEIGHT_INCHES = 8.0
    if fig_height > MAX_HEIGHT_INCHES:
        print("WARNING: fig_height too large:" + fig_height + 
              "so will reduce to" + MAX_HEIGHT_INCHES + "inches.")
        fig_height = MAX_HEIGHT_INCHES

    params = {'backend': 'ps',
              'text.latex.preamble': ['\usepackage{gensymb}'],
              'axes.labelsize': 8, # fontsize for x and y labels (was 10)
              'axes.titlesize': 8,
              'text.fontsize': 8, # was 10
              'legend.fontsize': 8, # was 10
              'xtick.labelsize': 8,
              'ytick.labelsize': 8,
              'text.usetex': True,
              'figure.figsize': [fig_width,fig_height],
              'font.family': 'serif'
    }

    matplotlib.rcParams.update(params)


def format_axes(ax):

    for spine in ['top', 'right']:
        ax.spines[spine].set_visible(False)

    for spine in ['left', 'bottom']:
        ax.spines[spine].set_color(SPINE_COLOR)
        ax.spines[spine].set_linewidth(0.5)

    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')

    for axis in [ax.xaxis, ax.yaxis]:
        axis.set_tick_params(direction='out', color=SPINE_COLOR)

    return ax



state_color_map = {'000' : [0.8, 0.9, 0.9, 1.0],
                   '001' : [0.75, 0.6, 0.6, 1.0],
                   '101' : [0.2, 0.3, 0.25, 1.0],
                   '011' : [0.3, 0.4, 0.6, 1.0]}

def GetBeginAndEndTime(data):
    return min(data), max(data)

def GetState(array):
    result = ""
    for elem in array:
        result += str(int(elem))
    return result


def StateChange(a_arr, b_arr):
    if GetState(a_arr) != GetState(b_arr):
        return True
    return False


def GenerateHorizontalBar(data):
    # Determine start and end time
    data_array = np.array([[float(j) for j in i] for i in data])
    start_time, end_time = GetBeginAndEndTime(data_array[:, 0])
    # determine segments
    # find activation times
    # build bitmask
    state_array = data_array[:,0:4]
    state_array[:,3] = state_array[:,3] > 0.18

    plot_state_segments = list()
    plot_state_segments.append((state_array[0][0], GetState(state_array[0][1:4])))
    for i in xrange(len(state_array)):
        # time, active, done, activation_level
        if i > 0:
            # Check for state chagnes
            if StateChange(state_array[i-1][1:4], state_array[i][1:4]):
                plot_state_segments.append((state_array[i][0], GetState(state_array[i][1:4])))
    plot_state_segments.append((state_array[-1][0], '000'))
    color = []
    segments = []
    for i in xrange(len(plot_state_segments)-1):
        segments.append(plot_state_segments[i+1][0] - plot_state_segments[i][0])
        color.append(plot_state_segments[i][1])

    # generate color and index
    return segments, color


def GraphData(data_hash, order):
    #update state color map
    # plt.figure(figsize=(16, 9))
    latexify()
    viridis = plt.get_cmap('magma')
    cnorm = colors.Normalize(vmin=0, vmax=100)
    scalarmap = cmx.ScalarMappable(norm=cnorm, cmap=viridis)
    state_color_map['000'] = scalarmap.to_rgba(100)
    state_color_map['001'] = scalarmap.to_rgba(75)
    state_color_map['101'] = scalarmap.to_rgba(50)
    state_color_map['011'] = scalarmap.to_rgba(25)

    bar_width = 0.4
    segment_list = list()
    max_length = -1
    # setup ordering list
    order_list = ['' for x in xrange(len(order))]
    for key in order:
        # find key in hash the most matches
        for elem in data_hash.keys():
            if key in elem:
                order_list[order[key]] = elem
    order_list.reverse()
    for key in order_list:
        print "Processing - [%s]" % key
        # Generate bar graph for key value pair
        bar_info = GenerateHorizontalBar(data_hash[key])

        if len(bar_info[0]) > max_length:
            max_length = len(bar_info[0])
        segment_list.append(bar_info)
    # graph segments
    x_offset = np.array([0.0] * len(segment_list))
    y_index = np.arange(bar_width/2, len(segment_list)*bar_width + bar_width/2, bar_width);
    for i in xrange(max_length):
        # extract row
        data = []
        color = []
        for elem in segment_list:
            if i < len(elem[0]):
                data.append(elem[0][i])
                color.append(state_color_map[elem[1][i]])
            else:
                data.append(0)
                color.append([0.0, 0.0, 0.0, 0.0])
        plt.barh(np.arange(0, len(data)*bar_width, bar_width), data, bar_width, color=color, left=x_offset)
        x_offset += data
    inactive  = patches.Patch(color=state_color_map['000'], label='Inactive')
    active    = patches.Patch(color=state_color_map['001'], label='Active')
    working   = patches.Patch(color=state_color_map['101'], label='Running')
    done      = patches.Patch(color=state_color_map['011'], label='Done')


    ylables = list()
    labels = {
        'THEN_0_1_001_state_Data_.csv'  : 'THEN\_0',
        'PLACE_3_1_002_state_Data_.csv' : 'PLACEMAT',
        'AND_3_1_003_state_Data_.csv'   : 'AND\_0',
        'OR_3_1_004_state_Data_.csv'    : 'OR\_0',
        'PLACE_3_1_005_state_Data_.csv' : 'SPOON',
        'PLACE_3_1_006_state_Data_.csv' : 'FORK',
        'THEN_0_1_007_state_Data_.csv'  : 'THEN\_1',
        'PLACE_3_1_008_state_Data_.csv' : 'KNIFE',
        'PLACE_3_1_009_state_Data_.csv' : 'WINEGLASS',
        'PLACE_3_1_010_state_Data_.csv' : 'CUP',
        'PLACE_3_1_011_state_Data_.csv' : 'SODA',
        'PLACE_3_1_012_state_Data_.csv' : 'PLATE',
        'PLACE_3_1_013_state_Data_.csv' : 'BOWL',
    }
    for key in order_list:
        ylables.append(labels[key])

    plt.yticks(y_index, ylables)
    plt.legend(handles=[inactive, active, working, done], ncol=4, mode="expand")
    plt.ylabel("Behaviors")
    plt.xlabel("t(s)")
    plt.tight_layout()
    ax = plt.gca()
    format_axes(ax)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Process directory input')
    parser.add_argument('-d', '--dir', type=str, required=True, help='Directory of behavior runtime results')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output that shows graph outputs')
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        logging.error('Directory not found - [%s]' % os.path.abspath(args.dir))
        return -1
    if not os.path.isdir(args.dir):
        logging.error('Error: Expected directory not file')
        return -1

    print 'Loading files...'
    files = [f for f in os.listdir(args.dir) if os.path.isfile(os.path.join(args.dir, f))]
    # load ordering dictionary
    with open('table_setting_demo_node_graph_order.txt', 'r') as file:
        order = pickle.loads(file.read())
    # Generate data dictionary to store information
    data = dict()
    for file in files:
        if file != 'remote_mutex.csv':
            if fnmatch.fnmatch(file, '*.csv'):
                print 'Loading [%s]' % file
                with open(os.path.join(args.dir, file), 'rb') as csvfile:
                    spamreader = csv.reader(csvfile, delimiter=',')
                    data[file] = list(spamreader)

    print 'Processing Data...'

    GraphData(data, order)

if __name__ == "__main__":
    main()
