#!/usr/bin/env python3

# Import
import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy
import re
import sys

def parse_log_files(path_file, env_file, bounds_file, point_robot, box_robot, box_robot_side):
    if point_robot:
        path_add = []
        with open(path_file) as data:
            for line in data:
                if "RealVectorState" in line:
                    cart_nums=[float(val) for val in re.findall(r"[-+]?(?:\d*\.*\d+)", line)]
                    path_add.append(cart_nums)
        path_arr = numpy.array(path_add, dtype=numpy.float64)
    
    elif box_robot:
        path_add1 = []
        path_add2 = []
        with open(path_file) as data:
            for line in data:
                if "RealVectorState" in line:
                    cart_nums=[float(val) for val in re.findall(r"[-+]?(?:\d*\.*\d+)", line)]
                    path_add1.append(cart_nums)
                elif "SO2State" in line:
                    rot_nums=[float(val) for val in re.findall(r"[-+]?(?:\d*\.*\d+)", line)]
                    path_add2.append(rot_nums[-1])
        path_arr = numpy.append(path_add1, numpy.array(path_add2).reshape(-1,1), axis=1)
    
    else:
        raise ValueError("Need to specify point or box robot.")

    env_add = []
    with open(env_file) as data:
        for line in data:
            env_nums = [float(val) for val in line.strip().split(',')]
            env_add.append(env_nums)
    env_arr = numpy.array(env_add, dtype=numpy.float64)
    
    bounds_add = []
    with open(bounds_file) as data:
        for line in data:
            bounds_nums = [float(val) for val in line.strip().split(',')]
            bounds_add.extend(bounds_nums)
    env_bounds = numpy.array(bounds_add, dtype=numpy.float64)

    return path_arr, env_arr, env_bounds
    
def gen_env_vis(env_arr, env_bounds):
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3]) 
    plt.show()

    return None

def gen_path_vis(path_arr, env_arr, env_bounds):
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3])

    # Add the computed path
    print(path_arr)
    plt.plot(path_arr[:,0], path_arr[:,1], '-', color='yellow')

    plt.show()

    return None

def main(argv):
    
    # Parse Input Arguments
    parser = argparse.ArgumentParser(description='Visualize and solution path and environment.')
    parser.add_argument('--point-robot', default=False, action='store_true', help='Option for point robot')
    parser.add_argument('--box-robot', default=False, action='store_true', help='Option for box robot')
    parser.add_argument('--box-robot-side', default=None, help='Specify the side length of a box robot. Must include this if visualizing a box robot')

    (options, args) = parser.parse_known_args()

    # Parse the log files to produce arrays for visualization
    path_arr, env_arr, env_bounds = parse_log_files(args[0], args[1], args[2], options.point_robot, options.box_robot, options.box_robot_side)

    # Generate the visualzation of environment only
    gen_env_vis(env_arr, env_bounds)

    # Generate the visualzation of path and environment
    gen_path_vis(path_arr, env_arr, env_bounds)
    
    return 0

if(__name__ == '__main__'):
   try:
       sys.exit(main(sys.argv[1:]))
   except IOError:
       sys.exit("I/O Error")
