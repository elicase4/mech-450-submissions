#!/usr/bin/env python3 

# Import
import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy
import re
import sys

def parse_log_files(path_file, env_file, bounds_file, pendulum, car, car_length):
    if pendulum:
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
    
    elif car:
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
        raise ValueError("Need to specify pendulum or car.")

    env_add = []
    with open(env_file) as data:
        for line in data:
            if line.strip():
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
    
def gen_env_vis(env_arr, env_bounds, file_num):
    
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3]) 
    
    plt.savefig('figures/env' + file_num + '.png')
    plt.show()

    return None

def gen_path_vis(path_arr, env_arr, env_bounds, file_num):
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3])

    # Add the computed path
    plt.plot(path_arr[:,0], path_arr[:,1], '-', color='blue')

    plt.savefig('figures/pathenv' + file_num + '.png')
    plt.show()

    return None

def main(argv):
    
    # Parse Input Arguments
    parser = argparse.ArgumentParser(description='Visualize and solution path and environment.')
    parser.add_argument('--pendulum', default=False, action='store_true', help='Option for pendulum robot')
    parser.add_argument('--car', default=False, action='store_true', help='Option for car robot')
    parser.add_argument('--car-length', default=None, help='Specify the length of the car')
    parser.add_argument('-n', '--file-num', default=None, help='Specify the file numering for output naming conventions.', required=True)

    (options, args) = parser.parse_known_args()

    # Parse the log files to produce arrays for visualization
    path_arr, env_arr, env_bounds = parse_log_files(args[0], args[1], args[2], options.pendulum, options.car, options.car_length)

    # Generate the visualzation of environment only
    gen_env_vis(env_arr, env_bounds, options.file_num)

    # Generate the visualzation of path and environment
    gen_path_vis(path_arr, env_arr, env_bounds, options.file_num)
    
    return 0

if(__name__ == '__main__'):
   try:
       sys.exit(main(sys.argv[1:]))
   except IOError:
       sys.exit("I/O Error")
