#!/usr/bin/env python3 

# Import
import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
import numpy
import sys

def parse_log_files(path_file, env_file, bounds_file, pendulum, car, car_length, start_goal, control_bounds):
    
    if pendulum:
        path_arr = numpy.flip(numpy.loadtxt(path_file, usecols=range(2), dtype=numpy.float64), axis=1)
    
    elif car:
        path_arr = numpy.loadtxt(path_file, usecols=range(4), dtype=numpy.float64)
    
    else:
        raise ValueError("Need to specify pendulum or car.")

    if (start_goal): 
        with open(start_goal) as data:
            for line in data:
                if line.strip():
                    start_goal_nums = [float(val) for val in line.strip().split(',')]

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

    if (control_bounds):
      control_bounds_nums = [float(val) for val in control_bounds.strip().split()]
    else:
      control_bounds_nums = None

    return path_arr, env_arr, env_bounds, start_goal_nums, control_bounds_nums
    
def gen_env_vis(env_arr, env_bounds, pendulum, car, file_num):
    
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3]) 
    
    if (pendulum):
        plt.title('Pendulum Environment')
        plt.xlabel(f'$\\omega$')
        plt.ylabel(f'$\\theta$')
        plt.savefig('figures/pendulum/env.png')
    
    elif (car):
        plt.title('Car Environment')
        plt.xlabel(f'$x$')
        plt.ylabel(f'$y$')
        plt.savefig('figures/car/env.png')
    
    else:
        raise ValueError("Need to specify pendulum or car.")

    return None

def gen_path_vis(path_arr, env_arr, env_bounds, start_goal, pendulum, car, file_num, planner, control_bounds, car_length):
    # Visualize the environment
    fig, ax = plt.subplots()

    for i in range(numpy.shape(env_arr)[0]):
        ax.add_patch(Rectangle((env_arr[i,0], env_arr[i,1]), env_arr[i,2], env_arr[i,3], edgecolor='steelblue', facecolor='grey', fill=True, lw=2))
    ax.set_xlim(env_bounds[0], env_bounds[1]) 
    ax.set_ylim(env_bounds[2], env_bounds[3])

    # Plot the car orientation
    if (car):
        car_length = float(car_length)
        for i in range(0, path_arr.shape[0], 5):
            car_rect = plt.Rectangle((path_arr[i,0] - 0.5*car_length, path_arr[i,1] - 0.5*car_length), width=car_length, height=car_length, color="red", transform=Affine2D().rotate_deg_around(*(path_arr[i,0], path_arr[i,1]), (180/3.14159)*path_arr[i,2])+ax.transData)
            ax.add_patch(car_rect)

    # Plot start point
    start_circle = plt.Circle((start_goal[0], start_goal[1]), start_goal[-1], fill=False, color='r')
    ax.add_artist(start_circle)
    
    # Plot goal point
    goal_circle = plt.Circle((start_goal[2], start_goal[3]), start_goal[-1], fill=False, color='r')
    ax.add_artist(goal_circle)
    
    # Add the computed path
    plt.plot(path_arr[:,0], path_arr[:,1], '-', color='blue')
    plt.grid()
    
    if (pendulum):
        plt.title('Pendulum Solution with ' + planner + ' Planner and Torque of ' + str(control_bounds[0]))
        plt.xlabel(f'$\\omega$')
        plt.ylabel(f'$\\theta$')
        plt.savefig('figures/pendulum/pathenv' + file_num + '.png')
  
    elif (car):
        plt.title('Car Solution with ' + planner)
        plt.xlabel(f'$x$')
        plt.ylabel(f'$y$')
        plt.savefig('figures/car/pathenv' + file_num + '.png')
    
    else:
        raise ValueError("Need to specify pendulum or car.")

    return None

def main(argv):
    
    # Parse Input Arguments
    parser = argparse.ArgumentParser(description='Visualize and solution path and environment.')
    parser.add_argument('--pendulum', default=False, action='store_true', help='Option for pendulum robot')
    parser.add_argument('--car', default=False, action='store_true', help='Option for car robot')
    parser.add_argument('--car-length', default=None, help='Specify the length of the car')
    parser.add_argument('-n', '--file-num', default=None, help='Specify the file numering for output naming conventions.', required=True)
    parser.add_argument('--start-goal', default=None, help='Option for specifying start and goal states.')
    parser.add_argument('-e', '--env', default=False, action='store_true', help='Option to visualize the environment only.')
    parser.add_argument('-p', '--path', default=False, action='store_true', help='Option to visualize the environment and path only.')
    parser.add_argument('--planner', default=None, help='Specify the planner used.', required=True)
    parser.add_argument('--control-bounds', default=None, help='Specify the control bounds.')

    (options, args) = parser.parse_known_args()

    # Parse the log files to produce arrays for visualization
    path_arr, env_arr, env_bounds, start_goal, control_bounds = parse_log_files(args[0], args[1], args[2], options.pendulum, options.car, options.car_length, options.start_goal, options.control_bounds)

    # Generate the visualzation of environment only
    if (options.env):
        gen_env_vis(env_arr, env_bounds, options.pendulum, options.car, options.file_num)

    # Generate the visualzation of path and environment
    if (options.path):
        gen_path_vis(path_arr, env_arr, env_bounds, start_goal, options.pendulum, options.car, options.file_num, options.planner, control_bounds, options.car_length)
    
    return 0

if(__name__ == '__main__'):
   try:
       sys.exit(main(sys.argv[1:]))
   except IOError:
       sys.exit("I/O Error")
