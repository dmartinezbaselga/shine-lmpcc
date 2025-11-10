#!/usr/bin/env python3
"""
This node provides a ROS Service that receives [x0, u] and uses the FORCES Pro Dynamics to compute its trajectory
@see PropagateDynamics.srv
"""


import sys, os, shutil

sys.path.append("../")
sys.path.append("")

this_dir = os.path.dirname(os.path.abspath(__file__) + "/scripts")
sys.path.append(os.path.dirname(this_dir))
print(sys.path)
# If your forces is in this directory add it
# from helpers import load_forces_path
import helpers

helpers.load_forces_path()

import rospy
import forcespro.nlp
import numpy as np
import copy

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from lmpcc_solver.srv import PropagateDynamics, PropagateDynamicsRequest, PropagateDynamicsResponse

import matplotlib.pyplot as plt

iterations = 10
save_fig_increase = 6


f = plt.figure(figsize=(10, 25))
ax = f.add_subplot(311)
ax2 = f.add_subplot(312)
ax3 = f.add_subplot(313)

def multiarray2d_to_np(multi_array):
    n_values = multi_array.layout.dim[0].size
    if len(multi_array.layout.dim) > 1:
        n_stages = multi_array.layout.dim[1].size
    else:
        n_stages = 1

    out = np.zeros((n_values, n_stages))
    for stage in range(n_stages):
        out[:, stage] = multi_array.data[stage * n_values : (stage + 1) * n_values]
    return out


def visualize(output):
    # ax = f.add_subplot(111)
    cmap = plt.get_cmap('viridis')
    trajectory = multiarray2d_to_np(output)

    # print(output['x'])
    # print(output['x'][0])
    ax.scatter(trajectory[0, :], trajectory[1, :], linewidth=10)  # np.tile(cmap(color_int / 5.), (20, 1)), linewidth=3.0)
    # ax.plot(trajectory['x_init'], trajectory['y_init'], marker='x', markersize=15, color=[1., 0.1, 0.1], linewidth=2.0)
    # plt.plot(np.array([-1., 1.], np.array([0., 0.]), linestyle='--', linewidth=2.0))
    ax.set_xlabel('x (m)', fontsize=20)
    ax.set_ylabel('y (m)', fontsize=20)
    # ax.set_ylim([-1.25, 1.25])
    # ax.set_xlim([-1.25, 1.25])
    plt.xticks(fontsize=20)
    plt.yticks(fontsize=20)
    plt.show()

def evaluate_objective(req, generated_solver):
    N = req.inputs.layout.dim[1].size
    objective = 0.
    for k in range(0, N):
        pass
    return generated_solver.objective

def propagate_dynamics(req, generated_solver):
    # Format input data
    N = req.inputs.layout.dim[1].size

    inputs = multiarray2d_to_np(req.inputs)
    x_init = multiarray2d_to_np(req.x_init)
    n_states = len(x_init)

    out = np.zeros((n_states, N))
    z_current = np.hstack([inputs[:, 0], x_init[:, 0]]) # [u0, x0]
    npar = int(generated_solver.param_decls[2][2] / generated_solver.nstages)
    for k in range(0, N):
        # help(generated_solver.dynamics)
        # Find x[k + 1] = f(x[k], u[k])
        x_next, j = generated_solver.dynamics(z_current, p=np.ones((npar, )), stage=k)

        # Save x[k + 1] as trajectory[k]
        out[:, k] = copy.deepcopy(np.reshape(x_next, -1)) # Save the output (we do not save the initial state)

        if k < N - 1: # Skip for the last stage
            # Define the next z[k + 1] = (u[k + 1], x[k + 1])
            z_next = np.hstack([inputs[:, k + 1], np.squeeze(x_next)])
            z_current = z_next

    out_array = Float64MultiArray()
    for k in range(N):
        for i in range(n_states):
            out_array.data.append(out[i, k])
    dims = MultiArrayDimension()
    dims.size = n_states
    out_array.layout.dim.append(copy.deepcopy(dims))
    dims.size = N
    out_array.layout.dim.append(copy.deepcopy(dims))

    return PropagateDynamicsResponse(out_array)


if __name__ == '__main__':

    rospy.init_node("online_dynamics_server", anonymous=True)

    # Load the solver
    try:
        generated_solver = forcespro.nlp.Solver.from_directory("JackalFORCESNLPsolver")
    except ImportError:
        print(os.path.dirname(os.path.abspath(__file__)) + "/JackalFORCESNLPsolver")
        generated_solver = forcespro.nlp.Solver.from_directory(os.path.dirname(os.path.abspath(__file__)) + "/JackalFORCESNLPsolver")

    # Propagate dynamics and return the result
    # Input: u : N x n_x, x_init : n_x
    server = rospy.Service('propagate_dynamics', PropagateDynamics, lambda req : propagate_dynamics(req, generated_solver))
    # objective_server = rospy.Service('evaluate_objective', EvaluateObjective, lambda req : evaluate_objective(req, generated_solver))

    if len(sys.argv) > 1 and sys.argv[1] == "test":
        input_test_array = Float64MultiArray()
        init_test_array = Float64MultiArray()
        for k in range(20):
            for i in range(2):
                input_test_array.data.append(2.0)

        for j in range(5):
            init_test_array.data.append(1.)

        dims = MultiArrayDimension()
        dims.size = 2
        input_test_array.layout.dim.append(copy.deepcopy(dims))
        dims.size = 20
        input_test_array.layout.dim.append(copy.deepcopy(dims))
        dims.size = 5
        init_test_array.layout.dim.append(copy.deepcopy(dims))
        req = PropagateDynamicsRequest()
        req.x_init = init_test_array
        req.inputs = input_test_array
        output = propagate_dynamics(req, generated_solver).outputs

        visualize(output)

    while not rospy.is_shutdown():
        rospy.spin()
