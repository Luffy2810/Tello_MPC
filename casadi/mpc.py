#!/usr/bin/env python3

import casadi
from time import time
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

def odom_callback(msg):
    node.get_logger().info(f"Received Odometry message: {msg.pose.pose}")

def shift_timestep(step_horizon, t0, state_init, u, f):
    # print("u: ",u[:,0])
    f_value = f(state_init, u[:,0])
    # print ("F_Value:",f_value,)
    # print ("States: ",state_init)
    # print (u[:,0])
    next_state = casadi.DM.full(state_init + (step_horizon * f_value))
    # print ("F_Values", f_value)
    t0 = t0 + step_horizon
    u0 = casadi.horzcat(
        u[:, 1:],
        casadi.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


def DM2Arr(dm):
    return np.array(dm.full())
# v_max = 0.6
# v_min = -0.6
# omega_max = math.pi/4
# omega_min = -omega_max
Q = casadi.diagcat(1,5,0.1)
R = casadi.diagcat(0.5,0.05)
step_horizon = 0.082  # time between steps in seconds
N = 100              # number of look ahead steps
rob_diam = 0.3      # diameter of the robot
wheel_radius = 1    # wheel radius
Lx = 0.3            # L in J Matrix (half robot x-axis length)
Ly = 0.3            # l in J Matrix (half robot y-axis length)
sim_time = 200      # simulation time

# specs
x_init = 0
y_init = 0
theta_init = 0
x_target = 20
y_target = 10
theta_target = pi/4

v_max = 1
v_min = -1
omega_min = -pi/4
omega_max = pi//4
x = casadi.SX.sym('x')
y = casadi.SX.sym('y')
theta = casadi.SX.sym('theta')
states = casadi.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()
v = casadi.SX.sym('v')
omega = casadi.SX.sym('omega')
controls = casadi.vertcat(v,omega)
# control symbolic variable
n_controls = controls.numel()


rhs = casadi.vertcat(v*cos(theta), v*sin(theta), omega)
f = casadi.Function('f', [states, controls], [rhs])
X = casadi.SX.sym('X', n_states, N + 1)

# matrix containing all control actions over all time steps (each column is an action vector)
U = casadi.SX.sym('U', n_controls, N)

# coloumn vector for storing initial state and target state
P = casadi.SX.sym('P', n_states + n_states)
g = X[:, 0] - P[:n_states]  # constraints in the equation
cost_fn = 0  # cost function
# print (P)
# print (Q)
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    cost_fn = cost_fn \
        + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) \
        # + con.T @ R @ con
    # print ("cost_fn: ",cost_fn)
    st_next = X[:, k+1]
    k1 = f(st, con)
    k2 = f(st + step_horizon/2*k1, con)
    k3 = f(st + step_horizon/2*k2, con)
    k4 = f(st + step_horizon * k3, con)
    st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    g = casadi.vertcat(g, st_next - st_next_RK4)


OPT_variables = casadi.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)
nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 2000,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

# solver = casadi.nlpsol('solver', 'ipopt', nlp_prob, opts)
solver = casadi.nlpsol('solver', 'fatrop', nlp_prob, {"structure_detection": "auto", "debug": True, "equality": [True for _ in range(N * x.numel())]})
lbx = casadi.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = casadi.DM.zeros((n_states*(N+1) + n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = -casadi.inf     # X lower bound
lbx[1: n_states*(N+1): n_states] = -casadi.inf     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -casadi.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = casadi.inf      # X upper bound
ubx[1: n_states*(N+1): n_states] = casadi.inf      # Y upper bound
ubx[2: n_states*(N+1): n_states] = casadi.inf      # theta upper bound

lbx[n_states*(N+1)::n_controls] = v_min                  # v lower bound for all V
ubx[n_states*(N+1)::n_controls] = v_max                  # v upper bound for all V
lbx[n_states*(N+1)+1::n_controls] = -pi/4                  # v lower bound for all V
ubx[n_states*(N+1)+1::n_controls] = pi/4

args = {
    'lbg': casadi.DM.zeros((n_states*(N+1), 1)),  # constraints lower bound
    'ubg': casadi.DM.zeros((n_states*(N+1), 1)),  # constraints upper bound
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0
state_init = casadi.DM([x_init, y_init, theta_init])        # initial state
state_target = casadi.DM([x_target, y_target, theta_target])  # target state

# xx = casadi.DM(state_init)
t = casadi.DM(t0)

u0 = casadi.DM.zeros((n_controls, N))  # initial control
X0 = casadi.repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states= DM2Arr(X0)
cat_controls= DM2Arr(u0[:, 0])
times = np.array([[0]])
###############################################################################
print (state_init)

if __name__ == '__main__':
    main_loop = time()  
    rclpy.init(args=None)

    # qos_profile = QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
    

    node = rclpy.create_node('Node')

    # subscriber = node.create_subscription(
    #     Odometry,
    #     '/drone1/odom',
    #     odom_callback,
    #     qos_profile # QoS profile, similar to publisher
    #     )
    publisher = node.create_publisher(Twist, '/drone1/cmd_vel', 10)


    while rclpy.ok():
        try:
            while (casadi.norm_2(state_init - state_target) > 5e-2) and (mpc_iter * step_horizon < sim_time):
                # print ("In While")
                t1 = time()
                args['p'] = casadi.vertcat(
                    state_init,    # current state
                    state_target   # target state
                )
                # optimization variable current state
                args['x0'] = casadi.vertcat(
                    casadi.reshape(X0, n_states*(N+1), 1),
                    casadi.reshape(u0, n_controls*N, 1)
                )

                sol = solver(
                    x0=args['x0'],
                    lbx=args['lbx'],
                    ubx=args['ubx'],
                    lbg=args['lbg'],
                    ubg=args['ubg'],
                    p=args['p']
                )

                u = casadi.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
                X0 = casadi.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)
                # print ("controls: ",u[:,0])
                # print (X0)
                cat_states = np.dstack((
                    cat_states,
                    DM2Arr(X0)
                ))

                cat_controls = np.vstack((
                    cat_controls,
                    DM2Arr(u[:, 0])
                ))
                t = np.vstack((
                    t,
                    t0
                ))

                t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)

                # print(X0)
                X0 = casadi.horzcat(
                    X0[:, 1:],
                    casadi.reshape(X0[:, -1], -1, 1)
                )

                # xx ...
                # print (u[:,0])
                linear_vel, angular_vel = np.array(u[:,0])
                # print (linear_vel, angular_vel)
                twist = Twist()
                twist.linear.x = float(linear_vel)/5.5
                twist.angular.z = float(angular_vel)/2
                publisher.publish(twist)
                node.get_logger().info(f'Published Twist message: linear={linear_vel}, angular={angular_vel}')
                t2 = time()
                print(mpc_iter)
                print(t2-t1)
                print ("States", state_init)
                # print ("Controls",u0)
                times = np.vstack((
                    times,
                    t2-t1
                ))

                mpc_iter = mpc_iter + 1

            main_loop_time = time()
            ss_error = casadi.norm_2(state_init - state_target)

            print('\n\n')
            print('Total time: ', main_loop_time - main_loop)
            print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
            print('final error: ', ss_error)
            twist = Twist()
            publisher.publish(twist)
            break
            # simulate(cat_states, cat_controls, times, step_horizon, N,
            #          np.array([x_init, y_init, theta_init, x_target, y_target, theta_target]), save=False)

        except ValueError:
            node.get_logger().warn('Invalid input. Please enter two float values separated by space (e.g., 0.5 0.2).')
        except KeyboardInterrupt:
            break

    node.destroy_node()
    rclpy.shutdown()