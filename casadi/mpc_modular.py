import casadi
import numpy as np
from time import time
from casadi import sin, cos, pi
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
 
class MPCController:
    def __init__(self,init_state,final_state):
        time_init = time()
        self.M = [[10,0,4.5],[20,0,4.5],[30,0,4.5],[40,0,4.5]]
        self.min_human_distance = 4
        self.Q, self.R, self.step_horizon, self.N = self.initialize_variables()
        self.states, self.n_states, self.controls, self.n_controls, self.f = self.setup_casadi_functions()
        self.solver = self.setup_solver()        
        self.state_init = casadi.DM(init_state)
        self.state_target = casadi.DM(final_state)
        self.X0 = casadi.repmat(self.state_init, 1, self.N+1)
        self.u0 = casadi.DM.zeros((self.n_controls, self.N))
        
        print ("time to init: ",1/(time()-time_init))

    def cbf(self, robot_state, human_position):
        rho = 0.2
        ds = 1.0   
        return (robot_state[0] - human_position[0])**2 + (robot_state[1] - human_position[1])**2 - (rho + ds)**2


    def initialize_variables(self):
        Q = casadi.diagcat(1, 1, 0.2)
        R = casadi.diagcat(0.5, 0.05)
        step_horizon = 0.1
        N = 7
        return Q, R, step_horizon, N
    
    def setup_casadi_functions(self):
        x = casadi.SX.sym('x')
        y = casadi.SX.sym('y')
        theta = casadi.SX.sym('theta')
        states = casadi.vertcat(x, y, theta)
        n_states = states.numel()
        v = casadi.SX.sym('v')
        omega = casadi.SX.sym('omega')
        controls = casadi.vertcat(v, omega)
        n_controls = controls.numel()
        rhs = casadi.vertcat(v*cos(theta), v*sin(theta), omega)
        f = casadi.Function('f', [states, controls], [rhs])
        return states, n_states, controls, n_controls, f
    
    def setup_solver(self):
        nlp_prob = self.setup_optimization()
        opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        solver = casadi.nlpsol('solver', 'ipopt', nlp_prob, opts)
        return solver
    
    def setup_optimization(self):
        gamma= 0.3
        X = casadi.SX.sym('X', self.n_states, self.N + 1)
        U = casadi.SX.sym('U', self.n_controls, self.N)
        P = casadi.SX.sym('P', self.n_states + self.n_states)
        g = X[:, 0] - P[:self.n_states]
        
        cost_fn = 0
        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            cost_fn += (st - P[self.n_states:]).T @ self.Q @ (st - P[self.n_states:])
            st_next = X[:, k+1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.step_horizon/2*k1, con)
            k3 = self.f(st + self.step_horizon/2*k2, con)
            k4 = self.f(st + self.step_horizon * k3, con)
            st_next_RK4 = st + (self.step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            g = casadi.vertcat(g, st_next - st_next_RK4)
            d_human = 0
            for j in range(len(self.M)):
                human_pos = self.M[j]
                d_human += casadi.fmax(0, self.min_human_distance-((st[0] - human_pos[0])**2 + (st[1] - human_pos[1])**2))
                h_k = self.cbf(st, human_pos)
                h_next = self.cbf(st_next, human_pos)
                g = casadi.vertcat(g, h_next - h_k + gamma * h_k)
            cost_fn += d_human
        OPT_variables = casadi.vertcat(
            X.reshape((-1, 1)),
            U.reshape((-1, 1))
        )
        nlp_prob = {
            'f': cost_fn,
            'x': OPT_variables,
            'g': g,
            'p': P
        }
        
        return nlp_prob
    
    def shift_timestep(self, t0, state_init, u):
        f_value = self.f(state_init, u[:, 0])
        next_state = casadi.DM.full(state_init + (self.step_horizon * f_value))
        t0 += self.step_horizon
        u0 = casadi.horzcat(
            u[:, 1:],
            casadi.reshape(u[:, -1], -1, 1)
        )
        return t0, next_state, u0
    
    def run(self):
    
        args = {
            'p': casadi.vertcat(self.state_init, self.state_target),
            'x0': casadi.vertcat(casadi.reshape(self.X0, self.n_states*(self.N+1), 1),
                                 casadi.reshape(self.u0, self.n_controls*self.N, 1)),
            'lbx': self.setup_lbx(),
            'ubx': self.setup_ubx(),
            'lbg': self.setup_lbg(),
            'ubg': self.setup_ubg(),
        }
        sol = self.solver(**args)
        u = casadi.reshape(sol['x'][self.n_states*(self.N+1):], self.n_controls, self.N)
        linear_vel, angular_vel = np.array(u[:, 0])
        twist = Twist()
        twist.linear.x = float(linear_vel) / 8
        twist.angular.z = float(angular_vel) / 3.14
        return twist


    
    def setup_lbx(self):
        lbx = casadi.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        print (lbx.shape)
        lbx[0: self.n_states*(self.N+1): self.n_states] = -casadi.inf
        lbx[1: self.n_states*(self.N+1): self.n_states] = -casadi.inf
        lbx[2: self.n_states*(self.N+1): self.n_states] = -pi
        lbx[self.n_states*(self.N+1)::self.n_controls] = -4
        lbx[self.n_states*(self.N+1)+1::self.n_controls] = -pi*3.14/8
        return lbx
    
    def setup_ubx(self):
        ubx = casadi.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        ubx[0: self.n_states*(self.N+1): self.n_states] = casadi.inf
        ubx[1: self.n_states*(self.N+1): self.n_states] = casadi.inf
        ubx[2: self.n_states*(self.N+1): self.n_states] = pi
        ubx[self.n_states*(self.N+1)::self.n_controls] = 4
        ubx[self.n_states*(self.N+1)+1::self.n_controls] = pi*3.14/8
        return ubx

    def setup_lbg(self):
        lbg = casadi.DM.zeros((self.n_states*(self.N+1) + len(self.M)*self.N, 1))
        return lbg

    def setup_ubg(self):
        ubg = casadi.DM.zeros((self.n_states*(self.N+1) + len(self.M)*self.N, 1))
        for j in range (len(self.M)):
            ubg[self.n_states*2+j::self.n_states*2+1] = casadi.inf  
        return ubg
    def odom_callback(self, msg):
        self.node.get_logger().info(f"Received Odometry message: {msg.pose.pose}")

if __name__ == '__main__':
    mpc = MPCController()
    mpc.run()
