import casadi
import numpy as np
from time import time
from casadi import sin, cos, pi
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile

class MPCController:
    def __init__(self):
        # Initialize parameters and variables
        self.Q, self.R, self.step_horizon, self.N = self.initialize_variables()
        self.states, self.n_states, self.controls, self.n_controls, self.f = self.setup_casadi_functions()
        self.solver = self.setup_solver()
        
        # Initialize other variables here
        self.state_init = casadi.DM([0, 0, 0])
        self.state_target = casadi.DM([20, 10, pi/4])
        self.X0 = casadi.repmat(self.state_init, 1, self.N+1)
        self.u0 = casadi.DM.zeros((self.n_controls, self.N))
        self.sim_time = 200
        
        self.publisher = None
        self.node = None
        self.subscriber = None
        
        # ROS 2 initialization
        rclpy.init(args=None)
        self.node = rclpy.create_node('MPC_Node')
        self.publisher = self.node.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.subscriber = self.node.create_subscription(
            Odometry,
            '/drone1/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)
        )
    
    def initialize_variables(self):
        Q = casadi.diagcat(1, 5, 0.1)
        R = casadi.diagcat(0.5, 0.05)
        step_horizon = 0.082
        N = 100
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
        main_loop_start = time()
        
        while rclpy.ok():
            try:
                mpc_iter = 0
                t0 = 0
                cat_states = DM2Arr(self.X0)
                cat_controls = DM2Arr(self.u0[:, 0])
                times = np.array([[0]])
                
                while (casadi.norm_2(self.state_init - self.state_target) > 5e-2) and (mpc_iter * self.step_horizon < self.sim_time):
                    t1 = time()
                    args = {
                        'p': casadi.vertcat(self.state_init, self.state_target),
                        'x0': casadi.vertcat(casadi.reshape(self.X0, self.n_states*(self.N+1), 1),
                                             casadi.reshape(self.u0, self.n_controls*self.N, 1)),
                        'lbx': self.setup_lbx(),
                        'ubx': self.setup_ubx(),
                        'lbg': casadi.DM.zeros((self.n_states*(self.N+1), 1)),
                        'ubg': casadi.DM.zeros((self.n_states*(self.N+1), 1))
                    }
                    
                    sol = self.solver(**args)
                    u = casadi.reshape(sol['x'][self.n_states*(self.N+1):], self.n_controls, self.N)
                    self.X0 = casadi.reshape(sol['x'][:self.n_states*(self.N+1)], self.n_states, self.N+1)
                    cat_states = np.dstack((cat_states, DM2Arr(self.X0)))
                    cat_controls = np.vstack((cat_controls, DM2Arr(u[:, 0])))
                    
                    t0, self.state_init, self.u0 = self.shift_timestep(t0, self.state_init, u)
                    self.X0 = casadi.horzcat(self.X0[:, 1:], casadi.reshape(self.X0[:, -1], -1, 1))
                    
                    linear_vel, angular_vel = np.array(u[:, 0])
                    twist = Twist()
                    twist.linear.x = float(linear_vel) / 5.5
                    twist.angular.z = float(angular_vel) / 2
                    self.publisher.publish(twist)
                    self.node.get_logger().info(f'Published Twist message: linear={linear_vel}, angular={angular_vel}')
                    
                    t2 = time()
                    times = np.vstack((times, t2 - t1))
                    mpc_iter += 1
                
                main_loop_time = time()
                ss_error = casadi.norm_2(self.state_init - self.state_target)
                
                print('\n\n')
                print('Total time: ', main_loop_time - main_loop_start)
                print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
                print('final error: ', ss_error)
                
                twist = Twist()
                self.publisher.publish(twist)
                break
            
            except ValueError:
                self.node.get_logger().warn('Invalid input. Please enter two float values separated by space (e.g., 0.5 0.2).')
            except KeyboardInterrupt:
                break
        
        self.node.destroy_node()
        rclpy.shutdown()
    
    def setup_lbx(self):
        lbx = casadi.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        lbx[0: self.n_states*(self.N+1): self.n_states] = -casadi.inf
        lbx[1: self.n_states*(self.N+1): self.n_states] = -casadi.inf
        lbx[2: self.n_states*(self.N+1): self.n_states] = -casadi.inf
        lbx[self.n_states*(self.N+1):-1:self.n_controls] = -1
        lbx[self.n_states*(self.N+1)+1:-1:self.n_controls] = -pi/4
        lbx[-1] = 0
        return lbx
    
    def setup_ubx(self):
        ubx = casadi.DM.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        ubx[0: self.n_states*(self.N+1): self.n_states] = casadi.inf
        ubx[1: self.n_states*(self.N+1): self.n_states] = casadi.inf
        ubx[2: self.n_states*(self.N+1): self.n_states] = pi
        ubx[self.n_states*(self.N+1):-1:self.n_controls] = 1
        ubx[self.n_states*(self.N+1)+1:-1:self.n_controls] = pi/4
        ubx = casadi.inf
        return ubx
    
    def odom_callback(self, msg):
        self.node.get_logger().info(f"Received Odometry message: {msg.pose.pose}")

if __name__ == '__main__':
    mpc = MPCController()
    mpc.run()
