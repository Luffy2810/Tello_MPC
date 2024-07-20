import os
import sys
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver,AcadosModel
import casadi as ca
from geometry_msgs.msg import Twist
import time
def safe_mkdir_recursive(directory, overwrite=False):
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError as exc:
            if exc.errno == errno.EEXIST and os.path.isdir(directory):
                pass
            else:
                raise
    else:
        if overwrite:
            try:
                shutil.rmtree(directory)
            except:
                print('Error while removing directory {}'.format(directory))

class MPCController:
    def __init__(self, init_state, final_state):
        t1= time.time()
        self.M = [[10,0,4.5],[20,0,4.5],[30,0,4.5],[40,0,4.5]]
        self.state_init = np.array(init_state)
        self.state_target = np.array(final_state)
        self.min_human_distance = 4
        self.Q, self.R, self.step_horizon, self.N = self.initialize_variables()
        self.acados_models_dir = './acados_models'
        safe_mkdir_recursive(os.path.join(os.getcwd(), self.acados_models_dir))
        self.acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, self.acados_source_path)        
        self.model = self.create_model()
        self.ocp = self.setup_ocp()
        self.solver = self.setup_solver()
        print ("init_time: ",1/(time.time()-t1))

    def initialize_variables(self):
        Q = np.diag([1, 1, 0.2])
        R = np.diag([0.5, 0.05])
        step_horizon = 0.02
        N = 40
        return Q, R, step_horizon, N

    # def create_model(self):
    #     model = AcadosModel()
    #     model.name = 'robot_model'

    #     # States
    #     x = ca.SX.sym('x')
    #     y = ca.SX.sym('y')
    #     theta = ca.SX.sym('theta')
    #     states = ca.vertcat(x, y, theta)
    #     model.x = states

    #     # Controls
    #     v = ca.SX.sym('v')
    #     omega = ca.SX.sym('omega')
    #     controls = ca.vertcat(v, omega)
    #     model.u = controls

    #     rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]
    #     x_dot = ca.SX.sym('x_dot', len(rhs))
    #     model.xdot = x_dot
    #     f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])

    #     f_impl = x_dot - f(states, controls)
    #     model.f_expl_expr = f(states, controls)
    #     model.f_impl_expr = f_impl

    #     # CBF constraints for humans
    #     # h = []
    #     # for human in self.M:
    #     #     h_i = (x - human[0])**2 + (y - human[1])**2 - (0.2 + 1.0)**2
    #     #     h.append(h_i)
    #     # model.con_h_expr = ca.vertcat(*h)

    #     return model
    def create_model(self):
        model = AcadosModel()
        model.name = 'robot_model'

        # States
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        model.x = states

        # Controls
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        model.u = controls

        # Dynamics
        rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]
        x_dot = ca.SX.sym('x_dot', len(rhs))
        model.xdot = x_dot
        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])

        f_impl = x_dot - f(states, controls)
        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        # model.p = ca.SX.sym('pos',5)

        num_objects = 4  # Adjust this based on how many objects you want to track
        model.p = ca.SX.sym('p',3+ 3 * num_objects)
        h = []
        rho = 0.3
        ds = 1.2
        gamma = 0.3
        
        for i in range(1,num_objects+1):
            object_x = model.p[3*i]
            object_y = model.p[3*i + 1]
            object_theta = model.p[3*i + 2]
            h_k = (x - object_x)**2 + (y - object_y)**2 - (rho + ds)**2
            h_next = (x + v*ca.cos(theta)*self.step_horizon - object_x)**2 + \
                     (y + v*ca.sin(theta)*self.step_horizon - object_y)**2 - (rho + ds)**2
            h_i = h_next - h_k + gamma * h_k
            h.append(h_i)
        model.con_h_expr = ca.vertcat(*h)
        # # External cost
        # cost = ca.mtimes([(states - self.state_target).T, self.Q, (states - self.state_target)]) + ca.mtimes([controls.T, self.R, controls])
        # model.cost_expr_ext_cost = cost
        # model.cost_expr_ext_cost_e = ca.mtimes([(states - self.state_target).T, self.Q, (states - self.state_target)])

        return model

    # def setup_ocp(self):
    #     ocp = AcadosOcp()
    #     ocp.acados_include_path = self.acados_source_path + '/include'
    #     ocp.acados_lib_path = self.acados_source_path + '/lib'
    #     # self.model, self.constraints = self.make_model()
    #     ocp.model = self.model
    #     ocp.dims.N = self.N
    #     ocp.solver_options.tf = self.N * self.step_horizon

    #     # Cost
    #     x = ocp.model.x
    #     u = ocp.model.u
    #     print ("Printing x: ",x)
    #     print ("Printing u: ",u)
    #     print ("Printing y_ref: ",ocp.cost.yref)
    #     # ocp.model.cost_expr_ext_cost = (st - P[self.n_states:]).T @ self.Q @ (st - P[self.n_states:])
    #     # ocp.model.cost_expr_ext_cost_e = (st - P[self.n_states:]).T @ self.Q @ (st - P[self.n_states:])
    #     # # https://docs.acados.org/python_interface/index.html#acados_template.acados_ocp.AcadosOcpCost
    #     # ocp.cost.cost_type = 'EXTERNAL'
    #     # ocp.cost.cost_type_e = 'EXTERNAL'

    #     ocp.cost.W = scipy.linalg.block_diag(self.Q, self.R)
    #     ocp.cost.W_e = self.Q

    #     nx = ocp.model.x.size()[0]
    #     nu = ocp.model.u.size()[0]

    #     ocp.cost.Vx = np.zeros((nx + nu, nx))
    #     ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    #     ocp.cost.Vu = np.zeros((nx + nu, nu))
    #     ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)

    #     ocp.cost.Vx_e = np.eye(nx)

        # ocp.cost.yref = np.zeros(nx + nu)
    #     ocp.cost.yref_e = np.zeros(nx)

    #     # Constraints
    #     ocp.constraints.lbu = np.array([-4, -np.pi*3.14/8])
    #     ocp.constraints.ubu = np.array([4, np.pi*3.14/8])
    #     ocp.constraints.idxbu = np.array([0, 1])

    #     ocp.constraints.x0 = self.state_init

    #     # CBF constraints
    #     # ocp.constraints.lh = np.zeros(len(self.M))
    #     # ocp.constraints.uh = 1000 * np.ones(len(self.M))

    #     # Solver options
    #     ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    #     ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    #     ocp.solver_options.integrator_type = 'ERK'
    #     ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    #     return ocp

    def setup_ocp(self):
        ocp = AcadosOcp()
        ocp.acados_include_path = self.acados_source_path + '/include'
        ocp.acados_lib_path = self.acados_source_path + '/lib'
        ocp.model = self.model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.N * self.step_horizon
        n_params = self.model.p.shape[0]
        ocp.dims.np = n_params
        ocp.parameter_values = np.zeros(n_params)
        # External cost
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        # Quadratic state and control cost
        obs_cost = 0
        for i in range (1,5):
            object_x = self.model.p[3*i]
            object_y = self.model.p[3*i + 1]
            object_theta = self.model.p[3*i + 2]
            obs_cost += 1/(0.1+10*((self.model.x[0]-object_x)**2+(self.model.x[1]-object_y)**2))
        ocp.model.cost_expr_ext_cost = (self.model.x-self.model.p[0:3]).T @ self.Q @ (self.model.x-self.model.p[0:3]) + self.model.u.T @ self.R @ self.model.u + obs_cost

        ocp.model.cost_expr_ext_cost_e = (self.model.x-self.model.p[0:3]).T @ self.Q @ (self.model.x-self.model.p[0:3]) + obs_cost

        # ocp.model.cost_expr_ext_cost = ca.norm_2(self.model.x-self.model.p[0:3])
        # ocp.model.cost_expr_ext_cost_e = ca.norm_2(self.model.x-self.model.p[0:3])

        # Constraints
        ocp.constraints.lbu = np.array([-4, -np.pi*3.14/8])
        ocp.constraints.ubu = np.array([4, np.pi*3.14/8])
        ocp.constraints.idxbu = np.array([0, 1])


        ocp.constraints.x0 = self.state_init


        # CBF constraints
        ocp.constraints.lh = -0.1 * np.ones(len(self.M))
        ocp.constraints.uh = 1000 * np.ones(len(self.M))

                # Solver options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'

        return ocp


    def setup_solver(self):
        json_file = 'acados_ocp_' + self.model.name + '.json'
        solver = AcadosOcpSolver(self.ocp, json_file=json_file)
        return solver

    def run(self):
        # Set initial state and reference
        self.solver.set(0, 'lbx', self.state_init)
        self.solver.set(0, 'ubx', self.state_init)
        object_params = []

        for obj in self.M:
            for coord in obj:
                object_params.append(coord)

        for i in range(self.N+1):
            self.solver.set(i, "x", self.state_init)
            self.solver.set(i, 'p', np.concatenate([self.state_target,np.array(object_params)]))
        # self.solver.set(self.N, 'yref', self.state_target)

        # Solve OCP
        status = self.solver.solve()

        if status != 0:
            print(f"acados returned status {status}")

        # Get optimal control input
        u0 = self.solver.get(0, 'u')
        # Convert to ROS Twist message (you'll need to implement this part)
        twist = self.create_twist_message(u0)
        
        return twist

    def create_twist_message(self, u):
        twist = Twist()
        twist.linear.x = float(u[0]) / 8
        twist.angular.z = float(u[1]) / 3.14
        return twist

if __name__ == '__main__':
    init_state = [0, 0, 0]
    final_state = [2, 2, 0]
    mpc = MPCController(init_state, final_state)
    twist = mpc.run()
    print (twist)
    # Publish twist to your robot (you'll need to implement this part)