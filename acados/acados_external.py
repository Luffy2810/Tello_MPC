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
        print ("*"*10)
        print ("state_init: ",self.state_init)
        print ("*"*10)
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
        R = np.diag([0.5,0.7, 0.05])
        step_horizon = 0.02
        N = 40
        return Q, R, step_horizon, N


    def convert_3D_2D(self,point,fx_d=922,fy_d=922,cx_d=480,cy_d=360):
        z,x,y = point 
        x=x*-1
        y=y*-1
        x_2d = x*fx_d/z + cx_d
        y_2d = y*fy_d/z + cy_d
        point_2d = np.array([x_2d,y_2d])
        return point_2d

    def convert_2D_3D(self,point,depth,fx_d=922,fy_d=922,cx_d=480,cy_d=360):
        x_d,y_d = point 
        x_3d = (x_d - cx_d) * depth / fx_d
        y_3d = (y_d - cy_d) * depth / fy_d
        z_3d = depth
        point_3d = np.array([z_3d,-x_3d,-y_3d])
        return point_3d

    def create_model(self):
        model = AcadosModel()
        model.name = 'robot_model'

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        z = ca.SX.sym('z')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x,y,z,theta)
        model.x = states
        v = ca.SX.sym('v')
        w = ca.SX.sym('w')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v,w,omega)
        model.u = controls

        # Dynamics
        rhs = [v*ca.cos(theta), v*ca.sin(theta),w, omega]
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
        # x_c,y_c = self.convert_3D_2D([x - v*ca.cos(theta)*self.step_horizon,y - v*ca.sin(theta)*self.step_horizon,z-w*self.step_horizon])
        # h.append(x_c-20)
        # h.append(y_c-20)
        # h.append(940-x_c)
        # h.append(720-y_c)
        # for i in range(1,num_objects+1):
        #     object_x = model.p[3*i]
        #     object_y = model.p[3*i + 1]
        #     object_theta = model.p[3*i + 2]
        #     h_k = (x - object_x)**2 + (y - object_y)**2 - (rho + ds)**2
        #     h_next = (x + v*ca.cos(theta)*self.step_horizon - object_x)**2 + \
        #              (y + v*ca.sin(theta)*self.step_horizon - object_y)**2 - (rho + ds)**2
        #     h_i = h_next - h_k + gamma * h_k
        #     h.append(h_i)
        # model.con_h_expr = ca.vertcat(*h)

        return model

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
        x,y,z = self.convert_2D_3D([self.model.p[0],self.model.p[1]],self.model.p[2])
        x_c,y_c = self.convert_3D_2D([self.model.x[0],self.model.x[1],self.model.x[2]])
        obs_cost = 0
        cx = 480
        cy = 360
        for i in range (1,5):
            object_x = self.model.p[3*i]
            object_y = self.model.p[3*i + 1]
            object_theta = self.model.p[3*i + 2]
            obs_cost += 1/(0.1+10*((self.model.x[0]-object_x)**2+(self.model.x[0]-object_x)**2))
        fx_d=922
        fy_d=922
        cx_d=480
        cy_d=360
        x_c = -(x-self.model.x[1])*fx_d/(z-self.model.x[0]) + cx_d
        y_c = -(y-self.model.x[2])*fy_d/(z-self.model.x[0]) + cy_d

        cost = (x_c - cx)/cx + (y_c-cy)/cy + self.model.u.T @ self.R @ self.model.u + obs_cost



        ocp.model.cost_expr_ext_cost = (x_c - cx)/cx + (y_c-cy)/cy + self.model.u.T @ self.R @ self.model.u

        ocp.model.cost_expr_ext_cost_e = (x_c - cx)/cx + (y_c-cy)/cy 


        # ocp.model.cost_expr_ext_cost = (self.model.x[0:3]-np.array([x-2,y-2,5])).T @ self.Q @ (self.model.x[0:3]-np.array([x-2,y-2,5])) + self.model.u.T @ self.R @ self.model.u 

        # ocp.model.cost_expr_ext_cost_e = (self.model.x[0:3]-np.array([x-2,y-2,5])).T @ self.Q @ (self.model.x[0:3]-np.array([x-2,y-2,5]))  
        ocp.constraints.lbu = np.array([-4,-1, -np.pi*3.14/8])
        ocp.constraints.ubu = np.array([4,1, np.pi*3.14/8])
        ocp.constraints.idxbu = np.array([0, 1,2])

        ocp.constraints.x0 = self.state_init


        # CBF constraints
        # ocp.constraints.lh = -0.1 * np.ones(4+len(self.M))
        # ocp.constraints.uh = 1000 * np.ones(4+len(self.M))

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
        twist.linear.z = float(u[1]) / 8
        twist.angular.z = float(u[2]) / 3.14
        return twist

if __name__ == '__main__':
    init_state = [0, 0, 0,0]
    final_state = [2, 2, 0]
    mpc = MPCController(init_state, final_state)
    twist = mpc.run()
    print (twist)
    # Publish twist to your robot (you'll need to implement this part)