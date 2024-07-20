import numpy as np
import casadi as ca
from acados_template import AcadosModel


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

class MobileRobotModel(object):
    def __init__(self,):
        self.model, self.constraint = make_model()
        self.T = 1.0
        self.N = 5
        os.chdir(os.path.dirname(os.path.realpath(__file__)))
        self.acados_models_dir = './acados_models'
        safe_mkdir_recursive(os.path.join(os.getcwd(), self.acados_models_dir))
        acados_source_path = os.environ['ACADOS_SOURCE_DIR']
        sys.path.insert(0, acados_source_path)
        nx = model.x.size()[0]
        self.nx = nx
        nu = model.u.size()[0]
        self.nu = nu
        ny = nx + nu
        n_params = len(model.p)
        ocp = AcadosOcp()
        ocp.acados_include_path = acados_source_path + '/include'
        ocp.acados_lib_path = acados_source_path + '/lib'
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.T



    def make_model(self):
        model = AcadosModel()
        constraint = ca.types.SimpleNamespace()
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        rhs = [v*ca.cos(theta), v*ca.sin(theta), omega]
        f = ca.Function('f', [states, controls], [ca.vcat(rhs)], ['state', 'control_input'], ['rhs'])
        x_dot = ca.SX.sym('x_dot', len(rhs))
        f_impl = x_dot - f(states, controls)
        model.f_expl_expr = f(states, controls)
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = x_dot
        model.u = controls
        model.p = []
        model.name = 'Drone'
        constraint.v_max = 4
        constraint.v_min = -4
        constraint.omega_max = np.pi*3.14/8.0
        constraint.omega_min = -np.pi*3.14/8.0
        constraint.x_min = -ca.inf
        constraint.x_max = ca.inf
        constraint.y_min = -ca.inf
        constraint.y_max = ca.inf
        constraint.expr = ca.vcat([v, omega])
        model = model
        constraint = constraint
        return model, constraint