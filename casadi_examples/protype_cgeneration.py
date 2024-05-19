import casadi as ca

# Define the variables
x = ca.MX.sym('x')
u = ca.MX.sym('u')

# Define the dynamics equation
xdot = ca.sin(u)

# Define the cost function
L = x**2 + u**2

# Define the constraints
g = [x - 1]

# Create an optimal control problem
ocp = {'x': x, 'u': u, 'p': [], 'f': L, 'g': g, 'ode': xdot}

# Generate C code for the optimal control problem
opts = {'main': False, 'mex': False, 'with_header': True}
code = ca.generate_code(ocp, 'ocp', opts)

# Print the generated C code
print(code['c'])
print(code['h'])