import casadi as ca
from casadi import jacobian

# Create a symbolic variable with two states
x = ca.MX.sym('x', 2)

# Expression for ODE right-hand side
z= 1-x[1]**2
rhs = ca.vertcat(z*x[0]-x[1], x[0])

ode = {} # dictionary to store the ODE
ode['x'] = x # state variables
ode['ode'] = rhs # right-hand side

# Construct a Function that integrates the ODE over 4 seconds
F = ca.integrator('F', 'cvodes', ode, 0,4)

# start from x=[0;1]
res = F(x0=[0,1])

# Print the result
print(res['xf'])

# Print the Jacobian of the ODE
res = F(x0=x)
S = ca.Function('S',[x],[jacobian(res["xf"],x)])
print(S([0,1]))