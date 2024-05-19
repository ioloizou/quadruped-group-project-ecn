from casadi import *

# Symbols/Expressions
x = MX.sym('x')
y = MX.sym('y')
z = MX.sym('z')
f = x**2 + 100*z**2
g = vertcat(x + y - z - 5, y + 2 - x)  # equality and inequality constraints combined

nlp = {} # NLP declaration
nlp['x'] = vertcat(x, y, z) # decision variables
nlp['f'] = f # objective
nlp['g'] = g # equality constraints

# Create an NLP solver
solver = nlpsol('solver', 'ipopt', nlp)

# Solve the NLP
res = solver(x0=[2.5, 3.0, 0.75], lbg=[0, -inf], ubg=[0, 0], lbx=-inf, ubx=inf)

# Print the solution
print(res['x'])
