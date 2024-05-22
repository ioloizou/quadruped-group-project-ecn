from casadi import *
import numpy as np
import matplotlib.pyplot as plt

T = 0.2 #sampling time
N = 3 #prediction horizon
robot_diameter = 0.3 

v_max = 0.5 
v_min = -v_max 
omega_max = np.pi/4
omega_min = -np.pi/4

# Define the state and control variables
x = SX.sym('x') 
y = SX.sym('y') 

theta = SX.sym('theta')
state = vertcat(x, y, theta)
n_state = state.size1()     #size1 = number of rows

v = SX.sym('v') 
omega = SX.sym('omega')
controls = vertcat(v, omega) 
n_controls = controls.size1() 


# Define the kinematic model (and put in a function that will map the state and control inputs to the derivative of the state)
robot_kmodel = vertcat(v*cos(theta), v*sin(theta), omega)
f = Function('f', [state, controls], [robot_kmodel]) 


# U = decision variables (controls for each step in the prediction horizon)
# P = parameters (initial state and reference state)
# X = prediction variables (state predictions from time 0 to N)
U = SX.sym('U', n_controls, N)
P = SX.sym('P', n_state + n_state)
X = SX.sym('X', n_state, (N+1))

# Fill up the prediction vector (initial state then from the kinematic model)
# Then wrap this process in a function
X[:,0] = P[0:3]
for k in range(N):
    X[:,k+1] = f(X[:,k], U[:,k])
    #X_1 = f(X_0, U_0)
    #X_2 = f(X_1, U_1)
    #X_3 = f(X_2, U_2) 

ff = Function('ff', [U, P], [X])


# Define the objective function
obj = 0
Q_diag = [1, 5, 0.1]
R_diag = [0.5, 0.05]
Q = diag(Q_diag)     
R = diag(R_diag)

for k in range(N):
    # obj = sum[(X[k] - x_ref[k]).T*Q*(X[k] - x_ref[k]) + U_k.T*R*U_k] from k=0 to N-1
    obj = obj + (X[:,k] - P[3:6]).T @ Q @ (X[:,k] - P[3:6]) + U[:,k].T @ R @ U[:,k]


# Define the constraints
# Box constraints on x and y, to ensure the robot stays within the bounds of a map
# Current state is known, we define the bounds for the predicted states starting from time step k+1
g = []
for k in range(N):
    #g = [x_k+1, y_k+1; x_k+2, y_k+2; ...; x_k+N+1, y_k+N+1]
    g = vertcat(g, X[0,k+1])    # g1 = x
    g = vertcat(g, X[1,k+1])    # g2 = y

# Make decision variables one long vector (2N rows and 1 column)
# OPT_variables = [v0, omega0, v1, omega1, ..., vN, omegaN]
OPT_variables = reshape(U, 2*N, 1)

# Bounds:
# Remember the structure of the OPT_variables: [v0, omega0, v1, omega1, ..., vN, omegaN]
# Odd indices correspond to v, even indices correspond to omega
args = {
    #do not confuse, here: lbx and ubx are the bounds for the decision variables (controls)
    #Based on how we defined OPT_variables, the even indexes of the decision variables are the linear velocity v
    #and the odd indexes are the angular velocity omega
    'lbx': [v_min if i % 2 == 0 else omega_min for i in range(2*N)], 
    'ubx': [v_max if i % 2 == 0 else omega_max for i in range(2*N)], 
    # -2 and 2 for the constraints on the state variables (map bounds defined above)
    'lbg': -2, 
    'ubg': 2
    }

# Create the Nonlinear Programming Problem
nlp_prob = {'f': obj, 'x': OPT_variables, 'g': g, 'p': P}

#solver options and instantiation of solver
opts = {
    'ipopt.max_iter': 100,
    'ipopt.print_level': 0,
    'print_time': 0,
    'ipopt.acceptable_tol': 1e-8,
    'ipopt.acceptable_obj_change_tol': 1e-6
    }
solver = nlpsol('solver', 'ipopt', nlp_prob, opts)


# Simulation loop (fix it)
t0 = 0
tf = 600

x_0 = np.array([0, 0, 0])
u_0 = np.zeros((n_controls, N))

x_ref = np.array([1.5, 1.5, 0])

x_hist = np.zeros((n_state, tf+1))
u_hist = np.zeros((n_controls, tf))

x_hist[:,0] = x_0
u_hist[:,0] = u_0[:,0]

mpc_iter = 0
xx1 = []
u_cl = []

while np.linalg.norm((x_0 - x_ref)[0:2]) > 1e-2 and mpc_iter < tf / T - 1:
    
    # Solve the NLP (call the solver function)
    p = np.concatenate([x_0, x_ref])    #initial state and reference state
    sol = solver(x0=u_0.flatten(), p=p.tolist(), lbg=args['lbg'], ubg=args['ubg'], lbx=args['lbx'], ubx=args['ubx'])

    # Get minimizer:
    u_star = reshape(sol['x'].full(), n_controls, N) #reshape the minimizer back to matrix form (2xN) - .full() gets data as np array

    # Calculate the state predictions using the minimizer control inputs and the current state
    ff_value = ff(u_star, p)

    # Update the state predicted trajectory and control input trajectory
    xx1.append(ff_value.full()[:, :3])
    u_cl.append(u_star[0, :])

    # Update simulation time, and shift the state and control inputs one step forward
    t0 = t0 + T
    x_0 = ff_value.full()[:, 1]
    u_0 = np.hstack((u[:, 1:], np.zeros((n_controls, 1))))

    # Store the results
    if mpc_iter < tf:
        x_hist[:, mpc_iter + 1] = x_0
        u_hist[:, mpc_iter] = u[:, 0]

    mpc_iter += 1

# Calculate performance metrics
ss_error = np.linalg.norm(x_0 - x_ref, 2)
average_mpc_time = tf / (mpc_iter + 1)

print("Steady-state error:", ss_error)
print("Average MPC time:", average_mpc_time)

# Plot the control inputs
time = np.arange(0, mpc_iter * T, T)[:u_hist.shape[1]]

# Plot the robot's trajectory
plt.figure()
plt.plot(x_hist[0, :], x_hist[1, :], label='Robot trajectory')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend()
plt.grid()

# Plot the control inputs
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(time, u_hist[0, :mpc_iter], label='v')
plt.xlabel('Time [s]')
plt.ylabel('Linear Velocity [m/s]')
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(time, u_hist[1, :mpc_iter], label='omega')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.legend()
plt.grid()

plt.show()
