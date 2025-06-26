import numpy as np
import cvxpy as cp
import matplotlib.pyplot as plt

# System dynamics: x[k+1] = Ax[k] + Bu[k]
A = np.array([[1.0]])
B = np.array([[1.0]])
x0 = np.array([[5.0]])  # Initial state

# MPC parameters
N = 10  # Horizon length
Q = np.eye(1) * 1.0
R = np.eye(1) * 0.1

x_hist = [x0.flatten()[0]]
x = x0.copy()

for t in range(20):
    # Define optimization variables
    u = cp.Variable((1, N))
    x_var = cp.Variable((1, N+1))

    # Objective function
    cost = 0
    constraints = [x_var[:, 0] == x.flatten()]
    for k in range(N):
        cost += cp.quad_form(x_var[:, k], Q) + cp.quad_form(u[:, k], R)
        constraints += [x_var[:, k+1] == A @ x_var[:, k] + B @ u[:, k],
                        cp.abs(u[:, k]) <= 1.0]

    # Solve MPC problem
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve()

    # Apply first control input
    u_opt = u[:, 0].value
    x = A @ x + B @ u_opt
    x_hist.append(x.flatten()[0])

# Plot result
plt.plot(x_hist, marker='o')
plt.title("MPC State Trajectory")
plt.xlabel("Time step")
plt.ylabel("State x")
plt.grid(True)
plt.savefig("example_plot.png")
plt.show()
