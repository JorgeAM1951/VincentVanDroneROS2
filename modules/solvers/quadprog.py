import numpy as np
from scipy import optimize
# e.g. least-squares problem
#print("    min. || G * x - a ||^2")
#print("    s.t. C * x <= b")
#print('\n')  

def solve_qp_scipy(G, a, C, b, meq):
    def f(x):
        return 0.5 * np.dot(x, G).dot(x) - np.dot(a, x)

    constraints = []
    if C is not None:
        constraints = [{
            'type': 'eq' if i < meq else 'ineq',
            'fun': lambda x, C=C, b=b, i=i: (np.dot(C.T, x) - b)[i]
        } for i in range(C.shape[1])]

    result = optimize.minimize(
        f, x0=np.zeros(len(G)), method='COBYQA', constraints=constraints,
        tol=1e-10, options={'maxiter': 2000})
    return result

# data: 
M = np.array([[1.0, 2.0, 0.0], [-8.0, 3.0, 2.0], [0.0, 1.0, 1.0]])
G = np.dot(M.T, M)  # this is a positive definite matrix
a = np.dot(np.array([3.0, 2.0, 3.0]), M)   # @ M
C = np.array([[1.0, 2.0, 1.0], [2.0, 0.0, 1.0], [-1.0, 2.0, -1.0]])
b = np.array([3.0, 2.0, -2.0])     #.reshape((3,))
meq = b.size

# for dense problem from https://github.com/qpsolvers/qpsolvers/blob/main/examples/quadratic_program.py
print("    min. 1/2 x^T P x + a^T x")
print("    s.t. G * x <= b")


# SciPy
result = solve_qp_scipy(G, a, C, b, meq)
print(result.x, '\n', result.fun, '\n')