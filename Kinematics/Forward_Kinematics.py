import numpy as np
import sympy as sym
from scipy.linalg import expm

M = [[0, -1, 0, .390],
     [0, 0, -1, .401],
     [1, 0, 0, .2155],
     [0, 0, 0, 1]]

w = [[0, 0, 1],     # w1
     [0, 1, 0],     # w2
     [0, 1, 0],     # w3
     [0, 1, 0],     # w4
     [1, 0, 0],     # w5
     [0, 1, 0]]     # w6

q = [[-.150, .150, .010],       # q1
     [-.150, .270, .162],       # q2
     [.094, .270, .162],        # q3
     [.307, .177, .162],        # q4
     [.307, .260, .162],        # q5
     [.390, .260, .162]]        # q6

"""
Helper function that builds the symbolic exponential matrix
"""
def buildSymExponential(w, q, theta):
    temp_w = np.transpose(w) * -1		# transpose for cross product
    q = np.transpose(q)		# transpose for cross product

    v = np.cross(temp_w, q)
    v = np.transpose(v)		# transpose to make temp matrix look cleaner
    temp = sym.Matrix(
            [[0, -w[2], w[1], v[0]],
            [w[2], 0, -w[0], v[1]],
            [-w[1], w[0], 0, v[2]],
            [0, 0, 0, 0]]) * theta

    e = temp.exp()  # create exponential matrix
    return e


"""
Calculates a symbolic solution to the forward kinematics equation. The
symbolic code starts with the M matrix, ω1 through ω6 and q1
through q6 and returns the symbolic T06 and the 6 matrices defined by
e^[S1]θ1 through e^[S6]θ6
"""
def symbolicSolution(M, w, q):
    # e^[S1]θ1
    theta1 = sym.Symbol('\u03F41')
    e1 = buildSymExponential(w[0], q[0], theta1)
    # e^[S2]θ2
    theta2 = sym.Symbol('\u03F42')
    e2 = buildSymExponential(w[1], q[1], theta2)
    # e^[S3]θ3
    theta3 = sym.Symbol('\u03F43')
    e3 = buildSymExponential(w[2], q[2], theta3)
    # e^[S4]θ4
    theta4 = sym.Symbol('\u03F44')
    e4 = buildSymExponential(w[3], q[3], theta4)
    # e^[S5]θ5
    theta5 = sym.Symbol('\u03F45')
    e5 = buildSymExponential(w[4], q[4], theta5)
    # e^[S6]θ6
    theta6 = sym.Symbol('\u03F46')
    e6 = buildSymExponential(w[5], q[5], theta6)

    e = [e1, e2, e3, e4, e5, e6]
    T = e[0]
    for i in range(1, 6):   # multiplies all e matrices in sequence
        T = np.matmul(T, e[i])
    T = np.matmul(T, M)

    return T, e


"""
Helper function that builds the numerical exponential matrix
"""
def buildExponential(w, q, theta):
    temp_w = np.transpose(w) * -1		# transpose for cross product
    q = np.transpose(q)		# transpose for cross product

    v = np.cross(temp_w, q)
    v = np.transpose(v)		# transpose to make temp matrix look cleaner
    temp = np.array([[0, -w[2], w[1], v[0]],
                [w[2], 0, -w[0], v[1]],
                [-w[1], w[0], 0, v[2]],
                [0, 0, 0, 0]])

    e = expm(temp*theta)		# create exponential matrix
    return e


"""
Calculates a numerical solution to the forward kinematics equation. The
numerical code starts with the M matrix, ω1 through ω6, q1
through q6, and the thetas and returns the numerical T06 and the 6 matrices defined by
e^[S1]θ1 through e^[S6]θ6
"""
def numericalSolution(M, w, q, thetas):
    e1 = buildExponential(w[0], q[0], thetas[0])    # e^[S1]θ1
    e2 = buildExponential(w[1], q[1], thetas[1])    # e^[S2]θ2
    e3 = buildExponential(w[2], q[2], thetas[2])    # e^[S3]θ3
    e4 = buildExponential(w[3], q[3], thetas[3])    # e^[S4]θ4
    e5 = buildExponential(w[4], q[4], thetas[4])    # e^[S5]θ5
    e6 = buildExponential(w[5], q[5], thetas[5])    # e^[S6]θ6

    e = [e1, e2, e3, e4, e5, e6]
    T = e[0]
    for i in range(1, 6):   # multiplies all e matrices in sequence
        T = np.matmul(T, e[i])
    T = np.matmul(T, M)

    return T, e


"""
For a given {θ1, θ2, θ3, θ4, θ5, θ6}, calculates the d06 vector
"""
def calcLocation(thetas):
    T, e = numericalSolution(M, w, q, thetas)
    d = [T[0][3], T[1][3], T[2][3]]     # extract location of end effector from T matrix
    return d


"""
MAIN CODE
"""
T, e = symbolicSolution(M, w, q)
print(e)

thetas1 = [np.radians(35), np.radians(-35), np.radians(25), np.radians(-20), np.radians(-90), np.radians(0)]
d = calcLocation(thetas1)
print("THETAS1: ")
print(d)
thetas2 = [np.radians(10), np.radians(-25), np.radians(35), np.radians(-45), np.radians(-90), np.radians(10)]
d = calcLocation(thetas2)
print("THETAS2: ")
print(d)
