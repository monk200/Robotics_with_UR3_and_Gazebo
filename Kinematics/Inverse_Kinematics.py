import numpy as np
import math
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

L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.083
L7 = 0.083
L8 = 0.082
L9 = 0.0535
L10 = 0.059

"""
Pass in (xw, yw, zw, yaw) and return an array of thetas in degrees
"""
def inverseKinematics(input):
    # (xcen, ycen, zcen) in terms of (xw, yw, zw, yaw)
    xcen = input[0] + .15 - (L9*np.cos(input[3]))
    ycen = input[1] - .15 - (L9*np.sin(input[3]))
    zcen = input[2] - .01

    # t1 in terms of (xcen, ycen, zcen)
    t1 = np.arctan2(ycen, xcen) - np.arcsin((L6+.027) / math.sqrt(math.pow(xcen, 2) + math.pow(ycen, 2)))

    # (x3end, y3end, z3end) in terms of (xcen, ycen, zcen, t1)
    xend = xcen + (L6+.027)*np.sin(t1) - L7*np.cos(t1)
    yend = ycen - (L6+.027)*np.cos(t1) - L7*np.sin(t1)
    zend = zcen + L8 + L10

    # intermediate values for side view
    a = np.arctan2(zend - L1, math.sqrt(math.pow(xend, 2) + math.pow(yend, 2)))
    R = math.sqrt(math.pow(zend - L1, 2) + math.pow(xend, 2) + math.pow(yend, 2))
    b = np.arccos((math.pow(R, 2) + math.pow(L3, 2) - math.pow(L5, 2)) / (2 * L3 * R))

    # t2 and t3 in terms of (x3end, y3end, z3end)
    t2 = -a - b
    t3 = np.pi - np.arccos((math.pow(L3, 2) + math.pow(L5, 2) - math.pow(R, 2)) / (2 * L3 * L5))

    # intermediate values for side view
    d = -b + t3
    c = (np.pi / 2) - d

    # t4 in terms of (x3end, y3end, z3end)
    t4 = (-np.pi / 2) + a + c
    # t5 is always -90 degrees
    t5 = -np.pi / 2
    # t6 in terms of (t1, yaw)
    t6 = t1 + (np.pi / 2) - input[3]

    return [np.degrees(t1), np.degrees(t2), np.degrees(t3), np.degrees(t4), np.degrees(t5), np.degrees(t6)]

"""
Helper function for forward kinematics that builds the numerical exponential matrix
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
For a given {θ1, θ2, θ3, θ4, θ5, θ6}, calculates the d06 vector
"""
def forwardKinematics(thetas):
    e1 = buildExponential(w[0], q[0], thetas[0])  # e^[S1]θ1
    e2 = buildExponential(w[1], q[1], thetas[1])  # e^[S2]θ2
    e3 = buildExponential(w[2], q[2], thetas[2])  # e^[S3]θ3
    e4 = buildExponential(w[3], q[3], thetas[3])  # e^[S4]θ4
    e5 = buildExponential(w[4], q[4], thetas[4])  # e^[S5]θ5
    e6 = buildExponential(w[5], q[5], thetas[5])  # e^[S6]θ6

    e = [e1, e2, e3, e4, e5, e6]
    T = e[0]
    for i in range(1, 6):  # multiplies all e matrices in sequence
        T = np.matmul(T, e[i])
    T = np.matmul(T, M)

    d = [T[0][3], T[1][3], T[2][3]]  # extract location of end effector from T matrix
    return d

def inverseAndCheck(input):
    t = inverseKinematics(input)
    print(t, " in degrees")
    t = [np.radians(t[0]), np.radians(t[1]), np.radians(t[2]), np.radians(t[3]), np.radians(t[4]), np.radians(t[5])]
    d = forwardKinematics(t)
    print(d)


print("TEST 1:")
test1 = [0.15, -0.1, 0.25, np.radians(-45)]
t = inverseKinematics(test1)
print(t)

print("TEST 2:")
test2 = [0.2, 0.4, 0.05, np.radians(45)]
t = inverseKinematics(test1)
print(t)
