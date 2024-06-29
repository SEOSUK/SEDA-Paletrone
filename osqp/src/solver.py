#!/usr/bin/env python3
# solver_publisher.py

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import sparse, identity
import osqp
import control
import time
import math
from scipy.sparse import csc_matrix, hstack, vstack
X0 = [0,0,0,0,0,0,0,0]
x_cal=[0,0,0,0,0,0,0,0]
Xr = [0,0,0,0,0,0,0,0]
t=1
i=0
def solve_qp_problem():
   	 
    # Model setting
    global X0 
    global Xr
    global x_cal
    global t
    global i
    x0 = np.array(X0)
    xr = np.array(Xr)+[0.001*i,0,0,0,0,0,0,0];
    #xr=np.array([np.deg2rad(10),0,np.deg2rad(10),0,0,0,0,0])
    N = 10
    dt = 1/100
    
    i=i+1
    x0=x0-x_cal
    
    umin = np.array([-(x0[0]+np.deg2rad(70)), -(np.deg2rad(-0))])
    umax = np.array([(x0[0]+np.deg2rad(120)), (np.deg2rad(0))])
    xmin = np.concatenate([-np.deg2rad(100) * np.ones(4), umin, umin])
    xmax = np.concatenate([np.deg2rad(100) * np.ones(4), umax, umax])
    Q = sparse.diags([100, 0.1, 0, 0, 0, 0, 0, 0]) #[q1 q1_dot q2 q2_dot theta1 theta1_dot theta2 theta2_dot]
    QN = Q
    R = sparse.diags([ 0.000001, 0]) #[theta1_des theta2_des]

    # SEA state parameter
    J1 = 0.420*0.24215*0.24215/12+0.420*0.13475*0.13475+0.225*0.1759*0.1759/12+0.225*(0.0562+0.24215)*(0.0562+0.24215)
    J2 = 0.225*0.1759*0.1759/12+0.225*0.0562*0.0562
    k1 = 0.007
    k2 = 6.75
    c1 = 3
    c2 = 1	



    mode=1 # if mode=1 -> Bessel Filter | if mode=2 -> Butterworth filter


    # Bessel Filter state parameter
    w0_1_Be=20
    w0_2_Be=0.4
    #Butterworth filter state space
    zeta = 0.5
    w0_1_Bu=5
    w0_2_Bu=5


    A_SEA = sparse.csc_matrix(
        [[0,                  1,           0,           0],
        [-(k1 + k2) / J1, -(c1 + c2) / J1, 0,    0],
        [0,                  0,           0,           1], 
        [k2 / J2,           c2 / J2,     -k2 / J2,    -c2 / J2]])

    B_SEA = sparse.csc_matrix(
        [[0,     0,     0,      0],
        [k1 / J1, c1 / J1, 0, 0],
        [0,     0,     0,      0],
        [0,     0,    k2 / J2, c2 / J2]])

    C_SEA = sparse.csc_matrix(
        [[1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    D_SEA = sparse.csc_matrix(np.zeros((4, 4)))


    if(mode==1):
        A_motor = sparse.csc_matrix(
            [[0,            1,          0,          0],
            [-3*w0_1_Be*w0_1_Be, -3*w0_1_Be, 0,          0],
            [0,            0,          0,          1],
            [0,            0,         -3*w0_2_Be*w0_2_Be, -3*w0_2_Be]])


        B_motor = sparse.csc_matrix(
            [[0,          0],
            [3*w0_1_Be*w0_1_Be, 0],
            [0,          0],
            [0,         3*w0_2_Be*w0_2_Be]])



    if(mode==2):
        A_motor = np.array([[0, 1, 0, 0],
                            [-w0_1_Bu**2, -2*zeta*w0_1_Bu, 0, 0],
                            [0, 0, 0, 1],
                            [0, 0, -w0_2_Bu*2, -2*zeta*w0_2_Bu]])

        B_motor = np.array([[0, 0],
                            [w0_1_Bu**2, 0],
                            [0, 0],
                            [0, w0_2_Bu**2]])


    A_model_top = hstack([A_SEA, B_SEA])
    A_model_bottom = hstack([csc_matrix((A_motor.shape[0], A_SEA.shape[1])), A_motor])
    A_model = vstack([A_model_top, A_model_bottom])

    B_model_top = csc_matrix((B_SEA.shape[0], 2))
    B_model = vstack([B_model_top, B_motor])
    
    C_model = identity(A_model.shape[0])

    D_model = np.zeros((A_model.shape[0], B_motor.shape[1])) 

    
    # 연속 시간 시스템 생성
    sys_continuous = control.StateSpace(A_model.toarray(), B_model.toarray(), C_model, D_model)
    # 이산 시간 시스템으로 변환
    sys_discrete = control.sample_system(sys_continuous, dt)

    Ad = sparse.csc_matrix(sys_discrete.A)
    Bd = sparse.csc_matrix(sys_discrete.B)

    [nx, nu] = Bd.shape


    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN, sparse.kron(sparse.eye(N), R)], format='csc')                     
    # - linear objective
    q = np.hstack([np.kron(np.ones(N), -Q@xr), -QN@xr, np.zeros(N*nu)])
    # - linear dynamics
    Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(N*nx)])
    ueq = leq
    # - input and state constraints
    Aineq = sparse.eye((N+1)*nx + N*nu)
    lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)]) 
    uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
    # - OSQP constraints
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])

    prob = osqp.OSQP()
    prob.setup(P, q, A, l, u, warm_starting=True)
    l[:nx] = -x0
    u[:nx] = -x0
    prob.update(l=l, u=u)
    res = prob.solve()
    ctrl = res.x[-N*nu:-(N-1)*nu]
    if(ctrl[0]<umin[0]):
        ctrl[0]=-np.deg2rad(150)
    if(ctrl[0]>umax[0]):
        ctrl[0]=np.deg2rad(150)

    if(ctrl[1]<umin[1]):
        ctrl[1]=umin[1]
    if(ctrl[1]>umax[1]):
        ctrl[1]=umax[1]

    rospy.loginfo("X0:::: %s/%s" % (x0*180/3.141592,xr*180/3.141592))
    rospy.loginfo("u: %s" %(ctrl*180/3.141592))

    return np.array([ctrl[0]+0*3.141592/180, 0])

def init_state_callback(msg):
    global X0
    X0= msg.data

def ref_state_callback(msg):
    global Xr
    Xr= msg.data



def publisher():
    rospy.init_node('solver_publisher', anonymous=True)
    # Subscribers
    rospy.Subscriber('init_state', Float64MultiArray, init_state_callback)
    rospy.Subscriber('ref_state', Float64MultiArray, ref_state_callback)
    
    pub = rospy.Publisher('solver_results', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100) # 1000 Hz
 
    while not rospy.is_shutdown():
        solution = solve_qp_problem()

        # Preparing the message
        msg = Float64MultiArray()
        msg.data = solution

        # Publishing
        
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
    	 
        publisher()
    except rospy.ROSInterruptException:
        pass


