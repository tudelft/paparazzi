import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate


f_N = open("IMAV_09_09_23_TEST1_ArucoNORTH_CompleteV2")
f_E = open("IMAV_09_09_23_TEST1_ArucoEAST_CompleteV2")

N = np.loadtxt(f_N, delimiter=",", dtype=str).astype(float)
E = np.loadtxt(f_E, delimiter=",", dtype=str).astype(float)

tM = N[:,1]
print("End time",tM[-1])

dt = 1.0 / 15.0
tsim = np.arange(0,tM[-1],dt)

f_N = N[0,0]
f_E = E[0,0]

x = np.asarray([[f_N],
                [f_E],
                [0],
                [0]])


A = np.asarray([[1,  0,  dt,  0],
                [0,  1,  0,   dt],
                [0,  0,  1,   0],
                [0,  0,  0,   1]] )

H = np.asarray([[1,  0, 0 ,0],
                [0,  1, 0 ,0]] )


print('A',A)
print('H',H)

K0 = 1e5
Kp = 1
Kv = 0.0001
Kpv = 0
Km = 1e5

Q = np.asarray([[Kp, 0, Kpv, 0],
                [0, Kp, 0, Kpv],
                [Kpv, 0, Kv, 0],
                [0, Kpv, 0 ,Kv]])
R = np.asarray([[Km],[Km]])
P = np.asarray([[K0, 0, 0, 0],
                [0, K0, 0, 0],
                [0, 0,  0, 0],
                [0, 0,  0, 0]])

print('P', P)
print('Q', Q)
print('R', R)

f_VN = 0
f_VE = 0

pf_N = []
pf_E = []

KP = 0.2
KV = 0.0001

i = 0
for t in np.nditer(tsim):
    # predict
    x = A @ x
    P = ((A @ P) @ A.T) + Q
    #print('x', x)
    #print('P',P)
 
    if tM[i] < t:
        # correct
        zk = np.asarray([[N[i,0]],
                         [E[i,0]]])
        #print('z',zk)
        yk = zk - H@x
        S = H @ P @ H.T + R
        #print('S',S)
        Si = np.linalg.inv(S)
        #print('S-1',Si)
        K = (P @ H.T) @ Si
        K = np.asarray([[KP, 0],[0, KP],[KV, 0],[0, KV]])
        #print('K',K)
        x = x + (K @ yk)
        #print('K*yk',K @ yk)
        P = (np.eye(4) - (K @ H)) @ P
        #print('P',P)
        
        while tM[i] < t:
            i+=1
        # print(i,np.round(t,2),np.round(tM[i],2))
    
    pf_N.append(x[0])
    pf_E.append(x[1])

for x in np.nditer(N[:,0]):
    f_N += f_VN
    f_N += (x - f_N) * KP
    f_VN += (x - f_N) * KV

for x in np.nditer(E[:,0]):
    f_E += f_VE
    f_E += (x - f_E) * KP
    f_VE += (x - f_E) * KV



plt.plot(N[:,0], E[:,0],'x')
plt.plot(pf_N, pf_E)
plt.plot(pf_N, pf_E, '*')
plt.ylabel('some numbers')
plt.grid()
plt.show()

plt.figure()
plt.plot(tsim,pf_N)
plt.show()
