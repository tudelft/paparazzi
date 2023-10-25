from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np

from KalmanFilter_V1 import init, predict, update

# READ LOGFILES

f_N = open("Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneNORTH_V3")
f_E = open("Measured_Variables/Outdoor_Tests/IMAV_13_09_23_TEST3_DroneEAST_V3")

E = np.loadtxt(f_N, delimiter=",", dtype=str).astype(float)
N = np.loadtxt(f_E, delimiter=",", dtype=str).astype(float)


tstart = 0
tend = -1


N = N[tstart:tend,:]
E = E[tstart:tend,:]

E[:,0] = -E[:,0] - 110
N[:,0] = -N[:,0] + 100

print(len(N))

# SIMULATED MAIN LOOP AT 15Hz

tM = N[:,1]
print("Simulation end time",tM[-1])
dt = 1.0 / 15.0
extra_time = 0
tsim = np.arange(0,tM[-1]+extra_time,dt)


VN = np.ediff1d(N[:,0]) / np.array(dt)
VE = np.ediff1d(E[:,1]) / np.array(dt)

#print(VN)

# LOGGING FOR PLOT
pf_N = []
pf_E = []
pf_VN = []
pf_VE = []


# init
x = init( [N[0,0], E[0,0], 25] );

i = 0
for t in np.nditer(tsim):
    # predict
    x = predict(dt)
    #print('predict x=', x)
    
    if tM[i] < t:
        # correct
        Z = [N[i,0], E[i,0], 25]
        #print('Z', Z)
        if i < (len(tM)-1):
            x = update( Z )
        #print('correct x=', x)

        #print(len(tM))
        while tM[i] < t:
            if i < (len(tM)-1):
                i+=1
            else:
                i = len(tM) - 1
                break

    pf_N.append(float(x[0]))
    pf_E.append(float(x[1]))
    pf_VN.append(float(x[2]))
    pf_VE.append(float(x[3]))



plt.plot(N[:,0], E[:,0],'x')
plt.plot(pf_N, pf_E)
#plt.plot(pf_N, pf_E, '*')
plt.ylabel('some numbers')
plt.axis('equal')
plt.grid()
plt.show()

Vtot = np.sqrt(VN**2 + VE **2)

fig, axs = plt.subplots(2, 2)
axs[0, 0].plot(tsim, pf_N)
axs[0, 0].plot(tM, N[:,0], 'x')
axs[0, 0].set_title('N')
axs[0, 0].grid()
axs[0, 1].plot(tsim, pf_E, 'tab:orange')
axs[0, 1].plot(tM, E[:,0], 'x')
axs[0, 1].set_title('E')
axs[0, 1].grid()
axs[1, 0].plot(tsim, pf_VN, 'tab:green')
#axs[1, 0].plot(tM[1:], VN, 'x')
axs[1, 0].set_title('VN')
axs[1, 0].grid()
axs[1, 1].plot(tsim, pf_VE, 'tab:red')
#axs[1, 1].plot(tM[1:], Vtot, 'x')
axs[1, 1].set_title('VE')
axs[1, 1].grid()
plt.show()
