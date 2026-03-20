import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import savgol_filter

df = pd.read_csv('/home/bimse/Documents/Sim_logs/20260225-142721.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/19700101-000717.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/19700101-000446.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/19700101-000736.csv')

df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt6/19700101-005237FORCEDHIGHERTHRUST.csv')
df = pd.read_csv('/home/bimse/Documents/Sim_logsg/20260310-123721.csv')
df = pd.read_csv('/home/bimse/Documents/Sim_logsg/20260310-124615.csv')
df = pd.read_csv('/home/bimse/Documents/Sim_logsg/20260310-182032-SINUSOID.csv')
df = pd.read_csv('/home/bimse/Documents/Sim_logsg/20260310-182301-STRAIGHTUP.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt10/SINUSOIDAL-MANUALTAKEOFF-SEVERELEFTSTEERING.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt10/HOVER-OFFCENTER-AFTERMANUALTAKEOFF.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt13/19700101-002132SINUSOIDAL_FLIGHT_LOWERKV_GAIN.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt13/19700101-000945LOWERKV.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt13/19700101-000604ORIGINIALKV.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt13/19700101-001400DECTREASEDROLLGAIN.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-000142FALLING_HEIGHT.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-000821GRAVITY_OTHER_DIRECTION_MAYBE_GOOD.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-001247SINUSOID.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-000424BADBIGSINUSOID.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-000229LONGER_SMALL_SIN.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/mrt17/19700101-000659BIGGER_SIN.csv')

problems_start = 80

time = df['timestamp']

pos_x = df['pos_x']
pos_x_ref = df['pos_ref_x']
vel_x = df['vel_x']
vel_x_ref = df['vel_ref_x']
acc_x = df['acc_x']
acc_x_ref = df['acc_ref_x']

acc_x_smooth = savgol_filter(acc_x, window_length=50, polyorder=3)

plt.figure(figsize=(18,15))

plt.subplot(3, 1, 1)
plt.plot(time, pos_x, label="pos_x", color="red")
plt.plot(time, pos_x_ref, label="pos_ref_x", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, vel_x, label="vel_x", color="red")
plt.plot(time, vel_x_ref, label="vel_ref_x", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, acc_x, label="acc_x", color="red")
plt.plot(time, acc_x_ref, label="acc_ref_x", color="blue")
plt.plot(time, acc_x_smooth, label="acc_x smooth", color="orange")
# plt.axvline(problems_start)
plt.legend()
plt.show()


pos_y = df['pos_y']
pos_y_ref = df['pos_ref_y']
vel_y = df['vel_y']
vel_y_ref = df['vel_ref_y']
acc_y = df['acc_y']
acc_y_ref = df['acc_ref_y']

acc_y_smooth = savgol_filter(acc_y, window_length=50, polyorder=3)

plt.figure(figsize=(18,15))

plt.subplot(3, 1, 1)
plt.plot(time, pos_y, label="pos_y", color="red")
plt.plot(time, pos_y_ref, label="pos_ref_y", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, vel_y, label="vel_y", color="red")
plt.plot(time, vel_y_ref, label="vel_ref_y", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, acc_y, label="acc_y", color="red")
plt.plot(time, acc_y_ref, label="acc_ref_y", color="blue")
plt.plot(time, acc_y_smooth, label="acc_y smooth", color="orange")
# plt.axvline(problems_start)
plt.legend()
plt.show()



pos_z = df['pos_z']
pos_z_ref = df['pos_ref_z']
vel_z = df['vel_z']
vel_z_ref = df['vel_ref_z']
acc_z = df['acc_z']
acc_z_ref = df['acc_ref_z']

acc_z_smooth = savgol_filter(acc_z, window_length=50, polyorder=3)

plt.figure(figsize=(18,15))

plt.subplot(3, 1, 1)
plt.plot(time, pos_z, label="pos_z", color="red")
plt.plot(time, pos_z_ref, label="pos_ref_z", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, vel_z, label="vel_z", color="red")
plt.plot(time, vel_z_ref, label="vel_ref_z", color="blue")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, acc_z, label="acc_z", color="red")
plt.plot(time, acc_z_ref, label="acc_ref_z", color="blue")
plt.plot(time, acc_z_smooth, label="acc_z smooth", color="orange")
# plt.axvline(problems_start)
plt.legend()
plt.show()

# plt.plot(time, acc_x, label="acc_x")
# plt.plot(time, acc_x_ref, label="acc_x_ref")
# plt.plot(time, acc_y, label="acc_y", color="blue")
# plt.plot(time, acc_y_ref, label="acc_y_ref", color="red")
# plt.plot(time, acc_z, label="acc_z")
# plt.plot(time, acc_z_ref, label="acc_z_ref")
# plt.legend()
# plt.show()


att_phi = df['att_phi']
att_theta = df['att_theta']
att_psi = df['att_psi']

plt.figure(figsize=(18,15))
plt.plot(time, att_phi, label="att_phi")
plt.plot(time, att_theta, label="att_theta")
plt.plot(time, att_psi, label="att_psi")
# plt.axvline(problems_start)
plt.legend()
plt.show()



rate_p = df['rate_p']
rate_q = df['rate_q']
roll_rate_calc = df['roll_rate_calc']
pitch_rate_calc = df['pitch_rate_calc']

rate_p_smooth = savgol_filter(rate_p, window_length=11, polyorder=3)
rate_q_smooth = savgol_filter(rate_q, window_length=11, polyorder=3)
roll_rate_calc_smooth = savgol_filter(roll_rate_calc, window_length=50, polyorder=3)
pitch_rate_calc_smooth = savgol_filter(pitch_rate_calc, window_length=50, polyorder=3)

plt.figure(figsize=(18,15))
plt.subplot(2, 1, 1)
plt.plot(time, roll_rate_calc, label="roll_rate_calc")
plt.plot(time, rate_p, label="rate_p")
# plt.plot(time, roll_rate_calc_smooth, label="roll_rate_calc smooth")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, pitch_rate_calc, label="pitch_rate_calc")
plt.plot(time, rate_q, label="rate_q")
# plt.plot(time, pitch_rate_calc_smooth, label="pitch_rate_calc smooth")
# plt.axvline(problems_start)
plt.legend()
plt.show()


d_accel_ref_v_x = df["d_accel_ref_v.x"]
d_accel_ref_v_y = df[" d_accel_ref_v.y"]
d_accel_ref_v_z = df[" d_accel_ref_v.z"]

d_accel_ref_b_x = df["d_accel_ref_b.x"]
d_accel_ref_b_y = df[" d_accel_ref_b.y"]
d_accel_ref_b_z = df[" d_accel_ref_b.z"]


plt.figure(figsize=(18,15))
plt.subplot(3, 1, 1)
plt.plot(time, d_accel_ref_v_x, label="d_accel_ref_v.x")
plt.plot(time, d_accel_ref_b_x, label="d_accel_ref_b.x")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, d_accel_ref_v_y, label="d_accel_ref_v.y")
plt.plot(time, d_accel_ref_b_y, label="d_accel_ref_b.y")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, d_accel_ref_v_z, label="d_accel_ref_v.z")
plt.plot(time, d_accel_ref_b_z, label="d_accel_ref_b.z")
# plt.axvline(problems_start)
plt.legend()
plt.show()


d_accel_ref_v_x_smooth = savgol_filter(d_accel_ref_v_x, window_length=50, polyorder=3)
d_accel_ref_v_y_smooth = savgol_filter(d_accel_ref_v_y, window_length=50, polyorder=3)
d_accel_ref_v_z_smooth = savgol_filter(d_accel_ref_v_z, window_length=50, polyorder=3)

d_accel_ref_b_x_smooth = savgol_filter(d_accel_ref_b_x, window_length=50, polyorder=3)
d_accel_ref_b_y_smooth = savgol_filter(d_accel_ref_b_y, window_length=50, polyorder=3)
d_accel_ref_b_z_smooth = savgol_filter(d_accel_ref_b_z, window_length=50, polyorder=3)


plt.figure(figsize=(18,15))
plt.subplot(3, 1, 1)
plt.plot(time, d_accel_ref_v_x_smooth, label="d_accel_ref_v.x smooth")
plt.plot(time, d_accel_ref_b_x_smooth, label="d_accel_ref_b.x smooth")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, d_accel_ref_v_y_smooth, label="d_accel_ref_v.y smooth")
plt.plot(time, d_accel_ref_b_y_smooth, label="d_accel_ref_b.y smooth")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, d_accel_ref_v_z_smooth, label="d_accel_ref_v.z smooth")
plt.plot(time, d_accel_ref_b_z_smooth, label="d_accel_ref_b.z smooth")
# plt.axvline(problems_start)
plt.legend()
plt.show()



T_guid = df["T_guid"]
T_cmd = df["T_cmd"]

plt.figure(figsize=(18,15))
plt.subplot(2, 1, 1)
plt.plot(time, T_guid, label="T_guid")
# plt.axvline(problems_start)
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, T_cmd, label="T_cmd")
# plt.axvline(problems_start)
plt.legend()
plt.show()


cmd_TL = df['cmd_TL']
cmd_TR = df['cmd_TR']
cmd_BL = df['cmd_BL']
cmd_BR = df['cmd_BR']

plt.figure(figsize=(18,15))
plt.plot(time, pos_y, label="pos_y", color="red")
plt.plot(time, pos_y_ref, label="pos_ref_y", color="blue")
plt.plot(time, cmd_TL, label="cmd_TL")
plt.plot(time, cmd_TR, label="cmd_TR")
plt.plot(time, cmd_BL, label="cmd_BL")
plt.plot(time, cmd_BR, label="cmd_BR")
plt.legend()
plt.show()


cmd_TL_smooth = savgol_filter(cmd_TL, window_length=50, polyorder=3)
cmd_TR_smooth = savgol_filter(cmd_TR, window_length=50, polyorder=3)
cmd_BL_smooth = savgol_filter(cmd_BL, window_length=50, polyorder=3)
cmd_BR_smooth = savgol_filter(cmd_BR, window_length=50, polyorder=3)
plt.figure(figsize=(18,15))
plt.plot(time, pos_y, label="pos_y", color="red")
plt.plot(time, pos_y_ref, label="pos_ref_y", color="blue")
plt.plot(time, cmd_TL_smooth, label="cmd_TL_smooth")
plt.plot(time, cmd_TR_smooth, label="cmd_TR_smooth")
plt.plot(time, cmd_BL_smooth, label="cmd_BL_smooth")
plt.plot(time, cmd_BR_smooth, label="cmd_BR_smooth")
plt.legend()
plt.show()