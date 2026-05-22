import matplotlib.pyplot as plt
import pandas as pd
from scipy.signal import savgol_filter

K_P = 4.0
K_V = 4.0

df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-000516FirstHoverAfterNewRods.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-000453FiltAccBAD.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-004352Filtered_daccel_somewhat_working_not_necessarily_improvement.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-001047Filtered_accel_wobbly.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-003356ThrustClamped.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr1/19700101-000230FiltInsideLoopBadThisTime.csv')

df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr20/19700101-001946FiltOf100OffCenterHoverWeirdCirclesActually.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr20/19700101-002709FiltOf100HigherGainsBetterOffCenterHover.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr21/19700101-000415PoorQualityOscillations.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr21/19700101-000229ShorterPeriodOscillations.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr21/19700101-001412HigherCutoffForOscillations.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr21/19700101-000316FilteredCmds.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr21/19700101-002026LowCOFreqBetterMaybe.csv')


# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr23/19700101-001522HoverCutOff8.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr23/19700101-000935HoverCutOff10.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr23/19700101-003942XSinusoid.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr24/19700101-002700VelFilterHoverOkayButUnstableIfOffCourse.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr29/19700101-000824HoverRealThrustForINDI.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr29/19700101-001302NewThrustSinNotAmazing.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/apr29/19700101-001957LowerKNoVisbileImprove.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may6/19700101-002701YawDamperAttempt3.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may6/19700101-003738NewYawDamperCodePosition.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may6/19700101-000341.csv')

df = pd.read_csv('/home/bimse/Documents/Flight_logs/may7/19700101-000553KpKvSetZero.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may7/19700101-001257KpKvSetZeroSin.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may7/19700101-001650KpKvSetZeroSin2.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may7/19700101-003233KpKvZeroEPsiTracking.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may7/19700101-003747NewPsiKpKvNotZero.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-000312KpKvZeroNewCodeStructureHover.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-003558IncludeGravityHover.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-003959LowerCOFreq.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-004421YawDamping.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-000125NewYawRecordingTypeInCode.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-000342Sin.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may18/19700101-000836SinAgainKpHigher.csv')

# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-000118YawHigherInAirframe.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-000710NewGains.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-001338NewThrustForMatrix.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-002544BodyAccLogging.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-000140LongerFligthSlightylyTunedYawDifferently.csv')
# df = pd.read_csv('/home/bimse/Documents/Flight_logs/may19/19700101-000630Circle.csv')

# df = pd.read_csv('/home/bimse/Documents/Sim_logs/may19/20260519-200451.csv')
# df = pd.read_csv('/home/bimse/Documents/Sim_logs/may19/20260519-202534.csv')
# df = pd.read_csv('/home/bimse/Documents/Sim_logs/may19/20260519-232140.csv')

df = pd.read_csv('/home/bimse/Documents/Flight_logs/may20/19700101-001710HoverAfterSimChanges.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may20/19700101-000205NewBattery.csv')
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may20/19700101-000909GainChanges.csv')

df = pd.read_csv('/home/bimse/Documents/Sim_logs/may20/20260520-230108.csv')

df = pd.read_csv('/home/bimse/Documents/Flight_logs/may20/19700101-001858.csv') #Sin shape in v and a visible
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may20/19700101-001916.csv') 

df = pd.read_csv('/home/bimse/Documents/Flight_logs/may22/19700101-000428CircleNoThrustCap.csv') 
df = pd.read_csv('/home/bimse/Documents/Flight_logs/may22/19700101-001907CircleLowerThrustCapLowerKp.csv') 




time = df['timestamp']

# plt.figure(figsize=(18,15))
# plt.plot(time, df['att_psi'], label="att_psi")
# plt.plot(time, df['rate_r'], label="rate_r")
# plt.plot(time, df['yaw_rate_calc'], label="cmd_yaw_rate")
# plt.legend()
# plt.show()

# plt.figure(figsize=(18,15))
# plt.plot(df['pos_x'], df['pos_y'], label="actual")
# plt.plot(df['pos_ref_x'], df['pos_ref_y'], label="ref")
# plt.legend()
# plt.show()

# plt.figure(figsize=(18,15))
# plt.plot(time, df['rate_q'], label="rate_q")
# plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
# plt.legend()
# plt.show()

# plt.figure(figsize=(18,15))
# plt.plot(time, df['rate_p'], label="rate_p")
# plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
# plt.legend()
# plt.show()

# plt.figure(figsize=(18,15))
# # plt.plot(time, df['rate_q'], label="rate_q")
# # plt.plot(time, df['rate_p'], label="rate_p")
# # plt.plot(time, df['rate_r'], label="rate_r")
# plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
# plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
# plt.plot(time, df['yaw_rate_calc'], label="cmd_yaw_rate")
# plt.legend()
# plt.show()

plt.figure(figsize=(18,15))
plt.title("Position tracking", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['pos_x'], label="pos_x", color="red")
plt.plot(time, df['pos_ref_x'], label="pos_ref_x", color="blue")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['pos_y'], label="pos_y", color="red")
plt.plot(time, df['pos_ref_y'], label="pos_ref_y", color="blue")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df['pos_z'], label="pos_z", color="red")
plt.plot(time, df['pos_ref_z'], label="pos_ref_z", color="blue")
plt.legend()
plt.show()


plt.figure(figsize=(18,15))
plt.title("x axis", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['pos_x'], label="pos_x", color="red")
plt.plot(time, df['pos_ref_x'], label="pos_ref_x", color="blue")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['vel_x'], label="vel_x", color="red")
plt.plot(time, df['vel_ref_x'], label="vel_ref_x", color="blue")
# plt.plot(time, df['vel_meas_x'], label="vel_meas_x", color="green")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df['acc_x'], label="acc_x", color="red")
plt.plot(time, df['acc_ref_x'], label="acc_ref_x", color="blue")
plt.plot(time, df['acc_meas_x'], label="acc_x meas", color="purple")
plt.legend()
plt.show()



plt.figure(figsize=(18,15))
plt.title("y axis", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['pos_y'], label="pos_y", color="red")
plt.plot(time, df['pos_ref_y'], label="pos_ref_y", color="blue")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['vel_y'], label="vel_y", color="red")
plt.plot(time, df['vel_ref_y'], label="vel_ref_y", color="blue")
# plt.plot(time, df['vel_meas_y'], label="vel_meas_y", color="green")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df['acc_y'], label="acc_y", color="red")
plt.plot(time, df['acc_ref_y'], label="acc_ref_y", color="blue")
plt.plot(time, df['acc_meas_y'], label="acc_y meas", color="purple")
plt.legend()
plt.show()



plt.figure(figsize=(18,15))
plt.title("z axis", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['pos_z'], label="pos_z", color="red")
plt.plot(time, df['pos_ref_z'], label="pos_ref_z", color="blue")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['vel_z'], label="vel_z", color="red")
plt.plot(time, df['vel_ref_z'], label="vel_ref_z", color="blue")
# plt.plot(time, df['vel_meas_z'], label="vel_meas_z", color="green")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df['acc_z'], label="acc_z", color="red")
plt.plot(time, df['acc_ref_z'], label="acc_ref_z", color="blue")
plt.plot(time, df['acc_meas_z'], label="acc_z meas", color="purple")
plt.legend()
plt.show()


plt.figure(figsize=(18,15))
plt.title("z axis 2", fontdict={'fontsize' : 30})

plt.subplot(4, 1, 1)
plt.plot(time, df['pos_z'], label="pos_z", color="red")
plt.plot(time, df['pos_ref_z'], label="pos_ref_z", color="blue")
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(time, df['vel_z'], label="vel_z", color="red")
plt.plot(time, df['vel_ref_z'], label="vel_ref_z", color="blue")
# plt.plot(time, df['vel_meas_z'], label="vel_meas_z", color="green")
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(time, df['acc_z'], label="acc_z", color="red")
plt.plot(time, df['acc_ref_z'], label="acc_ref_z", color="blue")
plt.plot(time, df['acc_meas_z'], label="acc_z meas", color="purple")
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(time, df["T_cmd"], label="T_cmd")
# plt.plot(time, df['vel_z']*3, label="vel_z", color="red")
plt.plot(time, (df['pos_z']+1.5)*3, label="pos_z", color="red")
# plt.plot(time, df['acc_meas_z']*1, label="acc_z", color="red")
plt.plot(time, df[" d_accel_ref_b.z"], label="d_accel_ref_b.z")
plt.axhline(0)
plt.legend()
plt.show()

plt.figure(figsize=(18,15))
plt.title("All axes", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['pos_x'], label="pos_x", color="red")
plt.plot(time, df['vel_x'], label="vel_x", color="green")
plt.plot(time, df['acc_meas_x'], label="acc_x meas", color="purple")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['pos_y'], label="pos_y", color="red")
plt.plot(time, df['vel_y'], label="vel_y", color="green")
plt.plot(time, df['acc_meas_x'], label="acc_x meas", color="purple")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df['pos_z'], label="pos_z", color="red")
plt.plot(time, df['vel_z'], label="vel_z", color="green")
plt.plot(time, df['acc_meas_z'], label="acc_z meas", color="purple")
plt.legend()
plt.show()




plt.figure(figsize=(18,15))
plt.title("attitude", fontdict={'fontsize' : 30})
plt.plot(time, df['att_phi'], label="att_phi")
plt.plot(time, df['att_theta'], label="att_theta")
plt.plot(time, df['att_psi'], label="att_psi")
plt.legend()
plt.show()

plt.figure(figsize=(18,15))
plt.subplot(2, 1, 1)
plt.title("angular rates", fontdict={'fontsize' : 30})
plt.plot(time, df['rate_p'], label="rate_p")
plt.plot(time, df['rate_q'], label="rate_q")
plt.plot(time, df['rate_r'], label="rate_r")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.plot(time, df['yaw_rate_calc'], label="yaw_rate_calc")
plt.legend()

plt.show()


rate_p_smooth = savgol_filter(df['rate_p'], window_length=11, polyorder=3)
rate_q_smooth = savgol_filter(df['rate_q'], window_length=11, polyorder=3)
roll_rate_calc_smooth = savgol_filter(df['roll_rate_calc'], window_length=50, polyorder=3)
pitch_rate_calc_smooth = savgol_filter(df['pitch_rate_calc'], window_length=50, polyorder=3)

plt.figure(figsize=(18,15))
plt.subplot(2, 1, 1)
plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
plt.plot(time, df['acc_meas_x'], label="acc_x meas", color="purple")
plt.plot(time, df['acc_ref_x'], label="acc_x_ref", color="red")
plt.plot(time, df["accel_ref_with_gains_x"], label="accel_ref_with_gains_x", color="green")
plt.plot(time, df[" d_accel_ref_b.y"], label="d_accel_ref_b.y")
plt.plot(time, df['rate_p'], label="rate_p")
plt.legend()



plt.subplot(2, 1, 2)
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.plot(time, df['acc_meas_y'], label="acc_y meas", color="purple")
plt.plot(time, df['acc_ref_y'], label="acc_y_ref", color="red")
plt.plot(time, df["d_accel_ref_b.x"], label="d_accel_ref_b.x")
plt.plot(time, df["accel_ref_with_gains_y"], label="accel_ref_with_gains_y", color="green")
plt.plot(time, df['rate_q'], label="rate_q")
plt.legend()
plt.show()



plt.figure(figsize=(18,15))
plt.title("acceleration comparisons")
plt.subplot(3, 1, 1)
plt.plot(time, df['acc_ref_x'] - df['acc_meas_x'], label="acc_x_ref - acc_x_meas", color="magenta")
plt.plot(time, df["d_accel_ref_v.x"], label="d_accel_ref_v.x")
plt.plot(time, df["d_accel_ref_b.x"], label="d_accel_ref_b.x")
plt.plot(time, df['att_phi'], label="att_phi")
plt.plot(time, df['att_theta'], label="att_theta")
plt.plot(time, df['att_psi'], label="att_psi")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df['acc_ref_y'] - df['acc_meas_y'], label="acc_y_ref - acc_y_meas", color="magenta")
plt.plot(time, df[" d_accel_ref_v.y"], label="d_accel_ref_v.y")
plt.plot(time, df[" d_accel_ref_b.y"], label="d_accel_ref_b.y")
plt.plot(time, df['att_phi'], label="att_phi")
plt.plot(time, df['att_theta'], label="att_theta")
plt.plot(time, df['att_psi'], label="att_psi")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, (df['acc_ref_z'] - df['acc_meas_z'] - 9.81 * 0.09)*5000, label="(acc_z_ref - acc_z_meas - 9.81 * 0.09)*5000", color="magenta")
plt.plot(time, df[" d_accel_ref_v.z"]*5000, label="d_accel_ref_v.z *5000")
plt.plot(time, df[" d_accel_ref_b.z"], label="d_accel_ref_b.z")
plt.legend()
plt.show()


d_accel_ref_v_x_smooth = savgol_filter(df["d_accel_ref_v.x"], window_length=50, polyorder=3)
d_accel_ref_v_y_smooth = savgol_filter(df[" d_accel_ref_v.y"], window_length=50, polyorder=3)
d_accel_ref_v_z_smooth = savgol_filter(df[" d_accel_ref_v.z"], window_length=50, polyorder=3)

d_accel_ref_b_x_smooth = savgol_filter(df["d_accel_ref_b.x"], window_length=50, polyorder=3)
d_accel_ref_b_y_smooth = savgol_filter(df[" d_accel_ref_b.y"], window_length=50, polyorder=3)
d_accel_ref_b_z_smooth = savgol_filter(df[" d_accel_ref_b.z"], window_length=50, polyorder=3)


T_guid_smooth = savgol_filter(df["T_guid"], window_length=50, polyorder=3)
T_cmd_smooth = savgol_filter(df["T_cmd"], window_length=50, polyorder=3)

plt.figure(figsize=(18,15))
plt.title("thrusts", fontdict={'fontsize' : 30})
plt.subplot(2, 1, 1)
plt.plot(time, df["T_guid"], label="T_guid")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time, df["T_cmd"], label="T_cmd")
plt.plot(time, df[" d_accel_ref_v.z"], label="d_accel_ref_v.z")
plt.legend()
plt.show()

plt.figure(figsize=(18,15))

plt.subplot(1, 1, 1)
plt.plot(time, df['pos_y'], label="pos_y", color="red")
plt.plot(time, df['pos_ref_y'], label="pos_ref_y", color="blue")
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.legend()

plt.show()


plt.figure(figsize=(18,15))
plt.title("acceleration and angular rate comparison", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
# plt.plot(time, df['acc_meas_x'] - df['acc_ref_x'], label="acc_x_meas - acc_x_ref", color="magenta")
plt.plot(time, df['d_accel_ref_v.x'], label="d_accel_ref_v.x", color="purple")
plt.plot(time, df['d_accel_ref_b.x'], label="d_accel_ref_b.x", color="green")
# plt.plot(time, df["d_accel_ref_b.x"], label="d_accel_ref_b.x")
# plt.plot(time, df["d_accel_ref_v.x"], label="d_accel_ref_v.x")
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
# plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
plt.axhline(0)
# plt.plot(time, df['rate_q'], label="rate_q")
plt.legend()

plt.subplot(3, 1, 2)
# plt.plot(time, df['acc_meas_y'] - df['acc_ref_y'], label="acc_y_meas - acc_y_ref", color="magenta")
plt.plot(time, df[' d_accel_ref_v.y'], label="d_accel_ref_v.y", color="purple")
plt.plot(time, df[' d_accel_ref_b.y'], label="d_accel_ref_b.y", color="green")
# plt.plot(time, df[" d_accel_ref_b.y"], label="d_accel_ref_b.y")
# plt.plot(time, df[" d_accel_ref_v.y"], label="d_accel_ref_v.y")
plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
# plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.axhline(0)
# plt.plot(time, df['rate_p'], label="rate_p")

plt.subplot(3, 1, 3)
plt.plot(time, df[' d_accel_ref_b.z'], label="d_accel_ref_b.z", color="green")
plt.axhline(0)
plt.plot(time, df["T_cmd"], label="T_cmd")
plt.legend()
plt.show()

plt.figure(figsize=(18,15))
plt.title("acceleration and angular rate comparison 2", fontdict={'fontsize' : 30})

plt.subplot(3, 1, 1)
plt.plot(time, df['d_accel_ref_b.x'], label="d_accel_ref_b.x", color="green")
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
# plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
plt.axhline(0)
plt.plot(time, df['rate_q'], label="rate_q")
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, df[' d_accel_ref_b.y'], label="d_accel_ref_b.y", color="green")
plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
# plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.axhline(0)
plt.plot(time, df['rate_p'], label="rate_p")
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, df[' d_accel_ref_b.z'], label="d_accel_ref_b.z", color="green")
# plt.plot(time, df['acc_body_x_z'], label="acc_body_z")
# plt.plot(time, (df['pos_z'] - df['pos_ref_z']) * K_P, label="pos_z - pos_z_ref", color="red")
# plt.plot(time, (df['vel_z'] - df['vel_ref_z']) * K_V, label="vel_z - vel_z_ref", color="blue")
# plt.plot(time, df['acc_meas_z'] - df['acc_ref_z'], label="acc_z - acc_z_ref", color="magenta")
plt.axhline(0)
plt.plot(time, df["T_cmd"], label="T_cmd")
plt.legend()
plt.show()

plt.figure(figsize=(18,15))
plt.title("acceleration and angular rate comparison 3", fontdict={'fontsize' : 30})

plt.subplot(2, 1, 1)
# plt.plot(time, df['acc_meas_x'] - df['acc_ref_x'], label="acc_x_meas - acc_x_ref", color="magenta")
plt.plot(time, df['d_accel_ref_b.x'], label="d_accel_ref_b.x", color="purple")
plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
# plt.plot(time, df['d_accel_ref_b.x']/df['acc_body_x_z'], label="acc_body_z")
plt.axhline(0)
# plt.plot(time, df['rate_q'], label="rate_q")
plt.legend()

plt.subplot(2, 1, 2)
# plt.plot(time, df['acc_meas_y'] - df['acc_ref_y'], label="acc_y_meas - acc_y_ref", color="magenta")
# plt.plot(time, df[' d_accel_ref_v.y'], label="d_accel_ref_v.y", color="purple")
plt.plot(time, df[' d_accel_ref_b.y'], label="d_accel_ref_b.y", color="green")
# plt.plot(time, df[" d_accel_ref_b.y"], label="d_accel_ref_b.y")
# plt.plot(time, df[" d_accel_ref_v.y"], label="d_accel_ref_v.y")
plt.plot(time, df['roll_rate_calc'], label="roll_rate_calc")
# plt.plot(time, df['acc_body_x_z'], label="acc_body_z")
# plt.plot(time, df['pitch_rate_calc'], label="pitch_rate_calc")
plt.axhline(0)
# plt.plot(time, df['rate_p'], label="rate_p")
plt.legend()
plt.show()


plt.figure(figsize=(18,15))
plt.title("motors", fontdict={'fontsize' : 30})
plt.plot(time, df['pos_y'], label="pos_y", color="red")
plt.plot(time, df['pos_ref_y'], label="pos_ref_y", color="blue")
plt.plot(time, df['cmd_TL'], label="cmd_TL")
plt.plot(time, df['cmd_TR'], label="cmd_TR")
plt.plot(time, df['cmd_BL'], label="cmd_BL")
plt.plot(time, df['cmd_BR'], label="cmd_BR")
plt.legend()
plt.show()


# cmd_TL_smooth = savgol_filter(df['cmd_TL'], window_length=50, polyorder=3)
# cmd_TR_smooth = savgol_filter(df['cmd_TR'], window_length=50, polyorder=3)
# cmd_BL_smooth = savgol_filter(df['cmd_BL'], window_length=50, polyorder=3)
# cmd_BR_smooth = savgol_filter(df['cmd_BR'], window_length=50, polyorder=3)

# plt.figure(figsize=(18,15))
# plt.plot(time, df['pos_y']*8000, label="pos_y*8000", color="red")
# plt.plot(time, df['pos_ref_y']*8000, label="pos_ref_y*8000", color="blue")
# plt.plot(time, cmd_TL_smooth, label="cmd_TL_smooth")
# plt.plot(time, cmd_TR_smooth, label="cmd_TR_smooth")
# plt.plot(time, cmd_BL_smooth, label="cmd_BL_smooth")
# plt.plot(time, cmd_BR_smooth, label="cmd_BR_smooth")
# plt.legend()
# plt.show()