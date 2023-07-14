import numpy as np
import scipy as sp
from scipy import signal
import matplotlib.pyplot as plt
import pandas as pd 

import sys
from os import path, getenv

class CtrlEffEst(object):
    def __init__(self, parsed_log, first_order_dyn_list, second_order_filter_cutoff, min_max_pwm_list, is_servo, actuator_indices = None, fs=500.):
        '''
        Initialization function of control effectiveness estimator
        '''
        self.data_indi = parsed_log.get_message_dict('STAB_ATTITUDE_FULL_INDI')
        self.data_actuators = parsed_log.get_message_dict('ACTUATORS')
        self.data_doublet = parsed_log.get_message_dict('DOUBLET')
        self.data_airdata = parsed_log.get_message_dict('AIR_DATA')
        # self.data_wing_rot = parsed_log.get_message_dict('ROT_WING_CONTROLLER')

        self.N_act = len(actuator_indices)
        self.actuator_indices = actuator_indices

        # Copy variables
        self.first_order_dyn_list = first_order_dyn_list # First order actuator dynamics constant
        self.second_order_filter_cutoff= second_order_filter_cutoff # Butterworth second order cutoff frequency [Hz]
        self.min_max_pwm_list = min_max_pwm_list # 2d list with lower and upper bound pwm signals
        self.is_servo = is_servo # bool list giving if actuator is servo (aerodynamic surface control)
        self.fs = fs # Sample frequency [Hz]
        self.dt = 1./fs # Sample time [s]

        # Check if first order dyn list is the same size as self.N_act
        if (self.N_act != len(self.first_order_dyn_list)):
            print('ERROR: length of actuator inputs not corresponding to first order actuator dynamics')

    def get_effectiveness_values(self, plot = True):
        '''
        Returns effectiveness values for each actuator and axis 
        '''
        # Order data arrays
        t_indi = np.array(self.data_indi['t'])
        t_act = np.array(self.data_actuators['t'])
        t_doublet = np.array(self.data_doublet['t'])
        t_airdata = np.array(self.data_airdata['t'])
        # t_rot_wing_controller = np.array(self.data_wing_rot['t'])

        doublet_activated = np.array(self.data_doublet['active']['data'])
        doublet_axis = np.array(self.data_doublet['axis']['data'])
        pwm_original = np.array(self.data_actuators['values']['data'])[:,self.actuator_indices]
        pwm_sliced = []
        airspeed = np.array(self.data_airdata['airspeed']['data'])
        # wing_rot = np.deg2rad(np.array(self.data_wing_rot['wing_angle_deg']['data']))
        # 2nd order bUtterworth noise filter
        b, a = sp.signal.butter(2, 0.1/(50/2), 'low', analog=False)
        airspeed_filtered = sp.signal.lfilter(b, a, airspeed)
        

        for i in range(self.N_act):
            pwm_interp = np.interp(t_indi, t_act, pwm_original.T[i])
            pwm_sliced.append(pwm_interp)

        pwm = np.array(pwm_sliced).T
        # Convert pwm to command
        min_pwm_array = np.array(self.min_max_pwm_list).T[0]
        max_pwm_array = np.array(self.min_max_pwm_list).T[1]
        pwm_range = max_pwm_array - min_pwm_array
        cmd = (pwm - min_pwm_array) / pwm_range * 9600.

        rate_p = np.array(self.data_indi['angular_rate_p']['data'])/180.*np.pi # rad/s
        rate_q = np.array(self.data_indi['angular_rate_q']['data'])/180.*np.pi # rad/s
        rate_r = np.array(self.data_indi['angular_rate_r']['data'])/180.*np.pi # rad/s
        rates = np.array([rate_p,rate_q,rate_r]).T
        acc_x = self.data_indi['body_accel_x']['data'] # m/s²
        acc_z = self.data_indi['body_accel_z']['data'] # m/s²

        # Apply actuator dynamics to commands
        for i in range(self.N_act):
            zi = sp.signal.lfilter_zi([self.first_order_dyn_list[i]], [1, -(1-self.first_order_dyn_list[i])])
            #filtered_cmd = sp.signal.lfilter([self.first_order_dyn_list[i]], [1, -(1-self.first_order_dyn_list[i])], cmd[:,i], zi=zi*cmd[:,i][0])[0]
            filtered_cmd = sp.signal.lfilter([self.first_order_dyn_list[i]], [1, -(1-self.first_order_dyn_list[i])], cmd[:,i])
            if i == 0:
                cmd_a_T = np.array([filtered_cmd]) # Transpose of cmd_a
            else:
                cmd_a_T = np.vstack((cmd_a_T, filtered_cmd))

        cmd_a = cmd_a_T.T

        # 2nd order bUtterworth noise filter
        b, a = sp.signal.butter(2, self.second_order_filter_cutoff/(self.fs/2), 'low', analog=False)

        # Filter signals and cmds
        #zi = np.array([sp.signal.lfilter_zi(b, a)]).T
        rates_f = sp.signal.lfilter(b, a, rates, axis=0)
        acc_x_f = sp.signal.lfilter(b, a, acc_x, axis=0)
        acc_z_f = sp.signal.lfilter(b, a, acc_z, axis=0)
        cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)

        # Apply finite difference methods to get anfular accelerarions
        d_rates = (np.vstack((np.zeros((1,3)), np.diff(rates_f,1,axis=0)))*self.fs)
        dd_rates = (np.vstack((np.zeros((1,3)), np.diff(d_rates,1,axis=0)))*self.fs)

        dd_rates = np.vstack((dd_rates.T,acc_x_f)).T
        dd_rates = np.vstack((dd_rates.T,acc_z_f)).T

        d_cmd= (np.vstack((np.zeros((1,self.N_act)), np.diff(cmd_af,1,axis=0)))*self.fs)
        dd_cmd = (np.vstack((np.zeros((1,self.N_act)), np.diff(d_cmd,1,axis=0)))*self.fs)

        t = t_indi
        # Construct A matrix
        for i in range(len(self.actuator_indices)):
            row = d_cmd[:,i]
            
            if i == 0:
                A = row
            else:
                A = np.vstack((A, row))

        A = A.T

        # Check timespans when doublets activated
        change_doublet_idx = np.where(doublet_activated[:-1] != doublet_activated[1:])[0]
        t_change_doublet = t_doublet[change_doublet_idx]
        #print(len(t_change_doublet))

        doublet_actuator = []
        eff_roll = []
        eff_pitch = []
        eff_yaw = []
        eff_acc_x = []
        eff_acc_z = []
        airspeed_doublet = []
        # wing_rot_doublet = []

        # Perform analysis
        for i in range(0, len(t_change_doublet), 2):
            t_start = t_change_doublet[i]
            t_end = t_change_doublet[i+1]
            doublet_actuator.append(doublet_axis[change_doublet_idx[i]])
            # Perform analysis up to 2 seconds after doublet ends
            # Search for index just before the doublet input
            start_idx = np.argmax(t_indi > t_start) - 1
            end_idx = np.argmax(t_indi > (t_end + 4.))

            A_sliced = A[start_idx:end_idx + 1]
            dd_rates_sliced = dd_rates[start_idx:end_idx + 1]

            g1_lstsq = np.linalg.lstsq(A_sliced,dd_rates_sliced, rcond=None)
            g1_matrix = g1_lstsq[0]
            g1_residuals = g1_lstsq[1]

            eff_roll.append(g1_matrix[doublet_actuator[-1]][0])
            eff_pitch.append(g1_matrix[doublet_actuator[-1]][1])
            eff_yaw.append(g1_matrix[doublet_actuator[-1]][2])
            eff_acc_x.append(g1_matrix[doublet_actuator[-1]][3])
            eff_acc_z.append(g1_matrix[doublet_actuator[-1]][4])

            start_idx_airspeed = np.argmax(t_airdata> t_start) - 1
            end_idx_airspeed = np.argmax(t_airdata > (t_end + 2.))
            airspeed_filtered_sliced = airspeed_filtered[start_idx_airspeed:end_idx_airspeed + 1]
            avg_airspeed = sum(airspeed_filtered_sliced) / len(airspeed_filtered_sliced)
            airspeed_doublet.append(avg_airspeed)

            # start_idx_wing_rot = np.argmax(t_rot_wing_controller > t_start) - 1
            # end_idx_wing_rot = np.argmax(t_rot_wing_controller > (t_end + 2.))
            # wing_rot_sliced = wing_rot[start_idx_wing_rot:end_idx_wing_rot]
            # avg_wing_angle = sum(wing_rot_sliced) / len(wing_rot_sliced)
            # wing_rot_doublet.append(avg_wing_angle)

        # export data to csv file
        # dict_data = {'idx': doublet_actuator, 'airspeed': airspeed_doublet, 'wing_angle' : wing_rot_doublet, 'roll_eff': eff_roll, 'pitch_eff': eff_pitch, 'yaw_eff': eff_yaw}
        dict_data = {'idx': doublet_actuator, 'airspeed': airspeed_doublet, 'roll_eff': eff_roll, 'pitch_eff': eff_pitch, 'yaw_eff': eff_yaw}
        df = pd.DataFrame(dict_data)
        df.to_csv('test_doublet.csv')  

        if plot:
            # Get indices of actuators
            idx0 = np.where(np.array(doublet_actuator) == 0)[0]
            idx1 = np.where(np.array(doublet_actuator) == 1)[0]
            idx2 = np.where(np.array(doublet_actuator) == 2)[0]
            idx3 = np.where(np.array(doublet_actuator) == 3)[0]
            idx5 = np.where(np.array(doublet_actuator) == 5)[0]
            idx6 = np.where(np.array(doublet_actuator) == 6)[0]

            plt.figure('Roll')
            plt.xlabel('doublet id')
            plt.ylabel('pprz_eff')
            plt.ylim(-0.02, 0.02)
            plt.grid()
            plt.scatter(idx0, np.array(eff_roll)[idx0], label="front")
            plt.scatter(idx1, np.array(eff_roll)[idx1], label="right")
            plt.scatter(idx2, np.array(eff_roll)[idx2], label="back")
            plt.scatter(idx3, np.array(eff_roll)[idx3], label="left")
            plt.scatter(idx6, np.array(eff_roll)[idx6], label="push")
            plt.legend()

            plt.figure('Pitch')
            plt.xlabel('doublet id')
            plt.ylabel('pprz_eff')
            plt.ylim(-0.0025, 0.0025)
            plt.grid()
            plt.scatter(idx0, np.array(eff_pitch)[idx0], label="front")
            plt.scatter(idx1, np.array(eff_pitch)[idx1], label="right")
            plt.scatter(idx2, np.array(eff_pitch)[idx2], label="back")
            plt.scatter(idx3, np.array(eff_pitch)[idx3], label="left")
            plt.scatter(idx5, np.array(eff_pitch)[idx5], label="elevator")
            plt.scatter(idx6, np.array(eff_pitch)[idx6], label="push")
            plt.legend()

            plt.figure('Yaw')
            plt.xlabel('doublet id')
            plt.ylabel('pprz_eff')
            plt.ylim(-0.001, 0.001)
            plt.grid()
            plt.scatter(idx0, np.array(eff_yaw)[idx0], label="front")
            plt.scatter(idx1, np.array(eff_yaw)[idx1], label="right")
            plt.scatter(idx2, np.array(eff_yaw)[idx2], label="back")
            plt.scatter(idx3, np.array(eff_yaw)[idx3], label="left")
            plt.scatter(idx6, np.array(eff_roll)[idx6], label="push")
            plt.legend()

            plt.figure('acc_z')
            plt.xlabel('doublet id')
            plt.ylabel('pprz_eff')
            #plt.ylim(-0.001, 0.001)
            plt.grid()
            plt.scatter(idx0, np.array(eff_acc_z)[idx0], label="front")
            plt.scatter(idx1, np.array(eff_acc_z)[idx1], label="right")
            plt.scatter(idx2, np.array(eff_acc_z)[idx2], label="back")
            plt.scatter(idx3, np.array(eff_acc_z)[idx3], label="left")
            plt.scatter(idx6, np.array(eff_acc_z)[idx6], label="push")
            plt.legend()

            # Fit roll effectiveness on airspeed
            A_roll_as_idx0 = np.append([np.array(airspeed_doublet)[idx0]], [np.ones(len(np.array(airspeed_doublet)[idx0]))], axis = 0).T
            A_roll_as_idx1 = np.append([np.array(airspeed_doublet)[idx1]], [np.ones(len(np.array(airspeed_doublet)[idx1]))], axis = 0).T
            A_roll_as_idx2 = np.append([np.array(airspeed_doublet)[idx2]], [np.ones(len(np.array(airspeed_doublet)[idx2]))], axis = 0).T
            A_roll_as_idx3 = np.append([np.array(airspeed_doublet)[idx3]], [np.ones(len(np.array(airspeed_doublet)[idx3]))], axis = 0).T

            roll_coef_as_idx0 = np.linalg.lstsq(A_roll_as_idx0, np.array(eff_roll)[idx0], rcond=-1)[0]
            A_roll_as_idx2 = np.append([np.array(airspeed_doublet)[idx1]], [np.ones(len(np.array(airspeed_doublet)[idx1]))], axis = 0).T
            roll_coef_as_idx1 = np.linalg.lstsq(A_roll_as_idx1, np.array(eff_roll)[idx1], rcond=-1)[0]
            A_roll_as_idx2 = np.append([np.array(airspeed_doublet)[idx2]], [np.ones(len(np.array(airspeed_doublet)[idx2]))], axis = 0).T
            roll_coef_as_idx2 = np.linalg.lstsq(A_roll_as_idx2, np.array(eff_roll)[idx2], rcond=-1)[0]
            A_roll_as_idx3 = np.append([np.array(airspeed_doublet)[idx3]], [np.ones(len(np.array(airspeed_doublet)[idx3]))], axis = 0).T
            roll_coef_as_idx3 = np.linalg.lstsq(A_roll_as_idx3, np.array(eff_roll)[idx3], rcond=-1)[0]


            plt.figure('roll eff vs airspeed')
            plt.xlabel('airspeed [m/s]')
            plt.ylabel('pprz effectiveness')
            plt.grid()
            plt.scatter(np.array(airspeed_doublet)[idx0], np.array(eff_roll)[idx0], label="front")
            plt.scatter(np.array(airspeed_doublet)[idx1], np.array(eff_roll)[idx1], label="right")
            plt.scatter(np.array(airspeed_doublet)[idx2], np.array(eff_roll)[idx2], label="back")
            plt.scatter(np.array(airspeed_doublet)[idx3], np.array(eff_roll)[idx3], label="left")
            plt.plot([0, 6], [roll_coef_as_idx3[1], roll_coef_as_idx3[1] + 6*roll_coef_as_idx3[0]])
            plt.legend()
            plt.figure('pitch eff vs airspeed')
            plt.xlabel('airspeed [m/s]')
            plt.ylabel('pprz effectiveness')
            plt.grid()
            plt.scatter(np.array(airspeed_doublet)[idx0], np.array(eff_pitch)[idx0], label="front")
            plt.scatter(np.array(airspeed_doublet)[idx1], np.array(eff_pitch)[idx1], label="right")
            plt.scatter(np.array(airspeed_doublet)[idx2], np.array(eff_pitch)[idx2], label="back")
            plt.scatter(np.array(airspeed_doublet)[idx3], np.array(eff_pitch)[idx3], label="left")
            plt.scatter(np.array(airspeed_doublet)[idx5], np.array(eff_pitch)[idx5], label="elevator")
            plt.legend()

            plt.figure('airspeed')
            plt.xlabel('t')
            plt.ylabel('airspeed [m/s]')
            plt.grid()
            plt.plot(t_airdata, airspeed)
            plt.plot(t_airdata, airspeed_filtered)
            plt.show()
