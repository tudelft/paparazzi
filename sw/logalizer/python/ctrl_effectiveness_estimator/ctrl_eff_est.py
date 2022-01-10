import numpy as np
import scipy as sp
from scipy import signal
import matplotlib.pyplot as plt

import sys
from os import path, getenv

class CtrlEffEst(object):
    def __init__(self, parsed_log, first_order_dyn_list, second_order_filter_cutoff, min_max_pwm_list, is_servo, actuator_indices = None, fs=500.):
        '''
        Initialization function of control effectiveness estimator
        '''
        self.data = parsed_log.get_message_dict('STAB_ATTITUDE_FULL_INDI')

        # Determine number of actuators from data
        if actuator_indices == None:
            self.N_act = len(self.data['u']['data'][0])
            self.actuator_indices = range(self.N_act)
        else:
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
        t = np.array(self.data['t'])
        pwm = np.array(self.data['u']['data'])[:,self.actuator_indices]
        airspeed = np.array(self.data['airspeed']['data'])
        # Convert pwm to command
        min_pwm_array = np.array(self.min_max_pwm_list).T[0]
        max_pwm_array = np.array(self.min_max_pwm_list).T[1]
        pwm_range = max_pwm_array - min_pwm_array
        cmd = (pwm - min_pwm_array) / pwm_range * 9600.

        rate_p = self.data['angular_rate_p']['data'] # rad/s²
        rate_q = self.data['angular_rate_q']['data'] # rad/s²
        rate_r = self.data['angular_rate_r']['data'] # rad/s²
        rates = np.array([rate_p,rate_q,rate_r]).T
        acc_z = self.data['body_accel_z'] # m/s²

        # Apply actuator dynamics to commands
        for i in range(self.N_act):
            zi = sp.signal.lfilter_zi([self.first_order_dyn_list[i]], [1, -(1-self.first_order_dyn_list[i])])
            filtered_cmd = sp.signal.lfilter([self.first_order_dyn_list[i]], [1, -(1-self.first_order_dyn_list[i])], cmd[:,i], zi=zi*cmd[:,i][0])[0]
            if i == 0:
                cmd_a_T = np.array([filtered_cmd]) # Transpose of cmd_a
            else:
                cmd_a_T = np.vstack((cmd_a_T, filtered_cmd))

        cmd_a = cmd_a_T.T

        # 2nd order bUtterworth noise filter
        b, a = sp.signal.butter(2, self.second_order_filter_cutoff/(self.fs/2), 'low', analog=False)

        # Filter signals and cmds
        zi = np.array([sp.signal.lfilter_zi(b, a)]).T
        rates_f = sp.signal.lfilter(b, a, rates, axis=0)
        cmd_af = sp.signal.lfilter(b, a, cmd_a, axis=0)
        airspeed_f = sp.signal.lfilter(b, a, airspeed, axis=0)

        # Apply finite difference methods to get anfular accelerarions
        d_rates = (np.vstack((np.zeros((1,3)), np.diff(rates_f,1,axis=0)))*self.fs)
        dd_rates = (np.vstack((np.zeros((1,3)), np.diff(d_rates,1,axis=0)))*self.fs)

        d_cmd= (np.vstack((np.zeros((1,self.N_act)), np.diff(cmd_af,1,axis=0)))*self.fs)
        dd_cmd = (np.vstack((np.zeros((1,self.N_act)), np.diff(d_cmd,1,axis=0)))*self.fs)

        t = t
        
        # Construct A matrix
        for i in range(len(self.actuator_indices)):
            if self.is_servo[i]:
                row = d_cmd[:,i] * airspeed_f**2
            else:
                row = d_cmd[:,i]
            
            if i == 0:
                A = row
            else:
                A = np.vstack((A, row))

        A = A.T

        # Remove first 2 seconds to align filters
        A = A[int(self.fs):-1]
        dd_rates_sliced = dd_rates[int(self.fs):-1]

        # Perform LMS to get effectiveness values per actuator per axis
        g1_lstsq = np.linalg.lstsq(A,dd_rates_sliced, rcond=0)
        g1_matrix = g1_lstsq[0]
        g1_residuals = g1_lstsq[1]
        print('g1_matrix: ', g1_matrix)
        print('g1_residuals: ', g1_residuals)

        if plot:
            plt.figure('roll d_cmd*g1')
            for i in self.actuator_indices:
                plt.subplot(self.N_act, 1, i+1)
                plt.plot(t, dd_rates.T[0])
                plt.plot(t, d_cmd.T[i] * g1_matrix[i][0])

            plt.figure('pitch d_cmd*g1')
            for i in self.actuator_indices:
                plt.subplot(self.N_act, 1, i+1)
                plt.plot(t, dd_rates.T[1])
                plt.plot(t, d_cmd.T[i] * g1_matrix[i][1])

            plt.figure('yaw d_cmd*g1')
            for i in self.actuator_indices:
                plt.subplot(self.N_act, 1, i+1)
                plt.plot(t, dd_rates.T[2])
                plt.plot(t, d_cmd.T[i] * g1_matrix[i][2])

            # Debug plots
            #plt.figure('Test')
            #plt.plot(t, d_cmd[:,0])
            #plt.plot(t, cmd_a[:,0])
            #plt.plot(t, cmd_af[:,0])
            #plt.figure('TEST2')
            #plt.plot(t, rates.T[0])
            #plt.plot(t, rates_f.T[0])
            #plt.plot(t, cmd_a.T[0])
            #plt.plot(t, cmd_af.T[0])
            plt.show()