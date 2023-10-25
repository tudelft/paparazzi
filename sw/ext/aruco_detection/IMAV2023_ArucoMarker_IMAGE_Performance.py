# -----------------------------------------------------------------------------------------
# Author:           Kevin Malkow
# Date:             04/07/23
# Affiliation:      TU Delft, IMAV 2023
#
# Version:          1.0 
# 
# Description:  
# Evaluate performance of Aruco marker detector for different heights and scaling values.
# -----------------------------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np

# -------------- Data --------------
# For 13m height:
tp_NO_SCALING_13 = 22
fn_NO_SCALING_13 = 24
fp_NO_SCALING_13 = 1

tp_30_SCALING_13 = 26
fn_30_SCALING_13 = 20
fp_30_SCALING_13 = 1

tp_90_SCALING_13 = 29
fn_90_SCALING_13 = 18
fp_90_SCALING_13 = 0

# For 9m height:
tp_NO_SCALING_9 = 18
fn_NO_SCALING_9 = 6
fp_NO_SCALING_9 = 0

tp_30_SCALING_9 = 24
fn_30_SCALING_9 = 0
fp_30_SCALING_9 = 0

tp_90_SCALING_9 = 23
fn_90_SCALING_9 = 1
fp_90_SCALING_9 = 0

# For 4.4m height:
tp_NO_SCALING_4_4 = 16
fn_NO_SCALING_4_4 = 6
fp_NO_SCALING_4_4 = 0

tp_30_SCALING_4_4 = 16
fn_30_SCALING_4_4 = 6
fp_30_SCALING_4_4 = 0

tp_90_SCALING_4_4 = 18
fn_90_SCALING_4_4 = 4
fp_90_SCALING_4_4 = 0

# -------------- Computation --------------
# Compute Precision
# For 13m height:
precision_NO_SCALING_13 =  tp_NO_SCALING_13/(tp_NO_SCALING_13 + fp_NO_SCALING_13)*100
precision_30_SCALING_13 =  tp_30_SCALING_13/(tp_30_SCALING_13 + fp_30_SCALING_13)*100
precision_90_SCALING_13 =  tp_90_SCALING_13/(tp_90_SCALING_13 + fp_90_SCALING_13)*100

# For 9m height:
precision_NO_SCALING_9 =  tp_NO_SCALING_9/(tp_NO_SCALING_9 + fp_NO_SCALING_9)*100
precision_30_SCALING_9 =  tp_30_SCALING_9/(tp_30_SCALING_9 + fp_30_SCALING_9)*100
precision_90_SCALING_9 =  tp_90_SCALING_9/(tp_90_SCALING_9 + fp_90_SCALING_9)*100

# For 4.4m height:
precision_NO_SCALING_4_4 =  tp_NO_SCALING_4_4/(tp_NO_SCALING_4_4 + fp_NO_SCALING_4_4)*100
precision_30_SCALING_4_4 =  tp_30_SCALING_4_4/(tp_30_SCALING_4_4 + fp_30_SCALING_4_4)*100
precision_90_SCALING_4_4 =  tp_90_SCALING_4_4/(tp_90_SCALING_4_4 + fp_90_SCALING_4_4)*100

# Compute Recall
# For 13m height:
recall_NO_SCALING_13 = tp_NO_SCALING_13/(tp_NO_SCALING_13 + fn_NO_SCALING_13)*100
recall_30_SCALING_13 = tp_30_SCALING_13/(tp_30_SCALING_13 + fn_30_SCALING_13)*100
recall_90_SCALING_13 = tp_90_SCALING_13/(tp_90_SCALING_13 + fn_90_SCALING_13)*100

# For 9m height:
recall_NO_SCALING_9 = tp_NO_SCALING_9/(tp_NO_SCALING_9 + fn_NO_SCALING_9)*100
recall_30_SCALING_9 = tp_30_SCALING_9/(tp_30_SCALING_9 + fn_30_SCALING_9)*100
recall_90_SCALING_9 = tp_90_SCALING_9/(tp_90_SCALING_9 + fn_90_SCALING_9)*100

# For 4.4m height:
recall_NO_SCALING_4_4 = tp_NO_SCALING_4_4/(tp_NO_SCALING_4_4 + fn_NO_SCALING_4_4)*100
recall_30_SCALING_4_4 = tp_30_SCALING_4_4/(tp_30_SCALING_4_4 + fn_30_SCALING_4_4)*100
recall_90_SCALING_4_4 = tp_90_SCALING_4_4/(tp_90_SCALING_4_4 + fn_90_SCALING_4_4)*100

# -------------- Plotting --------------
# Plotting the points 

# plt.figure()

# ax_1 = plt.plot(recall_NO_SCALING_13, precision_NO_SCALING_13, marker='o')
# ax_2 = plt.plot(recall_NO_SCALING_9, precision_NO_SCALING_9, marker='o')
# ax_3 = plt.plot(recall_NO_SCALING_4_4, precision_NO_SCALING_4_4, marker='o')

# ax_1.legend([13], ['Height [m]'])

# plt.xlim([0, 105])
# plt.ylim([0, 105])

# plt.xlabel('Recall [%]')
# plt.ylabel('Precision[%]')
# plt.title('Precision-Recall Plot')
  
# plt.show()

# plt.cla() 
# plt.clf() 



