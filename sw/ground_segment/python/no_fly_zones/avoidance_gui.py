# -*- coding: utf-8 -*-
"""
Created on Sat Aug 18 15:32:12 2018

@author: dennis
"""

import Tkinter as tk
import threading



class AvoidanceGUI(threading.Thread):
    def __init__(self, detm, resm, cas, reduced_tla_step, extended_tla, reduced_tla_margin, max_tla, reduced_resolution_margin, min_avoid_leg_time):
        threading.Thread.__init__(self)
        self.detm_init = detm
        self.resm_init = resm
        self.reduced_resolution_margin_init = reduced_resolution_margin
        self.cas_init = cas
        self.reduced_tla_step_init = reduced_tla_step
        self.extended_tla_init = extended_tla
        self.reduced_tla_margin_init = reduced_tla_margin
        self.max_tla_init = max_tla
        self.min_avoid_leg_time_init = min_avoid_leg_time
        self.resend = False

    def callback(self):
        self.root.quit()
        
    def set_resend(self):
        self.resend = True
    
    def clear_resend(self):
        self.resend = False
        
    def run(self):
        self.master = tk.Tk()
        self.master.title("Avoidance Settings")
        
        # Detection Margin
        self.w_detm_slider = tk.Scale(self.master, from_=0., to=300., orient=tk.HORIZONTAL)
        self.w_detm_slider.set(self.detm_init)
        self.w_detm_slider.pack()
        self.w_detm_label = tk.Label(self.master, text="Detection Margin [m]")
        self.w_detm_label.pack()
        
        # Reduced resolution margin
        self.w_reduced_resolution_margin_slider = tk.Scale(self.master, from_=0, to=300., orient=tk.HORIZONTAL)
        self.w_reduced_resolution_margin_slider.set(self.reduced_resolution_margin_init)
        self.w_reduced_resolution_margin_slider.pack()
        self.w_reduced_resolution_margin_label = tk.Label(self.master, text="Reduced Resolution Margin [m]").pack()
        
        # Resolution margin
        self.w_resm_slider = tk.Scale(self.master, from_=0., to=300., orient=tk.HORIZONTAL)
        self.w_resm_slider.set(self.resm_init)
        self.w_resm_slider.pack()
        self.w_resm_label = tk.Label(self.master, text="Resolution Margin [m]")
        self.w_resm_label.pack()
        
        # Commanced airspeed
        self.w_cas_slider = tk.Scale(self.master, from_=0., to=50., orient=tk.HORIZONTAL)
        self.w_cas_slider.set(self.cas_init)
        self.w_cas_slider.pack()
        self.w_cas_label = tk.Label(self.master, text="Commanded Airspeed [m/s]").pack()
        
        # min reduced_tla
        self.w_reduced_tla_step_slider = tk.Scale(self.master, from_=0.5, to=10., orient=tk.HORIZONTAL, resolution = 0.5)
        self.w_reduced_tla_step_slider.set(self.reduced_tla_step_init)
        self.w_reduced_tla_step_slider.pack()
        self.w_reduced_tla_step_label = tk.Label(self.master, text="Reduced TLA Step [s]").pack()
        
        # extended tla
        self.w_extended_tla_slider = tk.Scale(self.master, from_=0., to=120., orient=tk.HORIZONTAL)
        self.w_extended_tla_slider.set(self.extended_tla_init)
        self.w_extended_tla_slider.pack()
        self.w_extended_tla_label = tk.Label(self.master, text="Extended TLA [s]").pack()
        
        #reduced tla margin
        self.w_reduced_tla_margin_slider = tk.Scale(self.master, from_=0., to =60., orient=tk.HORIZONTAL)
        self.w_reduced_tla_margin_slider.set(self.reduced_tla_margin_init)
        self.w_reduced_tla_margin_slider.pack()
        self.w_reduced_tla_margin_label = tk.Label(self.master, text="Reduced TLA margin [s]").pack()
        
        # max tla
        self.w_max_tla_slider = tk.Scale(self.master, from_=0., to=240., orient=tk.HORIZONTAL)
        self.w_max_tla_slider.set(self.max_tla_init)
        self.w_max_tla_slider.pack()
        self.w_max_tla_label = tk.Label(self.master, text="Maximum lookahead time [s]").pack()
        
        # min avoid leg time
        self.w_min_avoid_leg_time_slider = tk.Scale(self.master, from_=0., to=60., orient=tk.HORIZONTAL)
        self.w_min_avoid_leg_time_slider.set(self.min_avoid_leg_time_init)
        self.w_min_avoid_leg_time_slider.pack()
        self.w_min_avoid_leg_time_label = tk.Label(self.master, text="Minimal Avoidance Leg Time [s]").pack()
        
        
        
        
        self.w_resend_button = tk.Button(self.master, text='Resend Mission', command=self.set_resend).pack()
        
        self.master.mainloop()