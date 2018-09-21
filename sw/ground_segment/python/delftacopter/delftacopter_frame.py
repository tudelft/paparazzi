#
# Copyright (C) 2016 TUDelft
#
# This file is part of paparazzi.
#
# paparazzi is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi.  If not, see <http://www.gnu.org/licenses/>.
#

import wx
import sys
import os
import time
import threading
import math
import pynotify
import array
from cStringIO import StringIO

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 850.0
HEIGHT = 700.0


class DelftaCopterFrame(wx.Frame):

    def message_recv(self, ac_id, msg):
        if msg.name =="ROTORCRAFT_FP_MIN":
            self.gspeed = round(float(msg['gspeed']) / 100.0 * 3.6 / 1.852,1)
            self.alt = round(float(msg['up']) * 0.0039063 * 3.28084 ,1)
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_FP":
            self.alt = round(float(msg['up']) * 0.0039063 ,1)
            self.alt_sp = round(float(msg['carrot_up']) * 0.0039063 ,1)
            wx.CallAfter(self.update)
        elif msg.name =="AIR_DATA":
            self.airspeed = round(float(msg['airspeed']),1)
            self.amsl = round(float(msg['amsl_baro']) * 3.28084,1)
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_NAV_STATUS":
            self.range = round(float(msg['dist_home']),1)
            wx.CallAfter(self.update)
        elif msg.name =="TEMP_ADC":
            self.motor_temp = round(float(msg['temp1']) )
            self.batt_temp = round(float(msg['temp2']) )
            wx.CallAfter(self.update)
        elif msg.name =="THROTTLE_CURVE":
            self.throttle = round(float(msg['throttle']) )
            self.rpm = round(float(msg['rpm_meas']) )
            wx.CallAfter(self.update)

        elif msg.name =="HYBRID_GUIDANCE":
            self.sideslip = round(float(msg['beta']) )
            wx.CallAfter(self.update)
        elif msg.name =="GPS_INT":
            self.gps_acc = round(float(msg['pacc'])/100.0 )
            self.gps_fix = round(float(msg['fix']) )
            self.gps_sv = round(float(msg['numsv']) )
            wx.CallAfter(self.update)
        elif msg.name =="VISION_OUTBACK":
            self.vision_status = round(float(msg['status']) )
            self.vision_marker_x = round(float(msg['marker_enu_x']) )
            self.vision_marker_y = round(float(msg['marker_enu_y']) )
            self.vision_height = round(float(msg['height']) )
            wx.CallAfter(self.update)

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize()[0]
        self.h = event.GetSize()[1]
        self.Refresh()

    def StatusBox(self, dc, nr, txt, percent, color):
        if percent < 0:
            percent = 0
        if percent > 1:
            percent = 1
        boxw = self.stat
        tdx = int(boxw * 10.0 / 300.0)
        tdy = int(boxw * 6.0 / 300.0)
        boxh = int(boxw * 40.0 / 300.0)
        boxw = self.stat - 2*tdx
        spacing = boxh+10

        dc.SetPen(wx.Pen(wx.Colour(0,0,0))) 
	dc.SetBrush(wx.Brush(wx.Colour(220,220,220))) 
        dc.DrawRectangle(tdx, int(nr*spacing+tdx), int(boxw), boxh) 
        if color < 0.2:
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx, int(nr*spacing+tdx), int(boxw * percent), boxh) 
        dc.DrawText(txt,18,int(nr*spacing+tdy+tdx)) 


    def OnPaint(self, e):

        w = self.w
        h = self.h

        stat = int(100/WIDTH)*(float(w))

        if (float(w)/float(h)) > (WIDTH/HEIGHT):
          w = int(h * WIDTH/HEIGHT)
        else:
          h = int(w * HEIGHT/WIDTH)

	tdy = int(w * 75.0 / WIDTH)
        tdx = int(w * 15.0 / WIDTH)
        
        dc = wx.PaintDC(self)
        #brush = wx.Brush("white")
        #dc.SetBackground(brush)
        #dc.Clear()

	fontscale = int(w * 40.0 / WIDTH)
        if fontscale < 6:
            fontscale = 6

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        #dc.DrawCircle(w/2,w/2,w/2-1)
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)
        dc.DrawText("Airspeed: " + str(self.airspeed) + " m/s",tdx,tdx)
        dc.DrawText("RPM: " + str(self.rpm) + "",tdx,tdx+tdy*1)

        dc.DrawText("Motor Temp: " + str(self.motor_temp) + "",tdx,tdx+tdy*2)
        dc.DrawText("Batt Temp: " + str(self.batt_temp) + "",tdx,tdx+tdy*3)

        dc.DrawText("SideSlip: " + str(self.sideslip) + "",tdx,tdx+tdy*4)
        dc.DrawText("Vision: " + str(int(self.vision_status)) + ", " + str(self.vision_height) + "m " + str(self.vision_marker_x) + "," + str(self.vision_marker_y) ,tdx,tdx+tdy*5)


        dc.DrawText("DeltaH: " + str(self.alt_sp - self.alt)  ,tdx,tdx+tdy*6)
        dc.DrawText("GPS-ACC: " + str(int(self.gps_fix)) + ", #" + str(int(self.gps_sv)) + ", " + str(self.gps_acc) + "m" ,tdx,tdx+tdy*7)
        #dc.DrawText("HMSL: " + str(self.hmsl) + " ft",tdx,tdx+tdy*6)

        #c = wx.Colour(0,0,0)
        #dc.SetBrush(wx.Brush(c, wx.SOLID))
        #dc.DrawCircle(int(w/2),int(w/2),10)



    def __init__(self):

        self.w = int(WIDTH)
        self.h = int(HEIGHT)

        self.motor_temp = -1
        self.batt_temp = -1
        self.rpm = -1

        self.airspeed = -1

        self.sideslip = -1

        self.vision_status = -1
        self.vision_marker_x = -1
        self.vision_marker_y = -1
        self.vision_height = -1

        self.range = -1
        self.alt = -1
        self.alt_sp = -1
        self.gps_acc = -1
        self.gps_fix = -1
        self.gps_sv = -1

        wx.Frame.__init__(self, id=-1, parent=None, name=u'DelftaCopter Center',
                          size=wx.Size(self.w, self.h), title=u'DelftaCopter Center')


        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/delftacopter/dc.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.interface = IvyMessagesInterface("DelftaCopter")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
