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
import array
from cStringIO import StringIO

PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 850.0
HEIGHT = 750.0


class DelftaCopterFrame(wx.Frame):

    def count_distance(self):
        dx = self.north - self.track_lastnorth
        dy = self.east - self.track_lasteast
        self.track_lastnorth = self.north
        self.track_lasteast = self.east
        self.track_distance = self.track_distance + math.sqrt(dx*dx+dy*dy)

    def message_recv(self, ac_id, msg):
        if msg.name =="ROTORCRAFT_FP_MIN":
            self.gspeed = round(float(msg['gspeed']) / 100.0 * 3.6 / 1.852,1)
            self.alt = round(float(msg['up']) * 0.0039063 * 3.28084 ,1)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_FP":
            self.east = float(msg['east']) * 0.0039063
            self.north = float(msg['north']) * 0.0039063
            self.count_distance()  # do not count_distance in 2 messages: use single source
            self.alt = round(float(msg['up']) * 0.0039063 ,1)
            self.alt_sp = round(float(msg['carrot_up']) * 0.0039063 ,1)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="AIR_DATA":
            self.airspeed = round(float(msg['airspeed']),1)
            self.amsl = round(float(msg['amsl_baro']) * 3.28084,1)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="ROTORCRAFT_NAV_STATUS":
            self.range = round(float(msg['dist_home']),1)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="TEMP_ADC":
            self.motor_temp = round(float(msg['temp1']) ,1)
            self.batt_temp = round(float(msg['temp2']) ,1)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="THROTTLE_CURVE":
            self.throttle = round(float(msg['throttle']) )
            self.rpm = round(float(msg['rpm_meas']) )
            self.toggle()
            wx.CallAfter(self.update)

        elif msg.name =="HYBRID_GUIDANCE":
            self.sideslip = round(float(msg['beta']),2)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="GPS_INT":
            self.gps_acc = round(float(msg['pacc'])/100.0 , 1)
            self.gps_fix = round(float(msg['fix']) )
            self.gps_sv = round(float(msg['numsv']) )
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name =="VISION_OUTBACK":
            self.vision_status = round(float(msg['status']) )
            self.vision_marker_x = round(float(msg['marker_enu_x']) ,1)
            self.vision_marker_y = round(float(msg['marker_enu_y']) ,1)
            self.vision_height = round(float(msg['height']) , 1)
            self.vision_version = round(float(msg['Version']) , 2)
            self.toggle()
            wx.CallAfter(self.update)
        elif msg.name == "TRIM":
            self.trim_pitch = int(float(msg['trim_pitch']))
            self.trim_roll = int(float(msg['trim_roll']))
            self.toggle()
            wx.CallAfter(self.update)

    def toggle(self):
        self.link_toggle = 1 - self.link_toggle

    def update(self):
        self.Refresh()

    def OnSize(self, event):
        self.w = event.GetSize().x
        self.h = event.GetSize().y
        self.cfg.Write("width", str(self.w));
        self.cfg.Write("height", str(self.h));
        self.Refresh()

    def OnMove(self, event):
        self.x = event.GetPosition().x
        self.y = event.GetPosition().y
        self.cfg.Write("left", str(self.x));
        self.cfg.Write("top", str(self.y));

    def StatusBox(self, dc, nr, txt, percent, color):
        if percent < 0:
            percent = 0
        if percent > 1:
            percent = 1

        tdx = int(self.stat * 0.1)
        tdy = int(self.stat * 0.1)
        boxw = self.stat - 2*tdx
        boxh = boxw

        dc.SetPen(wx.Pen(wx.Colour(0,0,0))) 
	dc.SetBrush(wx.Brush(wx.Colour(220,220,220))) 
        dc.DrawRectangle(tdx, int(nr*self.stat+tdx), int(boxw), int(boxh)) 
        if color < -0.2:
            dc.SetBrush(wx.Brush(wx.Colour(190,190,190)))
        elif color < 0.2:
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx, int(nr*self.stat+tdx), int(boxw * percent), int(boxh)) 
        dc.DrawText(txt,18,int(nr*self.stat+tdy+tdx)) 

    def motor_color(self):
        if self.motor_temp < 0:
            return -1
        if (self.motor_temp > 90):
            return 0
        elif (self.motor_temp > 60):
            return 0.1
        return 1

    def gps_color(self):
        if (self.gps_fix < 0):
            return -1
        if (self.gps_fix < 3):
            return 0
        if (self.gps_acc > 10):
            return 0
        elif (self.gps_acc > 3):
            return 0.5
        return 1

    def airspeed_color(self):
        if (self.airspeed < 9):
            return -1
        if (self.airspeed < 19):
            return 0
        elif (self.airspeed < 21):
            return 0.5
        return 1

    def trim_color(self):
        if (self.trim_pitch == 9999):
            return -1
        elif (self.trim_pitch > 1500) | (self.trim_pitch < -500):
            return 0
        elif (self.trim_pitch > 1000) | (self.trim_pitch < 0):
            return 0.5
        return 1

    def alt_color(self):
        if self.alt_sp < 0:
            return -1
        dh = self.alt_sp - self.alt
        if dh < 0:
            dh = -dh
        if dh > 15:
            return 0
        elif dh > 5:
            return 0.5
        return 1

    def sideslip_color(self):
        if self.sideslip > 900:
            return -1
        ss = abs( self.sideslip )
        if ss > 1.5:
            return 0
        elif ss > 0.5:
            return 0.5
        return 1

    def get_vision_status(self):
        if self.vision_status == 0:
            return "On"
        elif self.vision_status == 2:
            return "Version Error!"
        elif self.vision_status == 1:
            return "Off"
        return "?"

    def vision_color(self):
        if self.vision_status == 0:
            return 1.0
        elif self.vision_status == 2:
            return 0.0
        return -1

    def range_color(self):
        if self.range > 10000:
            return 0
        elif self.range > 5000:
            return 0.5
        elif self.range >= 0:
            return  1
        return -1

    def OnPaint(self, e):

        w = self.w
        h = self.h

        if (float(w)/float(h)) > (WIDTH/HEIGHT):
          w = int(h * WIDTH/HEIGHT)
        else:
          h = int(w * HEIGHT/WIDTH)

	tdy = int(w * 75.0 / WIDTH)
        tdx = int(w * 15.0 / WIDTH)

        self.stat = tdy
        
        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

	fontscale = int(w * 40.0 / WIDTH)
        if fontscale < 6:
            fontscale = 6

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0,0,0), wx.TRANSPARENT))
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)

        dc.DrawText("Airspeed: " + str(self.airspeed) + " m/s",self.stat+tdx,tdx)
        self.StatusBox(dc,0,"",1.0, self.airspeed_color())

        dc.DrawText("RPM: " + str(self.rpm) + "",self.stat+tdx,tdx+tdy*1)

        dc.DrawText("Motor Temp: " + str(self.motor_temp) + "",self.stat+tdx,tdx+tdy*2)
        self.StatusBox(dc,2,"",1.0, self.motor_color())

        dc.DrawText("Link Range: " + str(self.range / 1000.0) + " km",self.stat+tdx,tdx+tdy*3)
        self.StatusBox(dc,3,"",1.0, self.range_color())

        dc.DrawText("SideSlip: {:.2f}".format(self.sideslip),self.stat+tdx,tdx+tdy*4)
        self.StatusBox(dc,4,"",1.0, self.sideslip_color())

        dc.DrawText("Vision: " + self.get_vision_status() + " " + str(self.vision_height) + "m " + str(self.vision_marker_x) + "," + str(self.vision_marker_y) + " v" + str(self.vision_version) ,self.stat+tdx,tdx+tdy*5)
        self.StatusBox(dc,5,"",1.0, self.vision_color())


        dc.DrawText("DeltaH: " + str(self.alt_sp - self.alt)  ,self.stat+tdx,tdx+tdy*6)
        self.StatusBox(dc,6,"",1.0, self.alt_color())
        dc.DrawText("GPS-ACC: " + str(int(self.gps_fix)) + ", #" + str(int(self.gps_sv)) + ", " + str(self.gps_acc) + "m" ,self.stat+tdx,tdx+tdy*7)
        self.StatusBox(dc,7,"",1.0, self.gps_color())
        
        dc.DrawText("Trim elev {} ail {}".format(self.trim_pitch, self.trim_roll), self.stat+tdx, tdx+tdy*8)
        self.StatusBox(dc,8,"",1.0, self.trim_color())

        dc.DrawText("Flown Dist: " + str(round(self.track_distance / 1000.0,1)) + " km",self.stat+tdx,tdx+tdy*9)

        #c = wx.Colour(0,0,0)
        #dc.SetBrush(wx.Brush(c, wx.SOLID))
        #dc.DrawCircle(int(w/2),int(w/2),10)
        if self.link_toggle:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
            dc.DrawRectangle(0, self.h-5, self.w, 5)




    def __init__(self):

        self.w = int(WIDTH)
        self.h = int(HEIGHT)

        self.motor_temp = -1
        self.batt_temp = -1
        self.rpm = -1

        self.link_toggle = 0
        self.airspeed = -1
        self.sideslip = 999

        self.vision_status = -1
        self.vision_marker_x = -1
        self.vision_marker_y = -1
        self.vision_height = -1
        self.vision_version =-1

        self.range = -1
        self.alt = 0
        self.alt_sp = -1
        self.gps_acc = -1
        self.gps_fix = -1
        self.gps_sv = -1
        self.trim_pitch = 9999
        self.trim_roll = 9999

	self.track_distance = 0;
        self.track_lastnorth = 0;
        self.track_lasteast = 0;

        self.cfg = wx.Config('delftacopter_conf')
        if self.cfg.Exists('width'):
            self.w = int(self.cfg.Read('width'))
            self.h = int(self.cfg.Read('height'))


        wx.Frame.__init__(self, id=-1, parent=None, name=u'DelftaCopter Center',
                          size=wx.Size(self.w, self.h), title=u'DelftaCopter Center')

        if self.cfg.Exists('left'):
            self.x = int(self.cfg.Read('left'))
            self.y = int(self.cfg.Read('top'))
            self.SetPosition(wx.Point(self.x,self.y), wx.SIZE_USE_EXISTING)


        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/delftacopter/dc.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.interface = IvyMessagesInterface("DelftaCopter")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()
