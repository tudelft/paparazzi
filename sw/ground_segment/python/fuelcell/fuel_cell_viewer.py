#
# Copyright (C) 2018 TUDelft
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
import math
import datetime


PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

PPRZ_SRC = os.getenv("PAPARAZZI_SRC", os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                    '../../../..')))

sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface

WIDTH = 600
BARH = 140


def get_text_from_seconds(secs):
    m, s = divmod(int(secs), int(60))
    return "{:02d}:{:02d}".format(m, s)

class AirDataMessage(object):
    def __init__(self, msg):
        self.airspeed = float(msg['airspeed'])

class EnergyMessage(object):
    def __init__(self, msg):
        self.volt = float(msg['voltage'])
        self.current = float(msg['current'])*2.0
        self.power = float(msg['power'])
        self.energy = float(msg['energy'])

class TempMessage(object):
    def __init__(self, msg):
        self.motor = float(msg['temp1'])
        self.battery = float(msg['temp2'])

class EscMessage(object):
    def __init__(self, msg):
        self.id = float(msg['motor_id'])
        self.amp = float(msg['amps'])
        self.rpm = float(msg['rpm'])
        self.volt_b = float(msg['bat_volts'])
        self.volt_m = float(msg['motor_volts'])
        self.power = float(msg['power'])
        self.energy = float(msg['energy'])
    
    def get_current(self):
        return "Mot " + str(self.id) + " " +str(round(self.amp ,1)) + "A"
    def get_current_perc(self):
        return self.amp / 30

    def get_rpm(self):
        return "Mot " + str(self.id) + " " +str(round(self.rpm ,0)) + ""
    def get_rpm_perc(self):
        return self.rpm / 4000


    def get_volt(self):
        return "Mot " + str(self.id) + " " +str(round(self.volt_b ,0)) + "V"
    def get_volt_perc(self):
        return self.volt_b / (6*4.2)

class MotorList(object):
    def __init__(self):
        self.mot = []

    def fill_from_esc_msg(self, esc):
        added = False
        for i in range(len(self.mot)):
            if self.mot[i].id == esc.id:
                self.mot[i] = esc
                added = True
                break
        if not added:
            self.mot.append(esc)


class BatteryCell(object):
    def __init__(self):
        self.voltage = 0
        self.current = 0
        self.energy = 0
        self.model = 0
        self.temperature = 0
    def fill_from_energy_msg(self, energy):
        self.voltage = energy.volt / 6
        self.current = energy.current / 6
        self.energy  = energy.energy / 6.0
        self.model = 0 #bat.mah_from_volt_and_current(self.voltage,self.current)
    def fill_from_temp_msg(self, temp):
        self.temperature = temp.battery

    def get_volt(self):
        return "Cell Volt = "+str(round(self.voltage,2)) + " V"
    def get_mah_from_volt(self):
        return "Cap(U,I) = "+str(round(self.model/1000.0,2)) + " Ah"
    def get_current(self):
        return "Cell Amps = "+str(round(self.current,2)) + " A"
    def get_energy(self):
        return "Cell mAh  = "+str(round(self.energy/1000.0 ,2)) + " Ah"
    def get_temp(self):
        return "Cell Temp = "+str(round(self.temperature ,2))
    def get_power_text(self):
        return "Battery Power: {:.0f}W".format(self.get_power() * 6)
    def get_volt_perc(self):
        return self.get_volt_percent(self.voltage)
    def get_volt_percent(self,volt):
        return (volt - 2.5) / (4.3 - 2.5)
    def get_power(self):
        return self.voltage * self.current
    def get_power_per_cell(self):
        return self.get_power() / 6 / 6
    def get_temp_perc(self):
        return (self.temperature / 60)
    def get_current_perc(self):
        return (self.current / 10)
    def get_energy_perc(self):
        return (self.energy / 4* 4500 + 30000)
    def get_model_perc(self):
        return (self.model / 4* 4500 + 30000)
    def get_power_perc(self):
        return (self.get_power() - 200) / (800 - 200)

    def get_volt_color(self):
        if self.voltage < 3.2:
            return 0.1
        elif self.voltage < 3.6:
            return 0.5
        return 1

    def get_current_color(self):
        if self.current < 2.5:
            return 1
        elif self.current > 4.5:
            return 0.1
        return 0.5
    
    def get_energy_color(self):
        if self.energy > 3000:
            return 0.1
        elif self.energy < 2000:
            return 1
        return 0.5
  
    def get_temp_color(self):
        if (self.temperature > 20) & (self.temperature < 40):
            return 1
        elif (self.temperature > 10) & (self.temperature < 55):
            return 0.5
        return 0.1

    def get_power_color(self):
        return 0.5


class PayloadMessage(object):
    def __init__(self, msg):
        self.values = ''.join(chr(int(x)) for x in msg['values'])



class FuelCellStatus(object):

    def update(self,msg):
        self.msg = msg
        elements = self.msg.strip('<').strip('>').split(',')
        if (len(elements) == 4):
            self.tank = float(elements[0])
            self.battery = float(elements[1])
            self.status = elements[2]
            self.error = elements[3]

        else:
            print('ERROR: ' + msg)
    def get_tank(self):
        bar = round(5 + self.tank / 100 * 295,1)
        return 'Cylinder ' + str(bar) + ' Bar'
    def get_tank_perc(self):
        return (self.tank) / 100.0
    def get_tank_color(self):
        if (self.tank) < 10:
            return 0.1
        elif int(self.tank) < 20:
            return 0.5
        else:
            return 1
                
    def get_battery(self):
        volt = round( self.battery / 100.0 * (24.0-19.6) + 19.6, 2)
        return str(volt) + ' V'
    def get_battery_perc(self):
        return (self.battery) / 100.0
    def get_battery_color(self):
        if (self.battery) < 10:
            return 0.1
        elif (self.battery) < 20:
            return 0.5
        else:
            return 1
                

    def get_status(self):
        if int(self.status == 0):
            return 'Starting'
        elif int(self.status == 1):
            return 'Ready (wait for button press)'
        elif int(self.status == 2):
            return 'Running'
        elif int(self.status == 3):
            return 'Fault'
        elif int(self.status == 4):
            return 'Battery only'
        else:
            return self.status
    def get_status_perc(self):
        return 0.0
    def get_status_color(self):
        if (self.status) < 1:
            return 0.1
        elif (self.status) == 1:
            return 0.5
        else:
            return 0.5
                

    def get_error(self):
        return self.error
    def get_error_perc(self):
        return 1.0
    def get_error_color(self):
        return 0.1
                


class FuelCellFrame(wx.Frame):
    def message_recv(self, ac_id, msg):
        if msg.name == "ENERGY":
            self.bat = EnergyMessage(msg)
            self.cell.fill_from_energy_msg(self.bat)

            wx.CallAfter(self.update)
        elif msg.name == "TEMP_ADC":
            self.temp = TempMessage(msg)
            self.cell.fill_from_temp_msg(self.temp)
            wx.CallAfter(self.update)

        elif msg.name == "AIR_DATA":
            self.air_data = AirDataMessage(msg)
            wx.CallAfter(self.update)    

        elif msg.name == "ESC":
            self.esc = EscMessage(msg)
            self.motors.fill_from_esc_msg(self.esc)
            wx.CallAfter(self.update)    

        elif msg.name == "PAYLOAD":
            self.payload = PayloadMessage(msg)
            self.fuelcell.update(self.payload.values)
            #print("Payload: " + self.payload.values)


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


    def StatusBox(self, dc, row, col, txt, percent, color):
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
        dc.DrawRectangle(tdx +  col * 200, int(row*spacing+tdx), int(boxw), boxh)
        dc.SetTextForeground(wx.Colour(0, 0, 0))
        if color < 0.2:
            dc.SetTextForeground(wx.Colour(255, 255, 255))
            dc.SetBrush(wx.Brush(wx.Colour(250,0,0)))
        elif color < 0.6:
            dc.SetBrush(wx.Brush(wx.Colour(250,180,0)))
        else:
            dc.SetBrush(wx.Brush(wx.Colour(0,250,0)))
#        dc.DrawLine(200,50,350,50)
        dc.DrawRectangle(tdx +  col * 200, int(row*spacing+tdx), int(boxw * percent), boxh)
        dc.DrawText(txt,18 + col * 200,int(row*spacing+tdy+tdx))

    def plot_x(self, x):
        return int(self.stat+self.tdx +  x * (self.w-self.stat-2*self.tdx))

    def plot_y(self, y):
        return int(self.tdx + (1-y) * (self.h-self.tdx-self.tdx))

    def plot(self, dc, i1, i2):
        dc.DrawLine(self.plot_x(i1[1]/3500), self.plot_y((i1[0]-2.5)/(4.2-2.5)), self.plot_x(i2[1]/3500), self.plot_y((i2[0]-2.5)/(4.2-2.5)))

          

    def OnPaint(self, e):
        # Automatic Scaling
        w = self.w
        h = self.h - 25

        self.stat = int(w/8)
        if self.stat<100:
            self.stat=100

        dc = wx.PaintDC(self)
        brush = wx.Brush("white")
        dc.SetBackground(brush)
        dc.Clear()

        # Background
        dc.SetBrush(wx.Brush(wx.Colour(0, 0, 0), wx.TRANSPARENT))

        fontscale = int(w * 11.0 / 1600.0)
        if fontscale < 6:
            fontscale = 6
        font = wx.Font(fontscale, wx.FONTFAMILY_ROMAN, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD)
        dc.SetFont(font)

	# Draw Drone
        dc.SetPen(wx.Pen(wx.Colour(0,0,0))) 
        dc.SetBrush(wx.Brush(wx.Colour(240,240,220))) 
        # Fuselage
        dc.DrawRoundedRectangle(int(0.35*w), int(0.05*h),int(0.3*w), int(0.9*h), int(0.1*w))
        # Front Wing
        dc.DrawRectangle(int(0.05*w), int(0.25*h),int(0.9*w), int(0.15*h))
        # Back Wing
        dc.DrawRectangle(int(0.05*w), int(0.65*h),int(0.9*w), int(0.15*h))

        dc.SetBrush(wx.Brush(wx.Colour(100,100,100))) 
        dc.DrawRoundedRectangle(int(0.37*w), int(0.07*h),int(0.26*w), int(0.35*h), int(0.13*w))


        self.StatusBox(dc,0, 0, self.cell.get_volt(), self.cell.get_volt_perc(), self.cell.get_volt_color())
        self.StatusBox(dc,1, 0, self.cell.get_current(), self.cell.get_current_perc(), self.cell.get_current_color() )
        self.StatusBox(dc,2, 0, self.cell.get_energy(), self.cell.get_energy_perc(), self.cell.get_energy_color() )
        self.StatusBox(dc,3, 0, self.cell.get_mah_from_volt(), self.cell.get_energy_perc(), self.cell.get_energy_color() )
        self.StatusBox(dc,4, 0, self.cell.get_temp(), self.cell.get_temp_perc(), self.cell.get_temp_color())
        self.StatusBox(dc,5, 0, self.cell.get_power_text(), self.cell.get_power_perc(), self.cell.get_power_color())

        self.StatusBox(dc,0, 2, self.fuelcell.get_tank(), self.fuelcell.get_tank_perc(), self.fuelcell.get_tank_color())
        self.StatusBox(dc,1, 2, self.fuelcell.get_battery(), self.fuelcell.get_battery_perc(), self.fuelcell.get_battery_color())
        self.StatusBox(dc,2, 2, self.fuelcell.get_status(), self.fuelcell.get_status_perc(), self.fuelcell.get_status_color())
        self.StatusBox(dc,3, 2, self.fuelcell.get_error(), self.fuelcell.get_error_perc(), self.fuelcell.get_error_color())


        i = 6
        for m in self.motors.mot:
            self.StatusBox(dc,i, 0, m.get_current(), m.get_current_perc(), 1)
            self.StatusBox(dc,i, 1, m.get_rpm(), m.get_rpm_perc(), 1)
            self.StatusBox(dc,i, 2, m.get_volt(), m.get_volt_perc(), 1)
            i = i + 1



    def __init__(self):

        self.w = WIDTH
        self.h = WIDTH + BARH

        self.cfg = wx.Config('fuel_cell_conf')
        if self.cfg.Exists('width'):
            self.w = int(self.cfg.Read('width'))
            self.h = int(self.cfg.Read('height'))

        wx.Frame.__init__(self, id=-1, parent=None, name=u'FuelCellFrame',
                          size=wx.Size(self.w, self.h), title=u'Fuel Cell Monitoring')

        if self.cfg.Exists('left'):
            self.x = int(self.cfg.Read('left'))
            self.y = int(self.cfg.Read('top'))
            self.SetPosition(wx.Point(self.x,self.y), wx.SIZE_USE_EXISTING)

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.Bind(wx.EVT_SIZE, self.OnSize)
        self.Bind(wx.EVT_MOVE, self.OnMove)
        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ico = wx.Icon(PPRZ_SRC + "/sw/ground_segment/python/energy_mon/energy_mon.ico", wx.BITMAP_TYPE_ICO)
        self.SetIcon(ico)

        self.bat = {}
        self.temp = {}
        self.cell = BatteryCell()
        self.motors = MotorList()
        self.fuelcell = FuelCellStatus()
        self.fuelcell.update('<50,86,2,0x00020000>')
     
        self.interface = IvyMessagesInterface("fuelcellframe")
        self.interface.subscribe(self.message_recv)

    def OnClose(self, event):
        self.interface.shutdown()
        self.Destroy()

if __name__ == '__main__':
    energy_message = EnergyMessage({"bat": 22, "amp": 18, "power": 22 * 18, "energy": 10000})
    air_data_message = AirDataMessage({"airspeed": 21})
    esc_message = Esc({})
    cell = BatteryCell()
    cell.fill_from_energy_msg(energy_message)

    import matplotlib.pyplot as plt
    import numpy as np
    energies = np.arange(0, 3200*6, 10)
    seconds_left = np.zeros(energies.shape)

    # for i, energy in enumerate(energies):
    #     cell.fill_from_energy_msg(EnergyMessage({"bat": 22, "amp": 18, "power": 22 * 18, "energy": energy}))
    #     seconds_left[i] = energy_prediction.get_hover_seconds_left()
    #
    #
    # plt.plot(energies, seconds_left)
    # plt.show()
