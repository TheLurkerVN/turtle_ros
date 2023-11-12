import tkinter as tk
from tkinter import ttk
from tkinter import *
import struct
import numpy as np
from PIL import Image, ImageTk, GifImagePlugin
import paho.mqtt.client as mqtt

from control import Control
import backend_classes.topics as topics

#NAV_MQTT = "nav_mqtt"
#CMD_MQTT_KEY = "cmd_mqtt"
#STATUS_MQTT = "status_mqtt"
#BATTERY_MQTT = "battery_mqtt"

class ControlPanel(ttk.Frame):
    def __init__(self, container, mqtt):
        super().__init__(container)
        
        self.mqtt = mqtt
        self.mqtt.client.message_callback_add(topics.BATTERY_MQTT, self.battery_status)

        self.container = container
        self.config(height = 1000, width = 500)
        #self.place(x = 1000, y = 0)
        self.pack(fill='both', side='right', expand='True')
        self.pack_propagate(False)
        self.grid_propagate(False)
        self.bat = 0.0

        self.schoolLabel = Label(self, text='National University of Hanoi', font=('Arial', 25))
        self.schoolLabelTwo = Label(self, text='University of Engineering and Technology', font=('Arial', 20))
        self.studentLabel = Label(self, text='Nguyễn Hữu Đạt - 19020516')

        self.schoolLabel.pack()
        self.schoolLabelTwo.pack()
        self.studentLabel.pack()


        self.statusLabel = Label(self, text='Status')
        #self.statusLabel.place(x = 0, y = 0)
        #self.statusLabel.grid(column=0, row=0)
        self.statusLabel.pack(side='left', expand='True', anchor = tk.NW)

        self.batteryLabel = Label(self, text='Battery: 100%')
        #self.batteryLabel.place(x = 400, y = 0)
        #self.batteryLabel.grid(column=1, row=0)
        self.batteryLabel.pack(side='left', anchor = tk.NW)
        #self.batteryLabel.grid(column = 1, row = 0, padx = 5, pady = 5)

        self.buttonFrame = tk.Frame(self, bg='blue')
        #self.buttonFrame.pack(anchor=tk.CENTER)
        self.buttonFrame.place(x = 100, y = 200)

        self.navmode_button = ttk.Button(self.buttonFrame, text = 'Navigation Mode', command = self.navigationmode_command)
        self.nav_origin_button = ttk.Button(self.buttonFrame, text = 'Go To Origin', command = self.origin_navigation)
        self.cancel_button = ttk.Button(self.buttonFrame, text = 'Cancel Navigation', command = self.cancel_navigation)
        self.slammode_button = ttk.Button(self.buttonFrame, text = 'SLAM Mode', command = self.slammode_command)
        self.save_button = ttk.Button(self.buttonFrame, text = 'Save Map', command = self.savemap_command)
        self.update_button = ttk.Button(self.buttonFrame, text = 'Update Map', command = self.update_command)
        
        self.navmode_button.grid(column = 0, row = 0, sticky='nesw')
        self.nav_origin_button.grid(column = 1, row = 0, sticky='nesw')
        self.cancel_button.grid(column = 2, row = 0, sticky='nesw')
        self.slammode_button.grid(column = 0, row = 1, sticky='nesw')
        self.save_button.grid(column = 1, row = 1, sticky='nesw')
        self.update_button.grid(column = 2, row = 1, sticky='nesw')
        
        self.controlPad = Control(self, mqtt)

        self.controlPad.place(x = 100, y = 400)

    def battery_status(self, client, userdata, msg):
        self.bat = round(float(msg.payload[0:4]))
        if self.bat > 100.0:
            self.bat = 100
        self.batteryLabel.config(text = "Battery: {}%".format(self.bat))

    def origin_navigation(self):
        nav_coords = list((0.0, 0.0))
        nav_param = struct.pack("{}d".format(2), *nav_coords)
        self.mqtt.publishControl(topics.NAV_MQTT, nav_param)
        self.statusLabel.config(text = "Going to Origin")
        print("go to origin")

    def cancel_navigation(self):
        self.mqtt.publishControl(topics.CMD_MQTT_NAV, "cancelnav")
        self.statusLabel.config(text = "Cancelled Nav")
        print("cancel_navigation")

    def savemap_command(self):
        self.mqtt.publishControl(topics.CMD_MQTT_MAP, "savemap")
        self.statusLabel.config(text = "Saved Map")
        print("savemap")

    def update_command(self):
        self.mqtt.publishControl(topics.CMD_MQTT_MAP, "update")
        self.statusLabel.config(text = "Updating")
        print("update")

    def slammode_command(self):
        print("slammode")
        self.mqtt.publishControl(topics.CMD_MQTT_SLAM, "slam_mode")
        self.statusLabel.config(text = "SLAM Mode Initializing")
        self.navmode_button.config(state = DISABLED)
        

    def navigationmode_command(self):
        print("navmode")
        self.mqtt.publishControl(topics.CMD_MQTT_NAV_MODE, "navigation_mode")
        self.statusLabel.config(text = "Nav Mode Initializing")
        self.slammode_button.config(state = DISABLED)

    


