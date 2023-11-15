import tkinter as tk
from tkinter import ttk
from tkinter import *
import struct
import numpy as np
from PIL import Image, ImageTk, GifImagePlugin
import paho.mqtt.client as mqtt

from control import Control
import backend_classes.topics as topics

class ControlPanel(ttk.Frame):
    def __init__(self, container, mqtt):
        super().__init__(container)
        
        self.mqtt = mqtt
        self.mqtt.client.message_callback_add(topics.BATTERY_MQTT, self.battery_status)
        self.mqtt.client.message_callback_add(topics.RESULT_MQTT, self.gotopose_status)

        self.container = container
        self.config(height = 1000, width = 500)
        self.pack(fill='both', side='right', expand='True')
        self.pack_propagate(False)
        self.grid_propagate(False)

        self.bat = 0.0

        self.statusFrame = tk.Frame(self)
        self.statusFrame.pack(side=tk.TOP, fill=tk.X)

        self.buttonFrame = tk.Frame(self)
        self.buttonFrame.pack(side=tk.TOP, fill=tk.X)

        self.schoolLabel = Label(self.statusFrame, text='National University of Hanoi', font=('Arial', 25))
        self.schoolLabelTwo = Label(self.statusFrame, text='University of Engineering and Technology', font=('Arial', 20))
        self.studentLabel = Label(self.statusFrame, text='Nguyễn Hữu Đạt - 19020516')
        self.InstructorLabel = Label(self.statusFrame, text='GVHD: Ts. Phạm Duy Hưng')

        self.schoolLabel.grid(row = 0, column = 0, columnspan = 6)
        self.schoolLabelTwo.grid(row = 1, column = 0, columnspan = 6)
        self.studentLabel.grid(row = 2, column = 0, columnspan = 6)
        self.InstructorLabel.grid(row = 3, column = 0, columnspan = 6)
        self.statusLabel = Label(self.statusFrame, text='Select a mode...')
        self.statusLabel.grid(column = 0, row = 4, columnspan = 2, pady = 10)

        self.GoToPoseLabel = Label(self.statusFrame, text='Waiting...')
        self.GoToPoseLabel.grid(column = 2, row = 4, columnspan = 2, pady = 10)

        self.batteryLabel = Label(self.statusFrame, text='Battery: 100%')
        self.batteryLabel.grid(column = 4, row = 4, columnspan = 2, pady = 10)

        self.templabel = Label(self.buttonFrame, text='-----------------------------------------------', font=('Arial', 25))
        self.templabel.grid(row = 0, column = 0, columnspan = 6, sticky = 'nesw')
        self.navmode_button = tk.Button(self.buttonFrame, text = 'Navigation Mode', command = self.navigationmode_command)
        self.nav_origin_button = tk.Button(self.buttonFrame, text = 'Go To Origin', command = self.origin_navigation)
        self.cancel_button = tk.Button(self.buttonFrame, text = 'Cancel Navigation', command = self.cancel_navigation)
        self.slammode_button = tk.Button(self.buttonFrame, text = 'SLAM Mode', command = self.slammode_command)
        self.save_button = tk.Button(self.buttonFrame, text = 'Save Map', command = self.savemap_command)
        self.update_button = tk.Button(self.buttonFrame, text = 'Update Map', command = self.update_command)
        
        self.navmode_button.grid(column = 0, row = 1, columnspan = 3, sticky = 'nesw')
        self.slammode_button.grid(column = 3, row = 1, columnspan = 3, sticky = 'nesw')
        self.nav_origin_button.grid(column = 0, row = 2, columnspan = 2, sticky = 'nesw')
        self.cancel_button.grid(column = 2, row = 2, columnspan = 2, sticky = 'nesw')
        self.save_button.grid(column = 0, row = 2, columnspan = 6, sticky='nesw')
        self.update_button.grid(column = 4, row = 2, columnspan = 2, sticky = 'nesw')
        
        self.nav_origin_button.grid_forget()
        self.cancel_button.grid_forget()
        self.save_button.grid_forget()
        self.update_button.grid_forget()

        self.controlPad = Control(self, mqtt)

        self.controlPad.place(x = 100, y = 400)

    def battery_status(self, client, userdata, msg):
        self.bat = round(float(msg.payload[0:4]))
        if self.bat > 100.0:
            self.bat = 100
        self.batteryLabel.config(text = "Battery: {}%".format(self.bat))

    def gotopose_status(self, client, userdata, msg):
        self.GoToPoseLabel.config(text = msg.payload)

    def origin_navigation(self):
        nav_coords = list((0.0, 0.0))
        nav_param = struct.pack("{}d".format(2), *nav_coords)
        self.mqtt.publishControl(topics.NAV_MQTT, nav_param)
        self.GoToPoseLabel.config(text = "Going to Origin...")
        print("go to origin")

    def cancel_navigation(self):
        self.mqtt.publishControl(topics.CMD_MQTT_NAV, "cancelnav")
        self.GoToPoseLabel.config(text = "Cancelled Nav")
        print("cancel_navigation")

    def savemap_command(self):
        self.mqtt.publishControl(topics.CMD_MQTT_MAP, "savemap")
        self.GoToPoseLabel.config(text = "Saved Map")
        print("savemap")

    def update_command(self):
        self.mqtt.publishControl(topics.CMD_MQTT_NAV, "calibrate")
        self.statusLabel.config(text = "Calibrating...")
        print("calibrate")

    def slammode_command(self):
        print("slammode")
        self.mqtt.publishControl(topics.CMD_MQTT_MODE, "slam_mode")
        self.statusLabel.config(text = "SLAM Mode Initializing")
        self.slammode_button.config(bg = 'green')
        self.navmode_button.config(bg = 'red')
        self.nav_origin_button.grid_forget()
        self.cancel_button.grid_forget()
        self.update_button.grid_forget()
        self.save_button.grid(column = 0, row = 2, columnspan = 6, sticky='nesw')

    def navigationmode_command(self):
        print("navmode")
        self.mqtt.publishControl(topics.CMD_MQTT_MODE, "navigation_mode")
        self.statusLabel.config(text = "Nav Mode Initializing")
        self.navmode_button.config(bg = 'green')
        self.slammode_button.config(bg = 'red')
        self.save_button.grid_forget()
        self.nav_origin_button.grid(column = 0, row = 2, columnspan = 2, sticky = 'nesw')
        self.cancel_button.grid(column = 2, row = 2, columnspan = 2, sticky = 'nesw')
        self.update_button.grid(column = 4, row = 2, columnspan = 2, sticky = 'nesw')
