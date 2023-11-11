import tkinter as tk
from tkinter import ttk
from tkinter import *
from PIL import ImageTk

from backend_classes.calc_velocity import CalcVel
import backend_classes.topics as topics
import struct

class Control(tk.Frame):
    def __init__(self, container, mqtt):
        super().__init__(container)
        self.config(height=300, width=300)
        self.grid_propagate(False)
        """
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.columnconfigure(2, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)
        self.rowconfigure(3, weight=1)
        """
        print(self.winfo_width(), self.winfo_height())
        
        self.percentage = 0.0
        self.direction_calc = CalcVel()
        self.mqtt = mqtt

        blank_image = tk.PhotoImage()

        
        self.up_button = ttk.Button(self, image = blank_image, text = u'\u25b2', compound=tk.CENTER, command = lambda: self.test("up"))
        #self.up_button.config(width = 50, height = 50)
        self.up_button.grid(column = 1, row = 0, padx = 5, pady = 5)

        self.down_button = ttk.Button(self, image = blank_image, text = u'\u25bc', compound=tk.CENTER, command = lambda: self.test("down"))
        #self.down_button.config(width = 50, height = 50)
        self.down_button.grid(column = 1, row = 2, padx = 5, pady = 5)

        self.left_button = ttk.Button(self, image = blank_image, text = u'\u25c0', compound=tk.CENTER, command = lambda: self.test("left"))
        #self.left_button.config(width = 50, height = 50)
        self.left_button.grid(column = 0, row = 1, padx = 5, pady = 5)

        self.right_button = ttk.Button(self, image = blank_image, text = u'\u25b6', compound=tk.CENTER, command = lambda: self.test("right"))
        #self.right_button.config(width = 50, height = 50)
        self.right_button.grid(column = 2, row = 1, padx = 5, pady = 5)

        self.stop_button = ttk.Button(self, image = blank_image, text = 'Center', compound=tk.CENTER, command = lambda: self.test("stop"))
        #self.stop_button.config(width = 50, height = 50)
        self.stop_button.grid(column = 1, row = 1, padx = 5, pady = 5)

        
        self.speedLabel = Label(self, text='Speed: 0%')
        self.speedLabel.grid(column = 0, row = 3, padx = 5, pady = 5)

        self.speed_slider = ttk.Scale(
                self,
                from_=0,
                to=100,
                orient='horizontal',
                length=200,
                command=self.speed_changed
                )  
        self.speed_slider.grid(column = 1, row = 3, padx = 5, pady = 5, columnspan = 2)
        
        
    
    def speed_changed(self, event):
        print(self.speed_slider.get())
        self.percentage = round(self.speed_slider.get())
        self.speedLabel.config(text = "Speed: {}%".format(self.percentage))

    def test(self, dir):
        #print(dir)
        control_param = self.direction_calc.controlVel(dir, self.percentage / 100)
        #print(control_param)
        control_param = struct.pack("{}d".format(2), *control_param)
        
        #test = struct.unpack("{}d".format(2), control_param[0:16])
        #print(test)
        self.mqtt.publishControl(topics.KEY_MQTT, control_param)

    