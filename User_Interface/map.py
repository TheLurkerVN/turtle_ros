import tkinter as tk
from tkinter import ttk
from tkinter import *
import struct
import numpy as np
from PIL import Image, ImageTk, GifImagePlugin
import paho.mqtt.client as mqtt
import math

from control import Control
import backend_classes.topics as topics

# Resolution 0.05 m/cell
class Map(tk.Canvas):
    def __init__(self, container, mqtt, friendContainer):
        super().__init__(container)
        
        self.mqtt = mqtt
        self.mqtt.client.message_callback_add(topics.MAP_CONTENT_MQTT, self.map_receiver)
        self.mqtt.client.message_callback_add(topics.POSE_MQTT, self.pose_receiver)  

        self.container = container
        self.friendContainer = friendContainer
        self.config(bg = 'gray', height = 1000, width = 1000)    #CDCDCD
        self.pack(fill='both', side='left', expand='True')
        self.pack_propagate(False)

        self.mapImage = Image.open("User_Interface/images/map_default.pgm")
        newImage = self.mapImage.resize((1000, 1000), resample = Image.NEAREST)
        self.scanImage = ImageTk.PhotoImage(newImage)
        self.mapCanvas = self.create_image(0, 0, anchor="nw", image = self.scanImage)

        self.origin = (0.0, 0.0)
        self.pose = (0.0, 0.0)
        self.rotation = 0.0
        self.rotationOffset = [0.0, 0.0]
        self.ratio = 0.0
        self.actualPos = (2.0, 2.0)
        self.size = (0.0, 0.0)

        self.originDot = self.create_oval(self.actualPos[0] - 2, self.actualPos[1] - 2, self.actualPos[0] + 2, self.actualPos[1] + 2, fill = 'red')
        self.rotationLine = self.create_line(self.actualPos[0], self.actualPos[1], self.actualPos[0] + self.rotationOffset[0], self.actualPos[1] + self.rotationOffset[1], fill = 'red', width = 2)
        self.itemconfig(self.originDot, state = 'hidden')
        self.itemconfig(self.rotationLine, state = 'hidden')
        self.navDot = self.create_oval(0, 0, 4, 4, fill = 'green')
        self.itemconfig(self.navDot, state='hidden')

        self.bind("<Button-1>", self.callback) 

    
    def callback(self, event):
        if self.friendContainer.bat < 10:
            self.friendContainer.statusLabel.config(text = "Going to Point failed, please recharge")
        elif not self.valid_point(self.canvasx(event.x), self.canvasy(event.y)):
            self.friendContainer.statusLabel.config(text = "Invalid Point!")
        else:
            self.itemconfig(self.navDot, state='normal')
            x, y = self.canvasx(event.x), self.canvasy(event.y)
            self.moveto(self.navDot, x, y)
            self.tag_raise(self.navDot)
            x_nav = y // self.ratio * 0.05 - self.origin[0] * -1
            y_nav = x // self.ratio * 0.05 - self.origin[1] * -1
            nav_coords = list((x_nav, y_nav))
            nav_param = struct.pack("{}d".format(2), *nav_coords)
            self.mqtt.publishControl(topics.NAV_MQTT, nav_param)
            self.friendContainer.statusLabel.config(text = "Going to Point")
 
    def valid_point(self, x, y):
        if (x >= self.size[0] * self.ratio) or (y >= self.size[1] * self.ratio):
            return False
        return True
        
    def map_receiver(self, client, userdata, msg):
        print("Received map content")
        self.size = struct.unpack(">{}h".format(2), msg.payload[0:4])
        self.content = struct.unpack(">{}h".format(self.size[0] * self.size[1]), msg.payload[4:])

        self.size = self.size[::-1]
        mat = np.array(self.content)
        mat = mat.reshape(self.size)
        mat = np.fliplr(mat)

        for x in range(0, self.size[0]):
            for y in range(0, self.size[1]):
                if mat[x, y] < 0 or mat[x, y] > 100:
                    mat[x, y] = 205
                elif mat[x, y] <= 25:
                    mat[x, y] = 254
                elif mat[x, y] >= 65:
                    mat[x, y] = 0
                else:
                    mat[x, y] = 205

        image = Image.fromarray(mat.astype(np.uint8), 'L')
        max_wh = 1000
        width, height = self.size[::-1]
        self.ratio = min(max_wh / width, max_wh / height)

        newImage = image.resize((int(width*self.ratio), int(height*self.ratio)), resample = Image.NEAREST)
        newImage = newImage.rotate(90, resample = Image.NEAREST, expand = 1)
        newImage.save("test.pgm")

        self.scanImage = ImageTk.PhotoImage(newImage)
        self.itemconfig(self.mapCanvas, image = self.scanImage)
        self.tag_raise(self.navDot)
        self.tag_raise(self.rotationLine)
        # Assign random container attr with the image so it doesn't flicker
        # If not: garbage collector
        self.container.temp = self.scanImage    

    def pose_receiver(self, client, userdata, msg):
        print("Received pose")
        self.pose = struct.unpack(">{}f".format(5), msg.payload[0:40])
        self.rotation = self.pose[4]
        self.origin = self.pose[2:4]
        self.pose = self.pose[0:2]
        self.rotationOffset[0] = 20 * math.sin(self.rotation)
        self.rotationOffset[1] = 20 * math.cos(self.rotation)
        print(self.rotationOffset)
        self.actualPos = [x / 0.05 for x in self.pose]       
        self.actualPos = [x * self.ratio * -1.0 for x in self.actualPos]
        self.actualPos = self.actualPos[::-1]
        self.delete(self.rotationLine)
        self.rotationLine = self.create_line(self.actualPos[0] + 2, self.actualPos[1] + 2, 
                                             self.actualPos[0] + self.rotationOffset[0], self.actualPos[1] + self.rotationOffset[1], 
                                             fill = 'red', width = 2)
        self.moveto(self.originDot, self.actualPos[0], self.actualPos[1])
        self.itemconfig(self.originDot, state = 'normal')
        self.tag_raise(self.originDot)
        self.tag_raise(self.rotationLine)
        



        


    