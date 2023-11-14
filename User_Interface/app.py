import tkinter as tk
from tkinter import ttk
from tkinter.messagebox import showinfo
import paho.mqtt.client as mqtt
from control import Control
from backend_classes.mqtt import MQTTClient
from map import Map
from controlPanel import ControlPanel

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title('IoRT System')

        window_width = 1500
        window_height = 1000
        screen_width = self.winfo_screenwidth()
        screen_height = self.winfo_screenheight()
        center_x = int(screen_width/2 - window_width / 2)
        center_y = int(screen_height/2 - window_height / 2)
        self.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

    def get_key(self, event):
        print(event.char)

if __name__ == "__main__":
    app = App()

    mqtt = MQTTClient()

    controlFrame = ControlPanel(app, mqtt)
    mapFrame = Map(app, mqtt, controlFrame)

    app.mainloop()
