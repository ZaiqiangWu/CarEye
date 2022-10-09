#!/usr/bin/env python-
# -*- coding: utf-8 -*

import sys, os, math, json
import threading
# image processing (OpenCV)
import cv2
# GUI library
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
# library to control motors
# officially current version has some methods not being implemented yet
# instead, use that placed in the local folder
sys.path.insert(0, '../..')
from gs2d import SerialInterface, RobotisP20

def lerp(x, x0, x1, y0, y1):
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)

def config_save(obj, filename = "config_roboeye.json"):
    # save json file
    with open(filename, "w") as file:
        json.dump(obj, file)

def config_load(filename = "config_roboeye.json"):
    # create if not existing
    if not os.path.exists(filename):
        config_save({"command":{"left":{"x": 0, "y": 0}, "right":{"x": 0, "y": 0}},"range":{"hori": 0, "vert": 0}}, filename)
    # loading json file
    with open(filename, "r") as file:
        return json.load(file)

class RoboticEye():
    def __init__(self, port="COM3", baud=1000000):
        super().__init__()
        # open serial ports
        self.si = SerialInterface(device=port, baudrate=baud)
        # controlling manager for motors
        self.robotis = RobotisP20(self.si)
        # activated motor id
        self.sids = []

    def setup(self, id_motor, target_time=0.2, p_gain=400):
        ## Robitis Motor
        self.robotis.set_torque_enable(True, sid=id_motor)
        # set as initial position
        self.robotis.set_target_position(0, sid=id_motor)
        # setting the target time
        self.robotis.set_target_time(target_time, sid=id_motor)
        # P gains
        self.robotis.set_p_gain(p_gain, sid=id_motor)
        # register activated motor id
        self.sids.append(id_motor)

    def move(self, id_motor, angle, invert=False):
        #print("move")
        self.robotis.set_target_position(-angle if invert else angle, id_motor)
        
    def __del__(self):
        # disable motor
        for num in self.sids:
            self.robotis.set_torque_enable(False, sid=num)
        # close motor managers
        self.robotis.close()
        # close serial ports
        self.si.close()

class Application(tk.Frame):
    def __init__(self, master, video_source=0):
        super().__init__(master)
        # setup motor
        self.motor = RoboticEye(port="COM5")
        self.motor.setup(id_motor=1)
        self.motor.setup(id_motor=2)
        self.motor2 = RoboticEye(port="COM6")
        self.motor2.setup(id_motor=3)
        self.motor2.setup(id_motor=4)
        # variables
        self.angle_x = 0
        self.angle_y = 0
        self.config = config_load()
        ## parameters
        self.key_left_x = self.config["command"]["left"]["x"]
        self.key_left_y = self.config["command"]["left"]["y"]
        self.key_right_x = self.config["command"]["right"]["x"]
        self.key_right_y = self.config["command"]["right"]["y"]
        # window setting
        self.master.geometry("1368x912")
        self.master.title("Robotic Eye Controller")
        self.master.bind("<KeyPress>", self.key_pressed)
        self.width = 1620 #1368
        self.height = 1080 #912
        ## setting grid of the main window
        self.master.grid_rowconfigure(0, weight=1)
        self.master.grid_columnconfigure(0, weight=1)
        # open video
        self.vcap = cv2.VideoCapture(video_source, cv2.CAP_MSMF)
        ## camera configuration
        self.vcap.set(cv2.CAP_PROP_FPS, 30)
        self.vcap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.vcap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.vcap_width = self.vcap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.vcap_height = self.vcap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        # widget
        self.create_widgets()
        # protocol handler
        self.master.protocol("WM_DELETE_WINDOW", self.close)
        # canvas update
        self.draw()
        self.move()

    def create_widgets(self):
        # Frame GUI panel
        self.frame_cam = ttk.Frame(self.master)
        self.frame_cam.place(x = 0, y = 0)
        self.frame_cam.configure(width = self.width, height = self.height)
        self.frame_cam.grid(row=0, column=0, sticky="nsew")
        # Canvas
        self.canvas = tk.Canvas(self.frame_cam)
        self.canvas.configure(width = self.vcap_width, height = self.vcap_height)
        self.canvas.pack(expand=True, fill=tk.BOTH, side=tk.LEFT)
        ## binding mouse event
        self.canvas.bind("<Button-1>", self.mouse_pressed)
        self.canvas.bind("<B1-Motion>", self.mouse_pressed)
        self.canvas.bind("<Configure>", self.on_resize)
        # Frame GUI panel
        self.frame_gui = ttk.LabelFrame(self.master, text = "Control")
        #self.frame_gui.configure(width = 600, height = 200)
        #self.frame_gui.grid_propagate(0)
        ## horizontal angle range
        self.label_holi = ttk.Label(self.frame_gui, text="Horizontal")
        self.label_holi.grid(row=0, column=0)
        self.scale_hori = tk.IntVar(value=self.config["range"]["hori"])
        self.slider_hori = tk.Scale(self.frame_gui)
        self.slider_hori.config(variable=self.scale_hori, orient=tk.HORIZONTAL, resolution=5, tickinterval=5, from_=0, to=45, length=600)
        self.slider_hori.grid(row=0, column=1)
        ## vertical angle range
        self.label_holi = ttk.Label(self.frame_gui, text="Vertical")
        self.label_holi.grid(row=1, column=0)
        self.scale_vert = tk.IntVar(value=self.config["range"]["vert"])
        self.slider_vert = tk.Scale(self.frame_gui)
        self.slider_vert.config(variable=self.scale_vert, orient=tk.HORIZONTAL, resolution=5, tickinterval=5, from_=0, to=45, length=600)
        self.slider_vert.grid(row=1, column=1)
        # registration
        self.frame_regist_angle = ttk.LabelFrame(self.frame_gui, text="Register Motor Angles to Arrow Keys")
        self.frame_regist_angle.grid(row=2, column=0, columnspan=2, sticky=tk.E+tk.W)
        # motor angles when the left arrow key pressed
        self.button_regist_left = ttk.Button(self.frame_regist_angle, padding=5, text="Left Arrow", command=lambda: self.set_arrow_left(self.angle_x, self.angle_y))
        self.button_regist_left.pack(fill='x', padx=20, side='left', expand=True)
        # motor angles when the right arrow key pressed
        self.button_regist_right = ttk.Button(self.frame_regist_angle, padding=5, text="Right Arrow", command=lambda: self.set_arrow_right(self.angle_x, self.angle_y))
        self.button_regist_right.pack(fill='x', padx=20, side='left', expand=True)

    def set_arrow_right(self, x, y):
        self.key_right_x = int(x)
        self.key_right_y = int(y)
    
    def set_arrow_left(self, x, y):
        self.key_left_x = int(x)
        self.key_left_y = int(y)


    def eye_figure(self, x, y, size, radius, ax, ay):
        # outer side (white eye)
        self.canvas.create_oval(x, y, x+size, y+size, fill='white')
        # coordinate of the inner side
        pos_x = x + int(0.5 * size * (1 + math.sin(math.radians(ax))))
        pos_y = y + int(0.5 * size * (1 + math.sin(math.radians(ay))))
        # inner side (black eye)
        self.canvas.create_oval(pos_x-radius, pos_y-radius, pos_x+radius, pos_y+radius, fill='black')

    def draw(self):
        # self.photo -> Canvas
        # get a frame from the video source
        ret, frame = self.vcap.read()
        if ret:
            # resize capture frame (may slow down drastically)
            #frame = cv2.resize(frame, dsize = (self.width, self.height))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.photo = ImageTk.PhotoImage(image = Image.fromarray(frame))
            self.canvas.create_image(0, 0, image = self.photo, anchor = tk.NW)
        # draw eye position
        self.eye_figure(self.width-120, self.height-140, 100, 20, self.angle_x, self.angle_y)
        self.eye_figure(self.width-240, self.height-140, 100, 20, self.angle_x, self.angle_y)
        # call itself recursively
        self.after_id = self.master.after(33, self.draw)

    def move(self):
        # control motors
        self.motor.move(2, self.angle_x)
        self.motor.move(1, self.angle_y)
        self.motor2.move(4, self.angle_x)
        self.motor2.move(3, self.angle_y)
        # call itself recursively (multi-threading)
        self.timer = threading.Timer(0.05, self.move)
        self.timer.start()

    def on_resize(self, event):
        # get the current window size
        cw = event.width
        ch = event.height
        # cange canvas size
        if cw != self.width or ch != self.height:
            self.width = cw
            self.height = ch

    def mouse_pressed(self, event):
        #print(f'mouse pressed:({event.x}, {event.y})')
        self.angle_x = lerp(event.x, 0, event.widget.winfo_width(), -self.scale_hori.get(), self.scale_hori.get())
        self.angle_y = lerp(event.y, 0, event.widget.winfo_height(), -self.scale_vert.get(), self.scale_vert.get())

    def key_pressed(self, event):
        # move motors when the right arrow key pressed
        if event.keysym == "Right":
            self.angle_x = self.key_right_x
            self.angle_y = self.key_right_y
        # move motors when the left arrow key pressed
        elif event.keysym == "Left":
            self.angle_x = self.key_left_x
            self.angle_y = self.key_left_y
        # set motor as initial position
        elif event.keysym == "Down":
            self.angle_x = self.angle_y = 0
        # show config panel
        elif event.keysym == "c":
            self.frame_gui.place(x = 100, y = 100)
        # hide config panel
        elif event.keysym == "d":
            self.frame_gui.place_forget()
        # close application
        elif event.keysym == "q":
            self.close()
        else:
            print(event.keysym)

    def close(self):
        # cancel callback schedule
        self.timer.cancel()
        self.master.after_cancel(self.after_id)
        # close motor port
        del self.motor
        del self.motor2
        # save the current parameters
        config_save({"command":{"left":{"x": self.key_left_x, "y": self.key_left_y}, "right":{"x": self.key_right_x, "y": self.key_right_y}},"range":{"hori": self.scale_hori.get(), "vert": self.scale_vert.get()}})
        # release camera resource
        self.vcap.release()
        # destroy root window
        self.master.destroy()      

def main():
    # main loop
    root = tk.Tk()
    app = Application(master=root, video_source=2)
    app.mainloop()

if __name__ == "__main__":
    main()
