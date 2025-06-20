#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import String
import threading
from PIL import Image as PILImage, ImageTk
import datetime
from sensor_msgs.msg import Image as ROSImage
import cv2
import numpy as np
from ijsjes_robot.msg import IceTarget
from ijsjes_robot.msg import HmiToRobot
from ijsjes_robot.msg import RobotToHmi  
from cv_bridge import CvBridge


class LampControl:
    """Klasse voor het bedienen van de lichtzuil (stoplicht)."""
    def __init__(self, canvas):
        self.canvas = canvas
        self.lamp_frame = self.create_lamp_frame()
        self.lamps = {
            'green': self.create_lamp(140, 80, 'green'),
            'orange': self.create_lamp(140, 160, 'orange'),
            'red': self.create_lamp(140, 240, 'red')
        }
        self.blinking = False
        self.blink_color = None

    def create_lamp_frame(self):
        return self.canvas.create_rectangle(120, 60, 220, 320, fill="#2E2E2E", outline="#444", width=3)

    def create_lamp(self, x, y, color):
        return self.canvas.create_oval(x, y, x + 60, y + 60, fill="gray", outline=color, width=3)

    def set_lamps(self, green=False, orange=False, red=False):
        """Stel de lampen in afhankelijk van booleans (True = kleur aan, False = uit/gray)."""
        self.stop_blinking()  # Stop eventuele blinking
        self.canvas.itemconfig(self.lamps['green'], fill='green' if green else 'gray')
        self.canvas.itemconfig(self.lamps['orange'], fill='orange' if orange else 'gray')
        self.canvas.itemconfig(self.lamps['red'], fill='red' if red else 'gray')
        self.canvas.update()

    def blink_lamp(self, color):
        if self.blinking and self.blink_color == color:
            # Al bezig met knipperen in deze kleur, niks doen
            return
        self.stop_blinking()
        self.blinking = True
        self.blink_color = color
        threading.Thread(target=self._blink, daemon=True).start()

    def stop_blinking(self):
        self.blinking = False

    def _blink(self):
        while self.blinking:
            for lamp_color, lamp_id in self.lamps.items():
                current_fill = self.canvas.itemcget(lamp_id, "fill")
                new_fill = "gray" if current_fill == self.blink_color else self.blink_color if lamp_color == self.blink_color else "gray"
                self.canvas.itemconfig(lamp_id, fill=new_fill)
            self.canvas.update()
            rospy.sleep(0.5)


class RoboticArmInterface(tk.Frame):
    """Hoofdklasse voor de grafische HMI."""
    def __init__(self, master=None):
        super().__init__(master, bg='#f0f0f0')
        self.master = master
        self.master.title('Robotic Arm Control Interface')
        self.pack(fill=tk.BOTH, expand=True)

        self.is_dark_mode = False

        rospy.init_node('hmi_control_node', anonymous=True)

        # Subscribe op de camera beelden
        rospy.Subscriber('/camera/image_raw', ROSImage, self.image_callback)
        self.bridge = CvBridge()

        # Subscribe op de status vanuit robot (RobotToHmi)
        rospy.Subscriber('/robot_to_hmi', RobotToHmi, self.robot_status_callback)

        # Publisher naar robot
        self.pub = rospy.Publisher('/hmi_to_robot', HmiToRobot, queue_size=10)

        # Style voor ttk widgets (buttons etc)
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self._set_light_mode_styles()

        self.create_header()

        self.center_frame = tk.Frame(self, bg="#f0f0f0")
        self.center_frame.pack(pady=20)

        self.create_lamp_block(self.center_frame)
        self.create_controls(self.center_frame)
        self.create_camera_view(self.center_frame)

    def create_header(self):
        """Header met logo (links) en klok (rechts). Titel is verwijderd zoals gevraagd."""
        self.header_frame = tk.Frame(self, bg="#f0f0f0", height=100)
        self.header_frame.pack(fill=tk.X, side="top")
        self.header_frame.pack_propagate(False)

        self.header_frame.grid_columnconfigure(0, weight=1)
        self.header_frame.grid_columnconfigure(1, weight=1)

        logo_image = PILImage.open("avans_logo.png").resize((80, 80))
        logo_photo = ImageTk.PhotoImage(logo_image)
        self.logo_label = tk.Label(self.header_frame, image=logo_photo, bg="#f0f0f0")
        self.logo_label.image = logo_photo
        self.logo_label.grid(row=0, column=0, sticky="w", padx=20)

        self.clock_label = tk.Label(self.header_frame, font=("Segoe UI", 12, "bold"), bg="#f0f0f0")
        self.clock_label.grid(row=0, column=1, sticky="e", padx=20)
        self.update_clock()

    def create_lamp_block(self, parent):
        """Maak de canvas en lampen aan."""
        self.canvas = tk.Canvas(parent, width=300, height=400, bg='#f0f0f0', highlightthickness=0)
        self.canvas.pack()
        self.lamp_control = LampControl(self.canvas)

    def create_controls(self, parent):
        """Maak de knoppen en statuslabel aan."""
        self.controls_frame = tk.Frame(parent, bg='#f0f0f0')
        self.controls_frame.pack(pady=10)

        self.buttons_frame = tk.Frame(self.controls_frame, bg='#f0f0f0')
        self.buttons_frame.pack()

        self.control_frame = tk.Frame(self.buttons_frame, bg='#f0f0f0')
        self.control_frame.pack()

        # Buttons met ttk.Button
        self.start_button = ttk.Button(self.control_frame, text="Start", command=self.start, style="TButton")
        self.start_button.pack(side="left", padx=10)

        self.stop_button = ttk.Button(self.control_frame, text="Stop", command=self.stop, style="TButton")
        self.stop_button.pack(side="left", padx=10)

        self.single_cycle_button = ttk.Button(self.control_frame, text="Single Cycle", command=self.single_cycle, style="TButton")
        self.single_cycle_button.pack(side="left", padx=10)

        self.emergency_stop_button = ttk.Button(self.control_frame, text="Emergency Stop", command=self.emergency_stop, style="Red.TButton")
        self.emergency_stop_button.pack(side="left", padx=10)

        self.theme_button = ttk.Button(self.controls_frame, text="Dark mode", command=self.toggle_theme, style="TButton")
        self.theme_button.pack(pady=5)

        self.exit_button = ttk.Button(self.controls_frame, text="Close HMI", command=self.master.quit, style="TButton")
        self.exit_button.pack(pady=5)

        # Status label: hier tonen we current_action en passen kleur aan per theme (wit/zwart)
        self.status_label = ttk.Label(self.controls_frame, text="Current action: Initializing", foreground="black")
        self.status_label.pack(pady=5)

    def create_camera_view(self, parent):
        """Maak de camera preview aan."""
        self.camera_frame = tk.Frame(parent, bg="black")
        self.camera_frame.pack(pady=10)

        self.camera_label = tk.Label(self.camera_frame, bg='black')
        self.camera_label.pack(padx=8, pady=8)

        self.camera_frame_label = tk.Label(parent, text="OAK-D DepthAI camera", bg="#f0f0f0", font=("Segoe UI", 12, "bold"))
        self.camera_frame_label.pack()

    def toggle_theme(self):
        """Toggle tussen light en dark mode en pas kleuren aan."""
        self.is_dark_mode = not self.is_dark_mode
        if self.is_dark_mode:
            bg_color = '#1e1e1e'
            fg_color = '#ffffff'
            canvas_bg = '#2E2E2E'
            self._set_dark_mode_styles()
            self.theme_button.config(text="Light mode")
        else:
            bg_color = '#f0f0f0'
            fg_color = '#000000'
            canvas_bg = '#f0f0f0'
            self._set_light_mode_styles()
            self.theme_button.config(text="Dark mode")

        # Hoofd widgets aanpassen
        self.configure(bg=bg_color)
        self.master.configure(bg=bg_color)

        # Canvas achtergrond apart, want die heeft eigen kleur
        self.canvas.configure(bg=canvas_bg)

        # Alle frames en labels die zelf bg/fg hebben (background/foreground)
        widgets_bg = [
            self.header_frame, self.center_frame,
            self.controls_frame, self.buttons_frame, self.control_frame,
            self.camera_frame,
        ]
        for w in widgets_bg:
            w.configure(bg=bg_color)

        # Labels met fg-kleur aanpassen
        labels_fg = [
            self.logo_label, self.clock_label, self.camera_frame_label
        ]
        for lbl in labels_fg:
            lbl.configure(bg=bg_color, fg=fg_color)

        # Status label nu wit of zwart (niet oranje meer)
        self.status_label.configure(foreground=fg_color, background=bg_color)

        # Zet ook alle buttons via style (ttk) - die passen zich nu aan via _set_XXX_mode_styles()

        # Recursief alle child widgets bg/fg aanpassen (voor elk kind van self)
        self._set_widget_colors(self, bg_color, fg_color)

    def _set_widget_colors(self, parent, bg, fg):
        """Helper om recursief bg en fg van widgets aan te passen."""
        for child in parent.winfo_children():
            # Sommige widgets ondersteunen bg en fg
            try:
                child.configure(bg=bg)
            except tk.TclError:
                pass
            try:
                child.configure(fg=fg)
            except tk.TclError:
                pass
            self._set_widget_colors(child, bg, fg)

    def _set_dark_mode_styles(self):
        """Configureer ttk styles voor dark mode."""
        self.style.configure("TButton",
                             background="#3a3a3a",
                             foreground="white",
                             borderwidth=1,
                             focusthickness=3,
                             focuscolor='none')
        self.style.map("TButton",
                       background=[('active', '#555555'), ('pressed', '#222222')],
                       foreground=[('disabled', '#888888')])
        self.style.configure("Red.TButton",
                             foreground="red",
                             background="#3a3a3a")
        self.style.map("Red.TButton",
                       background=[('active', '#661111'), ('pressed', '#440000')],
                       foreground=[('disabled', '#bb5555')])

    def _set_light_mode_styles(self):
        """Configureer ttk styles voor light mode."""
        self.style.configure("TButton",
                             background="#f0f0f0",
                             foreground="black",
                             borderwidth=1)
        self.style.map("TButton",
                       background=[('active', '#d9d9d9'), ('pressed', '#c0c0c0')],
                       foreground=[('disabled', '#a3a3a3')])
        self.style.configure("Red.TButton",
                             foreground="red",
                             background="#f0f0f0")
        self.style.map("Red.TButton",
                       background=[('active', '#ffdddd'), ('pressed', '#ffbbbb')],
                       foreground=[('disabled', '#ff9999')])

    def update_clock(self):
        """Update klok op tijd."""
        now = datetime.datetime.now()
        self.clock_label.config(text=now.strftime("%H:%M:%S"))
        self.after(1000, self.update_clock)

    def image_callback(self, data):
        """Callback voor camerabeelden vanuit ROS topic /camera/image_raw."""
        try:
            # Image message naar numpy array
            np_arr = np.frombuffer(data.data, np.uint8)
            img_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
            img_pil = PILImage.fromarray(img_np)
            img_pil = img_pil.resize((320, 240), PILImage.ANTIALIAS)
            img_tk = ImageTk.PhotoImage(img_pil)

            # Update camera label image in main thread
            def update_image():
                self.camera_label.imgtk = img_tk
                self.camera_label.configure(image=img_tk)
            self.camera_label.after(0, update_image)
        except Exception as e:
            rospy.logwarn(f"Error processing camera image: {e}")

    def robot_status_callback(self, msg):
        """Callback om status van robot te ontvangen en interface aan te passen."""

        # Tekstkleur aanpassen op basis van thema
        fg_color = "#ffffff" if self.is_dark_mode else "#000000"
        current_action_text = f"Current action: {msg.current_action}"
        self.status_label.config(text=current_action_text, foreground=fg_color)

        # Speciale situatie: running én stop_active -> oranje knippert
        if msg.running and msg.stop_active:
            self.lamp_control.blink_lamp("orange")
            return  # Stop hier om geen vaste lampstatus meer te zetten

        # Andere situaties → eerst stoppen met knipperen
        self.lamp_control.stop_blinking()

        # Standaard lampstatussen
        green = False
        orange = False
        red = False

        if msg.emergency_active:
            red = True
        elif msg.error:
            green = True
            orange = True
        elif msg.running and not msg.stop_active:
            orange = True
        elif msg.ready:
            green = True

        self.lamp_control.set_lamps(green=green, orange=orange, red=red)


    # Button callbacks om commands naar robot te sturen
    def start(self):
        msg = HmiToRobot()
        msg.start = True
        msg.stop = False
        msg.single_cycle = False
        msg.emergency = False
        self.pub.publish(msg)

    def stop(self):
        msg = HmiToRobot()
        msg.start = False
        msg.stop = True
        msg.single_cycle = False
        msg.emergency = False
        self.pub.publish(msg)

    def single_cycle(self):
        msg = HmiToRobot()
        msg.start = False
        msg.stop = False
        msg.single_cycle = True
        msg.emergency = False
        self.pub.publish(msg)

    def emergency_stop(self):
        msg = HmiToRobot()
        msg.start = False
        msg.stop = False
        msg.single_cycle = False
        msg.emergency = True
        self.pub.publish(msg)


if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmInterface(master=root)
    root.protocol("WM_DELETE_WINDOW", root.quit)
    root.mainloop()
