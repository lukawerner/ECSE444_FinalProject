import tkinter as tk
import math
from queue import Queue

MAX_RADAR_DISTANCE_CM = 50

class RadarUI:

    def __init__(self, root, queue: Queue):
        self.root = root
        self.queue = queue
        self.root.title("Intruder Detection Radar")

        self.scan_start_angle = 30  # Scan starts at 30 degrees
        self.scan_end_angle = 150  # Scan ends at 150 degrees
        self.scan_sweep = self.scan_end_angle - self.scan_start_angle

        self.canvas_size = 600
        self.radar_radius = 250
        self.center_x = self.canvas_size // 2
        self.center_y = self.canvas_size // 2

        self.canvas = tk.Canvas(root, width=self.canvas_size, height=self.canvas_size, bg="black")
        self.canvas.pack(padx=10, pady=(30, 10))

        self.temp_var = tk.StringVar()
        self.temp_var.set(self.format_status_text("N/A"))
        self.temp_label = tk.Label(root, textvariable=self.temp_var, font=("Helvetica", 14, "bold"), fg="white",
                                   bg=root['bg'])
        self.temp_label.pack(pady=10)

        self.indicator_angle = self.scan_start_angle
        self.indicator_line = None

        self.intruders = {}
        self.baselines = {}

        self.draw_radar_background()
        self.update_queue()
        self.update_indicator()

    def format_status_text(self, temp):
        grid_step_cm = MAX_RADAR_DISTANCE_CM / 5
        return f"Temperature: {temp} °C\n\nNote: Each grid arc represents {int(grid_step_cm)}cm distance."

    def draw_radar_background(self):

        num_steps = 5

        for i in range(1, num_steps + 1):
            r = int((i / num_steps) * self.radar_radius)
            if r == 0: continue
            self.canvas.create_arc(
                self.center_x - r, self.center_y - r,
                self.center_x + r, self.center_y + r,
                start=self.scan_start_angle, extent=self.scan_sweep,
                outline="green", style=tk.ARC, width=1.5
            )

        for angle in range(self.scan_start_angle, self.scan_end_angle + 1, 30):
            rad = math.radians(angle)
            x = self.center_x + self.radar_radius * math.cos(rad)
            y = self.center_y - self.radar_radius * math.sin(rad)
            self.canvas.create_line(self.center_x, self.center_y, x, y, fill="green", width=1.5)

    def update_indicator(self):

        # Clear previous indicator
        if self.indicator_line:
            self.canvas.delete(self.indicator_line)

        rad = math.radians(self.indicator_angle)
        x = self.center_x + self.radar_radius * math.cos(rad)
        y = self.center_y - self.radar_radius * math.sin(rad)

        self.indicator_line = self.canvas.create_line(
            self.center_x, self.center_y, x, y, fill="white", width=1
        )

        # next update
        self.root.after(50, self.update_indicator)  # 20 FPS

    def update_queue(self):

        while not self.queue.empty():
            try:
                msg = self.queue.get_nowait()
                self.handle_message(msg)
            except Exception:
                pass

        self.root.after(100, self.update_queue)  # Check queue every 100ms

    def handle_message(self, msg: dict):

        try:
            msg_type = msg.get("type")

            if msg_type == "temperature":
                self.temp_var.set(self.format_status_text(msg.get('temperature', 'N/A')))

            elif msg_type == "normal":
                angle = msg["angle"]
                self.indicator_angle = angle
                if angle in self.intruders:
                    self.canvas.delete(self.intruders[angle])
                    del self.intruders[angle]

            elif msg_type == "intruder":
                angle = msg["angle"]
                self.indicator_angle = angle
                distance_cm = msg["distance"]
                if distance_cm > MAX_RADAR_DISTANCE_CM:
                    return

                distance_pixels = (distance_cm / MAX_RADAR_DISTANCE_CM) * self.radar_radius

                rad = math.radians(angle)
                x = self.center_x + distance_pixels * math.cos(rad)
                y = self.center_y - distance_pixels * math.sin(rad)

                if angle in self.intruders:
                    self.canvas.delete(self.intruders[angle])

                # Draw red intruder dot
                dot_size = 10
                dot = self.canvas.create_oval(
                    x - dot_size / 2, y - dot_size / 2,
                    x + dot_size / 2, y + dot_size / 2,
                    fill="red", outline="white"
                )
                self.intruders[angle] = dot

            elif msg_type == "baseline":
                angle = msg["angle"]
                self.indicator_angle = angle
                distance_cm = msg["distance"]
                if distance_cm > MAX_RADAR_DISTANCE_CM:
                    distance_cm = MAX_RADAR_DISTANCE_CM

                distance_pixels = (distance_cm / MAX_RADAR_DISTANCE_CM) * self.radar_radius

                rad = math.radians(angle)
                x = self.center_x + distance_pixels * math.cos(rad)
                y = self.center_y - distance_pixels * math.sin(rad)

                dot_size = 10
                dot = self.canvas.create_oval(
                    x - dot_size / 2, y - dot_size / 2,
                    x + dot_size / 2, y + dot_size / 2,
                    fill="green", outline="white"
                )
                self.baselines[angle] = dot

        except Exception as e:
            print(f"Unexpected error in handle_message: {e}")