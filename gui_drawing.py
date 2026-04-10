import os
import math
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk


class SketchRecorderApp:
    def __init__(self, root, background_path="pybullet_scene_snapshot.png"):
        self.root = root
        self.root.title("Trajectory Sketch GUI")

        self.canvas_width = 900
        self.canvas_height = 600
        self.bg_color = "white"
        self.background_path = background_path
        self.circle_radius = None

        # Drawing state
        self.current_tool = tk.StringVar(value="freehand")
        self.current_color = tk.StringVar(value="red")
        self.freehand_thickness = tk.IntVar(value=3)

        # Temporary interaction state
        self.start_x = None
        self.start_y = None
        self.preview_item = None

        # Triangle state
        self.triangle_points = []

        # Data recording
        self.all_drawn_points = []
        self.freehand_raw_points = []
        self.freehand_sampled_points = []
        self.circle_center = None
        self.circle_outline_points = []
        self.triangle_tip = None
        self.triangle_outline_points = []

        # Background image
        self.bg_image = None
        self.bg_item = None

        # Drawn item ids
        self.drawn_items = []

        self._build_ui()

    def load_background(self, image_path):
        if not os.path.exists(image_path):
            print(f"Background image not found: {image_path}")
            return

        image = Image.open(image_path)
        image = image.resize((self.canvas_width, self.canvas_height))
        self.bg_image = ImageTk.PhotoImage(image)

        self.bg_item = self.canvas.create_image(
            0, 0,
            image=self.bg_image,
            anchor="nw"
        )
        self.canvas.tag_lower(self.bg_item)

    def _build_ui(self):
        control_frame = ttk.Frame(self.root, padding=8)
        control_frame.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(control_frame, text="Tool:").pack(side=tk.LEFT, padx=(0, 5))

        ttk.Radiobutton(
            control_frame, text="Freehand",
            variable=self.current_tool, value="freehand",
            command=self._tool_changed
        ).pack(side=tk.LEFT)

        ttk.Radiobutton(
            control_frame, text="Circle",
            variable=self.current_tool, value="circle",
            command=self._tool_changed
        ).pack(side=tk.LEFT)

        ttk.Radiobutton(
            control_frame, text="Triangle",
            variable=self.current_tool, value="triangle",
            command=self._tool_changed
        ).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Label(control_frame, text="Thickness:").pack(side=tk.LEFT)
        thickness_scale = ttk.Scale(
            control_frame,
            from_=1,
            to=15,
            variable=self.freehand_thickness,
            orient=tk.HORIZONTAL,
            length=120
        )
        thickness_scale.pack(side=tk.LEFT, padx=(5, 12))

        ttk.Label(control_frame, text="Color:").pack(side=tk.LEFT)
        ttk.Radiobutton(
            control_frame, text="Red",
            variable=self.current_color, value="red"
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            control_frame, text="Green",
            variable=self.current_color, value="green"
        ).pack(side=tk.LEFT, padx=(0, 12))

        ttk.Button(
            control_frame,
            text="Show Reconstruction",
            command=self.show_reconstruction
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            control_frame,
            text="Clear",
            command=self.clear_canvas
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            control_frame,
            text="Done",
            command=self.root.destroy
        ).pack(side=tk.LEFT, padx=5)

        self.info_label = ttk.Label(
            self.root,
            text="Hover on the drawing to see pixel coordinates.",
            padding=(8, 2)
        )
        self.info_label.pack(side=tk.TOP, fill=tk.X)

        self.data_label = ttk.Label(
            self.root,
            text="Circle center=None | Triangle tip=None | Sampled trajectory points=0",
            padding=(8, 2)
        )
        self.data_label.pack(side=tk.TOP, fill=tk.X)

        self.canvas = tk.Canvas(
            self.root,
            width=self.canvas_width,
            height=self.canvas_height,
            bg=self.bg_color,
            highlightthickness=1,
            highlightbackground="gray"
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.load_background(self.background_path)

        self.canvas.bind("<ButtonPress-1>", self.on_mouse_down)
        self.canvas.bind("<B1-Motion>", self.on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_mouse_up)
        self.canvas.bind("<Motion>", self.on_mouse_move)

    def _tool_changed(self):
        self.triangle_points = []
        if self.preview_item is not None:
            self.canvas.delete(self.preview_item)
            self.preview_item = None

    def clear_canvas(self):
        for item in self.drawn_items:
            self.canvas.delete(item)
        self.drawn_items.clear()

        if self.preview_item is not None:
            self.canvas.delete(self.preview_item)
            self.preview_item = None

        self.start_x = None
        self.start_y = None
        self.triangle_points = []

        self.all_drawn_points = []
        self.freehand_raw_points = []
        self.freehand_sampled_points = []
        self.circle_center = None
        self.circle_radius = None
        self.circle_outline_points = []
        self.triangle_tip = None
        self.triangle_outline_points = []

        self.info_label.config(text="Hover on the drawing to see pixel coordinates.")
        self._update_data_label()

    def on_mouse_down(self, event):
        tool = self.current_tool.get()

        if tool in ("freehand", "circle"):
            self.start_x = event.x
            self.start_y = event.y

            if tool == "freehand":
                self.freehand_raw_points.append((event.x, event.y))
                self.all_drawn_points.append((event.x, event.y))

        elif tool == "triangle":
            self.handle_triangle_click(event.x, event.y)

    def on_mouse_drag(self, event):
        tool = self.current_tool.get()
        color = self.current_color.get()

        if tool == "freehand" and self.start_x is not None and self.start_y is not None:
            thickness = int(self.freehand_thickness.get())
            item = self.canvas.create_line(
                self.start_x, self.start_y, event.x, event.y,
                fill=color, width=thickness, capstyle=tk.ROUND, smooth=True
            )
            self.drawn_items.append(item)

            self.freehand_raw_points.append((event.x, event.y))
            self.all_drawn_points.append((event.x, event.y))

            self.start_x = event.x
            self.start_y = event.y

        elif tool == "circle" and self.start_x is not None and self.start_y is not None:
            if self.preview_item is not None:
                self.canvas.delete(self.preview_item)

            r = math.hypot(event.x - self.start_x, event.y - self.start_y)
            self.preview_item = self.canvas.create_oval(
                self.start_x - r, self.start_y - r,
                self.start_x + r, self.start_y + r,
                outline=color, width=2, dash=(4, 2)
            )

    def on_mouse_up(self, event):
        tool = self.current_tool.get()
        color = self.current_color.get()

        if tool == "circle" and self.start_x is not None and self.start_y is not None:
            if self.preview_item is not None:
                self.canvas.delete(self.preview_item)
                self.preview_item = None

            cx, cy = self.start_x, self.start_y
            r = math.hypot(event.x - cx, event.y - cy)

            if r > 2:
                item = self.canvas.create_oval(
                    cx - r, cy - r, cx + r, cy + r,
                    outline=color, width=2
                )
                self.drawn_items.append(item)

                self.circle_center = (int(cx), int(cy))
                self.circle_radius = float(r)
                self.circle_outline_points = self.sample_circle_points(cx, cy, r, num_points=60)
                self.all_drawn_points.extend(self.circle_outline_points)

            self.start_x = None
            self.start_y = None
            self._update_data_label()

        elif tool == "freehand":
            self.start_x = None
            self.start_y = None
            self.freehand_sampled_points = self.sample_polyline(self.freehand_raw_points, step=12)
            self._update_data_label()

    def on_mouse_move(self, event):
        nearest = self.find_nearest_point(event.x, event.y, threshold=10)
        if nearest is not None:
            self.info_label.config(
                text=f"Mouse: ({event.x}, {event.y}) | Nearest sketch pixel: ({nearest[0]}, {nearest[1]})"
            )
        else:
            self.info_label.config(
                text=f"Mouse: ({event.x}, {event.y}) | Nearest sketch pixel: None"
            )

    def handle_triangle_click(self, x, y):
        self.triangle_points.append((x, y))

        marker = self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="black", outline="black")
        self.drawn_items.append(marker)

        if len(self.triangle_points) == 1:
            self.triangle_tip = (x, y)
            self._update_data_label()

        elif len(self.triangle_points) == 2:
            p1, p2 = self.triangle_points
            item = self.canvas.create_line(
                p1[0], p1[1], p2[0], p2[1],
                fill=self.current_color.get(), width=2
            )
            self.drawn_items.append(item)

        elif len(self.triangle_points) == 3:
            p1, p2, p3 = self.triangle_points

            poly = self.canvas.create_polygon(
                [p1[0], p1[1], p2[0], p2[1], p3[0], p3[1]],
                outline=self.current_color.get(),
                fill="",
                width=2
            )
            self.drawn_items.append(poly)

            self.triangle_outline_points = self.sample_triangle_edges(p1, p2, p3, points_per_edge=30)
            self.all_drawn_points.extend(self.triangle_outline_points)

            self.triangle_points = []
            self._update_data_label()

    def sample_circle_points(self, cx, cy, r, num_points=60):
        pts = []
        for i in range(num_points):
            theta = 2 * math.pi * i / num_points
            x = int(cx + r * math.cos(theta))
            y = int(cy + r * math.sin(theta))
            pts.append((x, y))
        return pts

    def sample_line(self, p1, p2, num_points=20):
        x1, y1 = p1
        x2, y2 = p2
        pts = []
        for i in range(num_points + 1):
            t = i / num_points
            x = int(x1 + t * (x2 - x1))
            y = int(y1 + t * (y2 - y1))
            pts.append((x, y))
        return pts

    def sample_triangle_edges(self, p1, p2, p3, points_per_edge=30):
        pts = []
        pts.extend(self.sample_line(p1, p2, points_per_edge))
        pts.extend(self.sample_line(p2, p3, points_per_edge))
        pts.extend(self.sample_line(p3, p1, points_per_edge))
        return pts

    def sample_polyline(self, points, step=12):
        if not points:
            return []

        sampled = points[::step]
        if sampled[-1] != points[-1]:
            sampled.append(points[-1])

        cleaned = []
        prev = None
        for p in sampled:
            ip = (int(p[0]), int(p[1]))
            if ip != prev:
                cleaned.append(ip)
                prev = ip
        return cleaned

    def find_nearest_point(self, x, y, threshold=10):
        if not self.all_drawn_points:
            return None

        best_point = None
        best_dist_sq = threshold * threshold

        for px, py in self.all_drawn_points:
            d2 = (px - x) ** 2 + (py - y) ** 2
            if d2 <= best_dist_sq:
                best_dist_sq = d2
                best_point = (px, py)

        return best_point

    def _update_data_label(self):
        self.data_label.config(
            text=(
                f"Circle center={self.circle_center} | "
                f"Circle radius={self.circle_radius} | "
                f"Triangle tip={self.triangle_tip} | "
                f"Sampled trajectory points={len(self.freehand_sampled_points)}"
            )
        )

    def get_trajectory_data(self):
        return {
            "grasp": self.circle_center,
            "dropoff": self.triangle_tip,
            "grasp_radius": self.circle_radius,
            "waypoints": self.freehand_sampled_points
        }
    def show_reconstruction(self):
        recon = tk.Toplevel(self.root)
        recon.title("Reconstructed Trajectory")
        recon.geometry("950x700")

        info = ttk.Label(
            recon,
            text=(
                "Blue path = sampled trajectory waypoints | "
                "Orange = grasp point (circle center) | "
                "Purple = drop-off point (triangle tip)"
            ),
            padding=8
        )
        info.pack(side=tk.TOP, fill=tk.X)

        canvas = tk.Canvas(
            recon,
            width=self.canvas_width,
            height=self.canvas_height,
            bg="white",
            highlightthickness=1,
            highlightbackground="gray"
        )
        canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        pts = self.freehand_sampled_points
        if len(pts) >= 2:
            for i in range(len(pts) - 1):
                x1, y1 = pts[i]
                x2, y2 = pts[i + 1]
                canvas.create_line(x1, y1, x2, y2, width=2, fill="blue")
            for i, (x, y) in enumerate(pts):
                canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="blue", outline="blue")
                canvas.create_text(x + 18, y - 10, text=f"{i}:({x},{y})", font=("Arial", 8), fill="blue")
        elif len(pts) == 1:
            x, y = pts[0]
            canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="blue", outline="blue")
            canvas.create_text(x + 18, y - 10, text=f"0:({x},{y})", font=("Arial", 8), fill="blue")

        if self.circle_center is not None:
            x, y = self.circle_center
            canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill="orange", outline="black")
            canvas.create_text(x + 55, y, text=f"Grasp: ({x},{y})", fill="orange", font=("Arial", 10, "bold"))

        if self.triangle_tip is not None:
            x, y = self.triangle_tip
            canvas.create_oval(x - 7, y - 7, x + 7, y + 7, fill="purple", outline="black")
            canvas.create_text(x + 65, y, text=f"Drop-off: ({x},{y})", fill="purple", font=("Arial", 10, "bold"))

        if self.triangle_outline_points:
            for i in range(len(self.triangle_outline_points) - 1):
                x1, y1 = self.triangle_outline_points[i]
                x2, y2 = self.triangle_outline_points[i + 1]
                canvas.create_line(x1, y1, x2, y2, fill="gray")

        if len(self.circle_outline_points) > 1:
            for i in range(len(self.circle_outline_points)):
                x1, y1 = self.circle_outline_points[i]
                x2, y2 = self.circle_outline_points[(i + 1) % len(self.circle_outline_points)]
                canvas.create_line(x1, y1, x2, y2, fill="gray")

        summary = (
            f"Saved Data:\n"
            f"  Circle center (grasp point): {self.circle_center}\n"
            f"  Triangle tip (drop-off point): {self.triangle_tip}\n"
            f"  Sampled trajectory waypoints: {self.freehand_sampled_points}"
        )
        ttk.Label(recon, text=summary, padding=8, justify=tk.LEFT).pack(side=tk.BOTTOM, fill=tk.X)



def launch_gui(background_path="pybullet_scene_snapshot.png"):
    root = tk.Tk()
    app = SketchRecorderApp(root, background_path=background_path)
    root.mainloop()
    return app.get_trajectory_data()

if __name__ == "__main__":
    data = launch_gui()
    print("Returned data:", data)