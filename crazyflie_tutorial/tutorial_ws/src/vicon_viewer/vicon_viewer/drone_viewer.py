#!/usr/bin/env python3
"""
3D drone viewer using vispy, subscribing to Crazyswarm2's /poses topic.

Self-contained: all vispy scene building, data bridging, obstacle
rendering, and trajectory logging are embedded in this file.
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from motion_capture_tracking_interfaces.msg import NamedPoseArray

from vispy import app, scene
from vispy.scene import visuals as scene_visuals

# ---------------------------------------------------------------------------
# Theme — world geometry is in meters, screen values are in pixels
# ---------------------------------------------------------------------------

BG_COLOR        = (0.07, 0.085, 0.115, 1.0)
FLOOR_COLOR     = (0.105, 0.125, 0.155, 1.0)
GRID_MINOR      = (0.18, 0.21, 0.27, 0.55)
GRID_MAJOR      = (0.34, 0.42, 0.55, 0.85)
FLOOR_BORDER    = (0.45, 0.55, 0.72, 0.7)
AXIS_X_RGB      = (1.00, 0.32, 0.36)
AXIS_Y_RGB      = (0.42, 0.95, 0.50)
AXIS_Z_RGB      = (0.40, 0.65, 1.00)
TEXT_PRIMARY    = (0.88, 0.91, 0.96, 1.0)
TEXT_DIM        = (0.55, 0.60, 0.68, 1.0)
TEXT_ACCENT     = (0.55, 0.78, 1.00, 1.0)
PANEL_FILL      = (0.04, 0.06, 0.10, 0.78)
PANEL_BORDER    = (0.22, 0.28, 0.36, 0.9)
ORIGIN_DOT      = (0.78, 0.83, 0.92, 0.9)
EDGE_HIGHLIGHT  = (1.0, 1.0, 1.0, 0.85)

# Scene geometry constants (meters)
DEFAULT_FLOOR_HALF = 3.5       # 7m x 7m floor
DEFAULT_GRID_MINOR = 0.25      # minor grid spacing
DEFAULT_GRID_MAJOR = 1.0       # major grid spacing
DEFAULT_AXIS_LENGTH = 0.5      # length of XYZ axes
DEFAULT_MARKER_SIZE = 11.0     # pixels (screen-space, not meters)
FLOOR_Z_MINOR = 0.0005         # tiny lifts avoid z-fighting with the floor
FLOOR_Z_MAJOR = 0.0006
FLOOR_Z_BORDER = 0.0007
FLOOR_Z_VIS = 0.001            # shadow and drop-line floor projection height
LABEL_Z_OFFSET = 0.07          # height of name labels above each drone
AXIS_LABEL_PAD = 0.13          # distance from axis tip to label text

OBSTACLE_COLOR = (0.6, 0.3, 0.3, 0.15)
OBSTACLE_EDGE_COLOR = (0.8, 0.4, 0.4, 0.4)
OBSTACLE_HEIGHT = 0.4  # meters

# Bright, saturated palette designed to pop against the dark background.
_DRONE_COLORS = [
    (0.30, 0.70, 1.00), (1.00, 0.40, 0.45), (0.45, 0.95, 0.50),
    (1.00, 0.78, 0.25), (0.85, 0.50, 1.00), (0.30, 0.95, 0.85),
    (1.00, 0.55, 0.30), (0.95, 0.45, 0.75), (0.55, 0.90, 1.00),
    (0.90, 0.92, 0.40), (0.60, 0.75, 1.00), (1.00, 0.65, 0.55),
    (0.35, 0.85, 0.65), (0.80, 0.65, 1.00), (0.95, 0.85, 0.55),
    (0.40, 0.80, 0.95), (1.00, 0.50, 0.85), (0.70, 1.00, 0.55),
    (0.95, 0.70, 0.40), (0.55, 0.95, 1.00),
]

def drone_color(name: str) -> tuple[float, float, float, float]:
    """Return a stable RGBA color for a drone name."""
    # Do not use Python's built-in hash(); it is intentionally salted per run.
    import hashlib
    idx = int(hashlib.md5(name.encode("utf-8")).hexdigest(), 16) % len(_DRONE_COLORS)
    return _DRONE_COLORS[idx] + (1.0,)


def trail_color_array(rgb, n_points: int,
                      max_alpha: float = 0.95,
                      gamma: float = 1.4) -> np.ndarray:
    """Per-vertex RGBA where oldest point is transparent and newest is bright."""
    colors = np.tile(np.array([*rgb, 0.0], dtype=np.float32), (n_points, 1))
    t = np.linspace(0.0, 1.0, n_points, dtype=np.float32)
    colors[:, 3] = max_alpha * (t ** gamma)
    return colors


def shadow_color_array(n_points: int, max_alpha: float = 0.30) -> np.ndarray:
    """Greyscale alpha-fade for floor projections."""
    colors = np.zeros((n_points, 4), dtype=np.float32)
    t = np.linspace(0.0, 1.0, n_points, dtype=np.float32)
    colors[:, 3] = max_alpha * (t ** 1.6)
    return colors

class DataBridge:
    """Thread-safe store for drone positions and time-stamped trails."""

    def __init__(self, trail_duration: float = 30.0):
        self.trail_duration = trail_duration
        self.lock = threading.Lock()
        self.current: dict[str, tuple[float, float, float]] = {}
        self.trails: dict[str, deque] = {}
        self.frame_number = 0
        self.frame_rate = 0.0
        self.num_drones = 0

    def update(self, drones: dict[str, tuple[float, float, float]],
               frame_number: int, frame_rate: float):
        """Called from ROS2 callback. Stores latest positions."""
        now = time.time()
        with self.lock:
            self.current = dict(drones)
            self.frame_number = frame_number
            self.frame_rate = frame_rate
            self.num_drones = len(drones)

            for name, pos in drones.items():
                if name not in self.trails:
                    self.trails[name] = deque()
                self.trails[name].append((now, pos[0], pos[1], pos[2]))

            # Remove trail history for drones that disappeared
            for name in list(self.trails):
                if name not in drones:
                    self.trails[name].clear()
                    del self.trails[name]

    def snapshot(self):
        """Called from vispy timer. Returns a consistent snapshot."""
        now = time.time()
        cutoff = now - self.trail_duration
        with self.lock:
            trails_out: dict[str, list[tuple[float, float, float]]] = {}
            for name, trail in self.trails.items():
                # Prune entries older than trail_duration
                while trail and trail[0][0] < cutoff:
                    trail.popleft()
                if trail:
                    trails_out[name] = [(t[1], t[2], t[3]) for t in trail]
            return (
                dict(self.current),
                trails_out,
                self.frame_number,
                self.frame_rate,
                self.num_drones,
            )

class TrajectoryLogger:
    """CSV logger for /poses trajectories."""

    def __init__(self, output_dir: str = "recordings"):
        self._output_dir = output_dir
        self._file = None
        self._writer = None
        self._lock = threading.Lock()
        self._recording = False
        self._frame_count = 0
        self._start_time = 0.0
        self._path = ""

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def path(self) -> str:
        return self._path

    @property
    def start_time(self) -> float:
        return self._start_time

    def start(self) -> str:
        """Begin recording. Returns the CSV file path."""
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = os.path.join(self._output_dir, f"trajectory_{ts}.csv")
        with self._lock:
            self._file = open(self._path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow([
                "timestamp_s", "frame", "drone_name", "x_m", "y_m", "z_m",
            ])
            self._frame_count = 0
            self._start_time = time.time()
            self._recording = True
        return self._path

    def stop(self) -> str:
        """Stop recording. Returns the CSV file path."""
        with self._lock:
            self._recording = False
            if self._file is not None:
                self._file.close()
                self._file = None
                self._writer = None
            return self._path

    def log_frame(self, drones: dict[str, tuple[float, float, float]],
                  frame_number: int) -> None:
        """Write one frame of position data (if recording)."""
        with self._lock:
            if not self._recording or self._writer is None:
                return
            t = time.time() - self._start_time
            for name, (x, y, z) in drones.items():
                self._writer.writerow([
                    f"{t:.6f}", frame_number, name,
                    f"{x:.5f}", f"{y:.5f}", f"{z:.5f}",
                ])
            self._frame_count += 1

def build_vispy_scene() -> dict:
    """Build the full vispy scene and return mutable visual objects."""
    canvas = scene.SceneCanvas(
        keys="interactive",
        size=(1280, 800),
        title="Drone Viewer",
        bgcolor=BG_COLOR,
    )

    view = canvas.central_widget.add_view()
    view.camera = scene.cameras.TurntableCamera(
        elevation=22,
        azimuth=42,
        center=(0, 0, 0.4),
        distance=4.5,
        scale_factor=4.5,
        fov=45,
        up="+z",
    )

    f = DEFAULT_FLOOR_HALF

    # Floor mesh at z=0
    floor_verts = np.array([
        [-f, -f, 0], [f, -f, 0], [f, f, 0],
        [-f, -f, 0], [f, f, 0], [-f, f, 0],
    ], dtype=np.float32)
    scene_visuals.Mesh(vertices=floor_verts, color=FLOOR_COLOR, parent=view.scene)

    # Minor grid (0.25m spacing, skip where major lines go)
    minor_pts: list[list[list[float]]] = []
    n_minor = int(f / DEFAULT_GRID_MINOR)
    major_step = max(1, int(round(DEFAULT_GRID_MAJOR / DEFAULT_GRID_MINOR)))
    for i in range(-n_minor, n_minor + 1):
        if i % major_step == 0:
            continue
        offset = i * DEFAULT_GRID_MINOR
        minor_pts.append([[-f, offset, FLOOR_Z_MINOR], [f, offset, FLOOR_Z_MINOR]])
        minor_pts.append([[offset, -f, FLOOR_Z_MINOR], [offset, f, FLOOR_Z_MINOR]])
    if minor_pts:
        scene_visuals.Line(
            pos=np.array(minor_pts, dtype=np.float32).reshape(-1, 3),
            color=GRID_MINOR, connect="segments", method="gl",
            parent=view.scene,
        )

    # Major grid (1.0m spacing, thicker and brighter)
    major_pts: list[list[list[float]]] = []
    n_major = int(f / DEFAULT_GRID_MAJOR)
    for i in range(-n_major, n_major + 1):
        offset = i * DEFAULT_GRID_MAJOR
        major_pts.append([[-f, offset, FLOOR_Z_MAJOR], [f, offset, FLOOR_Z_MAJOR]])
        major_pts.append([[offset, -f, FLOOR_Z_MAJOR], [offset, f, FLOOR_Z_MAJOR]])
    if major_pts:
        scene_visuals.Line(
            pos=np.array(major_pts, dtype=np.float32).reshape(-1, 3),
            color=GRID_MAJOR, connect="segments", method="gl",
            parent=view.scene,
        )

    # Floor border
    border_pts = np.array([
        [-f, -f, FLOOR_Z_BORDER], [ f, -f, FLOOR_Z_BORDER],
        [ f, -f, FLOOR_Z_BORDER], [ f,  f, FLOOR_Z_BORDER],
        [ f,  f, FLOOR_Z_BORDER], [-f,  f, FLOOR_Z_BORDER],
        [-f,  f, FLOOR_Z_BORDER], [-f, -f, FLOOR_Z_BORDER],
    ], dtype=np.float32)
    scene_visuals.Line(
        pos=border_pts, color=FLOOR_BORDER,
        connect="segments", method="gl", parent=view.scene,
    )

    # XYZ axes (per-vertex colors: X=red, Y=green, Z=blue)
    axis_len = DEFAULT_AXIS_LENGTH
    axis_pts = np.array([
        [0, 0, 0], [axis_len, 0, 0],
        [0, 0, 0], [0, axis_len, 0],
        [0, 0, 0], [0, 0, axis_len],
    ], dtype=np.float32)
    axis_cols = np.array([
        [*AXIS_X_RGB, 1.0], [*AXIS_X_RGB, 1.0],
        [*AXIS_Y_RGB, 1.0], [*AXIS_Y_RGB, 1.0],
        [*AXIS_Z_RGB, 1.0], [*AXIS_Z_RGB, 1.0],
    ], dtype=np.float32)
    scene_visuals.Line(
        pos=axis_pts, color=axis_cols, connect="segments",
        method="gl", parent=view.scene, width=3.0,
    )

    # Axis labels (X, Y, Z)
    label_off = axis_len + AXIS_LABEL_PAD
    for char, pos, col in [
        ("X", (label_off, 0, 0), AXIS_X_RGB),
        ("Y", (0, label_off, 0), AXIS_Y_RGB),
        ("Z", (0, 0, label_off), AXIS_Z_RGB),
    ]:
        scene_visuals.Text(
            text=char, parent=view.scene, pos=pos,
            color=(*col, 1.0), font_size=14, bold=True,
            anchor_x="center", anchor_y="center",
        )

    # Origin dot
    scene_visuals.Markers(parent=view.scene).set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([list(ORIGIN_DOT)], dtype=np.float32),
        edge_color=None, size=6,
    )

    # Marker scatter: halo (larger, low-alpha) + core (normal-size, bright edge)
    halo_scatter = scene_visuals.Markers(scaling=False)
    halo_scatter.set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([[0, 0, 0, 0]], dtype=np.float32),
        edge_color=None, size=1,
    )
    view.add(halo_scatter)

    core_scatter = scene_visuals.Markers(scaling=False)
    core_scatter.set_data(
        pos=np.array([[0, 0, 0]], dtype=np.float32),
        face_color=np.array([[0, 0, 0, 0]], dtype=np.float32),
        edge_color=None, size=1,
    )
    view.add(core_scatter)

    # 2D overlay view for HUD, button, and legend
    overlay_view = canvas.central_widget.add_view()
    overlay_view.camera = scene.cameras.PanZoomCamera(
        rect=(0, 0, canvas.size[0], canvas.size[1]))
    overlay_view.interactive = False
    overlay_view.order = 1  # render on top of the 3D scene

    panel_w, panel_h = 480, 124
    panel_margin = 18
    panel_pad_x = 18
    title_off_y = 22
    line_spacing = 22
    line0_off_y = title_off_y + 30

    panel = scene_visuals.Rectangle(
        center=(panel_margin + panel_w / 2,
                canvas.size[1] - panel_margin - panel_h / 2),
        width=panel_w, height=panel_h,
        color=PANEL_FILL, border_color=PANEL_BORDER,
        parent=overlay_view.scene,
    )

    title_text = scene_visuals.Text(
        text="VICON /POSES VIEWER",
        pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - title_off_y),
        color=TEXT_ACCENT, font_size=11, bold=True,
        anchor_x="left", anchor_y="top",
        parent=overlay_view.scene,
    )
    conn_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y),
        color=TEXT_DIM, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )
    frame_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y - line_spacing),
        color=TEXT_PRIMARY, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )
    active_text = scene_visuals.Text(
        text="", pos=(panel_margin + panel_pad_x,
             canvas.size[1] - panel_margin - line0_off_y - 2 * line_spacing),
        color=TEXT_PRIMARY, font_size=10,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )

    # Hint text (bottom-right)
    hint_margin = 28
    hint_text = scene_visuals.Text(
        text="R reset view   |   +/- trail length   |   S rec   |   Q/Esc quit",
        pos=(canvas.size[0] - hint_margin, hint_margin),
        color=TEXT_DIM, font_size=10,
        anchor_x="right", anchor_y="bottom",
        parent=overlay_view.scene,
    )

    # Recording button and status (bottom-left)
    rec_btn_x, rec_btn_y = 20, 55
    rec_btn_w, rec_btn_h = 105, 30
    rec_btn_rect = scene_visuals.Rectangle(
        center=(rec_btn_x + rec_btn_w / 2, rec_btn_y + rec_btn_h / 2),
        width=rec_btn_w, height=rec_btn_h,
        color=(0.18, 0.19, 0.24, 0.88),
        border_color=(0.32, 0.33, 0.40, 0.9),
        parent=overlay_view.scene,
    )
    rec_btn_text = scene_visuals.Text(
        text="●  REC",
        pos=(rec_btn_x + rec_btn_w / 2, rec_btn_y + rec_btn_h / 2),
        color=(0.50, 0.52, 0.58, 1.0), font_size=11, bold=True,
        anchor_x="center", anchor_y="center",
        parent=overlay_view.scene,
    )
    rec_status_text = scene_visuals.Text(
        text="", pos=(rec_btn_x, rec_btn_y - 18),
        color=TEXT_DIM, font_size=9,
        anchor_x="left", anchor_y="top", face="monospace",
        parent=overlay_view.scene,
    )

    # Top-right legend panel
    legend_margin = 80
    legend_w = 380
    legend_pad_x = 12
    legend_title_off_y = 22
    legend_row_start_y = legend_title_off_y + 26
    legend_row_spacing = 18

    legend_panel = scene_visuals.Rectangle(
        center=(canvas.size[0] - legend_margin - legend_w / 2,
                canvas.size[1] - panel_margin - 100 / 2),
        width=legend_w, height=100,
        color=PANEL_FILL, border_color=PANEL_BORDER,
        parent=overlay_view.scene,
    )
    legend_title = scene_visuals.Text(
        text="DRONES",
        pos=(canvas.size[0] - legend_margin - legend_w + legend_pad_x,
             canvas.size[1] - panel_margin - legend_title_off_y),
        color=TEXT_ACCENT, font_size=11, bold=True,
        anchor_x="left", anchor_y="top",
        parent=overlay_view.scene,
    )

    # Pre-allocate legend rows (swatch + name text per row)
    max_legend_rows = 20
    legend_swatches = []
    legend_rows = []
    for _ in range(max_legend_rows):
        swatch = scene_visuals.Rectangle(
            center=(0, 0), width=10, height=10,
            color=(0, 0, 0, 0), border_color=None,
            parent=overlay_view.scene,
        )
        legend_swatches.append(swatch)
        row_text = scene_visuals.Text(
            text="", pos=(0, 0),
            color=TEXT_PRIMARY, font_size=9,
            anchor_x="left", anchor_y="top", face="monospace",
            parent=overlay_view.scene,
        )
        legend_rows.append(row_text)
    
    return {
        "canvas": canvas, "view": view,
        "halo_scatter": halo_scatter, "core_scatter": core_scatter,
        "trails": {}, "shadows": {}, "drops": {}, "labels": {},
        "overlay_view": overlay_view,
        "panel": panel, "title_text": title_text,
        "conn_text": conn_text, "frame_text": frame_text,
        "active_text": active_text, "hint_text": hint_text,
        "panel_w": panel_w, "panel_h": panel_h,
        "panel_margin": panel_margin, "panel_pad_x": panel_pad_x,
        "title_off_y": title_off_y, "line0_off_y": line0_off_y,
        "line_spacing": line_spacing, "hint_margin": hint_margin,
        "marker_size": DEFAULT_MARKER_SIZE,
        "rec_btn_rect": rec_btn_rect, "rec_btn_text": rec_btn_text,
        "rec_status_text": rec_status_text,
        "rec_btn_x": rec_btn_x, "rec_btn_y": rec_btn_y,
        "rec_btn_w": rec_btn_w, "rec_btn_h": rec_btn_h,
        "legend_margin": legend_margin, "legend_w": legend_w,
        "legend_pad_x": legend_pad_x,
        "legend_title_off_y": legend_title_off_y,
        "legend_row_start_y": legend_row_start_y,
        "legend_row_spacing": legend_row_spacing,
        "legend_panel": legend_panel, "legend_title": legend_title,
        "legend_swatches": legend_swatches, "legend_rows": legend_rows,
    }

def load_obstacles(csv_path: str) -> list[dict]:
    """Parse obstacles CSV. Columns: center_x, center_y, length_x, length_y, theta."""
    obstacles = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            obstacles.append({
                "center": (float(row["center_x"]), float(row["center_y"])),
                "length": (float(row["length_x"]), float(row["length_y"])),
                "theta": float(row["theta"]),
            })
    return obstacles

def add_obstacles_to_scene(view, obstacles: list[dict]):
    """Render obstacles as semi-transparent boxes with wireframe edges."""
    for obs in obstacles:
        cx, cy = obs["center"]
        lx, ly = obs["length"]
        theta = obs["theta"]

        # Rotate the four corners by theta
        c, s = math.cos(theta), math.sin(theta)
        half_x, half_y = lx / 2.0, ly / 2.0
        corners_local = np.array([
            [-half_x, -half_y], [half_x, -half_y],
            [half_x, half_y], [-half_x, half_y],
        ])
        R = np.array([[c, -s], [s, c]])
        corners_world = (R @ corners_local.T).T + np.array([cx, cy])

        # Bottom face (z=0) and top face (z=OBSTACLE_HEIGHT)
        bottom = np.array([
            [corners_world[i][0], corners_world[i][1], 0.0]
            for i in range(4)
        ], dtype=np.float32)
        top = bottom.copy()
        top[:, 2] = OBSTACLE_HEIGHT

        verts = np.vstack([bottom, top])
        faces = np.array([
            [0, 1, 2], [0, 2, 3],   # bottom
            [4, 5, 6], [4, 6, 7],   # top
            [0, 1, 5], [0, 5, 4],   # front
            [1, 2, 6], [1, 6, 5],   # right
            [2, 3, 7], [2, 7, 6],   # back
            [3, 0, 4], [3, 4, 7],   # left
        ], dtype=np.uint32)
        scene_visuals.Mesh(
            vertices=verts, faces=faces,
            color=OBSTACLE_COLOR, parent=view.scene,
        )

        # Wireframe edges (12 edges of the box)
        edge_pts = np.array([
            bottom[0], bottom[1], bottom[1], bottom[2],
            bottom[2], bottom[3], bottom[3], bottom[0],
            top[0], top[1], top[1], top[2],
            top[2], top[3], top[3], top[0],
            bottom[0], top[0], bottom[1], top[1],
            bottom[2], top[2], bottom[3], top[3],
        ], dtype=np.float32)
        scene_visuals.Line(
            pos=edge_pts, color=OBSTACLE_EDGE_COLOR,
            connect="segments", method="gl", parent=view.scene, width=1.0,
        )

class PoseSubscriber(Node):
    """Subscribe to /poses and feed the DataBridge."""

    def __init__(self, bridge: DataBridge):
        super().__init__("drone_viewer_pose_subscriber")
        self._bridge = bridge
        self._frame_count = 0
        self._last_msg_time: float | None = None
        self._frame_rate = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(NamedPoseArray, "/poses", self._on_poses, qos)
        self.get_logger().info("Listening on /poses")

    def _on_poses(self, msg: NamedPoseArray):
        """Callback for each /poses message."""
        drones: dict[str, tuple[float, float, float]] = {}
        for named_pose in msg.poses:
            p = named_pose.pose.position
            if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
                continue
            # /poses is already in meters — no unit conversion needed
            drones[named_pose.name] = (float(p.x), float(p.y), float(p.z))

        # Exponential moving average of frame rate
        now = time.time()
        if self._last_msg_time is not None:
            dt = now - self._last_msg_time
            if dt > 0:
                instant = 1.0 / max(dt, 1e-6)
                self._frame_rate = 0.9 * self._frame_rate + 0.1 * instant
        self._last_msg_time = now

        self._frame_count += 1
        self._bridge.update(drones, self._frame_count, self._frame_rate)
    
class DroneViewer:
    """Owns the vispy scene, ROS spin timer, HUD, legend, and per-frame updates."""

    def __init__(self, bridge: DataBridge, subscriber: PoseSubscriber,
                 logger: TrajectoryLogger | None = None):
        self.bridge = bridge
        self._subscriber = subscriber
        self._logger = logger

        # Build the 3D scene and unpack the returned dictionary
        s = build_vispy_scene()
        self.canvas        = s["canvas"]
        self._view         = s["view"]
        self.halo_scatter  = s["halo_scatter"]
        self.core_scatter  = s["core_scatter"]
        self._trails       = s["trails"]
        self._shadows      = s["shadows"]
        self._drops        = s["drops"]
        self._labels       = s["labels"]
        self._overlay_view = s["overlay_view"]
        self._panel        = s["panel"]
        self._title_text   = s["title_text"]
        self._conn_text    = s["conn_text"]
        self._frame_text   = s["frame_text"]
        self._active_text  = s["active_text"]
        self._hint_text    = s["hint_text"]
        self._panel_w      = s["panel_w"]
        self._panel_h      = s["panel_h"]
        self._panel_margin = s["panel_margin"]
        self._panel_pad_x  = s["panel_pad_x"]
        self._title_off_y  = s["title_off_y"]
        self._line0_off_y  = s["line0_off_y"]
        self._line_spacing = s["line_spacing"]
        self._hint_margin  = s["hint_margin"]
        self._marker_size  = s["marker_size"]

        self._rec_btn_rect    = s["rec_btn_rect"]
        self._rec_btn_text    = s["rec_btn_text"]
        self._rec_status_text = s["rec_status_text"]
        self._rec_btn_x       = s["rec_btn_x"]
        self._rec_btn_y       = s["rec_btn_y"]
        self._rec_btn_w       = s["rec_btn_w"]
        self._rec_btn_h       = s["rec_btn_h"]

        self._legend_margin      = s["legend_margin"]
        self._legend_w           = s["legend_w"]
        self._legend_pad_x       = s["legend_pad_x"]
        self._legend_title_off_y = s["legend_title_off_y"]
        self._legend_row_start_y = s["legend_row_start_y"]
        self._legend_row_spacing = s["legend_row_spacing"]
        self._legend_panel       = s["legend_panel"]
        self._legend_title       = s["legend_title"]
        self._legend_swatches    = s["legend_swatches"]
        self._legend_rows        = s["legend_rows"]

        self.known_names: set[str] = set()

        # Connect events
        self.canvas.events.resize.connect(self._relayout_hud)
        self.canvas.events.key_press.connect(self._on_key)
        self.canvas.events.mouse_press.connect(self._on_mouse_press)
        self._relayout_hud()  # initial layout

        # ROS2 executor (processes callbacks cooperatively on the main thread)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(subscriber)

        # Timers: display at 30Hz, ROS spin at 100Hz
        self._timer = app.Timer(
            interval=0.033, connect=self._update_display, start=True)
        self._ros_timer = app.Timer(
            interval=0.01, connect=self._spin_ros, start=True)

    def _relayout_hud(self, *_args):
        """Reposition HUD elements when the canvas is resized."""
        w, h = self.canvas.size
        self._overlay_view.camera.rect = (0, 0, w, h)

        self._panel.center = (
            self._panel_margin + self._panel_w / 2,
            h - self._panel_margin - self._panel_h / 2,
        )

        x = self._panel_margin + self._panel_pad_x
        self._title_text.pos  = (x, h - self._panel_margin - self._title_off_y)
        self._conn_text.pos   = (x, h - self._panel_margin - self._line0_off_y)
        self._frame_text.pos  = (x, h - self._panel_margin - self._line0_off_y - self._line_spacing)
        self._active_text.pos = (x, h - self._panel_margin - self._line0_off_y - 2 * self._line_spacing)
        self._hint_text.pos   = (w - self._hint_margin, self._hint_margin)

        legend_left = w - self._legend_margin - self._legend_w
        self._legend_title.pos = (
            legend_left + self._legend_pad_x,
            h - self._panel_margin - self._legend_title_off_y,
        )
    
    def _on_key(self, event):
        """Handle key presses."""
        key = event.key
        if key in ("Q", "Escape"):
            self.canvas.close()
        elif key == "R":
            self._view.camera.reset()
            self._view.camera.center = (0, 0, 0.4)
            self._view.camera.scale_factor = 4.5
        elif key in ("+", "="):
            self.bridge.trail_duration = min(120, self.bridge.trail_duration + 5)
            print(f"Trail duration: {self.bridge.trail_duration:.0f}s")
        elif key == "-":
            self.bridge.trail_duration = max(2, self.bridge.trail_duration - 5)
            print(f"Trail duration: {self.bridge.trail_duration:.0f}s")
        elif key == "S":
            self._toggle_recording()

    def _on_mouse_press(self, event):
        """Detect clicks on the recording button."""
        x, y = event.pos[:2]
        bx, by = self._rec_btn_x, self._rec_btn_y
        bw, bh = self._rec_btn_w, self._rec_btn_h
        if bx <= x <= bx + bw and by <= y <= by + bh:
            self._toggle_recording()

    def _toggle_recording(self):
        """Start or stop CSV recording."""
        if self._logger is None:
            return
        if self._logger.is_recording:
            path = self._logger.stop()
            print(f"Recording stopped: {path} ({self._logger.frame_count} frames)")
        else:
            path = self._logger.start()
            print(f"Recording started: {path}")
    
    def _update_display(self, _event):
        """Called at 30Hz. Reads latest data and updates all visuals."""
        current, trail_data, frame_num, frame_rate, num_drones = \
            self.bridge.snapshot()
    
        # --- Scatter (halo + core) ---
        if current:
            names = sorted(current.keys())
            positions = np.array([current[name] for name in names], dtype=np.float32)
            colors = np.array([drone_color(name) for name in names], dtype=np.float32)

            halo_colors = colors.copy()
            halo_colors[:, 3] = 0.28
            self.halo_scatter.set_data(
                pos=positions, face_color=halo_colors,
                edge_color=None, size=self._marker_size * 2.4,
            )

            edge = np.tile(np.array(EDGE_HIGHLIGHT, dtype=np.float32),
                           (len(names), 1))
            self.core_scatter.set_data(
                pos=positions, face_color=colors,
                edge_color=edge, edge_width=1.4,
                size=self._marker_size,
            )
        else:
            # Hide markers when no data is available
            invisible = np.array([[0, 0, 0]], dtype=np.float32)
            clear = np.array([[0, 0, 0, 0]], dtype=np.float32)
            self.halo_scatter.set_data(pos=invisible, face_color=clear,
                                       edge_color=None, size=1)
            self.core_scatter.set_data(pos=invisible, face_color=clear,
                                       edge_color=None, size=1)
    
        # --- Remove visuals for drones that disappeared ---
        active_names = set(trail_data.keys())
        for name in list(self.known_names):
            if name not in active_names:
                for group in (self._trails, self._shadows,
                              self._drops, self._labels):
                    if name in group:
                        group[name].parent = None
                        del group[name]
        self.known_names.clear()
        self.known_names.update(active_names)

        # --- Update per-drone visuals ---
        for name in active_names:
            pts = trail_data[name]
            if len(pts) < 2:
                continue

            base = drone_color(name)
            arr = np.array(pts, dtype=np.float32)
            n = len(arr)

            # Trail line (alpha gradient from old to new)
            tcol = trail_color_array(base[:3], n, max_alpha=0.95, gamma=1.4)
            if name not in self._trails:
                line = scene_visuals.Line(
                    pos=arr, color=tcol, connect="strip",
                    method="gl", width=2.5,
                )
                self._view.add(line)
                self._trails[name] = line
            else:
                self._trails[name].set_data(pos=arr, color=tcol)

            # Floor shadow (projection at z=FLOOR_Z_VIS)
            shadow_pts = arr.copy()
            shadow_pts[:, 2] = FLOOR_Z_VIS
            scol = shadow_color_array(n, max_alpha=0.30)
            if name not in self._shadows:
                shadow = scene_visuals.Line(
                    pos=shadow_pts, color=scol, connect="strip",
                    method="gl", width=1.4,
                )
                self._view.add(shadow)
                self._shadows[name] = shadow
            else:
                self._shadows[name].set_data(pos=shadow_pts, color=scol)

            # Drop line (vertical from drone to floor)
            if name in current:
                x, y, z = current[name]
                drop_pts = np.array(
                    [[x, y, z], [x, y, FLOOR_Z_VIS]], dtype=np.float32)
                drop_col = np.array([
                    [base[0], base[1], base[2], 0.55],   # top (drone)
                    [base[0], base[1], base[2], 0.08],   # bottom (floor)
                ], dtype=np.float32)
                if name not in self._drops:
                    drop = scene_visuals.Line(
                        pos=drop_pts, color=drop_col, connect="strip",
                        method="gl", width=1.2,
                    )
                    self._view.add(drop)
                    self._drops[name] = drop
                else:
                    self._drops[name].set_data(pos=drop_pts, color=drop_col)

                # Text label (drone name floating above the marker)
                label_pos = (x, y, z + LABEL_Z_OFFSET)
                if name in self._labels:
                    self._labels[name].pos = label_pos
                    self._labels[name].text = name
                else:
                    self._labels[name] = scene_visuals.Text(
                        text=name, parent=self._view.scene,
                        pos=label_pos, color=base[:3] + (0.95,),
                        font_size=10, bold=True,
                        anchor_x="center", anchor_y="bottom",
                    )
    
        # --- HUD text ---
        self._conn_text.text = "/poses  ·  BEST_EFFORT  ·  meters"
        self._frame_text.text = (
            f"Frame  {frame_num:>10d}     {frame_rate:6.1f} Hz")
        self._active_text.text = (
            f"Active {num_drones:>10d}     "
            f"Trail {self.bridge.trail_duration:5.1f} s")

        # --- Legend panel ---
        w, h = self.canvas.size
        legend_left = w - self._legend_margin - self._legend_w
        active_names_sorted = sorted(current.keys()) if current else []

        for i in range(len(self._legend_swatches)):
            if i < len(active_names_sorted):
                name = active_names_sorted[i]
                x, y, z = current[name]
                row_y = (h - self._panel_margin - self._legend_row_start_y
                         - i * self._legend_row_spacing)
                swatch_x = legend_left + self._legend_pad_x + 4

                self._legend_swatches[i].center = (swatch_x, row_y - 4)
                self._legend_swatches[i].color = drone_color(name)[:3] + (0.95,)

                self._legend_rows[i].pos = (
                    legend_left + self._legend_pad_x + 14, row_y)
                self._legend_rows[i].text = (
                    f"{name:<8s} ({x:+.3f}, {y:+.3f}, {z:+.3f}) m")
                self._legend_rows[i].color = TEXT_PRIMARY
            else:
                # Hide unused rows
                self._legend_swatches[i].center = (-100, -100)
                self._legend_rows[i].text = ""

        # Resize legend panel to fit visible rows
        n = max(len(active_names_sorted), 1)
        legend_panel_h = (self._legend_row_start_y
                          + n * self._legend_row_spacing + 10)
        self._legend_panel.height = legend_panel_h
        self._legend_panel.center = (
            legend_left + self._legend_w / 2,
            h - self._panel_margin - legend_panel_h / 2,
        )

        # --- Recording UI ---
        if self._logger is not None and self._logger.is_recording:
            if current:
                self._logger.log_frame(current, frame_num)
            elapsed = time.time() - self._logger.start_time
            mins, secs = divmod(int(elapsed), 60)
            self._rec_status_text.text = (
                f"● REC  {mins:02d}:{secs:02d}  ·  "
                f"{self._logger.frame_count} frames")
            self._rec_status_text.color = (1.0, 0.35, 0.40, 1.0)
            self._rec_btn_rect.color = (0.55, 0.12, 0.15, 0.90)
            self._rec_btn_rect.border_color = (0.90, 0.25, 0.30, 0.95)
            self._rec_btn_text.text = "■  STOP"
            self._rec_btn_text.color = (1.0, 0.35, 0.40, 1.0)
        elif self._logger is not None:
            self._rec_status_text.text = ""
            self._rec_btn_rect.color = (0.18, 0.19, 0.24, 0.88)
            self._rec_btn_rect.border_color = (0.32, 0.33, 0.40, 0.9)
            self._rec_btn_text.text = "●  REC"
            self._rec_btn_text.color = (0.50, 0.52, 0.58, 1.0)

        self.canvas.update()

    def _spin_ros(self, _event):
        """Drain ROS2 callbacks without blocking the GUI. Called at ~100Hz."""
        for _ in range(10):
            self._executor.spin_once(timeout_sec=0.0)

    def run(self):
        """Show the canvas and enter the vispy event loop. Blocks until closed."""
        try:
            self.canvas.show()
            app.run()
        except KeyboardInterrupt:
            pass
        finally:
            if self._logger is not None and self._logger.is_recording:
                path = self._logger.stop()
                print(f"Recording stopped: {path} "
                      f"({self._logger.frame_count} frames)")
            self._executor.remove_node(self._subscriber)
    
def main():
    parser = argparse.ArgumentParser(description="Drone 3D Viewer")
    parser.add_argument("--output-dir", default="recordings",
                        help="CSV recording output directory")
    parser.add_argument("--trail", type=float, default=30.0,
                        help="Trail duration in seconds (default: 30)")
    parser.add_argument("--obstacles", default="",
                        help="Path to obstacles CSV file")

    # ROS2 may append --ros-args. Keep argparse focused on this script's args.
    argv = sys.argv[1:]
    ros_args = []
    clean_args = []
    for i, arg in enumerate(argv):
        if arg == "--ros-args":
            ros_args = argv[i:]
            break
        clean_args.append(arg)

    args = parser.parse_args(clean_args)

    rclpy.init(args=ros_args if ros_args else None)
    bridge = DataBridge(trail_duration=args.trail)
    subscriber = PoseSubscriber(bridge)
    logger = TrajectoryLogger(output_dir=args.output_dir)
    viewer = DroneViewer(bridge, subscriber, logger)

    # Load obstacles if provided
    if args.obstacles:
        if os.path.exists(args.obstacles):
            obstacles = load_obstacles(args.obstacles)
            add_obstacles_to_scene(viewer._view, obstacles)
            print(f"Loaded {len(obstacles)} obstacles from {args.obstacles}")
        else:
            print(f"Obstacles file not found: {args.obstacles}", file=sys.stderr)

    try:
        viewer.run()
    finally:
        subscriber.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()