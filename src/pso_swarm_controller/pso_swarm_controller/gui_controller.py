#!/usr/bin/env python3
"""
gui_controller.py  –  Swarm Command Center (Alpha Drone Dashboard)
===================================================================
A full-featured Tkinter GUI for controlling the Alpha leader drone.

Controls
--------
  W / S        – Forward / Backward  (linear.x)
  A / D        – Strafe Left / Right (linear.y)
  Q / E        – Rotate CCW / CW     (angular.z)
  R / F        – Ascend / Descend    (linear.z)
  SPACE        – Immediate full stop

All key-combinations are valid simultaneously (proper multi-key state tracking).
Speed slider applies to all translational axes.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
from tkinter import font as tkfont
from threading import Thread
import time


# ──────────────────────────────────────────────────────────────
#  Colour palette
# ──────────────────────────────────────────────────────────────
BG          = "#0d0f14"
PANEL       = "#151820"
BORDER      = "#1e2330"
ACCENT      = "#00e5ff"
ACCENT2     = "#ff6b35"
SUCCESS     = "#00ff88"
WARNING     = "#ffd700"
DANGER      = "#ff3355"
TEXT_PRI    = "#e8eaf0"
TEXT_SEC    = "#606880"
BTN_IDLE    = "#1c2030"
BTN_ACTIVE  = "#00e5ff"
BTN_TXT_ACT = "#0d0f14"


class KeyStateTracker:
    """Tracks currently held-down keys so multiple keys work simultaneously."""

    def __init__(self):
        self._held: set[str] = set()

    def press(self, key: str):
        self._held.add(key.lower())

    def release(self, key: str):
        self._held.discard(key.lower())

    def is_held(self, key: str) -> bool:
        return key.lower() in self._held

    def any_of(self, *keys) -> bool:
        return any(k.lower() in self._held for k in keys)


class DroneDashboard(Node):
    """ROS 2 node + Tkinter GUI for the Alpha (leader) drone."""

    # Axis configuration: key → (linear.x, linear.y, linear.z, angular.z)
    KEY_AXES = {
        'w': ( 1, 0, 0, 0),
        's': (-1, 0, 0, 0),
        'a': ( 0, 1, 0, 0),
        'd': ( 0,-1, 0, 0),
        'r': ( 0, 0, 1, 0),
        'f': ( 0, 0,-1, 0),
        'q': ( 0, 0, 0, 1),
        'e': ( 0, 0, 0,-1),
    }
    TURN_SPEED = 1.2   # rad/s for rotation keys

    def __init__(self):
        super().__init__('drone_dashboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_keyboard', 10)

        self.keys = KeyStateTracker()
        self.running = True

        # Last published twist (for live telemetry display)
        self.last_twist = Twist()

        # Publish at 20 Hz regardless of key events (cleaner than event-only)
        self.create_timer(0.05, self._publish_from_keys)

        self._build_gui()

    # ──────────────────────────────────────────────────────────
    #  GUI Construction
    # ──────────────────────────────────────────────────────────

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("Swarm Command Center")
        self.root.geometry("460x640")
        self.root.resizable(False, False)
        self.root.configure(bg=BG)

        # ── Header ──
        hdr = tk.Frame(self.root, bg=BG)
        hdr.pack(fill=tk.X, padx=20, pady=(18, 0))

        tk.Label(hdr, text="◈  SWARM", bg=BG, fg=ACCENT,
                 font=("Courier New", 20, "bold")).pack(side=tk.LEFT)
        tk.Label(hdr, text="COMMAND CENTER", bg=BG, fg=TEXT_PRI,
                 font=("Courier New", 20, "bold")).pack(side=tk.LEFT, padx=8)

        self._status_dot = tk.Label(hdr, text="●", bg=BG, fg=SUCCESS,
                                    font=("Courier New", 14))
        self._status_dot.pack(side=tk.RIGHT)
        tk.Label(hdr, text="ONLINE", bg=BG, fg=SUCCESS,
                 font=("Courier New", 10)).pack(side=tk.RIGHT, padx=(0, 4))

        tk.Frame(self.root, bg=BORDER, height=1).pack(fill=tk.X, padx=20, pady=10)

        # ── Speed + Z-speed sliders ──
        sliders = tk.Frame(self.root, bg=BG)
        sliders.pack(fill=tk.X, padx=24)

        self.speed_scale = self._make_slider(
            sliders, "XY SPEED  (m/s)", 0.2, 5.0, 2.0, ACCENT)
        self.z_speed_scale = self._make_slider(
            sliders, "Z  SPEED  (m/s)", 0.2, 3.0, 1.0, ACCENT2)

        tk.Frame(self.root, bg=BORDER, height=1).pack(fill=tk.X, padx=20, pady=8)

        # ── WASD key pad ──
        pad_outer = tk.Frame(self.root, bg=BG)
        pad_outer.pack()

        tk.Label(pad_outer, text="FLIGHT CONTROLS", bg=BG, fg=TEXT_SEC,
                 font=("Courier New", 9, "bold")).pack(pady=(0, 8))

        pad = tk.Frame(pad_outer, bg=BG)
        pad.pack()

        # Row 0: Q  W  E
        # Row 1: A  S  D
        # Row 2: (space)  RF col
        layout = [
            [('q', 0, 0, "Q\nROT←"), ('w', 0, 1, "W\nFWD"),  ('e', 0, 2, "E\nROT→")],
            [('a', 1, 0, "A\n←"),    ('s', 1, 1, "S\nBACK"), ('d', 1, 2, "D\n→")],
        ]

        self._btn_refs: dict[str, tk.Label] = {}
        for row in layout:
            for key, r, c, label in row:
                btn = self._make_key_btn(pad, label, r, c)
                self._btn_refs[key] = btn

        # Altitude buttons (right column)
        alt_col = tk.Frame(pad_outer, bg=BG)
        alt_col.pack(pady=(10, 0))

        tk.Label(alt_col, text="ALTITUDE", bg=BG, fg=TEXT_SEC,
                 font=("Courier New", 8)).pack()
        rf_row = tk.Frame(alt_col, bg=BG)
        rf_row.pack()
        self._btn_refs['r'] = self._make_key_btn(rf_row, "R\n▲", 0, 0,
                                                  active_color="#00ff88")
        self._btn_refs['f'] = self._make_key_btn(rf_row, "F\n▼", 0, 1,
                                                  active_color="#ff6b35")

        # ── Live telemetry ──
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill=tk.X, padx=20, pady=10)
        self._build_telemetry()

        # ── Emergency stop ──
        stop = tk.Button(
            self.root, text="⛔  EMERGENCY STOP",
            bg=DANGER, fg="white",
            font=("Courier New", 13, "bold"),
            relief=tk.FLAT, bd=0,
            activebackground="#cc0022", activeforeground="white",
            command=self._emergency_stop,
            cursor="hand2"
        )
        stop.pack(pady=(4, 18), fill=tk.X, padx=24)

        # Key bindings
        self.root.bind("<KeyPress>",   self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # Telemetry refresh loop
        self.root.after(80, self._refresh_telemetry)

    def _make_slider(self, parent, label, lo, hi, default, color):
        frame = tk.Frame(parent, bg=BG)
        frame.pack(fill=tk.X, pady=4)
        tk.Label(frame, text=label, bg=BG, fg=TEXT_SEC,
                 font=("Courier New", 9)).pack(side=tk.LEFT, padx=(0, 10))
        scale = tk.Scale(frame, from_=lo, to=hi, resolution=0.1,
                         orient=tk.HORIZONTAL, bg=BG, fg=TEXT_PRI,
                         troughcolor=BORDER, highlightthickness=0,
                         activebackground=color, sliderrelief=tk.FLAT,
                         showvalue=True, length=200)
        scale.set(default)
        scale.pack(side=tk.RIGHT)
        return scale

    def _make_key_btn(self, parent, label, row, col,
                      active_color=None, size=70):
        if active_color is None:
            active_color = BTN_ACTIVE
        btn = tk.Label(
            parent, text=label,
            width=5, height=3,
            bg=BTN_IDLE, fg=TEXT_SEC,
            font=("Courier New", 9, "bold"),
            relief=tk.FLAT, bd=0,
            justify=tk.CENTER
        )
        btn.grid(row=row, column=col, padx=4, pady=4,
                 ipadx=6, ipady=4)
        btn._active_color = active_color
        return btn

    def _build_telemetry(self):
        tele_frame = tk.Frame(self.root, bg=PANEL, bd=0)
        tele_frame.pack(fill=tk.X, padx=24, pady=4)

        tk.Label(tele_frame, text=" LIVE TELEMETRY",
                 bg=PANEL, fg=TEXT_SEC,
                 font=("Courier New", 8, "bold")).pack(anchor=tk.W,
                                                        padx=8, pady=(6, 2))

        row = tk.Frame(tele_frame, bg=PANEL)
        row.pack(fill=tk.X, padx=8, pady=(0, 8))

        self._tele_vars: dict[str, tk.StringVar] = {}
        axes = [
            ("VX", ACCENT),  ("VY", ACCENT),
            ("VZ", ACCENT2), ("WZ", WARNING),
        ]
        for i, (name, color) in enumerate(axes):
            col_f = tk.Frame(row, bg=PANEL)
            col_f.pack(side=tk.LEFT, expand=True)
            tk.Label(col_f, text=name, bg=PANEL, fg=TEXT_SEC,
                     font=("Courier New", 8)).pack()
            var = tk.StringVar(value="0.00")
            self._tele_vars[name] = var
            tk.Label(col_f, textvariable=var, bg=PANEL, fg=color,
                     font=("Courier New", 13, "bold")).pack()

    # ──────────────────────────────────────────────────────────
    #  Key handling
    # ──────────────────────────────────────────────────────────

    def _on_key_press(self, event):
        k = event.keysym.lower()
        self.keys.press(k)
        if k in self._btn_refs:
            btn = self._btn_refs[k]
            btn.config(bg=btn._active_color, fg=BTN_TXT_ACT)

    def _on_key_release(self, event):
        k = event.keysym.lower()
        self.keys.release(k)
        if k in self._btn_refs:
            btn = self._btn_refs[k]
            btn.config(bg=BTN_IDLE, fg=TEXT_SEC)

    # ──────────────────────────────────────────────────────────
    #  Velocity computation & publishing  (ROS timer, 20 Hz)
    # ──────────────────────────────────────────────────────────

    def _publish_from_keys(self):
        """Called by ROS timer at 20 Hz. Builds and publishes Twist from held keys."""
        s   = self.speed_scale.get()
        sz  = self.z_speed_scale.get()

        twist = Twist()
        for key, (ax, ay, az, aw) in self.KEY_AXES.items():
            if self.keys.is_held(key):
                twist.linear.x  += ax * s
                twist.linear.y  += ay * s
                twist.linear.z  += az * sz
                twist.angular.z += aw * self.TURN_SPEED

        # Clamp to prevent double-key over-speed
        twist.linear.x  = max(-s,  min(s,  twist.linear.x))
        twist.linear.y  = max(-s,  min(s,  twist.linear.y))
        twist.linear.z  = max(-sz, min(sz, twist.linear.z))
        twist.angular.z = max(-self.TURN_SPEED, min(self.TURN_SPEED, twist.angular.z))

        self.last_twist = twist
        self.publisher_.publish(twist)

    def _emergency_stop(self):
        self.keys._held.clear()
        self.last_twist = Twist()
        self.publisher_.publish(Twist())
        # Flash all buttons red momentarily
        for btn in self._btn_refs.values():
            btn.config(bg=DANGER, fg="white")
        self.root.after(300, self._reset_all_btns)

    def _reset_all_btns(self):
        for btn in self._btn_refs.values():
            btn.config(bg=BTN_IDLE, fg=TEXT_SEC)

    # ──────────────────────────────────────────────────────────
    #  Telemetry refresh  (Tk after-loop, 12 Hz)
    # ──────────────────────────────────────────────────────────

    def _refresh_telemetry(self):
        t = self.last_twist
        self._tele_vars["VX"].set(f"{t.linear.x:+.2f}")
        self._tele_vars["VY"].set(f"{t.linear.y:+.2f}")
        self._tele_vars["VZ"].set(f"{t.linear.z:+.2f}")
        self._tele_vars["WZ"].set(f"{t.angular.z:+.2f}")
        self.root.after(80, self._refresh_telemetry)

    # ──────────────────────────────────────────────────────────
    #  Lifecycle
    # ──────────────────────────────────────────────────────────

    def _on_close(self):
        self.running = False
        self.publisher_.publish(Twist())   # zero velocities on exit
        self.root.destroy()

    def run(self):
        def _ros_spin():
            while rclpy.ok() and self.running:
                rclpy.spin_once(self, timeout_sec=0.05)

        t = Thread(target=_ros_spin, daemon=True)
        t.start()
        self.root.mainloop()
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    dashboard = DroneDashboard()
    dashboard.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()