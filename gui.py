import tkinter as tk
from tkinter import ttk
import math
import time
import threading
import logging
from typing import Optional, Callable

from config import SystemConfig, TankId, PumpAction
from modbus_client import ModbusRTUClient
from tank_controller import TankController
from imu_reader import IMUReader
from attitude_controller import AttitudeController

logger = logging.getLogger(__name__)


class GUILogHandler(logging.Handler):
    def __init__(self, callback):
        super().__init__()
        self._callback = callback

    def emit(self, record):
        try:
            msg = self.format(record)
            level = record.levelname
            if level in ("ERROR", "CRITICAL"):
                direction = "ERR"
            elif level == "WARNING":
                direction = "WARN"
            else:
                direction = "LOG"
            if self._callback:
                self._callback(direction, msg)
        except Exception:
            pass


COLORS = {
    "bg": "#FFFFFF",
    "card": "#F4F4F4",
    "border": "#EEEEEE",
    "accent": "#3E6AE1",
    "accent_light": "#EBF0FA",
    "danger": "#FF3B30",
    "danger_light": "#FFEBE9",
    "success": "#34C759",
    "success_light": "#E8F9EE",
    "warning": "#FF9500",
    "warning_light": "#FFF3E6",
    "text": "#171A20",
    "text_secondary": "#5C5E62",
    "text_tertiary": "#8E8E8E",
    "drain": "#FF3B30",
    "drain_light": "#FFEBE9",
    "fill": "#4ECDC4",
    "fill_light": "#E0F7F5",
    "stop": "#8E8E8E",
    "stop_light": "#F4F4F4",
    "pitch_color": "#3E6AE1",
    "roll_color": "#FF9500",
    "yaw_color": "#5C5E62",
    "dark_surface": "#171A20",
    "pale_silver": "#D0D1D2",
}


class AttitudeIndicator(tk.Canvas):
    def __init__(self, parent, size=320, **kwargs):
        super().__init__(parent, width=size, height=size, bg=COLORS["bg"],
                         highlightthickness=0, **kwargs)
        self._size = size
        self._cx = size / 2
        self._cy = size / 2
        self._r = size / 2 - 30
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self.bind("<Configure>", self._on_resize)

    def update_angles(self, pitch: float, roll: float, yaw: float = 0.0):
        self._pitch = pitch
        self._roll = roll
        self._yaw = yaw
        self.redraw()

    def _on_resize(self, event):
        self._size = min(event.width, event.height)
        self._cx = event.width / 2
        self._cy = event.height / 2
        self._r = self._size / 2 - 30
        self.redraw()

    def redraw(self):
        self.delete("all")
        cx, cy, r = self._cx, self._cy, self._r

        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         fill=COLORS["card"], outline=COLORS["border"], width=1)

        roll_rad = math.radians(self._roll)
        pitch_offset = self._pitch * (r / 45.0)
        pitch_offset = max(-r * 0.8, min(r * 0.8, pitch_offset))

        dx = math.sin(roll_rad) * pitch_offset
        dy = -math.cos(roll_rad) * pitch_offset

        half_len = r * 1.5
        x1 = cx - math.cos(roll_rad) * half_len + dx
        y1 = cy - math.sin(roll_rad) * half_len + dy
        x2 = cx + math.cos(roll_rad) * half_len + dx
        y2 = cy + math.sin(roll_rad) * half_len + dy

        self.create_line(x1, y1, x2, y2, fill=COLORS["accent"], width=2)

        for angle in [-30, -20, -10, 0, 10, 20, 30]:
            offset = angle * (r / 45.0)
            offset = max(-r * 0.8, min(r * 0.8, offset))
            ly = cy + offset
            if (ly - cy) ** 2 < r ** 2:
                half_w = math.sqrt(r ** 2 - (ly - cy) ** 2) * 0.3
                lx1 = cx - half_w
                lx2 = cx + half_w
                self.create_line(lx1, ly, lx2, ly, fill=COLORS["text_tertiary"], width=1, dash=(3, 3))
                self.create_text(lx1 - 15, ly, text=f"{-angle}", fill=COLORS["text_tertiary"],
                                 font=("Consolas", 7))

        self.create_line(cx - 15, cy, cx + 15, cy, fill=COLORS["danger"], width=2)
        self.create_line(cx, cy - 15, cx, cy + 15, fill=COLORS["danger"], width=2)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=COLORS["danger"], outline="")

        marker_r = r + 15
        for angle_deg in range(0, 360, 30):
            angle_rad = math.radians(angle_deg - 90)
            x = cx + math.cos(angle_rad) * marker_r
            y = cy + math.sin(angle_rad) * marker_r
            if angle_deg % 90 == 0:
                self.create_oval(x - 3, y - 3, x + 3, y + 3, fill=COLORS["accent"], outline="")
            else:
                self.create_oval(x - 1.5, y - 1.5, x + 1.5, y + 1.5, fill=COLORS["text_tertiary"], outline="")

        roll_marker_angle = math.radians(self._roll - 90)
        rm_x = cx + math.cos(roll_marker_angle) * (r + 8)
        rm_y = cy + math.sin(roll_marker_angle) * (r + 8)
        self.create_polygon(
            rm_x, rm_y,
            rm_x + math.cos(roll_marker_angle + 0.3) * 10,
            rm_y + math.sin(roll_marker_angle + 0.3) * 10,
            rm_x + math.cos(roll_marker_angle - 0.3) * 10,
            rm_y + math.sin(roll_marker_angle - 0.3) * 10,
            fill=COLORS["warning"], outline=""
        )

        self.create_text(cx, cy - r - 15, text=f"PITCH {self._pitch:+.1f}\u00b0",
                         fill=COLORS["pitch_color"], font=("Consolas", 10))
        self.create_text(cx, cy + r + 15, text=f"ROLL {self._roll:+.1f}\u00b0",
                         fill=COLORS["roll_color"], font=("Consolas", 10))
        self.create_text(cx + r + 5, cy - r + 5, text=f"YAW {self._yaw:+.1f}\u00b0",
                         fill=COLORS["text_tertiary"], font=("Consolas", 9), anchor="ne")


class TankVisualWidget(tk.Canvas):
    def __init__(self, parent, tank_name: str, size=90, **kwargs):
        super().__init__(parent, width=size, height=size + 20, bg=COLORS["bg"],
                         highlightthickness=0, **kwargs)
        self._size = size
        self._name = tank_name
        self._state = PumpAction.STOP
        self._water_level = 0.5
        self.redraw()

    def set_state(self, state: PumpAction, water_level: float = 0.5):
        self._state = state
        self._water_level = max(0, min(1, water_level))
        self.redraw()

    def redraw(self):
        self.delete("all")
        s = self._size
        pad = 8
        tank_w = s - pad * 2
        tank_h = s - pad * 2 - 10
        x1, y1 = pad, pad
        x2, y2 = pad + tank_w, pad + tank_h

        self.create_rectangle(x1, y1, x2, y2, fill=COLORS["card"],
                              outline=COLORS["border"], width=1)

        water_h = tank_h * self._water_level
        water_y = y2 - water_h
        if self._state == PumpAction.DRAIN:
            water_color = COLORS["drain_light"]
        elif self._state == PumpAction.FILL:
            water_color = COLORS["fill_light"]
        else:
            water_color = COLORS["accent_light"]

        if water_h > 0:
            self.create_rectangle(x1 + 2, water_y, x2 - 2, y2 - 2,
                                  fill=water_color, outline="")

        if self._state == PumpAction.DRAIN:
            indicator_color = COLORS["drain"]
            label = "\u6392\u6c34"
        elif self._state == PumpAction.FILL:
            indicator_color = COLORS["fill"]
            label = "\u6ce8\u6c34"
        else:
            indicator_color = COLORS["stop"]
            label = "\u505c\u6b62"

        self.create_oval(x2 - 18, y1 + 4, x2 - 4, y1 + 18,
                         fill=indicator_color, outline="")
        self.create_text((x1 + x2) / 2, y2 + 12, text=f"{self._name}",
                         fill=COLORS["text"], font=("Microsoft YaHei", 9))
        self.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=label,
                         fill=indicator_color, font=("Microsoft YaHei", 9))


class CageTopView(tk.Canvas):
    def __init__(self, parent, size=260, **kwargs):
        super().__init__(parent, width=size, height=size, bg=COLORS["bg"],
                         highlightthickness=0, **kwargs)
        self._size = size
        self._states = {tid: PumpAction.STOP for tid in TankId}
        self.redraw()

    def set_states(self, states: dict):
        self._states = states
        self.redraw()

    def _draw_tank(self, cx, cy, w, h, name, state, is_center=False):
        if state == PumpAction.DRAIN:
            color = COLORS["drain"]
            fill = COLORS["drain_light"]
        elif state == PumpAction.FILL:
            color = COLORS["fill"]
            fill = COLORS["fill_light"]
        else:
            color = COLORS["border"]
            fill = COLORS["card"]

        if is_center:
            r = min(w, h) / 2
            self.create_oval(cx - r, cy - r, cx + r, cy + r,
                             fill=fill, outline=color, width=1)
        else:
            self.create_rectangle(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2,
                                  fill=fill, outline=color, width=1)

        self.create_text(cx, cy - 6, text=name, fill=COLORS["text"],
                         font=("Microsoft YaHei", 9))
        action_text = {PumpAction.STOP: "\u505c", PumpAction.DRAIN: "\u6392", PumpAction.FILL: "\u6ce8"}
        self.create_text(cx, cy + 10, text=action_text.get(state, "\u505c"),
                         fill=color, font=("Microsoft YaHei", 8))

    def redraw(self):
        self.delete("all")
        s = self._size
        cx, cy = s / 2, s / 2
        gap = 8
        rect_w = 50
        rect_h = 22

        self.create_rectangle(cx - 60, cy - 60, cx + 60, cy + 60,
                              fill="", outline=COLORS["border"], width=1, dash=(4, 4))

        self._draw_tank(cx, cy, 40, 40, "\u4e2d\u5fc3", self._states[TankId.CENTER], is_center=True)
        self._draw_tank(cx, cy - 45 - gap, rect_w, rect_h, "\u5317", self._states[TankId.NORTH])
        self._draw_tank(cx, cy + 45 + gap, rect_w, rect_h, "\u5357", self._states[TankId.SOUTH])
        self._draw_tank(cx + 45 + gap, cy, rect_h, rect_w, "\u4e1c", self._states[TankId.EAST])
        self._draw_tank(cx - 45 - gap, cy, rect_h, rect_w, "\u897f", self._states[TankId.WEST])

        self.create_text(cx, 10, text="\u9876\u89c6\u56fe", fill=COLORS["text_secondary"],
                         font=("Microsoft YaHei", 9))
        self.create_text(cx + 55, cy - 55, text="N", fill=COLORS["accent"],
                         font=("Consolas", 10))
        self.create_text(cx + 55, cy + 55, text="S", fill=COLORS["text_secondary"],
                         font=("Consolas", 10))
        self.create_text(cx + 65, cy, text="E", fill=COLORS["text_secondary"],
                         font=("Consolas", 10))
        self.create_text(cx - 65, cy, text="W", fill=COLORS["text_secondary"],
                         font=("Consolas", 10))


class ControlButton(tk.Frame):
    def __init__(self, parent, text="", color=COLORS["accent"], width=80, height=40,
                 callback: Optional[Callable] = None, primary=False, **kwargs):
        super().__init__(parent, bg="#FFFFFF" if not primary else color,
                         height=height, cursor="hand2",
                         highlightthickness=0, **kwargs)
        self._text = text
        self._color = color
        self._btn_w = width
        self._btn_h = height
        self._callback = callback
        self._pressed = False
        self._hover = False
        self._primary = primary

        if primary:
            btn_bg = color
            btn_fg = "#FFFFFF"
        else:
            btn_bg = "#FFFFFF"
            btn_fg = color

        self._default_bg = btn_bg
        self._default_fg = btn_fg

        self._btn = tk.Label(self, text=text, bg=btn_bg, fg=btn_fg,
                             font=("Microsoft YaHei", 11),
                             relief=tk.FLAT, borderwidth=0, cursor="hand2",
                             padx=8, pady=4)
        self._btn.pack(fill=tk.BOTH, expand=True)

        if not primary:
            self.configure(highlightbackground=COLORS["border"], highlightthickness=1)

        self._btn.bind("<ButtonPress-1>", self._on_press)
        self._btn.bind("<ButtonRelease-1>", self._on_release)
        self._btn.bind("<Enter>", self._on_enter)
        self._btn.bind("<Leave>", self._on_leave)
        self.bind("<ButtonPress-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)

    def _on_press(self, e):
        self._pressed = True
        if self._primary:
            self._btn.config(bg=self._color, fg="#FFFFFF")
            self.configure(bg=self._color)
        else:
            self._btn.config(bg=self._color, fg="#FFFFFF")

    def _on_release(self, e):
        self._pressed = False
        if self._hover:
            if self._primary:
                hover_bg = self._color
                self._btn.config(bg=hover_bg, fg="#FFFFFF")
                self.configure(bg=hover_bg)
            else:
                self._btn.config(bg=COLORS["card"], fg=self._color)
        else:
            self._btn.config(bg=self._default_bg, fg=self._default_fg)
            if not self._primary:
                self.configure(bg="#FFFFFF")
        if self._callback:
            self._callback()

    def _on_enter(self, e):
        self._hover = True
        if not self._pressed:
            if self._primary:
                self._btn.config(bg=self._color, fg="#FFFFFF")
            else:
                self._btn.config(bg=COLORS["card"], fg=self._color)

    def _on_leave(self, e):
        self._hover = False
        self._pressed = False
        self._btn.config(bg=self._default_bg, fg=self._default_fg)
        if not self._primary:
            self.configure(bg="#FFFFFF")

    def update_appearance(self, text=None, color=None):
        if text is not None:
            self._text = text
            self._btn.config(text=text)
        if color is not None:
            self._color = color
            if self._primary:
                self._default_bg = color
                if not self._pressed and not self._hover:
                    self._btn.config(bg=color, fg="#FFFFFF")
                    self.configure(bg=color)
            else:
                self._default_fg = color
                if not self._pressed and not self._hover:
                    self._btn.config(fg=color)

    def flash_press(self):
        self._pressed = True
        self._btn.config(bg=self._color, fg="#FFFFFF")

    def flash_release(self):
        self._pressed = False
        self._btn.config(bg=self._default_bg, fg=self._default_fg)
        if not self._primary:
            self.configure(bg="#FFFFFF")


class MainWindow(tk.Tk):
    def __init__(self, config: SystemConfig):
        super().__init__()
        self._config = config
        self._client = ModbusRTUClient(config.serial)
        self._tank = TankController(self._client, config)
        self._imu = IMUReader(self._client, config)
        self._attitude = AttitudeController(config.attitude, self._imu)
        self._connected = False
        self._auto_stable = False
        self._key_states = {}

        self.title("\u5347\u964d\u7f51\u7bb1\u6a21\u578b\u63a7\u5236\u7cfb\u7edf v1.0")
        self.geometry("1280x720")
        self.minsize(1024, 600)
        self.configure(bg=COLORS["bg"])

        self._setup_styles()
        self._build_ui()
        self._bind_keys()

        self._update_timer = None
        self._schedule_update()

    def _setup_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure("TCombobox",
                         fieldbackground="#FFFFFF",
                         background="#FFFFFF",
                         foreground=COLORS["text"],
                         arrowcolor=COLORS["text_secondary"],
                         bordercolor=COLORS["border"],
                         focuscolor=COLORS["accent"],
                         selectbackground=COLORS["accent"],
                         selectforeground="#FFFFFF",
                         padding=(8, 4))
        style.map("TCombobox",
                   fieldbackground=[("readonly", "#FFFFFF")],
                   selectbackground=[("readonly", COLORS["accent"])],
                   selectforeground=[("readonly", "#FFFFFF")])

        style.configure("TNotebook",
                         background=COLORS["card"],
                         borderwidth=0,
                         tabmargins=[0, 0, 0, 0])
        style.configure("TNotebook.Tab",
                         background="#FFFFFF",
                         foreground=COLORS["text_secondary"],
                         padding=[16, 6],
                         borderwidth=0,
                         font=("Microsoft YaHei", 10))
        style.map("TNotebook.Tab",
                   background=[("selected", COLORS["card"])],
                   foreground=[("selected", COLORS["text"])],
                   expand=[("selected", [0, 0, 0, 2])])

    def _build_ui(self):
        main_frame = tk.Frame(self, bg=COLORS["bg"])
        main_frame.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)

        top_frame = tk.Frame(main_frame, bg=COLORS["bg"])
        top_frame.pack(fill=tk.X, pady=(0, 10))

        self._build_connection_panel(top_frame)

        content_frame = tk.Frame(main_frame, bg=COLORS["bg"])
        content_frame.pack(fill=tk.BOTH, expand=True)

        left_frame = tk.Frame(content_frame, bg=COLORS["bg"])
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self._build_attitude_panel(left_frame)
        self._build_tank_panel(left_frame)

        right_frame = tk.Frame(content_frame, bg=COLORS["bg"], width=440)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        right_frame.pack_propagate(False)

        self._build_control_panel(right_frame)
        self._build_settings_panel(right_frame)

        self._build_log_panel(main_frame)

    def _card(self, parent, title=""):
        frame = tk.Frame(parent, bg=COLORS["card"], highlightthickness=0)
        frame.pack(fill=tk.X, pady=(0, 10), ipady=6, ipadx=8)

        if title:
            header = tk.Frame(frame, bg=COLORS["card"])
            header.pack(fill=tk.X, padx=16, pady=(12, 4))
            tk.Label(header, text=title, bg=COLORS["card"], fg=COLORS["text"],
                     font=("Microsoft YaHei", 13)).pack(side=tk.LEFT)
            tk.Frame(header, bg=COLORS["accent"], height=2).pack(side=tk.LEFT, fill=tk.X,
                                                                   expand=True, padx=(12, 0), pady=4)
        return frame

    def _build_connection_panel(self, parent):
        card = self._card(parent, "\u8fde\u63a5\u7ba1\u7406")
        ctrl = tk.Frame(card, bg=COLORS["card"])
        ctrl.pack(fill=tk.X, padx=16, pady=(4, 10))

        labels = ["\u4e32\u53e3:", "\u6ce2\u7279\u7387:", "\u6821\u9a8c:"]
        self._serial_var = tk.StringVar(value=self._config.serial.port)
        self._baud_var = tk.StringVar(value=str(self._config.serial.baudrate))
        self._parity_var = tk.StringVar(value=self._config.serial.parity)

        for i, lbl in enumerate(labels):
            tk.Label(ctrl, text=lbl, bg=COLORS["card"], fg=COLORS["text"],
                     font=("Microsoft YaHei", 11)).grid(row=0, column=i * 2, padx=(0, 4))

        serial_entry = ttk.Combobox(ctrl, textvariable=self._serial_var, width=8,
                                     values=["COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8"])
        serial_entry.grid(row=0, column=1, padx=(0, 16))

        baud_entry = ttk.Combobox(ctrl, textvariable=self._baud_var, width=7,
                                   values=["9600", "19200", "38400", "115200"])
        baud_entry.grid(row=0, column=3, padx=(0, 16))

        parity_entry = ttk.Combobox(ctrl, textvariable=self._parity_var, width=4,
                                     values=["N", "E", "O"])
        parity_entry.grid(row=0, column=5, padx=(0, 16))

        self._connect_btn = ControlButton(ctrl, "\u8fde\u63a5", COLORS["accent"], 90, 36,
                                           callback=self._toggle_connection, primary=True)
        self._connect_btn.grid(row=0, column=6, padx=(8, 0))

        self._status_label = tk.Label(ctrl, text="\u25cf \u672a\u8fde\u63a5", bg=COLORS["card"],
                                       fg=COLORS["danger"], font=("Microsoft YaHei", 11))
        self._status_label.grid(row=0, column=7, padx=(16, 0))

    def _build_attitude_panel(self, parent):
        card = self._card(parent, "\u59ff\u6001\u76d1\u63a7")
        container = tk.Frame(card, bg=COLORS["card"])
        container.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 10))

        self._attitude_indicator = AttitudeIndicator(container, size=280)
        self._attitude_indicator.pack(side=tk.LEFT, padx=(0, 24), pady=4)

        info_frame = tk.Frame(container, bg=COLORS["card"])
        info_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        gauge_row = tk.Frame(info_frame, bg=COLORS["card"])
        gauge_row.pack(fill=tk.X, pady=(8, 0))

        pitch_frame = tk.Frame(gauge_row, bg=COLORS["card"])
        pitch_frame.pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Label(pitch_frame, text="PITCH", bg=COLORS["card"], fg=COLORS["pitch_color"],
                 font=("Consolas", 16)).pack()
        self._pitch_value = tk.Label(pitch_frame, text="+0.00\u00b0", bg=COLORS["card"],
                                      fg=COLORS["pitch_color"],
                                      font=("Consolas", 24, "bold"))
        self._pitch_value.pack()

        sep1 = tk.Frame(gauge_row, bg=COLORS["border"], width=1)
        sep1.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)

        roll_frame = tk.Frame(gauge_row, bg=COLORS["card"])
        roll_frame.pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Label(roll_frame, text="ROLL", bg=COLORS["card"], fg=COLORS["roll_color"],
                 font=("Consolas", 16)).pack()
        self._roll_value = tk.Label(roll_frame, text="+0.00\u00b0", bg=COLORS["card"],
                                     fg=COLORS["roll_color"],
                                     font=("Consolas", 24, "bold"))
        self._roll_value.pack()

        sep2 = tk.Frame(gauge_row, bg=COLORS["border"], width=1)
        sep2.pack(side=tk.LEFT, fill=tk.Y, padx=8, pady=8)

        yaw_frame = tk.Frame(gauge_row, bg=COLORS["card"])
        yaw_frame.pack(side=tk.LEFT, expand=True, fill=tk.X)
        tk.Label(yaw_frame, text="YAW", bg=COLORS["card"], fg=COLORS["yaw_color"],
                 font=("Consolas", 16)).pack()
        self._yaw_value = tk.Label(yaw_frame, text="+0.00\u00b0", bg=COLORS["card"],
                                    fg=COLORS["yaw_color"],
                                    font=("Consolas", 24, "bold"))
        self._yaw_value.pack()

        auto_frame = tk.Frame(info_frame, bg=COLORS["card"])
        auto_frame.pack(pady=(32, 4))
        self._auto_btn = ControlButton(auto_frame, "\u81ea\u52a8\u7a33\u5b9a: \u5173", COLORS["accent"], 180, 44,
                                        callback=self._toggle_auto, primary=True)
        self._auto_btn.pack()

    def _build_control_panel(self, parent):
        card = self._card(parent, "\u952e\u76d8\u63a7\u5236 (W/A/S/D/Q/Z/\u7a7a\u683c)")
        ctrl = tk.Frame(card, bg=COLORS["card"])
        ctrl.pack(padx=16, pady=(4, 10))

        btn_w, btn_h = 100, 48
        self._key_buttons = {}

        row1 = tk.Frame(ctrl, bg=COLORS["card"])
        row1.pack(pady=3)
        self._key_buttons["w"] = ControlButton(row1, "W \u524d\u503e", COLORS["pitch_color"], btn_w, btn_h,
                      lambda: self._on_key_action("w"))
        self._key_buttons["w"].pack(side=tk.LEFT, padx=4)

        row2 = tk.Frame(ctrl, bg=COLORS["card"])
        row2.pack(pady=3)
        self._key_buttons["a"] = ControlButton(row2, "A \u5de6\u503e", COLORS["roll_color"], btn_w, btn_h,
                      lambda: self._on_key_action("a"))
        self._key_buttons["a"].pack(side=tk.LEFT, padx=4)
        self._key_buttons[" "] = ControlButton(row2, "\u505c\u6b62", COLORS["danger"], btn_w, btn_h,
                      lambda: self._on_key_action(" "))
        self._key_buttons[" "].pack(side=tk.LEFT, padx=4)
        self._key_buttons["d"] = ControlButton(row2, "D \u53f3\u503e", COLORS["roll_color"], btn_w, btn_h,
                      lambda: self._on_key_action("d"))
        self._key_buttons["d"].pack(side=tk.LEFT, padx=4)

        row3 = tk.Frame(ctrl, bg=COLORS["card"])
        row3.pack(pady=3)
        self._key_buttons["s"] = ControlButton(row3, "S \u540e\u4ef0", COLORS["pitch_color"], btn_w, btn_h,
                      lambda: self._on_key_action("s"))
        self._key_buttons["s"].pack(side=tk.LEFT, padx=4)

        row4 = tk.Frame(ctrl, bg=COLORS["card"])
        row4.pack(pady=(10, 3))
        self._key_buttons["q"] = ControlButton(row4, "Q \u4e0a\u6d6e", COLORS["success"], btn_w, btn_h,
                      lambda: self._on_key_action("q"))
        self._key_buttons["q"].pack(side=tk.LEFT, padx=4)
        self._key_buttons["z"] = ControlButton(row4, "Z \u4e0b\u6f5c", COLORS["warning"], btn_w, btn_h,
                      lambda: self._on_key_action("z"))
        self._key_buttons["z"].pack(side=tk.LEFT, padx=4)
        self._key_buttons["space"] = ControlButton(row4, "\u7d27\u6025\u505c\u6b62", COLORS["danger"], 130, btn_h,
                      lambda: self._on_key_action(" "), primary=True)
        self._key_buttons["space"].pack(side=tk.LEFT, padx=4)

    def _build_tank_panel(self, parent):
        card = self._card(parent, "\u538b\u8f7d\u8231\u72b6\u6001")
        container = tk.Frame(card, bg=COLORS["card"])
        container.pack(fill=tk.X, padx=16, pady=(4, 10))

        left_part = tk.Frame(container, bg=COLORS["card"])
        left_part.pack(side=tk.LEFT, padx=(8, 24), pady=4)

        self._cage_top_view = CageTopView(left_part, size=220)
        self._cage_top_view.pack(pady=4)

        right_part = tk.Frame(container, bg=COLORS["card"])
        right_part.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(8, 8))

        self._tank_buttons = {}
        for tid in TankId:
            from tank_controller import TANK_NAMES
            name = TANK_NAMES[tid]
            f = tk.Frame(right_part, bg=COLORS["card"])
            f.pack(fill=tk.X, pady=4)
            tk.Label(f, text=f"{name}\u8231:", bg=COLORS["card"], fg=COLORS["text"],
                     font=("Microsoft YaHei", 11), width=5, anchor="e").pack(side=tk.LEFT)
            ControlButton(f, "\u6392\u6c34", COLORS["drain"], 60, 30,
                          lambda t=tid: self._tank.set_pump(t, PumpAction.DRAIN)).pack(side=tk.LEFT, padx=3)
            ControlButton(f, "\u6ce8\u6c34", COLORS["fill"], 60, 30,
                          lambda t=tid: self._tank.set_pump(t, PumpAction.FILL)).pack(side=tk.LEFT, padx=3)
            ControlButton(f, "\u505c", COLORS["stop"], 45, 30,
                          lambda t=tid: self._tank.set_pump(t, PumpAction.STOP)).pack(side=tk.LEFT, padx=3)

    def _build_settings_panel(self, parent):
        card = self._card(parent, "\u53c2\u6570\u914d\u7f6e")

        notebook = ttk.Notebook(card)
        notebook.pack(fill=tk.X, padx=8, pady=4)

        pid_frame = tk.Frame(notebook, bg=COLORS["card"])
        notebook.add(pid_frame, text="  PID  ")
        self._build_pid_tab(pid_frame)

        addr_frame = tk.Frame(notebook, bg=COLORS["card"])
        notebook.add(addr_frame, text="  \u5730\u5740  ")
        self._build_addr_tab(addr_frame)

    def _build_pid_tab(self, parent):
        axes = [("pitch", "\u4fef\u4ef0\u8f74"), ("roll", "\u6a2a\u6eda\u8f74")]
        self._pid_vars = {}

        for axis_key, axis_name in axes:
            frame = tk.LabelFrame(parent, text=axis_name, bg=COLORS["card"],
                                   fg=COLORS["text"], font=("Microsoft YaHei", 10),
                                   relief=tk.FLAT, highlightbackground=COLORS["border"],
                                   highlightthickness=1)
            frame.pack(fill=tk.X, padx=8, pady=6, ipady=4)

            params = getattr(self._config.attitude, f"{axis_key}_pid")
            vars_dict = {}
            for i, (pname, pval) in enumerate([("Kp", params.kp), ("Ki", params.ki), ("Kd", params.kd)]):
                tk.Label(frame, text=pname, bg=COLORS["card"], fg=COLORS["text_secondary"],
                         font=("Consolas", 11)).grid(row=0, column=i * 2, padx=(8, 2))
                var = tk.StringVar(value=f"{pval}")
                vars_dict[pname] = var
                entry = tk.Entry(frame, textvariable=var, width=6, font=("Consolas", 11),
                                 justify=tk.CENTER, bg="#FFFFFF", fg=COLORS["text"],
                                 relief=tk.FLAT, highlightbackground=COLORS["border"],
                                 highlightthickness=1, insertbackground=COLORS["accent"])
                entry.grid(row=0, column=i * 2 + 1, padx=(0, 8), pady=4)

            self._pid_vars[axis_key] = vars_dict

        btn_frame = tk.Frame(parent, bg=COLORS["card"])
        btn_frame.pack(fill=tk.X, padx=8, pady=6)
        ControlButton(btn_frame, "\u5e94\u7528PID\u53c2\u6570", COLORS["accent"], 130, 34,
                      self._apply_pid, primary=True).pack(side=tk.LEFT)

        target_frame = tk.Frame(parent, bg=COLORS["card"])
        target_frame.pack(fill=tk.X, padx=8, pady=6)
        tk.Label(target_frame, text="\u76ee\u6807Pitch:", bg=COLORS["card"], fg=COLORS["text"],
                 font=("Microsoft YaHei", 10)).pack(side=tk.LEFT)
        self._target_pitch_var = tk.StringVar(value="0.0")
        tk.Entry(target_frame, textvariable=self._target_pitch_var, width=6,
                 font=("Consolas", 11), justify=tk.CENTER,
                 bg="#FFFFFF", fg=COLORS["text"], relief=tk.FLAT,
                 highlightbackground=COLORS["border"], highlightthickness=1,
                 insertbackground=COLORS["accent"]).pack(side=tk.LEFT, padx=4)
        tk.Label(target_frame, text="\u76ee\u6807Roll:", bg=COLORS["card"], fg=COLORS["text"],
                 font=("Microsoft YaHei", 10)).pack(side=tk.LEFT, padx=(12, 0))
        self._target_roll_var = tk.StringVar(value="0.0")
        tk.Entry(target_frame, textvariable=self._target_roll_var, width=6,
                 font=("Consolas", 11), justify=tk.CENTER,
                 bg="#FFFFFF", fg=COLORS["text"], relief=tk.FLAT,
                 highlightbackground=COLORS["border"], highlightthickness=1,
                 insertbackground=COLORS["accent"]).pack(side=tk.LEFT, padx=4)
        ControlButton(target_frame, "\u5e94\u7528", COLORS["accent"], 70, 30,
                      self._apply_target, primary=True).pack(side=tk.LEFT, padx=6)

    def _build_addr_tab(self, parent):
        self._addr_vars = {}
        from tank_controller import TANK_NAMES

        for tid in TankId:
            name = TANK_NAMES[tid]
            dev = self._config.tanks[tid]
            frame = tk.Frame(parent, bg=COLORS["card"])
            frame.pack(fill=tk.X, padx=8, pady=3)

            tk.Label(frame, text=f"{name}\u8231:", bg=COLORS["card"], fg=COLORS["text"],
                     font=("Microsoft YaHei", 10), width=5, anchor="e").pack(side=tk.LEFT)
            var = tk.StringVar(value=str(dev.address))
            self._addr_vars[tid] = var
            tk.Entry(frame, textvariable=var, width=4, font=("Consolas", 11),
                     justify=tk.CENTER, bg="#FFFFFF", fg=COLORS["text"],
                     relief=tk.FLAT, highlightbackground=COLORS["border"],
                     highlightthickness=1, insertbackground=COLORS["accent"]).pack(side=tk.LEFT, padx=4)

        imu_frame = tk.Frame(parent, bg=COLORS["card"])
        imu_frame.pack(fill=tk.X, padx=8, pady=3)
        tk.Label(imu_frame, text="IMU:", bg=COLORS["card"], fg=COLORS["text"],
                 font=("Microsoft YaHei", 10), width=5, anchor="e").pack(side=tk.LEFT)
        self._imu_addr_var = tk.StringVar(value=str(self._config.imu.address))
        tk.Entry(imu_frame, textvariable=self._imu_addr_var, width=4,
                 font=("Consolas", 11), justify=tk.CENTER,
                 bg="#FFFFFF", fg=COLORS["text"], relief=tk.FLAT,
                 highlightbackground=COLORS["border"], highlightthickness=1,
                 insertbackground=COLORS["accent"]).pack(side=tk.LEFT, padx=4)

        btn_frame = tk.Frame(parent, bg=COLORS["card"])
        btn_frame.pack(fill=tk.X, padx=8, pady=10)
        ControlButton(btn_frame, "\u5e94\u7528\u5730\u5740", COLORS["accent"], 100, 34,
                      self._apply_addresses, primary=True).pack(side=tk.LEFT)

    def _build_log_panel(self, parent):
        card = self._card(parent, "\u901a\u4fe1\u65e5\u5fd7")
        container = tk.Frame(card, bg=COLORS["card"])
        container.pack(fill=tk.X, padx=16, pady=(4, 10))

        log_frame = tk.Frame(container, bg=COLORS["dark_surface"], height=120)
        log_frame.pack(fill=tk.X)
        log_frame.pack_propagate(False)

        self._log_text = tk.Text(log_frame, bg=COLORS["dark_surface"], fg="#C8C8C8",
                                  font=("Consolas", 9), height=5,
                                  relief=tk.FLAT, wrap=tk.WORD,
                                  state=tk.DISABLED, cursor="arrow",
                                  insertbackground="#C8C8C8",
                                  selectbackground=COLORS["accent"],
                                  selectforeground="#FFFFFF",
                                  padx=8, pady=6)
        scrollbar = tk.Scrollbar(log_frame, command=self._log_text.yview,
                                  bg=COLORS["dark_surface"], troughcolor=COLORS["dark_surface"],
                                  activebackground=COLORS["text_tertiary"])
        self._log_text.config(yscrollcommand=scrollbar.set)

        self._log_text.tag_configure("TX", foreground=COLORS["accent"])
        self._log_text.tag_configure("RX", foreground=COLORS["success"])
        self._log_text.tag_configure("ERR", foreground=COLORS["danger"])
        self._log_text.tag_configure("WARN", foreground=COLORS["warning"])
        self._log_text.tag_configure("LOG", foreground="#C8C8C8")
        self._log_text.tag_configure("INFO", foreground=COLORS["warning"])
        self._log_text.tag_configure("TIME", foreground=COLORS["text_tertiary"])

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        btn_row = tk.Frame(container, bg=COLORS["card"])
        btn_row.pack(fill=tk.X, pady=(6, 0))
        ControlButton(btn_row, "\u6e05\u7a7a\u65e5\u5fd7", COLORS["text_secondary"], 80, 26,
                      self._clear_log).pack(side=tk.LEFT)
        self._log_paused = False
        self._pause_btn = ControlButton(btn_row, "\u6682\u505c\u6eda\u52a8", COLORS["text_secondary"], 80, 26,
                                         self._toggle_log_pause)
        self._pause_btn.pack(side=tk.LEFT, padx=6)

        self._client.set_log_callback(self._on_modbus_log)
        self._log_max_lines = 500

        gui_handler = GUILogHandler(self._on_modbus_log)
        gui_handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
        logging.getLogger().addHandler(gui_handler)

    def _on_modbus_log(self, direction: str, message: str):
        self.after(0, self._append_log, direction, message)

    def _append_log(self, direction: str, message: str):
        import datetime
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        tag = direction if direction in ("TX", "RX", "ERR", "WARN", "LOG") else "INFO"

        direction_map = {
            "TX": "TX >>",
            "RX": "RX <<",
            "ERR": "ERR!!",
            "WARN": "WARN >",
            "LOG": "LOG  >",
            "INFO": "INFO ",
        }
        prefix = direction_map.get(direction, direction)

        self._log_text.config(state=tk.NORMAL)
        self._log_text.insert(tk.END, f"[{ts}] ", "TIME")
        self._log_text.insert(tk.END, f"{prefix} ", tag)
        self._log_text.insert(tk.END, f"{message}\n")

        line_count = int(self._log_text.index("end-1c").split(".")[0])
        if line_count > self._log_max_lines:
            self._log_text.delete("1.0", f"{line_count - self._log_max_lines}.0")

        if not self._log_paused:
            self._log_text.see(tk.END)
        self._log_text.config(state=tk.DISABLED)

    def _clear_log(self):
        self._log_text.config(state=tk.NORMAL)
        self._log_text.delete("1.0", tk.END)
        self._log_text.config(state=tk.DISABLED)

    def _toggle_log_pause(self):
        self._log_paused = not self._log_paused
        if self._log_paused:
            self._pause_btn.update_appearance(text="\u6062\u590d\u6eda\u52a8", color=COLORS["accent"])
        else:
            self._pause_btn.update_appearance(text="\u6682\u505c\u6eda\u52a8", color=COLORS["text_secondary"])
            self._log_text.config(state=tk.NORMAL)
            self._log_text.see(tk.END)
            self._log_text.config(state=tk.DISABLED)

    def _bind_keys(self):
        self.bind_all("<KeyPress>", self._on_key_press)
        self.bind_all("<KeyRelease>", self._on_key_release)
        self.focus_set()

    def _on_key_press(self, event):
        focus_widget = self.focus_get()
        if focus_widget and isinstance(focus_widget, (tk.Entry, ttk.Combobox)):
            return
        key = event.keysym.lower()
        if key == "space":
            key = " "
        if key in self._key_states and self._key_states[key]:
            return
        self._key_states[key] = True
        btn_key = "space" if key == " " else key
        if btn_key in self._key_buttons:
            self._key_buttons[btn_key].flash_press()
        self._on_key_action(key)

    def _on_key_release(self, event):
        focus_widget = self.focus_get()
        if focus_widget and isinstance(focus_widget, (tk.Entry, ttk.Combobox)):
            return
        key = event.keysym.lower()
        if key == "space":
            key = " "
        self._key_states[key] = False
        btn_key = "space" if key == " " else key
        if btn_key in self._key_buttons:
            self._key_buttons[btn_key].flash_release()

    def _on_key_action(self, key: str):
        if not self._connected and key not in (" ",):
            return
        if key in ("w", "a", "s", "d", "q", "z", " "):
            self._tank.apply_keyboard_action(key)
            self._update_tank_display()
            self.focus_set()

    def _toggle_connection(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        self._config.serial.port = self._serial_var.get()
        self._config.serial.baudrate = int(self._baud_var.get())
        self._config.serial.parity = self._parity_var.get()

        if self._client.connect():
            self._connected = True
            self._imu.start_polling()
            self._status_label.config(text="\u25cf \u5df2\u8fde\u63a5", fg=COLORS["success"])
            self._connect_btn.update_appearance(text="\u65ad\u5f00", color=COLORS["danger"])
        else:
            self._status_label.config(text="\u25cf \u8fde\u63a5\u5931\u8d25", fg=COLORS["danger"])

    def _disconnect(self):
        self._imu.stop_polling()
        self._tank.stop_all()
        self._client.disconnect()
        self._connected = False
        self._status_label.config(text="\u25cf \u672a\u8fde\u63a5", fg=COLORS["danger"])
        self._connect_btn.update_appearance(text="\u8fde\u63a5", color=COLORS["accent"])

    def _toggle_auto(self):
        self._auto_stable = not self._auto_stable
        if self._auto_stable:
            self._attitude.enable()
            self._auto_btn.update_appearance(text="\u81ea\u52a8\u7a33\u5b9a: \u5f00", color=COLORS["success"])
        else:
            self._attitude.disable()
            self._auto_btn.update_appearance(text="\u81ea\u52a8\u7a33\u5b9a: \u5173", color=COLORS["accent"])

    def _apply_pid(self):
        try:
            for axis_key in ("pitch", "roll"):
                params = getattr(self._config.attitude, f"{axis_key}_pid")
                params.kp = float(self._pid_vars[axis_key]["Kp"].get())
                params.ki = float(self._pid_vars[axis_key]["Ki"].get())
                params.kd = float(self._pid_vars[axis_key]["Kd"].get())
            self._attitude._pitch_pid = type(self._attitude._pitch_pid)(
                self._config.attitude.pitch_pid)
            self._attitude._roll_pid = type(self._attitude._roll_pid)(
                self._config.attitude.roll_pid)
            logger.info("PID\u53c2\u6570\u5df2\u66f4\u65b0")
        except ValueError:
            logger.error("PID\u53c2\u6570\u683c\u5f0f\u9519\u8bef")

    def _apply_target(self):
        try:
            self._config.attitude.target_pitch = float(self._target_pitch_var.get())
            self._config.attitude.target_roll = float(self._target_roll_var.get())
        except ValueError:
            logger.error("\u76ee\u6807\u89d2\u5ea6\u683c\u5f0f\u9519\u8bef")

    def _apply_addresses(self):
        try:
            for tid in TankId:
                self._config.tanks[tid].address = int(self._addr_vars[tid].get())
            self._config.imu.address = int(self._imu_addr_var.get())
            logger.info("\u8bbe\u5907\u5730\u5740\u5df2\u66f4\u65b0")
        except ValueError:
            logger.error("\u5730\u5740\u683c\u5f0f\u9519\u8bef")

    def _schedule_update(self):
        self._update_loop()

    def _update_loop(self):
        if self._auto_stable and self._connected:
            pitch_out, roll_out = self._attitude.compute()
            self._tank.apply_attitude_output(pitch_out, roll_out)

        if self._connected:
            pitch, roll, yaw = self._imu.get_angles()
            self._attitude_indicator.update_angles(pitch, roll, yaw)
            self._pitch_value.config(text=f"{pitch:+.2f}\u00b0")
            self._roll_value.config(text=f"{roll:+.2f}\u00b0")
            self._yaw_value.config(text=f"{yaw:+.2f}\u00b0")

        self._update_tank_display()

        self._update_timer = self.after(100, self._update_loop)

    def _update_tank_display(self):
        states = self._tank.get_all_states()
        self._cage_top_view.set_states(states)

    def destroy(self):
        if self._update_timer:
            self.after_cancel(self._update_timer)
        if self._connected:
            self._disconnect()
        super().destroy()
