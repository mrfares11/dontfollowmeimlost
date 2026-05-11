#!/usr/bin/env python3
"""
landmark_mission_gui.py

Professional phase-2 GUI for the AMR project.

It supports:
1. Direct navigation to saved landmarks.
2. Mission execution:
   A. Grocery delivery:
      Docking station -> Supermarket -> selected house -> Docking station
   B. Food delivery:
      Docking station -> Restaurant -> selected house -> Docking station
   C. Fire emergency:
      Docking station -> Firefighting center -> selected house -> Docking station
   D. Medical help:
      Docking station -> Pharmacy -> selected house -> Docking station

Unavailable missions/houses are disabled and colored gray when the required
landmark is missing from landmarks.yaml.
"""

import math
import os
import re
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


DEFAULT_LANDMARKS_FILE = "/home/hadi/amr_project/param/landmarks.yaml"
NAV_ACTION_NAME = "navigate_to_pose"

# Watchdog: cancel goal if no progress for this long (seconds)
NO_PROGRESS_TIMEOUT_S = 30.0
# Progress means distance_remaining decreased by this much (meters)
PROGRESS_DELTA_M = 0.5
# How often watchdog timer runs (seconds)
WATCHDOG_PERIOD_S = 2.0
# Retries allowed per goal before giving up
WATCHDOG_RETRY_LIMIT = 1


# ============================================================
# UI theme
# ============================================================
COLORS = {
    "app_bg": "#EEF2F7",
    "panel": "#FFFFFF",
    "panel_soft": "#F8FAFC",
    "border": "#D7DEE8",
    "text": "#172033",
    "muted": "#667085",
    "primary": "#2563EB",
    "primary_dark": "#1D4ED8",
    "success": "#16A34A",
    "success_dark": "#15803D",
    "success_soft": "#DCFCE7",
    "warning": "#F59E0B",
    "warning_dark": "#D97706",
    "danger": "#DC2626",
    "danger_dark": "#B91C1C",
    "danger_soft": "#FEE2E2",
    "disabled": "#E5E7EB",
    "disabled_text": "#8A94A6",
    "selected": "#DBEAFE",
    "mission_soft": "#EEF2FF",
    "house_soft": "#E0F2FE",
}

FONT = "Arial"


# ============================================================
# Landmark aliases
# ============================================================
def normalize_name(name: str) -> str:
    return re.sub(r"[^A-Z0-9]", "", str(name).upper())


ALIASES = {
    "DOCK": [
        "DOCK", "DOCKING_STATION", "DOCKING STATION",
        "DOCKINGSTATION", "HOME", "BASE",
    ],
    "SUPERMARKET": [
        "SUPERMARKET", "SUPER_MARKET", "SUPER MARKET",
        "MARKET", "GROCERY", "GROCERY_STORE", "GROCERY STORE",
    ],
    "RESTAURANT": [
        "RESTAURANT", "FOOD", "FOOD_PLACE", "FOOD PLACE",
    ],
    "FIRE_STATION": [
        "FIRE_STATION", "FIRE STATION", "FIRESTATION",
        "FIREFIGHTING_CENTER", "FIRE_FIGHTING_CENTER",
        "FIREFIGHTING CENTER", "FIRE FIGHTING CENTER",
        "FIRE_CENTER", "FIRE CENTER",
    ],
    "PHARMACY": [
        "PHARMACY", "PHARMA", "MEDICAL",
        "MEDICAL_CENTER", "MEDICAL CENTER", "CLINIC",
    ],
}

for i in range(1, 11):
    ALIASES[f"HOUSE_{i}"] = [
        f"HOUSE_{i}", f"HOUSE {i}", f"HOUSE{i}", f"H{i}",
    ]


MISSION_DEFS = {
    "GROCERY": {
        "letter": "A",
        "display": "Grocery delivery",
        "service_key": "SUPERMARKET",
        "service_display": "Supermarket",
        "description": "Pick up groceries and deliver them to a selected house.",
        "route": "Dock → Supermarket → House → Dock",
    },
    "FOOD": {
        "letter": "B",
        "display": "Food delivery",
        "service_key": "RESTAURANT",
        "service_display": "Restaurant",
        "description": "Pick up food from the restaurant and deliver it to a selected house.",
        "route": "Dock → Restaurant → House → Dock",
    },
    "FIRE": {
        "letter": "C",
        "display": "Fire emergency",
        "service_key": "FIRE_STATION",
        "service_display": "Firefighting center",
        "description": "Pick up firefighting tools and travel to the selected house.",
        "route": "Dock → Firefighting center → House → Dock",
    },
    "MEDICAL": {
        "letter": "D",
        "display": "Medical help",
        "service_key": "PHARMACY",
        "service_display": "Pharmacy",
        "description": "Pick up medical supplies and deliver them to a selected house.",
        "route": "Dock → Pharmacy → House → Dock",
    },
}


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class LandmarkMissionGui(Node):
    def __init__(self):
        super().__init__("landmark_mission_gui")

        self.declare_parameter("landmarks_file", DEFAULT_LANDMARKS_FILE)

        self.landmarks_file = (
            self.get_parameter("landmarks_file").get_parameter_value().string_value
        )

        self.action_client = ActionClient(self, NavigateToPose, NAV_ACTION_NAME)

        self.landmarks = {}
        self.alias_map = {}

        self.current_goal_handle = None
        # Watchdog state for no-progress detection
        self.watchdog_last_dist = None
        self.watchdog_last_progress_time = None
        self.watchdog_retries_done = 0
        self.watchdog_last_goal_name = None
        self.watchdog_last_goal_lm = None
        # Start watchdog periodic timer
        self.watchdog_timer = self.create_timer(
            WATCHDOG_PERIOD_S, self._watchdog_check
        )
        self.current_goal_name = None

        self.active_sequence = []
        self.active_sequence_names = []
        self.active_sequence_index = 0
        self.active_mission_name = None

        self.selected_mission_key = None
        self.selected_house_key = None

        self.root = tk.Tk()
        self.root.title("AMR Phase 2 Control Center")
        self.root.geometry("980x720")
        self.root.minsize(900, 680)
        self.root.configure(bg=COLORS["app_bg"])

        self.setup_styles()
        self.build_header()
        self.build_body()
        self.build_footer()

        self.refresh_landmarks()

        self.root.after(100, self.spin_ros_once)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ============================================================
    # Style helpers
    # ============================================================
    def setup_styles(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure(
            "TNotebook",
            background=COLORS["app_bg"],
            borderwidth=0,
        )
        style.configure(
            "TNotebook.Tab",
            font=(FONT, 11, "bold"),
            padding=(20, 10),
            background="#E3EAF5",
            foreground=COLORS["text"],
            borderwidth=0,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLORS["panel"])],
            foreground=[("selected", COLORS["primary"])],
        )

    def style_button(self, button, kind="primary", enabled=True):
        if not enabled:
            button.configure(
                state="disabled",
                bg=COLORS["disabled"],
                fg=COLORS["disabled_text"],
                activebackground=COLORS["disabled"],
                activeforeground=COLORS["disabled_text"],
                relief="flat",
                bd=0,
                cursor="arrow",
            )
            return

        schemes = {
            "primary": (COLORS["primary"], "#FFFFFF", COLORS["primary_dark"]),
            "success": (COLORS["success"], "#FFFFFF", COLORS["success_dark"]),
            "warning": (COLORS["warning"], "#FFFFFF", COLORS["warning_dark"]),
            "danger": (COLORS["danger"], "#FFFFFF", COLORS["danger_dark"]),
            "light": (COLORS["panel_soft"], COLORS["text"], "#E5EAF3"),
            "mission": (COLORS["mission_soft"], COLORS["text"], "#E0E7FF"),
            "house": (COLORS["house_soft"], COLORS["text"], "#BAE6FD"),
            "selected": (COLORS["selected"], COLORS["primary"], "#BFDBFE"),
        }
        bg, fg, active = schemes.get(kind, schemes["primary"])
        button.configure(
            state="normal",
            bg=bg,
            fg=fg,
            activebackground=active,
            activeforeground=fg,
            relief="flat",
            bd=0,
            cursor="hand2",
        )

    def card(self, parent, title=None, subtitle=None):
        outer = tk.Frame(
            parent,
            bg=COLORS["panel"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
            bd=0,
        )
        inner = tk.Frame(outer, bg=COLORS["panel"])
        inner.pack(fill="both", expand=True, padx=18, pady=16)

        if title:
            tk.Label(
                inner,
                text=title,
                font=(FONT, 13, "bold"),
                fg=COLORS["text"],
                bg=COLORS["panel"],
            ).pack(anchor="w")
        if subtitle:
            tk.Label(
                inner,
                text=subtitle,
                font=(FONT, 10),
                fg=COLORS["muted"],
                bg=COLORS["panel"],
                justify="left",
                wraplength=820,
            ).pack(anchor="w", pady=(3, 10))
        elif title:
            tk.Frame(inner, bg=COLORS["panel"], height=8).pack()

        return outer, inner

    # ============================================================
    # Layout
    # ============================================================
    def build_header(self):
        header = tk.Frame(self.root, bg=COLORS["app_bg"])
        header.pack(fill="x", padx=24, pady=(20, 10))

        left = tk.Frame(header, bg=COLORS["app_bg"])
        left.pack(side="left", fill="x", expand=True)

        tk.Label(
            left,
            text="AMR Phase 2 Control Center",
            font=(FONT, 23, "bold"),
            fg=COLORS["text"],
            bg=COLORS["app_bg"],
        ).pack(anchor="w")

        tk.Label(
            left,
            text="Saved-map navigation, landmark destinations, and mission execution",
            font=(FONT, 11),
            fg=COLORS["muted"],
            bg=COLORS["app_bg"],
        ).pack(anchor="w", pady=(2, 0))

        right = tk.Frame(header, bg=COLORS["app_bg"])
        right.pack(side="right")

        self.landmark_count_badge = tk.Label(
            right,
            text="0 landmarks",
            font=(FONT, 10, "bold"),
            fg=COLORS["primary"],
            bg="#E0EAFF",
            padx=14,
            pady=7,
        )
        self.landmark_count_badge.pack(anchor="e")

    def build_body(self):
        self.status_var = tk.StringVar()
        self.status_var.set("Loading system...")

        status_card = tk.Frame(
            self.root,
            bg=COLORS["panel"],
            highlightbackground=COLORS["border"],
            highlightthickness=1,
        )
        status_card.pack(fill="x", padx=24, pady=(0, 12))

        self.status_dot = tk.Label(
            status_card,
            text="●",
            font=(FONT, 14, "bold"),
            fg=COLORS["primary"],
            bg=COLORS["panel"],
        )
        self.status_dot.pack(side="left", padx=(16, 8), pady=10)

        self.status_label = tk.Label(
            status_card,
            textvariable=self.status_var,
            font=(FONT, 11),
            fg=COLORS["text"],
            bg=COLORS["panel"],
            anchor="w",
            justify="left",
        )
        self.status_label.pack(side="left", fill="x", expand=True, pady=10)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=24, pady=(0, 12))

        self.direct_tab = tk.Frame(self.notebook, bg=COLORS["app_bg"])
        self.mission_tab = tk.Frame(self.notebook, bg=COLORS["app_bg"])

        self.notebook.add(self.direct_tab, text="Go to specific location")
        self.notebook.add(self.mission_tab, text="Assign mission")

        self.build_direct_tab()
        self.build_mission_tab()

    def build_footer(self):
        footer = tk.Frame(self.root, bg=COLORS["app_bg"])
        footer.pack(fill="x", padx=24, pady=(0, 18))

        self.refresh_button = tk.Button(
            footer,
            text="Refresh landmarks",
            font=(FONT, 11, "bold"),
            padx=16,
            pady=9,
            command=self.refresh_landmarks,
        )
        self.refresh_button.pack(side="left")
        self.style_button(self.refresh_button, "light", True)

        self.global_cancel_button = tk.Button(
            footer,
            text="Emergency Stop Goal / Mission",
            font=(FONT, 11, "bold"),
            padx=18,
            pady=9,
            command=self.cancel_current_goal_or_mission,
        )
        self.global_cancel_button.pack(side="right")

    def build_direct_tab(self):
        tab = tk.Frame(self.direct_tab, bg=COLORS["app_bg"])
        tab.pack(fill="both", expand=True, padx=0, pady=0)

        left_card, left = self.card(
            tab,
            "Saved destinations",
            "Choose a landmark discovered during the mapping phase.",
        )
        left_card.pack(side="left", fill="both", expand=True, padx=(0, 8), pady=12)

        self.direct_listbox = tk.Listbox(
            left,
            font=(FONT, 12),
            height=17,
            exportselection=False,
            relief="flat",
            bd=0,
            bg=COLORS["panel_soft"],
            fg=COLORS["text"],
            selectbackground=COLORS["primary"],
            selectforeground="#FFFFFF",
            highlightthickness=1,
            highlightbackground=COLORS["border"],
        )
        self.direct_listbox.pack(fill="both", expand=True)
        self.direct_listbox.bind("<<ListboxSelect>>", self.on_direct_select)

        right_card, right = self.card(
            tab,
            "Destination details",
            "Review the target pose before sending the goal to Nav2.",
        )
        right_card.pack(side="right", fill="both", expand=True, padx=(8, 0), pady=12)

        self.direct_info = tk.Text(
            right,
            height=15,
            font=(FONT, 11),
            wrap="word",
            state="disabled",
            relief="flat",
            bd=0,
            bg=COLORS["panel_soft"],
            fg=COLORS["text"],
            padx=12,
            pady=12,
        )
        self.direct_info.pack(fill="both", expand=True)

        button_row = tk.Frame(right, bg=COLORS["panel"])
        button_row.pack(fill="x", pady=(14, 0))

        self.direct_cancel_button = tk.Button(
            button_row,
            text="Cancel Destination",
            font=(FONT, 11, "bold"),
            padx=14,
            pady=9,
            command=self.cancel_current_goal_or_mission,
        )
        self.direct_cancel_button.pack(side="left")

        self.direct_confirm_button = tk.Button(
            button_row,
            text="Confirm and Navigate",
            font=(FONT, 11, "bold"),
            padx=16,
            pady=9,
            command=self.confirm_direct_navigation,
        )
        self.direct_confirm_button.pack(side="right")
        self.style_button(self.direct_confirm_button, "success", True)

    def build_mission_tab(self):
        tab = tk.Frame(self.mission_tab, bg=COLORS["app_bg"])
        tab.pack(fill="both", expand=True)

        top_card, top = self.card(
            tab,
            "Mission control",
            "Select a mission type and target house. The robot will execute the route step by step.",
        )
        top_card.pack(fill="x", pady=(12, 8))

        control_row = tk.Frame(top, bg=COLORS["panel"])
        control_row.pack(fill="x")

        self.start_mission_button = tk.Button(
            control_row,
            text="Start Mission",
            font=(FONT, 12, "bold"),
            padx=18,
            pady=10,
            command=self.start_mission,
        )
        self.start_mission_button.pack(side="left")

        self.stop_mission_button = tk.Button(
            control_row,
            text="Stop Mission",
            font=(FONT, 12, "bold"),
            padx=18,
            pady=10,
            command=self.cancel_current_goal_or_mission,
        )
        self.stop_mission_button.pack(side="left", padx=(10, 0))

        self.clear_mission_button = tk.Button(
            control_row,
            text="Clear Selection",
            font=(FONT, 11, "bold"),
            padx=16,
            pady=10,
            command=self.clear_mission_selection,
        )
        self.clear_mission_button.pack(side="right")
        self.style_button(self.clear_mission_button, "light", True)

        middle = tk.Frame(tab, bg=COLORS["app_bg"])
        middle.pack(fill="both", expand=True, pady=(0, 8))

        mission_card, mission_frame = self.card(
            middle,
            "Mission type",
            "Unavailable missions are disabled if their required service location is missing.",
        )
        mission_card.pack(side="left", fill="both", expand=True, padx=(0, 8))

        self.mission_buttons = {}
        for mission_key, mission in MISSION_DEFS.items():
            btn = tk.Button(
                mission_frame,
                text=(
                    f"{mission['letter']}. {mission['display']}\n"
                    f"{mission['route']}\n"
                    f"{mission['description']}"
                ),
                font=(FONT, 10),
                justify="left",
                anchor="w",
                padx=12,
                pady=10,
                command=lambda key=mission_key: self.select_mission(key),
            )
            btn.pack(fill="x", pady=4)
            self.mission_buttons[mission_key] = btn

        house_card, house_frame = self.card(
            middle,
            "Target house",
            "Only houses that exist in landmarks.yaml are available.",
        )
        house_card.pack(side="right", fill="both", expand=True, padx=(8, 0))

        house_grid = tk.Frame(house_frame, bg=COLORS["panel"])
        house_grid.pack(fill="x")

        self.house_buttons = {}
        for i in range(1, 11):
            house_key = f"HOUSE_{i}"
            btn = tk.Button(
                house_grid,
                text=f"House {i}",
                font=(FONT, 10, "bold"),
                width=10,
                padx=8,
                pady=10,
                command=lambda key=house_key: self.select_house(key),
            )
            btn.grid(row=(i - 1) // 5, column=(i - 1) % 5, padx=5, pady=5, sticky="ew")
            house_grid.grid_columnconfigure((i - 1) % 5, weight=1)
            self.house_buttons[house_key] = btn

        preview_card, preview = self.card(
            tab,
            "Mission preview",
            "The sequence below will be sent to Nav2 one goal at a time.",
        )
        preview_card.pack(fill="x", pady=(0, 12))

        self.mission_preview = tk.Text(
            preview,
            height=7,
            font=(FONT, 11),
            wrap="word",
            state="disabled",
            relief="flat",
            bd=0,
            bg=COLORS["panel_soft"],
            fg=COLORS["text"],
            padx=12,
            pady=10,
        )
        self.mission_preview.pack(fill="x")

    # ============================================================
    # Landmark loading
    # ============================================================
    def refresh_landmarks(self):
        self.landmarks, self.alias_map = self.load_landmarks(self.landmarks_file)

        self.direct_listbox.delete(0, tk.END)
        for name in sorted(self.landmarks.keys()):
            self.direct_listbox.insert(tk.END, f"  {name}")

        self.show_direct_info(None)

        count = len(self.landmarks)
        self.landmark_count_badge.configure(text=f"{count} landmark{'s' if count != 1 else ''}")

        if self.landmarks:
            self.set_status(
                f"Loaded {count} valid map-frame landmark(s) from {self.landmarks_file}",
                "ready",
            )
        else:
            self.set_status(
                f"No valid map-frame landmarks found in {self.landmarks_file}",
                "warning",
            )

        self.update_mission_availability()
        self.update_mission_preview()

    def load_landmarks(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f"Landmarks file does not exist: {path}")
            return {}, {}

        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f"Could not read landmarks file: {exc}")
            return {}, {}

        valid = {}
        for name, entry in data.items():
            if not isinstance(entry, dict):
                continue

            ref = entry.get("reference_frame", "")
            pose = entry.get("approach_pose", None)

            if ref != "map":
                self.get_logger().warn(
                    f"Skipping {name}: reference_frame is '{ref}', expected 'map'"
                )
                continue

            if not isinstance(pose, dict):
                self.get_logger().warn(f"Skipping {name}: missing approach_pose")
                continue

            if not all(k in pose for k in ("x", "y", "yaw")):
                self.get_logger().warn(
                    f"Skipping {name}: approach_pose must contain x, y, yaw"
                )
                continue

            try:
                valid[str(name)] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "yaw": float(pose["yaw"]),
                    "sightings": int(entry.get("sightings", 0)),
                    "best_quality": float(entry.get("best_quality", 0.0)),
                    "last_updated": str(entry.get("last_updated", "")),
                }
            except Exception as exc:
                self.get_logger().warn(f"Skipping {name}: invalid pose values: {exc}")

        alias_map = {}
        normalized_actual = {normalize_name(name): name for name in valid.keys()}

        for canonical, aliases in ALIASES.items():
            for alias in aliases:
                norm = normalize_name(alias)
                if norm in normalized_actual:
                    alias_map[canonical] = normalized_actual[norm]
                    break

        return valid, alias_map

    def actual_name(self, canonical_key):
        return self.alias_map.get(canonical_key)

    def has_location(self, canonical_key):
        return self.actual_name(canonical_key) is not None

    def set_status(self, message, level="ready"):
        self.status_var.set(message)
        if level == "ready":
            self.status_dot.configure(fg=COLORS["success"])
        elif level == "warning":
            self.status_dot.configure(fg=COLORS["warning"])
        elif level == "danger":
            self.status_dot.configure(fg=COLORS["danger"])
        else:
            self.status_dot.configure(fg=COLORS["primary"])

    # ============================================================
    # Direct navigation
    # ============================================================
    def selected_direct_name(self):
        selection = self.direct_listbox.curselection()
        if not selection:
            return None
        return self.direct_listbox.get(selection[0]).strip()

    def on_direct_select(self, _event=None):
        self.show_direct_info(self.selected_direct_name())

    def show_direct_info(self, name):
        self.direct_info.configure(state="normal")
        self.direct_info.delete("1.0", tk.END)

        if name is None:
            self.direct_info.insert(tk.END, "No destination selected.\n\n")
            self.direct_info.insert(
                tk.END,
                "Select a landmark from the list, then press Confirm and Navigate.",
            )
        else:
            lm = self.landmarks[name]
            self.direct_info.insert(tk.END, f"Destination\n{name}\n\n")
            self.direct_info.insert(tk.END, "Navigation frame\nmap\n\n")
            self.direct_info.insert(tk.END, "Approach pose\n")
            self.direct_info.insert(tk.END, f"  x:   {lm['x']:.3f} m\n")
            self.direct_info.insert(tk.END, f"  y:   {lm['y']:.3f} m\n")
            self.direct_info.insert(tk.END, f"  yaw: {math.degrees(lm['yaw']):.1f} deg\n\n")
            self.direct_info.insert(tk.END, "Landmark quality\n")
            self.direct_info.insert(tk.END, f"  sightings:     {lm['sightings']}\n")
            self.direct_info.insert(tk.END, f"  best quality:  {lm['best_quality']:.3f}\n")
            if lm["last_updated"]:
                self.direct_info.insert(tk.END, f"  last updated:  {lm['last_updated']}\n")

        self.direct_info.configure(state="disabled")

    def confirm_direct_navigation(self):
        name = self.selected_direct_name()
        if name is None:
            messagebox.showwarning("No destination selected", "Select a location first.")
            return

        if self.current_goal_handle is not None:
            messagebox.showwarning(
                "Robot is busy",
                "A goal or mission is already active. Cancel it first.",
            )
            return

        lm = self.landmarks[name]
        ok = messagebox.askyesno(
            "Confirm navigation",
            (
                f"Send robot to {name}?\n\n"
                f"x = {lm['x']:.2f} m\n"
                f"y = {lm['y']:.2f} m\n"
                f"yaw = {math.degrees(lm['yaw']):.1f} deg"
            ),
        )
        if not ok:
            return

        self.clear_active_mission_state()
        self.send_goal(name, lm)

    # ============================================================
    # Mission selection
    # ============================================================
    def update_mission_availability(self):
        dock_ok = self.has_location("DOCK")

        for mission_key, mission in MISSION_DEFS.items():
            service_ok = self.has_location(mission["service_key"])
            available = dock_ok and service_ok

            btn = self.mission_buttons[mission_key]
            if available:
                selected = mission_key == self.selected_mission_key
                self.style_button(btn, "selected" if selected else "mission", True)
                btn.configure(
                    text=(
                        f"{mission['letter']}. {mission['display']}\n"
                        f"{mission['route']}\n"
                        f"{mission['description']}"
                    )
                )
            else:
                missing = []
                if not dock_ok:
                    missing.append("Docking station")
                if not service_ok:
                    missing.append(mission["service_display"])

                self.style_button(btn, "mission", False)
                btn.configure(
                    text=(
                        f"{mission['letter']}. {mission['display']}\n"
                        f"Unavailable: missing {', '.join(missing)}"
                    )
                )

        for house_key, btn in self.house_buttons.items():
            if self.has_location(house_key):
                selected = house_key == self.selected_house_key
                self.style_button(btn, "selected" if selected else "house", True)
            else:
                self.style_button(btn, "house", False)

        self.update_start_stop_button_states()

    def select_mission(self, mission_key):
        self.selected_mission_key = mission_key
        self.update_mission_availability()
        self.update_mission_preview()

    def select_house(self, house_key):
        self.selected_house_key = house_key
        self.update_mission_availability()
        self.update_mission_preview()

    def clear_mission_selection(self):
        self.selected_mission_key = None
        self.selected_house_key = None
        self.update_mission_availability()
        self.update_mission_preview()

    def update_start_stop_button_states(self):
        mission_ready = (
            self.selected_mission_key is not None
            and self.selected_house_key is not None
            and self.has_location("DOCK")
            and self.has_location(self.selected_house_key)
            and self.has_location(MISSION_DEFS[self.selected_mission_key]["service_key"])
            and self.current_goal_handle is None
        )

        self.style_button(self.start_mission_button, "success", mission_ready)

        active = self.current_goal_handle is not None or self.active_mission_name is not None
        self.style_button(self.stop_mission_button, "danger", active)
        self.style_button(self.direct_cancel_button, "warning", active)
        self.style_button(self.global_cancel_button, "danger", active)

    def update_mission_preview(self):
        self.mission_preview.configure(state="normal")
        self.mission_preview.delete("1.0", tk.END)

        if self.selected_mission_key is None:
            self.mission_preview.insert(tk.END, "Select a mission type.\n\n")
            self.mission_preview.insert(tk.END, "Then select the target house and press Start Mission.")
        elif self.selected_house_key is None:
            mission = MISSION_DEFS[self.selected_mission_key]
            self.mission_preview.insert(tk.END, f"Mission\n{mission['display']}\n\n")
            self.mission_preview.insert(tk.END, f"Route\n{mission['route']}\n\n")
            self.mission_preview.insert(tk.END, "Next step\nSelect a target house.")
        else:
            mission = MISSION_DEFS[self.selected_mission_key]
            house_actual = self.actual_name(self.selected_house_key) or self.selected_house_key
            dock_actual = self.actual_name("DOCK") or "Docking station"
            service_actual = self.actual_name(mission["service_key"]) or mission["service_display"]

            self.mission_preview.insert(tk.END, f"Mission\n{mission['display']}\n\n")
            self.mission_preview.insert(tk.END, "Execution sequence\n")
            self.mission_preview.insert(tk.END, f"  1. {dock_actual}\n")
            self.mission_preview.insert(tk.END, f"  2. {service_actual}\n")
            self.mission_preview.insert(tk.END, f"  3. {house_actual}\n")
            self.mission_preview.insert(tk.END, f"  4. {dock_actual}\n\n")
            self.mission_preview.insert(tk.END, "Press Start Mission to begin.")

        self.mission_preview.configure(state="disabled")

    def start_mission(self):
        if self.current_goal_handle is not None:
            messagebox.showwarning(
                "Robot is busy",
                "A goal or mission is already active. Cancel it first.",
            )
            return

        if self.selected_mission_key is None or self.selected_house_key is None:
            messagebox.showwarning(
                "Incomplete mission",
                "Select a mission type and a target house first.",
            )
            return

        mission = MISSION_DEFS[self.selected_mission_key]
        service_key = mission["service_key"]

        required_keys = ["DOCK", service_key, self.selected_house_key, "DOCK"]
        missing = [key for key in required_keys if not self.has_location(key)]
        if missing:
            messagebox.showerror(
                "Mission unavailable",
                f"Missing required landmark(s): {', '.join(sorted(set(missing)))}",
            )
            return

        actual_names = [self.actual_name(key) for key in required_keys]
        readable_sequence = " → ".join(actual_names)

        ok = messagebox.askyesno(
            "Confirm mission",
            f"Start mission?\n\n{mission['display']}\n\nSequence:\n{readable_sequence}",
        )
        if not ok:
            return

        self.active_mission_name = mission["display"]
        self.active_sequence_names = actual_names
        self.active_sequence = [self.landmarks[name] for name in actual_names]
        self.active_sequence_index = 0

        self.set_status(f"Starting mission: {self.active_mission_name}", "info")
        self.update_start_stop_button_states()
        self.send_next_sequence_goal()

    def send_next_sequence_goal(self):
        if self.active_sequence_index >= len(self.active_sequence):
            finished = self.active_mission_name or "Mission"
            self.set_status(f"{finished} completed.", "ready")
            messagebox.showinfo("Mission complete", f"{finished} completed.")
            self.clear_active_mission_state()
            self.update_start_stop_button_states()
            return

        name = self.active_sequence_names[self.active_sequence_index]
        lm = self.active_sequence[self.active_sequence_index]

        self.set_status(
            f"{self.active_mission_name}: step "
            f"{self.active_sequence_index + 1}/{len(self.active_sequence)} → {name}",
            "info",
        )
        self.send_goal(name, lm)

    # ============================================================
    # Nav2 action
    # ============================================================
    def send_goal(self, name, lm):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            messagebox.showerror(
                "Nav2 not ready",
                "NavigateToPose action server is not available yet.\n"
                "Wait until Nav2 is fully active, then try again.",
            )
            self.clear_active_mission_state()
            self.update_start_stop_button_states()
            return

        # Reset watchdog state for new goal
        self.watchdog_last_dist = None
        self.watchdog_last_progress_time = self.get_clock().now()
        self.watchdog_last_goal_name = name
        self.watchdog_last_goal_lm = lm

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = lm["x"]
        goal_msg.pose.pose.position.y = lm["y"]
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(lm["yaw"])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.current_goal_name = name

        self.get_logger().info(
            f"Sending Nav2 goal for {name}: "
            f"x={lm['x']:.3f}, y={lm['y']:.3f}, "
            f"yaw={math.degrees(lm['yaw']):.1f} deg"
        )

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(lambda fut: self.goal_response_callback(fut, name))

    def goal_response_callback(self, future, name):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.set_status(f"Failed to send goal: {name}", "danger")
            self.get_logger().error(f"Failed to send goal {name}: {exc}")
            messagebox.showerror("Goal failed", f"Failed to send goal {name}:\n{exc}")
            self.clear_active_mission_state()
            self.update_start_stop_button_states()
            return

        if not goal_handle.accepted:
            self.set_status(f"Goal rejected by Nav2: {name}", "danger")
            self.get_logger().error(f"Goal rejected by Nav2: {name}")
            messagebox.showerror(
                "Goal rejected",
                f"Nav2 rejected the goal for {name}.\n\n"
                "Check localization, map→chassis TF, and whether the target is reachable.",
            )
            self.current_goal_handle = None
            self.current_goal_name = None
            self.clear_active_mission_state()
            self.update_start_stop_button_states()
            return

        self.current_goal_handle = goal_handle
        self.current_goal_name = name
        self.set_status(f"Goal accepted: navigating to {name}", "info")
        self.get_logger().info(f"Goal accepted: {name}")

        self.update_start_stop_button_states()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self.result_callback(fut, name))

    def _watchdog_check(self):
        """Periodically check if the current goal is making progress."""
        if self.current_goal_handle is None:
            return
        if self.watchdog_last_progress_time is None:
            return
        elapsed = (self.get_clock().now() - self.watchdog_last_progress_time).nanoseconds / 1e9
        if elapsed < NO_PROGRESS_TIMEOUT_S:
            return

        # No progress for too long
        self.get_logger().warn(
            f"Watchdog: no progress for {elapsed:.1f}s, canceling current goal"
        )
        name = self.watchdog_last_goal_name
        lm = self.watchdog_last_goal_lm

        # Cancel
        try:
            self.current_goal_handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().error(f"Watchdog cancel failed: {e}")
        self.current_goal_handle = None
        self.set_status(f"No progress 30s, canceled: {name}", "warning")

        # Retry once, then give up
        if self.watchdog_retries_done < WATCHDOG_RETRY_LIMIT and name and lm:
            self.watchdog_retries_done += 1
            self.get_logger().warn(
                f"Watchdog: retrying {name} (attempt {self.watchdog_retries_done})"
            )
            self.set_status(
                f"Retrying {name} (attempt {self.watchdog_retries_done})",
                "info",
            )
            # Schedule the retry on the GUI thread shortly
            self.root.after(2000, lambda: self.send_goal(name, lm))
        else:
            self.get_logger().error(
                f"Watchdog: giving up on {name} after {self.watchdog_retries_done} retries"
            )
            self.set_status(f"Gave up: {name}", "danger")
            self.watchdog_retries_done = 0
            self.watchdog_last_goal_name = None
            self.watchdog_last_goal_lm = None

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        try:
            dist = feedback.distance_remaining
            # Watchdog: record progress if distance dropped enough
            if self.watchdog_last_dist is None:
                self.watchdog_last_dist = dist
                self.watchdog_last_progress_time = self.get_clock().now()
            elif dist < self.watchdog_last_dist - PROGRESS_DELTA_M:
                self.watchdog_last_dist = dist
                self.watchdog_last_progress_time = self.get_clock().now()
            if self.active_mission_name:
                step = self.active_sequence_index + 1
                total = len(self.active_sequence)
                current = self.active_sequence_names[self.active_sequence_index]
                self.set_status(
                    f"{self.active_mission_name}: step {step}/{total}, "
                    f"going to {current}. Distance remaining: {dist:.2f} m",
                    "info",
                )
            else:
                current = self.current_goal_name or "destination"
                self.set_status(
                    f"Navigating to {current}. Distance remaining: {dist:.2f} m",
                    "info",
                )
        except Exception:
            self.set_status("Navigating...", "info")

    def result_callback(self, future, name):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.set_status(f"Navigation result error: {name}", "danger")
            self.get_logger().error(f"Navigation result error for {name}: {exc}")
            self.current_goal_handle = None
            self.current_goal_name = None
            # If in a mission, skip this leg and continue; otherwise clear.
            if self.active_mission_name:
                self.active_sequence_index += 1
                self.watchdog_retries_done = 0
                self.root.after(500, self.send_next_sequence_goal)
            else:
                self.clear_active_mission_state()
            self.update_start_stop_button_states()
            return

        self.current_goal_handle = None
        self.current_goal_name = None

        # action_msgs/msg/GoalStatus:
        # 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if status == 4:
            self.get_logger().info(f"Navigation succeeded: {name}")
            # Reset watchdog retry budget for the next goal
            self.watchdog_retries_done = 0

            if self.active_mission_name:
                self.active_sequence_index += 1
                self.root.after(500, self.send_next_sequence_goal)
            else:
                self.set_status(f"Arrived at {name}", "ready")
                messagebox.showinfo("Navigation complete", f"Arrived at {name}.")

        elif status == 5:
            self.get_logger().warn(f"Navigation canceled: {name}")
            if self.active_mission_name:
                # Skip this leg, continue with the rest of the mission.
                self.set_status(
                    f"Skipped {name} (canceled), continuing mission",
                    "warning",
                )
                self.active_sequence_index += 1
                self.watchdog_retries_done = 0
                self.root.after(500, self.send_next_sequence_goal)
            else:
                self.set_status(f"Goal canceled: {name}", "warning")
                self.clear_active_mission_state()

        else:
            self.get_logger().error(f"Navigation failed/aborted: {name}, status={status}")
            if self.active_mission_name:
                # Skip this leg, continue with the rest of the mission.
                self.set_status(
                    f"Skipped {name} (failed, status={status}), continuing mission",
                    "warning",
                )
                self.active_sequence_index += 1
                self.watchdog_retries_done = 0
                self.root.after(500, self.send_next_sequence_goal)
            else:
                self.set_status(f"Navigation failed/aborted: {name}", "danger")
                messagebox.showerror(
                    "Navigation failed",
                    f"Navigation failed or was aborted at {name}.\nStatus code: {status}",
                )
                self.clear_active_mission_state()

        self.update_start_stop_button_states()

    def cancel_current_goal_or_mission(self):
        mission_was_active = self.active_mission_name is not None
        self.clear_active_mission_state()

        if self.current_goal_handle is None:
            self.set_status("No active goal or mission to cancel.", "warning")
            self.update_start_stop_button_states()
            return

        if mission_was_active:
            self.set_status("Stopping mission and canceling current goal...", "warning")
        else:
            self.set_status("Canceling destination...", "warning")

        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(lambda _fut: self.set_status("Cancel request sent.", "warning"))
        self.update_start_stop_button_states()

    def clear_active_mission_state(self):
        self.active_mission_name = None
        self.active_sequence = []
        self.active_sequence_names = []
        self.active_sequence_index = 0

    # ============================================================
    # ROS/Tkinter loop
    # ============================================================
    def spin_ros_once(self):
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as exc:
            self.get_logger().error(f"spin_once error: {exc}")
        self.root.after(50, self.spin_ros_once)

    def on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = LandmarkMissionGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
