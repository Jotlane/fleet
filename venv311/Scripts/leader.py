#!/usr/bin/env python3
"""
Leader–Follower GUI (path-locked + improved stop + PANIC DISARM)
- Path-locked followers with stop-latch: when leader stops, followers freeze a reference, stop yawing,
  and once within STOP_RADIUS, switch to HOLD and zero speed so they park cleanly.
- PANIC DISARM: Immediate HOLD + forced DISARM (param2=21196) to ALL links, no confirmation, do NOT stop groups.
"""

import math, time, threading, queue
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple
from collections import deque

import tkinter as tk
from tkinter import ttk, messagebox

# Python 3.10+ shim for DroneKit
import collections, collections.abc
for _n in ("MutableMapping","Mapping","Sequence","Iterable"):
    if not hasattr(collections,_n):
        setattr(collections,_n,getattr(collections.abc,_n))

from dronekit import connect, VehicleMode
from pymavlink import mavutil

# ---- Helpers ----

def _time_boot_ms():
    return int((time.monotonic() * 1000) % 0xFFFFFFFF)

EARTH_RADIUS_M = 6378137.0

def add_ne_offset_latlon(lat, lon, north_m, east_m):
    if lat is None or lon is None:
        return None, None
    dlat = north_m / EARTH_RADIUS_M
    denom = EARTH_RADIUS_M * max(1e-6, math.cos(math.radians(lat)))
    dlon = east_m / denom
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def rel_offset_from_heading(hdg_deg, right_m, back_m, fwd_extra_m=0.0):
    if hdg_deg is None or not math.isfinite(hdg_deg):
        hdg_deg = 0.0
    yaw = math.radians(hdg_deg)
    fwd = -back_m + fwd_extra_m
    r = right_m
    north = fwd * math.cos(yaw) - r * math.sin(yaw)
    east  = fwd * math.sin(yaw) + r * math.cos(yaw)
    return north, east

def _safe_int1e7(x, lo, hi):
    if x is None or not math.isfinite(x):
        return None
    x = max(lo, min(hi, x))
    try:
        return int(x * 1e7)
    except Exception:
        return None

def send_pos_target_global_int(v, lat_deg, lon_deg, alt_m, yaw_deg=None):
    lat_i = _safe_int1e7(lat_deg, -90.0, 90.0)
    lon_i = _safe_int1e7(lon_deg, -180.0, 180.0)
    if lat_i is None or lon_i is None:
        return
    mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
    if yaw_deg is None or not math.isfinite(yaw_deg):
        mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    try:
        v._master.mav.set_position_target_global_int_send(
            _time_boot_ms(),
            v._master.target_system, v._master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT, mask,
            lat_i, lon_i, float(alt_m), 0, 0, 0, 0, 0, 0,
            math.radians(yaw_deg) if (yaw_deg is not None and math.isfinite(yaw_deg)) else 0.0,
            0.0
        )
    except Exception:
        pass

def cmd_change_speed(v, s):
    try:
        v.message_factory.command_long_send(
            v._master.target_system, v._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
            1, float(s), -1, 0, 0, 0, 0
        )
    except Exception:
        pass

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dlon/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
    x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1)))
    brg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    return brg

def point_back_along_path(points, back_m):
    if len(points) < 2:
        return None
    remain = max(0.0, back_m)
    for i in range(len(points)-1, 0, -1):
        lat2, lon2 = points[i]
        lat1, lon1 = points[i-1]
        seg = haversine_m(lat1, lon1, lat2, lon2)
        if seg >= remain:
            u = (seg - remain) / max(seg, 1e-6)
            lat = lat1 + (lat2 - lat1) * u
            lon = lon1 + (lon2 - lon1) * u
            hdg = bearing_deg(lat1, lon1, lat2, lon2)
            return (lat, lon, hdg)
        remain -= seg
    lat, lon = points[0]
    hdg = bearing_deg(points[0][0], points[0][1], points[1][0], points[1][1]) if len(points) > 1 else 0.0
    return (lat, lon, hdg)

# ---- Models ----

@dataclass
class FollowerCfg:
    endpoint: str = ""
    side: str = "Right"
    lateral_m: float = 0.0
    back_m: float = 0.0

@dataclass
class GroupConfig:
    name: str
    leader: str
    followers: List[FollowerCfg] = field(default_factory=list)
    fixed_ned: bool = False
    hold_until_auto: bool = False
    match_speed: bool = True
    min_speed: float = 0.5
    max_speed: float = 4.0
    smooth_alpha: float = 0.4
    speed_cmd_period: float = 0.5
    speed_epsilon: float = 0.1
    enable_lead: bool = True
    lead_time_s: float = 1.5
    lead_max_m: float = 8.0
    rate_hz: float = 8.0

@dataclass
class FollowerState:
    link: any = None
    last_speed_cmd: float = float("nan")
    last_speed_time: float = 0.0
    parked: bool = False  # reached stop radius and switched to HOLD

@dataclass
class GroupState:
    thread: Optional[threading.Thread] = None
    stop_evt: threading.Event = field(default_factory=threading.Event)
    running: bool = False
    leader_link: any = None
    followers: List[FollowerState] = field(default_factory=list)
    filt_speed: Optional[float] = None

# ---- Worker ----

class LeaderFollowerWorker:
    def __init__(self, cfg: GroupConfig, state: GroupState, log_q: queue.Queue):
        self.cfg = cfg
        self.state = state
        self.log_q = log_q
        self.breadcrumb = deque(maxlen=800)  # ~100s @ 8Hz
        # stop-latch for stable final positioning
        self.stop_latch = False
        self.stop_since = 0.0
        self.stop_target = None  # (lat, lon, yaw)

    def log(self, msg): self.log_q.put(f"[{self.cfg.name}] {msg}")

    def connect_vehicle(self, endpoint, src_sysid):
        v = connect(endpoint, wait_ready=True, baud=57600, source_system=src_sysid)
        v.wait_ready('autopilot_version', timeout=15)
        return v

    def ensure_guided(self, v):
        if v.mode.name != "GUIDED":
            v.mode = VehicleMode("GUIDED")
            t = time.time()
            while v.mode.name != "GUIDED":
                if time.time() - t > 10:
                    raise RuntimeError("Failed to enter GUIDED")
                time.sleep(0.2)

    def get_heading(self, v):
        if hasattr(v, "heading") and v.heading is not None:
            return float(v.heading)
        try:
            return (math.degrees(v.attitude.yaw) + 360.0) % 360.0
        except Exception:
            return 0.0

    def get_speed(self, v):
        try:
            if v.groundspeed is not None:
                return float(v.groundspeed)
        except Exception:
            pass
        try:
            vx, vy, vz = v.velocity
            if vx is not None:
                return math.hypot(vx, vy)
        except Exception:
            pass
        return 0.0

    def maybe_send_speed(self, fs: FollowerState, link, v_cmd):
        now = time.time()
        if now - fs.last_speed_time < self.cfg.speed_cmd_period:
            return
        if math.isfinite(fs.last_speed_cmd) and abs(v_cmd - fs.last_speed_cmd) < self.cfg.speed_epsilon:
            return
        cmd_change_speed(link, v_cmd)
        fs.last_speed_cmd = v_cmd
        fs.last_speed_time = now

    def run(self):
        try:
            self.log(f"Connecting Leader: {self.cfg.leader}")
            L = self.connect_vehicle(self.cfg.leader, 240)
            self.state.leader_link = L

            self.state.followers = []
            src = 250
            active = []
            for fcfg in self.cfg.followers:
                if not fcfg.endpoint.strip():
                    continue
                self.log(f"Connecting Follower @ {fcfg.endpoint}")
                F = self.connect_vehicle(fcfg.endpoint.strip(), src)
                src += 1
                self.ensure_guided(F)
                fs = FollowerState(link=F)
                active.append((fcfg, fs))
                self.state.followers.append(fs)
            self.log(f"Active followers: {len(active)}")

            period = 1.0 / max(1.0, self.cfg.rate_hz)
            self.state.running = True

            last_lat = last_lon = None

            while not self.state.stop_evt.is_set():
                loc = L.location.global_frame
                if not loc or loc.lat is None or loc.lon is None:
                    time.sleep(period)
                    continue
                lat, lon = float(loc.lat), float(loc.lon)
                hdg = self.get_heading(L)
                vL  = self.get_speed(L)

                # Breadcrumb (append if moved)
                if (last_lat is None) or (haversine_m(last_lat, last_lon, lat, lon) > 0.15):
                    self.breadcrumb.append((lat, lon))
                    last_lat, last_lon = lat, lon

                # Stop latch: freeze target once leader is effectively stopped
                STOP_V = 0.3    # m/s threshold
                STOP_T = 1.0    # seconds below threshold to engage
                STOP_RADIUS = 0.8  # m radius to "park" follower and switch to HOLD
                now = time.time()
                if vL < STOP_V:
                    if not self.stop_latch:
                        if self.stop_since == 0.0:
                            self.stop_since = now
                        elif (now - self.stop_since) >= STOP_T:
                            self.stop_latch = True
                            self.stop_target = None
                    stopped_ignore_yaw = True
                else:
                    self.stop_latch = False
                    self.stop_since = 0.0
                    self.stop_target = None
                    # unpark followers if they were parked
                    for _, fs in active:
                        fs.parked = False
                    stopped_ignore_yaw = False

                # Optional hold until AUTO
                if self.cfg.hold_until_auto and (L.mode.name.upper() != "AUTO"):
                    time.sleep(period)
                    continue

                # Speed target
                if self.cfg.match_speed:
                    if self.state.filt_speed is None:
                        self.state.filt_speed = vL
                    else:
                        a = max(0.0, min(1.0, self.cfg.smooth_alpha))
                        self.state.filt_speed = a * vL + (1.0 - a) * self.state.filt_speed
                    v_cmd = min(max(self.state.filt_speed, self.cfg.min_speed), self.cfg.max_speed)
                else:
                    v_cmd = None

                # Lead distance (suppressed when stopped)
                lead_fwd = 0.0
                if self.cfg.enable_lead and not self.stop_latch:
                    lead_fwd = min(max(vL * max(0.0, self.cfg.lead_time_s), 0.0), max(0.0, self.cfg.lead_max_m))

                for fcfg, fs in active:
                    if fs.parked:
                        # Already parked in HOLD near the final offset; just keep zero speed occasionally.
                        if v_cmd is not None:
                            self.maybe_send_speed(fs, fs.link, 0.0)
                        continue

                    if self.cfg.fixed_ned:
                        right = fcfg.lateral_m if fcfg.side == "Right" else -fcfg.lateral_m
                        north, east = rel_offset_from_heading(hdg, right, fcfg.back_m, fwd_extra_m=0.0 if self.stop_latch else lead_fwd)
                        tlat, tlon = add_ne_offset_latlon(lat, lon, north, east)
                        tgt_yaw = None if stopped_ignore_yaw else hdg
                    else:
                        if len(self.breadcrumb) < 2:
                            continue
                        back_distance = fcfg.back_m
                        if self.stop_latch and self.stop_target is not None:
                            ref_lat, ref_lon, ref_hdg = self.stop_target
                        else:
                            ref = point_back_along_path(list(self.breadcrumb), max(0.0, back_distance))
                            if not ref:
                                continue
                            ref_lat, ref_lon, ref_hdg = ref
                            if self.stop_latch and self.stop_target is None:
                                self.stop_target = (ref_lat, ref_lon, ref_hdg)
                        right = fcfg.lateral_m if fcfg.side == "Right" else -fcfg.lateral_m
                        north, east = rel_offset_from_heading(ref_hdg, right, 0.0, fwd_extra_m=0.0)
                        tlat, tlon = add_ne_offset_latlon(ref_lat, ref_lon, north, east)
                        tgt_yaw = None if stopped_ignore_yaw else ref_hdg

                    if tlat is None or tlon is None:
                        continue

                    # Distance to target
                    dist = haversine_m(fs.link.location.global_frame.lat, fs.link.location.global_frame.lon, tlat, tlon) \
                           if (fs.link and fs.link.location and fs.link.location.global_frame and
                               fs.link.location.global_frame.lat is not None and fs.link.location.global_frame.lon is not None) else None

                    if self.stop_latch and dist is not None and dist <= STOP_RADIUS:
                        # Park: HOLD + zero speed; mark parked to suppress further guidance chatter
                        try:
                            fs.link.mode = VehicleMode("HOLD")
                        except Exception:
                            pass
                        try:
                            cmd_change_speed(fs.link, 0.0)
                        except Exception:
                            pass
                        fs.parked = True
                        continue

                    # Command follower toward target
                    send_pos_target_global_int(fs.link, tlat, tlon, 0.0, yaw_deg=tgt_yaw)
                    if v_cmd is not None:
                        # When approaching parked state, command a gentle low speed
                        desired = 0.0 if self.stop_latch else v_cmd
                        self.maybe_send_speed(fs, fs.link, desired)

                time.sleep(period)

        except Exception as e:
            self.log(f"ERROR: {e}")
        finally:
            # Do NOT auto-close links here; keep behavior consistent with existing Stop logic
            self.state.running = False
            self.log("Stopped.")

# ---- GUI ----

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Leader–Follower GUI – Path-locked + Improved Stop + PANIC DISARM")
        self.geometry("1400x900")
        self.resizable(True, True)
        self.groups: Dict[str, Tuple[GroupConfig, GroupState]] = {}
        self.log_q = queue.Queue()
        self._build_ui()
        self.after(100, self._drain_logs)
        self.after(600, self._hb_monitor)

    def _build_ui(self):
        head = ttk.Frame(self); head.pack(fill=tk.X, padx=8, pady=6)
        ttk.Label(head, text="Global Heartbeat:").pack(side=tk.LEFT)
        self.hb_label = ttk.Label(head, text="HB: UNKNOWN")
        self.hb_label.pack(side=tk.LEFT, padx=8)

        frm = ttk.LabelFrame(self, text="Group Config"); frm.pack(fill=tk.X, padx=8, pady=6)
        labels = ["Name", "Leader (MAVLink)",
                  "F1 (MAVLink)", "F1 Side", "F1 Lat(m)", "F1 Back(m)",
                  "F2 (MAVLink)", "F2 Side", "F2 Lat(m)", "F2 Back(m)",
                  "F3 (MAVLink)", "F3 Side", "F3 Lat(m)", "F3 Back(m)",
                  "F4 (MAVLink)", "F4 Side", "F4 Lat(m)", "F4 Back(m)",
                  "Match Speed", "Min", "Max", "α", "CmdT(s)",
                  "Lead", "LeadT(s)", "LeadMax(m)", "Rate(Hz)"]
        for c, t in enumerate(labels):
            ttk.Label(frm, text=t).grid(row=0, column=c, sticky="w", padx=3, pady=4)

        self.name_var = tk.StringVar(value="Group1")
        self.leader_var = tk.StringVar(value="udpin:0.0.0.0:15021")
        self.f_vars = []
        for _ in range(4):
            self.f_vars.append((tk.StringVar(value=""), tk.StringVar(value="Right"),
                                tk.StringVar(value="5"), tk.StringVar(value="10")))

        self.fixedned_var = tk.BooleanVar(value=False)
        self.hold_until_auto_var = tk.BooleanVar(value=False)
        self.match_speed_var = tk.BooleanVar(value=True)
        self.min_speed_var = tk.StringVar(value="0.5")
        self.max_speed_var = tk.StringVar(value="3.0")
        self.smooth_alpha_var = tk.StringVar(value="0.4")
        self.cmd_period_var = tk.StringVar(value="0.5")
        self.enable_lead_var = tk.BooleanVar(value=True)
        self.lead_time_var = tk.StringVar(value="1.5")
        self.lead_max_var = tk.StringVar(value="8.0")
        self.rate_var = tk.StringVar(value="8")

        r = 1; col = 0
        ttk.Entry(frm, textvariable=self.name_var, width=12).grid(row=r, column=col, padx=3, pady=4, sticky="we"); col += 1
        ttk.Entry(frm, textvariable=self.leader_var, width=24).grid(row=r, column=col, padx=3, pady=4, sticky="we"); col += 1
        for i in range(4):
            ep, side, lat, back = self.f_vars[i]
            ttk.Entry(frm, textvariable=ep, width=24).grid(row=r, column=col, padx=3, pady=4); col += 1
            cb = ttk.Combobox(frm, textvariable=side, values=["Left", "Right"], width=7, state="readonly")
            cb.grid(row=r, column=col, padx=3, pady=4); col += 1
            ttk.Entry(frm, textvariable=lat, width=7).grid(row=r, column=col, padx=3, pady=4); col += 1
            ttk.Entry(frm, textvariable=back, width=7).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Checkbutton(frm, variable=self.match_speed_var).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.min_speed_var, width=6).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.max_speed_var, width=6).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.smooth_alpha_var, width=6).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.cmd_period_var, width=7).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Checkbutton(frm, variable=self.enable_lead_var).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.lead_time_var, width=7).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.lead_max_var, width=8).grid(row=r, column=col, padx=3, pady=4); col += 1
        ttk.Entry(frm, textvariable=self.rate_var, width=6).grid(row=r, column=col, padx=3, pady=4); col += 1

        r = 2
        ttk.Button(frm, text="Add/Update Group", command=self.add_update_group).grid(row=r, column=0, padx=4, pady=6, sticky="we", columnspan=2)
        ttk.Button(frm, text="Remove Group", command=self.remove_selected).grid(row=r, column=2, padx=4, pady=6, sticky="we")
        ttk.Button(frm, text="Start Group", command=self.start_selected).grid(row=r, column=3, padx=4, pady=6, sticky="we")
        ttk.Button(frm, text="Stop Group", command=self.stop_selected).grid(row=r, column=4, padx=4, pady=6, sticky="we")
        ttk.Button(frm, text="Start All", command=self.start_all).grid(row=r, column=5, padx=4, pady=6, sticky="we")
        ttk.Button(frm, text="Stop All", command=self.stop_all).grid(row=r, column=6, padx=4, pady=6, sticky="we")
        ttk.Button(frm, text="PANIC DISARM (HOLD+DISARM)", command=self.panic_disarm_all).grid(row=r, column=7, padx=4, pady=6, sticky="we")

        # Behavior panel
        behavior = ttk.LabelFrame(self, text="Behavior")
        behavior.pack(fill=tk.X, padx=8, pady=6)
        ttk.Checkbutton(behavior, text="Fixed NED (offsets in map frame)", variable=self.fixedned_var).grid(row=0, column=0, padx=6, pady=6, sticky="w")
        ttk.Checkbutton(behavior, text="Hold until Leader AUTO", variable=self.hold_until_auto_var).grid(row=0, column=1, padx=6, pady=6, sticky="w")

        # Groups table
        tbl_frm = ttk.LabelFrame(self, text="Groups"); tbl_frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)
        cols = ("name","leader","followers","fixedned","holdauto","matchspd","minv","maxv","alpha","period","lead","leadT","leadMax","rate","status")
        self.tree = ttk.Treeview(tbl_frm, columns=cols, show="headings", height=12)
        heads = ["Name","Leader","Followers (active)","FixedNED","HoldUntilAUTO","MatchSpd","Min","Max","α","Tcmd","Lead","Tlead","LeadMax","Rate","Status"]
        widths= [120,220,160,80,120,90,60,60,50,60,60,60,80,60,220]
        for i,(h,w) in enumerate(zip(heads,widths)):
            self.tree.heading(cols[i], text=h)
            self.tree.column(cols[i], width=w, anchor="w")
        self.tree.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        vsb = ttk.Scrollbar(tbl_frm, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscroll=vsb.set); vsb.pack(side=tk.RIGHT, fill=tk.Y)

        # Log
        log_frm = ttk.LabelFrame(self, text="Log"); log_frm.pack(fill=tk.BOTH, expand=True, padx=8, pady=6)
        self.log_txt = tk.Text(log_frm, height=10, wrap="word")
        self.log_txt.pack(fill=tk.BOTH, expand=True)
        ttk.Label(self, text="Tip: Use udpin:0.0.0.0:<port> for Mission Planner mirror ports.", foreground="#555").pack(fill=tk.X, padx=8, pady=4)

        self.tree.bind("<<TreeviewSelect>>", self.on_select_row)

    # ---- log drain ----
    def _drain_logs(self):
        try:
            while True:
                line = self.log_q.get_nowait()
                self.log_txt.insert(tk.END, line + "\n")
                self.log_txt.see(tk.END)
        except queue.Empty:
            pass
        self.after(150, self._drain_logs)

    # ---- heartbeat monitor ----
    def _hb_monitor(self):
        FRESH = 3.0
        any_running = False; all_ok = True; any_ok = True
        for name, (cfg, state) in self.groups.items():
            if state.running:
                any_running = True; l_ok = False; f_ok = 0
                try:
                    if state.leader_link and state.leader_link.last_heartbeat < FRESH: l_ok = True
                    for fs in state.followers:
                        if fs.link and fs.link.last_heartbeat < FRESH: f_ok += 1
                except Exception:
                    pass
                self.tree.set(name, "status", f"running [L:{'✔' if l_ok else '✖'} F:{f_ok}/{len(state.followers)}]")
                all_ok = all_ok and l_ok and (f_ok == len(state.followers))
                any_ok = any_ok and (l_ok or f_ok > 0)
            else:
                self.tree.set(name, "status", "stopped")
        if not any_running:
            self.hb_label.config(text="HB: IDLE", background="#ddd")
        else:
            if all_ok:
                self.hb_label.config(text="HB: ALL OK", background="#8fdb6b")
            elif any_ok:
                self.hb_label.config(text="HB: PARTIAL", background="#ffcc66")
            else:
                self.hb_label.config(text="HB: LOST", background="#ff6b6b")
        self.after(600, self._hb_monitor)

    # ---- collect cfg ----
    def _collect_group_from_inputs(self) -> Optional[GroupConfig]:
        try:
            followers = []
            for ep, side, lat, back in self.f_vars:
                followers.append(FollowerCfg(
                    endpoint=ep.get().strip(),
                    side=side.get().strip() or "Right",
                    lateral_m=float(lat.get() or 0.0),
                    back_m=float(back.get() or 0.0),
                ))
            cfg = GroupConfig(
                name=self.name_var.get().strip(),
                leader=self.leader_var.get().strip(),
                followers=followers,
                fixed_ned=bool(self.fixedned_var.get()),
                hold_until_auto=bool(self.hold_until_auto_var.get()),
                match_speed=bool(self.match_speed_var.get()),
                min_speed=float(self.min_speed_var.get() or 0.5),
                max_speed=float(self.max_speed_var.get() or 3.0),
                smooth_alpha=float(self.smooth_alpha_var.get() or 0.4),
                speed_cmd_period=float(self.cmd_period_var.get() or 0.5),
                enable_lead=bool(self.enable_lead_var.get()),
                lead_time_s=float(self.lead_time_var.get() or 1.5),
                lead_max_m=float(self.lead_max_var.get() or 8.0),
                rate_hz=float(self.rate_var.get() or 8.0),
            )
        except ValueError:
            messagebox.showerror("Invalid Input", "Numeric fields are invalid.")
            return None
        if not cfg.name:
            messagebox.showerror("Invalid Input", "Please provide a group name.")
            return None
        cfg.smooth_alpha = max(0.0, min(1.0, cfg.smooth_alpha))
        cfg.min_speed = max(0.0, cfg.min_speed)
        cfg.max_speed = max(cfg.min_speed, cfg.max_speed)
        cfg.speed_cmd_period = max(0.1, cfg.speed_cmd_period)
        cfg.lead_time_s = max(0.0, cfg.lead_time_s)
        cfg.lead_max_m = max(0.0, cfg.lead_max_m)
        cfg.rate_hz = max(1.0, cfg.rate_hz)
        return cfg

    # ---- group ops ----
    def add_update_group(self):
        cfg = self._collect_group_from_inputs()
        if not cfg:
            return
        state = self.groups.get(cfg.name, (cfg, GroupState()))[1]
        self.groups[cfg.name] = (cfg, state)
        active = sum(1 for f in cfg.followers if f.endpoint.strip())
        values = (cfg.name, cfg.leader, active, cfg.fixed_ned, cfg.hold_until_auto, cfg.match_speed, cfg.min_speed,
                  cfg.max_speed, cfg.smooth_alpha, cfg.speed_cmd_period, cfg.enable_lead, cfg.lead_time_s, cfg.lead_max_m, cfg.rate_hz,
                  "running" if state.running else "stopped")
        if self._find_row(cfg.name): self.tree.item(cfg.name, values=values)
        else: self.tree.insert("", tk.END, iid=cfg.name, values=values)
        self.log_q.put(f"[{cfg.name}] Added/Updated. Followers active: {active}. FixedNED={cfg.fixed_ned} HoldUntilAUTO={cfg.hold_until_auto}")

    def remove_selected(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("Remove Group", "Select a group first.")
            return
        name = sel[0]
        self._stop_group(name)
        if name in self.groups: del self.groups[name]
        try: self.tree.delete(name)
        except Exception: pass
        self.log_q.put(f"[{name}] Removed.")

    def _find_row(self, name):
        try:
            self.tree.item(name); return name
        except Exception:
            return None

    def on_select_row(self, _=None):
        sel = self.tree.selection()
        if not sel: return
        name = sel[0]
        cfg, _state = self.groups.get(name, (None, None))
        if not cfg: return
        self.name_var.set(cfg.name); self.leader_var.set(cfg.leader)
        for i, fcfg in enumerate(cfg.followers + [FollowerCfg()] * (4 - len(cfg.followers))):
            ep, side, lat, back = self.f_vars[i]
            ep.set(fcfg.endpoint); side.set(fcfg.side if fcfg.side in ("Left","Right") else "Right")
            lat.set(str(fcfg.lateral_m)); back.set(str(fcfg.back_m))
        self.fixedned_var.set(cfg.fixed_ned); self.hold_until_auto_var.set(cfg.hold_until_auto)
        self.match_speed_var.set(cfg.match_speed); self.min_speed_var.set(str(cfg.min_speed))
        self.max_speed_var.set(str(cfg.max_speed)); self.smooth_alpha_var.set(str(cfg.smooth_alpha))
        self.cmd_period_var.set(str(cfg.speed_cmd_period)); self.enable_lead_var.set(cfg.enable_lead)
        self.lead_time_var.set(str(cfg.lead_time_s)); self.lead_max_var.set(str(cfg.lead_max_m))
        self.rate_var.set(str(cfg.rate_hz))

    def start_selected(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("Start Group", "Select a group first.")
            return
        self._start_group(sel[0])

    def stop_selected(self):
        sel = self.tree.selection()
        if not sel:
            messagebox.showinfo("Stop Group", "Select a group first.")
            return
        self._stop_group(sel[0])

    def start_all(self):
        for name in list(self.groups.keys()):
            self._start_group(name)

    def stop_all(self):
        for name in list(self.groups.keys()):
            self._stop_group(name)

    def _start_group(self, name):
        cfg, state = self.groups.get(name, (None, None))
        if not cfg:
            messagebox.showerror("Error", f"Group '{name}' not found.")
            return
        if state.running:
            self.log_q.put(f"[{name}] Already running.")
            return
        state.stop_evt.clear()
        worker = LeaderFollowerWorker(cfg, state, self.log_q)
        t = threading.Thread(target=worker.run, daemon=True, name=f"{name}-thread")
        state.thread = t
        t.start()
        self.tree.set(name, "status", "running [L:✖ F:0/?]")
        self.log_q.put(f"[{name}] Starting… (FixedNED={cfg.fixed_ned} HoldUntilAUTO={cfg.hold_until_auto})")

    def _stop_group(self, name):
        cfg, state = self.groups.get(name, (None, None))
        if not cfg:
            return
        if not state.running and not state.thread:
            self.tree.set(name, "status", "stopped")
            return
        self.log_q.put(f"[{name}] Stopping…")
        try:
            state.stop_evt.set()
            if state.thread and state.thread.is_alive():
                state.thread.join(timeout=3.0)
        except Exception:
            pass
        finally:
            state.thread = None
            self.tree.set(name, "status", "stopped")

    # ---- PANIC DISARM (do NOT stop groups; no confirmation) ----
    def _cmd_panic_stop_disarm(self, veh):
        try:
            # HOLD immediately
            try:
                veh.mode = VehicleMode("HOLD")
            except Exception:
                pass
            # Zero speed
            try:
                veh.message_factory.command_long_send(
                    veh._master.target_system, veh._master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
                    1, 0.0, -1, 0, 0, 0, 0
                )
            except Exception:
                pass
            # Forced DISARM (param2 = 21196)
            veh.message_factory.command_long_send(
                veh._master.target_system,
                veh._master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 21196, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self.log_q.put(f"[PANIC] Error: {e}")

    def panic_disarm_all(self):
        count = 0
        for name, (cfg, state) in self.groups.items():
            if state.leader_link:
                self._cmd_panic_stop_disarm(state.leader_link); count += 1
            for fs in state.followers:
                if fs.link:
                    self._cmd_panic_stop_disarm(fs.link); count += 1
        self.log_q.put(f"[PANIC] Sent HOLD+DISARM to ~{count} links. Verify in GCS.")

    # ---- Table/map helpers ----
    def _find_row(self, name):
        try:
            self.tree.item(name); return name
        except Exception:
            return None

if __name__ == "__main__":
    App().mainloop()
