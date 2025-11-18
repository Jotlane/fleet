#!/usr/bin/env python3

# --------------------------------------------------------
# Python 3.10+ compatibility shim for DroneKit/pymavlink
# --------------------------------------------------------
import collections, collections.abc
for _n in ("MutableMapping", "Mapping", "Sequence", "Iterable"):
    if not hasattr(collections, _n):
        setattr(collections, _n, getattr(collections.abc, _n))

import tkinter as tk
from tkinter import ttk, messagebox
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import threading
import time

# --------------------------------------------------------
# Panic logic
# --------------------------------------------------------
def panic_disarm(vehicle):
    try:
        try:
            vehicle.mode = VehicleMode("HOLD")
        except:
            pass

        try:
            vehicle.message_factory.command_long_send(
                vehicle._master.target_system,
                vehicle._master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1, 0.0, -1, 0, 0, 0, 0
            )
        except:
            pass

        vehicle.message_factory.command_long_send(
            vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 21196,
            0, 0, 0, 0, 0
        )

    except Exception as e:
        print("PANIC ERROR:", e)



# --------------------------------------------------------
# Main GUI App
# --------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("5-Drone Panic Controller")
        self.geometry("520x350")

        self.ports = [15021, 15022, 15023, 15024, 15025]
        self.vehicles = [None] * 5
        self.status_labels = []

        frame = ttk.Frame(self)
        frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        ttk.Label(frame, text="Fleet Connections (UDP 15021–15025)", font=("Segoe UI", 12, "bold")).pack(pady=5)

        grid = ttk.Frame(frame)
        grid.pack()

        # Create 5 connect buttons + status labels
        for i, port in enumerate(self.ports):
            row = ttk.Frame(grid)
            row.pack(fill=tk.X, pady=4)

            idx = i
            btn = ttk.Button(row, text=f"Connect Drone {i+1} (UDP {port})",
                             command=lambda k=idx: self.start_connect_thread(k))
            btn.pack(side=tk.LEFT)

            status = ttk.Label(row, text="● Disconnected", foreground="red")
            status.pack(side=tk.LEFT, padx=10)
            self.status_labels.append(status)

        # Panic button
        ttk.Button(frame, text="🚨 PANIC DISARM ALL 5 🚨", command=self.panic_all).pack(fill=tk.X, pady=20)

        # Start periodic status refresh
        self.after(500, self.refresh_status)

    # ----------------------------------------------------
    # Thread-safe connect starter
    # ----------------------------------------------------
    def start_connect_thread(self, index):
        threading.Thread(target=self.connect_worker, args=(index,), daemon=True).start()

    # ----------------------------------------------------
    # Worker thread: connect()
    # ----------------------------------------------------
    def connect_worker(self, index):
        port = self.ports[index]
        endpoint = f"udp:0.0.0.0:{port}"
        print(f"[Drone {index+1}] Connecting to {endpoint}...")

        try:
            v = connect(endpoint, wait_ready=True, baud=115200, heartbeat_timeout=1)
            self.vehicles[index] = v

            # Initialize reconnect & heartbeat tracking
            v._reconnecting = False
            v._lost_triggered = False
            v._hb_prev = None
            v._hb_count = 0

            self.after(0, lambda:
                self.status_labels[index].config(text="● Connected", foreground="green")
            )
            print(f"[Drone {index+1}] Connected")

        except Exception as e:
            print(f"[Drone {index+1}] Connect failed: {e}")
            msg = f"Drone {index+1} ({endpoint}) failed:\n{e}"
            self.after(0, lambda m=msg: messagebox.showwarning("Connection Failed", m))

    # ----------------------------------------------------
    # Auto-Reconnect starter
    # ----------------------------------------------------
    def start_reconnect_thread(self, index):
        v = self.vehicles[index]

        # If vehicle still exists and says it's reconnecting, block
        if v is not None and getattr(v, "_reconnecting", False):
            print(f"[Drone {index+1}] Already reconnecting...")
            return

        print(f"[Drone {index+1}] Starting reconnect thread...")

        # Mark reconnecting state (on a simple dict entry)
        self.vehicles[index] = None
        threading.Thread(target=self.reconnect_worker, args=(index,), daemon=True).start()

    # ----------------------------------------------------
    # Auto-Reconnect worker
    # ----------------------------------------------------
    def reconnect_worker(self, index):
        port = self.ports[index]
        endpoint = f"udp:0.0.0.0:{port}"

        print(f"[Drone {index+1}] Attempting reconnect to {endpoint}...")

        try:
            v = connect(endpoint, wait_ready=True, heartbeat_timeout=1)
            self.vehicles[index] = v

            # Reset tracking after successful reconnect
            v._reconnecting = False
            v._lost_triggered = False
            v._hb_prev = None
            v._hb_count = 0

            self.after(0, lambda:
                self.status_labels[index].config(text="● Reconnected", foreground="green")
            )
            print(f"[Drone {index+1}] Reconnect successful")

        except Exception as e:
            print(f"[Drone {index+1}] Reconnect failed: {e}")
            # Retry after 2 seconds
            self.after(2000, lambda: self.start_reconnect_thread(index))

    # ----------------------------------------------------
    # Heartbeat monitor + auto reconnect
    # ----------------------------------------------------
    def refresh_status(self):
        for i, v in enumerate(self.vehicles):

            # No vehicle currently
            if v is None:
                self.status_labels[i].config(text="● Disconnected", foreground="red")
                continue

            # Attempt to read last heartbeat safely
            try:
                curr_hb = v.last_heartbeat
            except:
                curr_hb = None

            # No heartbeat yet — waiting
            if curr_hb is None or curr_hb == 0:
                v._hb_prev = None
                v._hb_count = 0
                self.status_labels[i].config(text="● Waiting for heartbeat…", foreground="blue")
                continue

            # First heartbeat seen
            if v._hb_prev is None:
                v._hb_prev = curr_hb
                v._hb_count = 1

            # Same heartbeat value again → increment stagnation counter
            elif curr_hb == v._hb_prev:
                v._hb_count += 1

            # Heartbeat changed → reset
            else:
                v._hb_prev = curr_hb
                v._hb_count = 1

            # 5 stagnated heartbeat reads = LOST
            if v._hb_count >= 5:
                self.status_labels[i].config(text="● Lost (Reconnecting...)", foreground="orange")

                if not v._lost_triggered:
                    v._lost_triggered = True
                    # Kill the dead vehicle object to avoid stale flags
                    self.vehicles[i] = None
                    self.start_reconnect_thread(i)

                continue

            # Otherwise considered connected
            self.status_labels[i].config(text="● Connected", foreground="green")

        self.after(500, self.refresh_status)

    # ----------------------------------------------------
    # Panic button – always runs instantly
    # ----------------------------------------------------
    def panic_all(self):
        threading.Thread(target=self.panic_worker, daemon=True).start()

    def panic_worker(self):
        any_connected = any(v is not None for v in self.vehicles)
        if not any_connected:
            self.after(0, lambda:
                messagebox.showerror("No Vehicles", "None of the 5 drones are connected.")
            )
            return

        for v in self.vehicles:
            if v:
                try:
                    panic_disarm(v)
                except Exception as e:
                    print("PANIC ERROR:", e)

        self.after(0, lambda:
            messagebox.showinfo("PANIC", "Forced DISARM sent to all connected drones.")
        )



# --------------------------------------------------------
if __name__ == "__main__":
    App().mainloop()
