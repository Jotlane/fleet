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
        self.title("5‑Drone Panic Controller")
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
            btn = ttk.Button(row, text=f"Connect Drone {i+1} (UDP {port})", command=lambda k=idx: self.connect_single(k))
            btn.pack(side=tk.LEFT)

            status = ttk.Label(row, text="● Disconnected", foreground="red")
            status.pack(side=tk.LEFT, padx=10)
            self.status_labels.append(status)

        # Panic button
        ttk.Button(frame, text="🚨 PANIC DISARM ALL 5 🚨", command=self.panic_all).pack(fill=tk.X, pady=20)

        # Start periodic status refresh
        self.after(500, self.refresh_status)

    # ----------------------------------------------------
    def connect_single(self, index):
        print("connecting single")
        port = self.ports[index]
        endpoint = f"udp:0.0.0.0:{port}"

        try:
            print(f"Connecting to {endpoint}")
            v = connect(endpoint, wait_ready=True, baud=115200, heartbeat_timeout=1)
            self.vehicles[index] = v
            self.status_labels[index].config(text="● Connected", foreground="green")
        except Exception as e:
            print("Connection Failed", f"Drone {index+1} ({endpoint}) failed:\n{e}\nRetrying connection")
            self.status_labels[index].config(text="● Lost", foreground="orange")

    # ----------------------------------------------------
    def refresh_status(self):
        for i, v in enumerate(self.vehicles):
            if v is None:
                self.status_labels[i].config(text="● Disconnected", foreground="red")
            else:
                # Vehicle has heartbeat? If not, consider it disconnected
                try:
                    _ = v.last_heartbeat
                    self.status_labels[i].config(text="● Connected", foreground="green")
                except:
                    self.status_labels[i].config(text="● Lost", foreground="orange")
        self.after(500, self.refresh_status)

    # ----------------------------------------------------
    def panic_all(self):
        any_connected = any(v is not None for v in self.vehicles)
        if not any_connected:
            messagebox.showerror("No Vehicles", "None of the 5 drones are connected.")
            return

        for v in self.vehicles:
            if v:
                panic_disarm(v)

        messagebox.showinfo("PANIC", "Forced DISARM sent to all connected drones.")


# --------------------------------------------------------
if __name__ == "__main__":
    App().mainloop()
