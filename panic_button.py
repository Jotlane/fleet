#!/usr/bin/env python3

# --------------------------------------------------------
# Python 3.10+ compatibility shim for DroneKit/pymavlink
# --------------------------------------------------------
import collections, collections.abc
for _n in ("MutableMapping", "Mapping", "Sequence", "Iterable"):
    if not hasattr(collections, _n):
        setattr(collections, _n, getattr(collections.abc, _n))

import threading
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# --------------------------------------------------------
# GPIO Setup
# --------------------------------------------------------
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

LED_PINS = [5, 6, 13, 19, 26]     # one LED per drone
PANIC_BUTTON_PIN = 21            # input panic button

for pin in LED_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.setup(PANIC_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # active LOW

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
# Main controller (no GUI)
# --------------------------------------------------------
class PanicController:
    def __init__(self):
        self.ports = [15021, 15022, 15023, 15024, 15025]
        self.vehicles = [None] * 5
        self.lock = threading.Lock()

    # -----------------------------
    # Connect worker
    # -----------------------------
    def connect_worker(self, index):
        port = self.ports[index]
        endpoint = f"udp:0.0.0.0:{port}"
        print(f"[Drone {index+1}] Connecting to {endpoint}...")

        try:
            v = connect(endpoint, wait_ready=True, baud=115200, heartbeat_timeout=1)

            # Setup heartbeat tracking
            v._reconnecting = False
            v._lost_triggered = False
            v._hb_prev = None
            v._hb_count = 0

            with self.lock:
                self.vehicles[index] = v

            print(f"[Drone {index+1}] Connected")
            GPIO.output(LED_PINS[index], GPIO.HIGH)

        except Exception as e:
            print(f"[Drone {index+1}] Connection failed: {e}")
            GPIO.output(LED_PINS[index], GPIO.LOW)

    # -----------------------------
    # Auto reconnect
    # -----------------------------
    def reconnect_worker(self, index):
        port = self.ports[index]
        endpoint = f"udp:0.0.0.0:{port}"
        print(f"[Drone {index+1}] Attempting reconnect to {endpoint}...")

        try:
            v = connect(endpoint, wait_ready=True, heartbeat_timeout=1)

            v._reconnecting = False
            v._lost_triggered = False
            v._hb_prev = None
            v._hb_count = 0

            with self.lock:
                self.vehicles[index] = v

            print(f"[Drone {index+1}] Reconnected")
            GPIO.output(LED_PINS[index], GPIO.HIGH)

        except Exception as e:
            print(f"[Drone {index+1}] Reconnect failed: {e}")
            time.sleep(2)
            self.start_reconnect(index)

    def start_reconnect(self, index):
        threading.Thread(target=self.reconnect_worker, args=(index,), daemon=True).start()

    # -----------------------------
    # Heartbeat monitor
    # -----------------------------
    def monitor_heartbeats(self):
        while True:
            for i, v in enumerate(self.vehicles):

                if v is None:
                    GPIO.output(LED_PINS[i], GPIO.LOW)
                    continue

                try:
                    curr_hb = v.last_heartbeat
                except:
                    curr_hb = None

                if curr_hb is None or curr_hb == 0:
                    v._hb_prev = None
                    v._hb_count = 0
                    continue

                if v._hb_prev is None:
                    v._hb_prev = curr_hb
                    v._hb_count = 1

                elif curr_hb == v._hb_prev:
                    v._hb_count += 1
                else:
                    v._hb_prev = curr_hb
                    v._hb_count = 1

                if v._hb_count >= 5:  # LOST heartbeat
                    print(f"[Drone {i+1}] Heartbeat lost → Reconnecting...")
                    GPIO.output(LED_PINS[i], GPIO.LOW)

                    if not v._lost_triggered:
                        v._lost_triggered = True
                        self.vehicles[i] = None
                        self.start_reconnect(i)
                else:
                    GPIO.output(LED_PINS[i], GPIO.HIGH)

            time.sleep(0.5)

    # -----------------------------
    # Panic button input
    # -----------------------------
    def panic_button_loop(self):
        while True:
            if GPIO.input(PANIC_BUTTON_PIN) == GPIO.LOW:   # button pressed
                print("🚨 PANIC BUTTON PRESSED — DISARMING ALL DRONES")
                self.panic_all()
                time.sleep(1)  # debounce
            time.sleep(0.05)

    def panic_all(self):
        for v in self.vehicles:
            if v:
                try:
                    panic_disarm(v)
                except Exception as e:
                    print("PANIC ERROR:", e)

    # -----------------------------
    # Main
    # -----------------------------
    def start(self):
        print("Starting Drone Panic Controller (no GUI)")

        # Start connection threads
        for i in range(5):
            threading.Thread(target=self.connect_worker, args=(i,), daemon=True).start()

        # Start heartbeat monitor
        threading.Thread(target=self.monitor_heartbeats, daemon=True).start()

        # Start panic button monitor
        threading.Thread(target=self.panic_button_loop, daemon=True).start()

        # Keep main thread alive forever
        while True:
            time.sleep(1)


# --------------------------------------------------------
# Run
# --------------------------------------------------------
if __name__ == "__main__":
    try:
        PanicController().start()
    finally:
        GPIO.cleanup()
