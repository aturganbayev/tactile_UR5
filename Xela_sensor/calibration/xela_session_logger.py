#!/usr/bin/env python3
"""
RUNS ON THIS WORKSTATION (the machine with the CAN-USB adapter / xela_server),
NOT on the DAQ PC.

Continuously logs raw XELA taxel counts with wall-clock timestamps to a CSV,
for later alignment (by timestamp) against a nano17_press_session.py run on
the DAQ PC - see that script's docstring for the split-machine rationale.

This does NOT read the Nano17 or drive the robot - it only reads whatever
xela_server is already broadcasting over its WebSocket.

PREREQUISITES:
  1. can0 is up:            sudo ip link set up can0 type can bitrate 1000000
  2. xela_server running:   cd Xela_sensor && ./xela_server -f xServ.ini --ip 0.0.0.0
  3. Sensor stream triggered if needed: cansend can0 203#07.00

Usage:
    python3 xela_session_logger.py [location_label]

Start this BEFORE starting nano17_press_session.py on the DAQ PC, and leave
it running (Ctrl-C to stop) until that session finishes retracting - a
little extra idle data on either end doesn't hurt, missing the press does.
"""

import csv
import json
import os
import sys
import threading
import time

import websocket

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_THIS_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

XELA_WS_URL = "ws://127.0.0.1:5000"
BASELINE_CONFIRM_SAMPLES = 60   # ~0.5s at ~120Hz, just to prove data is live


class XelaReader:
    """Background thread holding the latest parsed XELA sensor-1 message."""

    def __init__(self, url=XELA_WS_URL, sensor_key="1"):
        self._url = url
        self._sensor_key = sensor_key
        self._lock = threading.Lock()
        self._latest = None    # (recv_time, [ (x,y,z) x16 ])
        self._ws = None
        self._thread = None
        self.error = None

    def _on_message(self, ws, message):
        recv_time = time.time()
        try:
            data = json.loads(message)
            s = data[self._sensor_key]
            vals = [int(v, 16) for v in s["data"].split(",")]
            taxels = [tuple(vals[i:i + 3]) for i in range(0, len(vals), 3)]
        except Exception:
            return
        with self._lock:
            self._latest = (recv_time, taxels)

    def _on_error(self, ws, error):
        self.error = error

    def start(self):
        self._ws = websocket.WebSocketApp(
            self._url, on_message=self._on_message, on_error=self._on_error)
        self._thread = threading.Thread(target=self._ws.run_forever, daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._latest

    def wait_for_data(self, timeout=5.0):
        t0 = time.time()
        while self.latest() is None:
            if self.error is not None:
                raise RuntimeError(f"XELA websocket error: {self.error}")
            if time.time() - t0 > timeout:
                raise TimeoutError(
                    "No data from XELA server - is xela_server running and "
                    "has the sensor been triggered (cansend can0 203#07.00)?")
            time.sleep(0.05)

    def stop(self):
        if self._ws is not None:
            self._ws.close()


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else input(
        "Location label (should match the DAQ PC session's label): ").strip()
    if not label:
        print("A location label is required.")
        return
    out_path = os.path.join(DATA_DIR, f"{label}_xela.csv")

    print("Connecting to XELA server ...")
    xela = XelaReader()
    xela.start()
    xela.wait_for_data()
    print("XELA OK - data is live.")

    input("\nConfirm the sensor is NOT currently touched, then press Enter "
          "to start logging (Ctrl-C to stop once the DAQ PC press session "
          "has finished) ...")

    fieldnames = ["t"]
    for i in range(16):
        fieldnames += [f"x{i}", f"y{i}", f"z{i}"]

    rows_written = 0
    last_taxels = None
    print(f"Logging to {out_path} - press Ctrl-C to stop.")
    try:
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(fieldnames)
            while True:
                latest = xela.latest()
                if latest is not None:
                    recv_time, taxels = latest
                    if taxels != last_taxels:
                        flat = [v for xyz in taxels for v in xyz]
                        w.writerow([recv_time] + flat)
                        rows_written += 1
                        last_taxels = taxels
                time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        xela.stop()
        print(f"\nStopped. Logged {rows_written} row(s) -> {out_path}")


if __name__ == "__main__":
    main()
