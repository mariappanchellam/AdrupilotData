#!/usr/bin/env python3
"""
survey_gcs.py  –  Auto Survey Ground Control Interface
======================================================
Connects to ArduPilot/SITL via MAVLink UDP and provides:
  • AUTO SURVEY button  (sets SURV_ENABLE=1)
  • STOP SURVEY button  (sets SURV_ENABLE=0)
  • Live parameter editor for all SURV_* settings
  • Live vehicle status (mode, armed, altitude, GPS)
  • Console showing STATUSTEXT messages from the drone

Usage:
  pip3 install pymavlink
  python3 survey_gcs.py

For SITL:   host=127.0.0.1  port=14550
For real FC via WiFi:  enter the IP of the telemetry bridge
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import queue
from pymavlink import mavutil, mavlink

# ── parameter definitions ────────────────────────────────────────
# (param_name, label, default, min, max, decimal_places)
SURVEY_PARAMS = [
    ("SURV_LAT",      "Centre Latitude  (°)",   12.7478,  -90.0,  90.0,  4),
    ("SURV_LON",      "Centre Longitude (°)",   79.9474, -180.0, 180.0,  4),
    ("SURV_ALT",      "Altitude  (m AGL)",       50.0,    10.0,  150.0,  0),
    ("SURV_WIDTH",    "Width  E-W  (m)",        200.0,    50.0, 2000.0,  0),
    ("SURV_LEN",      "Length  N-S  (m)",       300.0,    50.0, 2000.0,  0),
    ("SURV_SPACING",  "Lane Spacing  (m)",       30.0,     5.0,  200.0,  0),
    ("SURV_INTERVAL", "Repeat Interval  (min)",  10.0,     1.0,  240.0,  0),
]

COPTER_MODES = {
    0: "STABILIZE", 2: "ALT_HOLD", 3: "AUTO",
    4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND",
}


class SurveyGCS:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Auto Survey GCS  |  ArduPilot")
        self.root.resizable(False, False)

        self.mav = None
        self.connected = False
        self.param_vars: dict[str, tk.StringVar] = {}
        self.msg_queue: queue.Queue = queue.Queue()

        self._build_ui()
        self._start_rx_thread()
        self._poll_ui()

    # ── UI construction ──────────────────────────────────────────
    def _build_ui(self):
        pad = {"padx": 8, "pady": 4}

        # ── connection row ────────────────────────────────────────
        cf = ttk.LabelFrame(self.root, text="Connection")
        cf.pack(fill="x", **pad)

        ttk.Label(cf, text="Host:").grid(row=0, column=0, sticky="w", padx=4)
        self.host_var = tk.StringVar(value="127.0.0.1")
        ttk.Entry(cf, textvariable=self.host_var, width=14).grid(row=0, column=1, padx=2)

        ttk.Label(cf, text="UDP Port:").grid(row=0, column=2, sticky="w", padx=4)
        self.port_var = tk.StringVar(value="14550")
        ttk.Entry(cf, textvariable=self.port_var, width=7).grid(row=0, column=3, padx=2)

        self.conn_btn = ttk.Button(cf, text="Connect", command=self._toggle_connect)
        self.conn_btn.grid(row=0, column=4, padx=6)

        self.conn_lbl = tk.Label(cf, text="● Disconnected", fg="red", font=("Arial", 9, "bold"))
        self.conn_lbl.grid(row=0, column=5, padx=6)

        # ── vehicle status ────────────────────────────────────────
        sf = ttk.LabelFrame(self.root, text="Vehicle Status")
        sf.pack(fill="x", **pad)

        self.mode_lbl  = ttk.Label(sf, text="Mode: —", width=18)
        self.armed_lbl = ttk.Label(sf, text="Armed: —", width=12)
        self.alt_lbl   = ttk.Label(sf, text="Alt: —", width=12)
        self.gps_lbl   = ttk.Label(sf, text="GPS: —", width=16)
        for i, w in enumerate([self.mode_lbl, self.armed_lbl, self.alt_lbl, self.gps_lbl]):
            w.grid(row=0, column=i, padx=8, pady=2)

        # ── survey parameters ─────────────────────────────────────
        pf = ttk.LabelFrame(self.root, text="Survey Parameters")
        pf.pack(fill="x", **pad)

        for row, (pname, label, default, lo, hi, dp) in enumerate(SURVEY_PARAMS):
            ttk.Label(pf, text=label + ":", anchor="w", width=24).grid(
                row=row, column=0, sticky="w", padx=6, pady=2)
            var = tk.StringVar(value=str(default))
            self.param_vars[pname] = var
            ttk.Entry(pf, textvariable=var, width=12).grid(row=row, column=1, padx=4, pady=2)
            ttk.Label(pf, text=f"[{lo} – {hi}]", foreground="gray").grid(
                row=row, column=2, sticky="w", padx=2)

        btn_row = ttk.Frame(pf)
        btn_row.grid(row=len(SURVEY_PARAMS), column=0, columnspan=3, pady=6)
        ttk.Button(btn_row, text="⬆  Upload to Vehicle",
                   command=self._upload_params).pack(side="left", padx=6)
        ttk.Button(btn_row, text="⬇  Read from Vehicle",
                   command=self._read_params).pack(side="left", padx=6)

        # ── big action buttons ────────────────────────────────────
        af = ttk.Frame(self.root)
        af.pack(pady=10)

        self.start_btn = tk.Button(
            af, text="▶   AUTO SURVEY",
            font=("Arial", 14, "bold"),
            bg="#1a7f37", fg="white",
            activebackground="#145f29", activeforeground="white",
            width=18, height=2,
            command=self._start_survey,
            state="disabled"
        )
        self.start_btn.grid(row=0, column=0, padx=10)

        self.stop_btn = tk.Button(
            af, text="■   STOP SURVEY",
            font=("Arial", 14, "bold"),
            bg="#c0392b", fg="white",
            activebackground="#922b21", activeforeground="white",
            width=18, height=2,
            command=self._stop_survey,
            state="disabled"
        )
        self.stop_btn.grid(row=0, column=1, padx=10)

        # ── console ───────────────────────────────────────────────
        cf2 = ttk.LabelFrame(self.root, text="Vehicle Messages")
        cf2.pack(fill="both", expand=True, **pad)

        self.console = scrolledtext.ScrolledText(
            cf2, height=8, state="disabled",
            font=("Monospace", 8), bg="#1e1e1e", fg="#d4d4d4"
        )
        self.console.pack(fill="both", expand=True)

    # ── connection ────────────────────────────────────────────────
    def _toggle_connect(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        host = self.host_var.get().strip()
        port = self.port_var.get().strip()
        try:
            self.mav = mavutil.mavlink_connection(f"udp:{host}:{port}")
            self._log_console(f"Connecting to udp:{host}:{port} ...")
            self.mav.wait_heartbeat(timeout=8)
            self.connected = True
            self.conn_lbl.config(text="● Connected", fg="green")
            self.conn_btn.config(text="Disconnect")
            self.start_btn.config(state="normal")
            self._log_console("Heartbeat received — vehicle online")
            self._read_params()
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.mav = None

    def _disconnect(self):
        self.connected = False
        if self.mav:
            self.mav.close()
            self.mav = None
        self.conn_lbl.config(text="● Disconnected", fg="red")
        self.conn_btn.config(text="Connect")
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self._log_console("Disconnected")

    # ── parameter handling ────────────────────────────────────────
    def _upload_params(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to vehicle first")
            return

        def _do():
            for pname, var in self.param_vars.items():
                try:
                    val = float(var.get())
                except ValueError:
                    self.msg_queue.put(("log", f"Bad value for {pname}: {var.get()}"))
                    continue
                self.mav.mav.param_set_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    pname.encode(),
                    val,
                    mavlink.MAV_PARAM_TYPE_REAL32
                )
                time.sleep(0.08)
            self.msg_queue.put(("log", "Parameters uploaded"))

        threading.Thread(target=_do, daemon=True).start()

    def _read_params(self):
        if not self.connected:
            return

        def _do():
            for pname in self.param_vars:
                self.mav.mav.param_request_read_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    pname.encode(),
                    -1
                )
                msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
                if msg:
                    pid = msg.param_id.strip('\x00')
                    if pid == pname and pname in self.param_vars:
                        # find decimal places
                        dp = next((d for n, _, _, _, _, d in SURVEY_PARAMS if n == pname), 0)
                        fmt = f"{{:.{dp}f}}"
                        self.msg_queue.put(("param", pname, fmt.format(msg.param_value)))
            self.msg_queue.put(("log", "Parameters read from vehicle"))

        threading.Thread(target=_do, daemon=True).start()

    # ── survey control ────────────────────────────────────────────
    def _start_survey(self):
        if not self.connected:
            return
        self._upload_params()
        time.sleep(0.6)
        self._set_enable(1)
        self.start_btn.config(state="disabled")
        self.stop_btn.config(state="normal")
        self._log_console("AUTO SURVEY started")

    def _stop_survey(self):
        if not self.connected:
            return
        self._set_enable(0)
        self.start_btn.config(state="normal")
        self.stop_btn.config(state="disabled")
        self._log_console("Survey STOPPED")

    def _set_enable(self, val: float):
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            b"SURV_ENABLE",
            float(val),
            mavlink.MAV_PARAM_TYPE_REAL32
        )

    # ── background MAVLink receiver ───────────────────────────────
    def _start_rx_thread(self):
        def _loop():
            while True:
                if not self.connected or not self.mav:
                    time.sleep(0.5)
                    continue
                try:
                    msg = self.mav.recv_match(
                        type=["HEARTBEAT", "STATUSTEXT",
                              "GLOBAL_POSITION_INT", "GPS_RAW_INT"],
                        blocking=True, timeout=1
                    )
                    if msg:
                        self.msg_queue.put(("mavlink", msg))
                except Exception:
                    pass
        threading.Thread(target=_loop, daemon=True).start()

    # ── UI poll (processes msg_queue on main thread) ──────────────
    def _poll_ui(self):
        try:
            while True:
                item = self.msg_queue.get_nowait()
                kind = item[0]

                if kind == "log":
                    self._log_console(item[1])

                elif kind == "param":
                    _, pname, val_str = item
                    if pname in self.param_vars:
                        self.param_vars[pname].set(val_str)

                elif kind == "mavlink":
                    msg = item[1]
                    t = msg.get_type()

                    if t == "HEARTBEAT":
                        armed = bool(msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                        mode  = COPTER_MODES.get(msg.custom_mode,
                                                  f"MODE_{msg.custom_mode}")
                        self.mode_lbl.config(text=f"Mode: {mode}")
                        self.armed_lbl.config(
                            text="Armed: YES" if armed else "Armed: No",
                            foreground="orange" if armed else "black"
                        )

                    elif t == "GLOBAL_POSITION_INT":
                        alt_rel = msg.relative_alt / 1000.0
                        self.alt_lbl.config(text=f"Alt: {alt_rel:.1f} m")

                    elif t == "GPS_RAW_INT":
                        fixes = {0: "None", 1: "No Fix", 2: "2D", 3: "3D", 6: "RTK"}
                        fix = fixes.get(msg.fix_type, str(msg.fix_type))
                        self.gps_lbl.config(text=f"GPS: {fix} / {msg.satellites_visible} sats")

                    elif t == "STATUSTEXT":
                        text = msg.text.strip('\x00').strip()
                        if text:
                            self._log_console(f"[DRONE] {text}")

        except queue.Empty:
            pass

        self.root.after(150, self._poll_ui)

    # ── console helper ────────────────────────────────────────────
    def _log_console(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.console.config(state="normal")
        self.console.insert("end", f"[{ts}] {msg}\n")
        self.console.see("end")
        self.console.config(state="disabled")


# ── entry point ──────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app  = SurveyGCS(root)
    root.mainloop()
