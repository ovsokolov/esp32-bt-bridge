#!/usr/bin/env python3
import os
import queue
import re
import subprocess
import threading
import time

os.environ.setdefault("TK_SILENCE_DEPRECATION", "1")

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


LOG_RE = re.compile(r"(?P<level>[IWEVD]) \((?P<ms>\d+)\) (?P<tag>[^:]+): (?P<msg>.*)")
DISPLAY_LOG_RE = re.compile(r"(?P<level>[IWEVD])\s+(?P<ms>\d+)\s+(?P<tag>[^:]+):\s(?P<msg>.*)")
DEVICE_RE = re.compile(r"DEVICE:(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}):(?P<name>.*)")
DEVICE_NAME_RE = re.compile(r"DEVICE_NAME:(?P<name>.*?)\|(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})$")
BT_FOUND_BDA_RE = re.compile(r"bda=(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})")
BT_FOUND_BD_NAME_RE = re.compile(r'bd_name="(?P<name>[^"]*)"')
BT_FOUND_EIR_NAME_RE = re.compile(r'eir_name="(?P<name>[^"]*)"')
BT_FOUND_RSSI_RE = re.compile(r"rssi=(?P<rssi>-?\d+)")


class SerialWorker(threading.Thread):
    def __init__(self, port, baud, events):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self.stop_event = threading.Event()
        self.tx = queue.Queue()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5)
            self.events.put(("state", f"Connected to {self.port}"))
        except Exception as exc:
            self.events.put(("error", f"Open failed: {exc}"))
            return

        while not self.stop_event.is_set():
            try:
                while not self.tx.empty():
                    cmd = self.tx.get_nowait()
                    self.ser.write((cmd + "\n").encode("utf-8"))

                raw = self.ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    self.events.put(("line", line))
            except Exception as exc:
                self.events.put(("error", str(exc)))
                break

        if self.ser:
            self.ser.close()
        self.events.put(("state", "Disconnected"))

    def send(self, cmd):
        self.tx.put(cmd)

    def stop(self):
        self.stop_event.set()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP-A2DP Control")
        self.geometry("980x620")
        self.events = queue.Queue()
        self.worker = None
        self.devices = {}
        self.label_to_bda = {}
        self.scan_in_progress = False
        self.audio_mode = "STOPPED"

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.pin_var = tk.StringVar()
        self.call_number_var = tk.StringVar(value="5551234567")
        self.device_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.play_pause_var = tk.StringVar(value="Play")

        self._build_ui()
        self._refresh_ports()
        self.after(250, self._show_window)
        self.after(100, self._poll_events)

    def _show_window(self):
        self.deiconify()
        self.lift()
        self.attributes("-topmost", True)
        self.after(100, lambda: self.attributes("-topmost", False))
        self.focus_force()
        try:
            subprocess.run(
                ["osascript", "-e", 'tell application "Python" to activate'],
                check=False,
                timeout=1,
            )
        except Exception:
            pass

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=28)
        self.port_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Label(top, text="Baud").pack(side=tk.LEFT, padx=(16, 0))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=6)
        ttk.Button(top, text="Connect", command=self._connect).pack(side=tk.LEFT)
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side=tk.LEFT, padx=6)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=12)

        controls = ttk.Frame(self, padding=(8, 0, 8, 8))
        controls.pack(fill=tk.X)
        self.scan_button = ttk.Button(controls, text="Scan", command=self._scan)
        self.scan_button.pack(side=tk.LEFT)
        self.stop_scan_button = ttk.Button(controls, text="Stop Scan", command=self._stop_scan, state=tk.DISABLED)
        self.stop_scan_button.pack(side=tk.LEFT, padx=6)
        self.device_combo = ttk.Combobox(controls, textvariable=self.device_var, width=34, state="readonly")
        self.device_combo.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(controls, text="Connect Device", command=self._connect_device).pack(side=tk.LEFT)
        ttk.Separator(controls, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Button(controls, text="Previous", command=lambda: self._send("PREVIOUS")).pack(side=tk.LEFT)
        self.play_pause_button = ttk.Button(
            controls,
            textvariable=self.play_pause_var,
            command=self._toggle_play_pause,
            width=10,
        )
        self.play_pause_button.pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Next", command=lambda: self._send("NEXT")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Stop", command=lambda: self._send("STOP")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Status", command=lambda: self._send("STATUS")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Passive Reconnect", command=lambda: self._send("PASSIVE_RECONNECT")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Active Reconnect", command=lambda: self._send("ACTIVE_RECONNECT")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Clear Pairing", command=lambda: self._send("CLEAR_PAIRING")).pack(side=tk.LEFT)
        ttk.Label(controls, text="PIN").pack(side=tk.LEFT, padx=(18, 0))
        ttk.Entry(controls, textvariable=self.pin_var, width=14).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Send PIN", command=self._send_pin).pack(side=tk.LEFT)

        hfp = ttk.Frame(self, padding=(8, 0, 8, 8))
        hfp.pack(fill=tk.X)
        ttk.Label(hfp, text="Call").pack(side=tk.LEFT)
        ttk.Entry(hfp, textvariable=self.call_number_var, width=18).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Incoming", command=self._call_incoming).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Add Incoming", command=self._call_second_incoming).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Outgoing", command=self._call_outgoing).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Answer", command=lambda: self._send("CALL_ANSWER")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Reject", command=lambda: self._send("CALL_REJECT")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="End", command=lambda: self._send("CALL_END")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Swap/Hold", command=lambda: self._send("CALL_SWAP")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Merge", command=lambda: self._send("CALL_MERGE")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Release Waiting/Held", command=lambda: self._send("CALL_RELEASE_WAITING")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Release Active", command=lambda: self._send("CALL_RELEASE_ACTIVE")).pack(side=tk.LEFT)

        self.log_text = scrolledtext.ScrolledText(self, wrap=tk.NONE, height=24, undo=False)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        bottom = ttk.Frame(self, padding=(8, 0, 8, 8))
        bottom.pack(fill=tk.X)
        ttk.Button(bottom, text="Clear Logs", command=lambda: self.log_text.delete("1.0", tk.END)).pack(side=tk.LEFT)

    def _refresh_ports(self):
        if list_ports is None:
            messagebox.showerror("Missing dependency", "Install pyserial: python3 -m pip install pyserial")
            return
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _connect(self):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial: python3 -m pip install pyserial")
            return
        if self.worker:
            return
        self.worker = SerialWorker(self.port_var.get(), int(self.baud_var.get()), self.events)
        self.worker.start()

    def _disconnect(self):
        if self.worker:
            self.worker.stop()
            self.worker = None

    def _send(self, cmd):
        if not self.worker:
            self._append_line(f"HOST_TX_BLOCKED:not connected:{cmd}")
            self.status_var.set("Connect serial first")
            return
        self._append_line(f"HOST_TX: {cmd}")
        self.worker.send(cmd)

    def _toggle_play_pause(self):
        if self.audio_mode in ("I2S", "SIMULATE"):
            self._send("PAUSE")
        else:
            self._send("SIMULATE")

    def _set_audio_mode(self, mode):
        mode = mode.strip().upper()
        if not mode:
            return
        self.audio_mode = mode
        self.play_pause_var.set("Pause" if mode in ("I2S", "SIMULATE") else "Play")

    def _scan(self):
        if self.scan_in_progress:
            return
        self.scan_in_progress = True
        self._set_scan_controls()
        self.devices.clear()
        self.label_to_bda.clear()
        self.device_combo["values"] = []
        self.device_var.set("")
        self._send("SCAN")

    def _stop_scan(self):
        self._send("STOP_SCAN")

    def _connect_device(self):
        label = self.device_var.get()
        bda = self.label_to_bda.get(label)
        if not bda:
            messagebox.showwarning("No device", "Select a discovered Bluetooth device first.")
            return
        self._send(f"CONNECT:{bda}")

    def _send_pin(self):
        pin = self.pin_var.get().strip()
        if not pin.isdigit():
            messagebox.showwarning("Invalid PIN", "PIN must contain digits only.")
            return
        self._send(f"PIN:{pin}")

    def _call_incoming(self):
        number = self.call_number_var.get().strip()
        self._send(f"CALL_IN:{number}" if number else "CALL_IN")

    def _call_second_incoming(self):
        number = self.call_number_var.get().strip()
        self._send(f"CALL_IN2:{number}" if number else "CALL_IN2")

    def _call_outgoing(self):
        number = self.call_number_var.get().strip()
        self._send(f"CALL_OUT:{number}" if number else "CALL_OUT")

    def _append_line(self, line):
        match = LOG_RE.match(line) or DISPLAY_LOG_RE.match(line)
        if match:
            message = match.group("msg")
            display = f"{match.group('level')} {match.group('ms'):>8} {match.group('tag')}: {message}"
        else:
            message = line
            display = line
        self.log_text.insert(tk.END, display + "\n")
        self.log_text.see(tk.END)

        if "PIN_REQUIRED:" in message:
            self.status_var.set("PIN required")
        elif "BT_FOUND:" in message:
            self._add_bt_found_device(message)
        elif "DEVICE_NAME:" in message or "DEVICE:" in message:
            self._add_device_from_line(message)
        elif "SCAN_STARTED" in message or "SCAN:STARTED" in message:
            self.scan_in_progress = True
            self._set_scan_controls()
            self.status_var.set("Scanning")
        elif "SCAN_DONE:" in message:
            self.scan_in_progress = False
            self._set_scan_controls()
            count = message.rsplit("SCAN_DONE:", 1)[1].strip()
            self.status_var.set(f"Scan done: {count} device(s)")
        elif "SCAN_FAILED:" in message:
            self.scan_in_progress = False
            self._set_scan_controls()
            self.status_var.set(message.rsplit("SCAN_FAILED:", 1)[1].strip())
        elif "SCAN_ALREADY_STOPPED" in message:
            self.scan_in_progress = False
            self._set_scan_controls()
        elif "CONNECT_PENDING:" in message:
            self.status_var.set("Stopping scan before connect")
        elif "CONNECTING:" in message:
            self.status_var.set(message)
        elif "A2DP_CONNECTION:" in message:
            self.status_var.set(message.split("A2DP_CONNECTION:", 1)[1])
        elif "AUDIO_MODE:" in message:
            mode = message.split("AUDIO_MODE:", 1)[1]
            self._set_audio_mode(mode)
            self.status_var.set(mode)
        elif "HFP_AG_CALL:" in message:
            self.status_var.set(message.split("HFP_AG_CALL:", 1)[1])

    def _add_device_from_line(self, line):
        match = DEVICE_NAME_RE.search(line) or DEVICE_RE.search(line)
        if not match:
            return
        bda = match.group("bda").lower()
        name = match.group("name").strip() or "unnamed"
        self._upsert_device(bda, name)

    def _add_bt_found_device(self, line):
        bda_match = BT_FOUND_BDA_RE.search(line)
        if not bda_match:
            return
        bda = bda_match.group("bda").lower()

        bd_name_match = BT_FOUND_BD_NAME_RE.search(line)
        eir_name_match = BT_FOUND_EIR_NAME_RE.search(line)
        bd_name = bd_name_match.group("name").strip() if bd_name_match else ""
        eir_name = eir_name_match.group("name").strip() if eir_name_match else ""
        name = bd_name or eir_name or "unnamed"

        rssi_match = BT_FOUND_RSSI_RE.search(line)
        rssi = int(rssi_match.group("rssi")) if rssi_match else None
        self._upsert_device(bda, name, rssi)

    def _upsert_device(self, bda, name, rssi=None):
        selected_bda = self.label_to_bda.get(self.device_var.get())
        existing = self.devices.get(bda, {})
        current_name = existing.get("name", "")
        preferred_name = current_name

        if name and name != "unnamed" and (not current_name or current_name == "unnamed"):
            preferred_name = name
        elif not preferred_name:
            preferred_name = name or "unnamed"

        best_rssi = existing.get("rssi")
        if rssi is not None and (best_rssi is None or rssi > best_rssi):
            best_rssi = rssi

        self.devices[bda] = {
            "bda": bda,
            "name": preferred_name,
            "rssi": best_rssi,
        }
        self._refresh_device_dropdown(selected_bda=selected_bda or bda)

    def _refresh_device_dropdown(self, selected_bda=None):
        labels = []
        selected_label = ""
        self.label_to_bda.clear()
        for bda, info in sorted(self.devices.items(), key=lambda item: item[1].get("name", "")):
            rssi = info.get("rssi")
            suffix = f", {rssi} dBm" if rssi is not None else ""
            label = f"{info.get('name', 'unnamed')} ({bda}{suffix})"
            labels.append(label)
            self.label_to_bda[label] = bda
            if selected_bda == bda:
                selected_label = label

        self.device_combo.configure(values=labels)
        if selected_label:
            self.device_var.set(selected_label)
        elif self.device_var.get() not in labels:
            self.device_var.set(labels[0] if labels else "")

    def _set_scan_controls(self):
        if self.scan_in_progress:
            self.scan_button.configure(state=tk.DISABLED)
            self.stop_scan_button.configure(state=tk.NORMAL)
        else:
            self.scan_button.configure(state=tk.NORMAL)
            self.stop_scan_button.configure(state=tk.DISABLED)

    def _poll_events(self):
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self._append_line(payload)
            elif kind == "state":
                self.status_var.set(payload)
            elif kind == "error":
                self._append_line(f"E ({int(time.time() * 1000)}) HOST: {payload}")
                self.status_var.set("Error")
        self.after(100, self._poll_events)

    def destroy(self):
        self._disconnect()
        super().destroy()


if __name__ == "__main__":
    App().mainloop()
