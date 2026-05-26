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
DISCOVERED_RE = re.compile(r'DISCOVERED:bda=(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}) name="(?P<name>[^"]*)"')
BT_FOUND_BDA_RE = re.compile(r"bda=(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})")
BT_FOUND_BD_NAME_RE = re.compile(r'bd_name="(?P<name>[^"]*)"')
BT_FOUND_EIR_NAME_RE = re.compile(r'eir_name="(?P<name>[^"]*)"')
BT_FOUND_RSSI_RE = re.compile(r"rssi=(?P<rssi>-?\d+)")
BOARD_ID_RE = re.compile(
    r"BOARD_ID:role=(?P<role>HF|AG)\s+name=(?P<name>\S+)"
    r"(?:\s+build=(?P<build>\S+))?\s+profiles=(?P<profiles>.*)"
)
A2DP_RATE_RE = re.compile(
    r"A2DP_AUDIO_RATE:codec=(?P<codec>\S+) sample_rate=(?P<rate>\d+) channels=(?P<channels>\S+)"
    r"(?: bitpool=(?P<bitpool>\S+) blocks=(?P<blocks>\d+) subbands=(?P<subbands>\d+) alloc=(?P<alloc>\S+))?"
)
HFP_RATE_RE = re.compile(
    r"HFP_HF_AUDIO_RATE:codec=(?P<codec>\S+) sample_rate=(?P<rate>\d+) channels=(?P<channels>\S+) frame=(?P<frame>\d+)"
)
HFP_AG_CALL_RE = re.compile(
    r"HFP_AG_CALL:(?P<source>\S+)\s+count=(?P<count>\d+)\s+active=(?P<active>\d+)\s+held=(?P<held>\d+)\s+setup=(?P<setup>\d+)"
)
HFP_AG_CALL_SLOT_RE = re.compile(
    r'HFP_AG_CALL_SLOT:(?P<idx>\d+)\s+mode=(?P<mode>\d+)\s+dir=(?P<dir>\d+)\s+mpty=(?P<mpty>\d+)\s+num="(?P<num>[^"]*)"'
)
HFP_AG_CLCC_RE = re.compile(
    r'HFP_AG_CLCC:idx=(?P<idx>\d+)\s+dir=(?P<dir>\d+)\s+status=(?P<status>\d+)\s+mpty=(?P<mpty>\d+)\s+num="(?P<num>[^"]*)"'
)

BRIDGE_TX_PREFIX = "BRIDGE_TX:"
BRIDGE_RX_PREFIX = "BRIDGE_RX:"

AG_CALL_MODE_NAMES = {
    0: "Idle",
    1: "Incoming",
    2: "Outgoing",
    3: "Active",
    4: "Held",
    5: "Merged",
}

CLCC_STATUS_NAMES = {
    0: "Active",
    1: "Held",
    2: "Dialing",
    3: "Alerting",
    4: "Incoming",
    5: "Waiting",
}


class SerialEndpoint(threading.Thread):
    def __init__(self, role, port, baud, events):
        super().__init__(daemon=True)
        self.role = role
        self.port = port
        self.baud = baud
        self.events = events
        self.stop_event = threading.Event()
        self.tx = queue.Queue()
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1, write_timeout=0.5)
            self.events.put((self.role, "state", f"Connected to {self.port}", self))
        except Exception as exc:
            self.events.put((self.role, "error", f"Open failed: {exc}", self))
            self.events.put((self.role, "closed", None, self))
            return

        while not self.stop_event.is_set():
            try:
                while not self.tx.empty():
                    cmd = self.tx.get_nowait()
                    self.ser.write((cmd + "\n").encode("utf-8"))

                raw = self.ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    self.events.put((self.role, "line", line, self))
            except Exception as exc:
                self.events.put((self.role, "error", str(exc), self))
                break

        if self.ser:
            self.ser.close()
        self.events.put((self.role, "state", "Disconnected", self))
        self.events.put((self.role, "closed", None, self))

    def send(self, cmd):
        self.tx.put(cmd)

    def stop(self):
        self.stop_event.set()


class BridgeApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP-A2DP Bridge Control")
        self.geometry("1180x900")
        self.events = queue.Queue()
        self.endpoints = {"HF": None, "AG": None}
        self.want_connected = {"HF": False, "AG": False}
        self.devices = {"HF": {}, "AG": {}}
        self.label_to_bda = {"HF": {}, "AG": {}}
        self.port_label_to_device = {}
        self.scan_in_progress = {"HF": False, "AG": False}

        self.port_vars = {"HF": tk.StringVar(), "AG": tk.StringVar()}
        self.baud_vars = {"HF": tk.StringVar(value="115200"), "AG": tk.StringVar(value="115200")}
        self.status_vars = {"HF": tk.StringVar(value="HF disconnected"), "AG": tk.StringVar(value="AG disconnected")}
        self.identity_vars = {"HF": tk.StringVar(value="Expected: ESP-A2DP-HF"), "AG": tk.StringVar(value="Expected: ESP-A2DP AG")}
        self.a2dp_rate_vars = {"HF": tk.StringVar(value="A2DP: --"), "AG": tk.StringVar(value="A2DP: --")}
        self.hfp_rate_vars = {"HF": tk.StringVar(value="HFP: --"), "AG": tk.StringVar(value="HFP: --")}
        self.device_vars = {"HF": tk.StringVar(), "AG": tk.StringVar()}
        self.raw_vars = {"HF": tk.StringVar(), "AG": tk.StringVar()}
        self.auto_forward_var = tk.BooleanVar(value=True)
        self.ag_call_summary_var = tk.StringVar(value="AG calls: no call table yet")
        self.ag_call_rows = {}

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
        ttk.Button(top, text="Refresh Ports", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Checkbutton(top, text="Forward bridge payloads", variable=self.auto_forward_var).pack(side=tk.LEFT, padx=16)
        ttk.Button(top, text="Clear Logs", command=lambda: self.log_text.delete("1.0", tk.END)).pack(side=tk.RIGHT)

        boards = ttk.Frame(self, padding=(8, 0, 8, 8))
        boards.pack(fill=tk.X)
        self._build_hf_panel(boards)
        self._build_ag_panel(boards)

        bridge = ttk.LabelFrame(self, text="Bridge Traffic", padding=8)
        bridge.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.log_text = scrolledtext.ScrolledText(bridge, wrap=tk.NONE, height=28, undo=False)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_configure("HF", foreground="#075985")
        self.log_text.tag_configure("AG", foreground="#7c2d12")
        self.log_text.tag_configure("BRIDGE", foreground="#166534")
        self.log_text.tag_configure("ERROR", foreground="#b91c1c")

    def _build_common_connection(self, parent, role):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(row, text="Port").pack(side=tk.LEFT)
        combo = ttk.Combobox(row, textvariable=self.port_vars[role], width=34, state="readonly")
        combo.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        setattr(self, f"{role.lower()}_port_combo", combo)
        ttk.Label(row, text="Baud").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Entry(row, textvariable=self.baud_vars[role], width=8).pack(side=tk.LEFT, padx=6)
        ttk.Button(row, text="Connect", command=lambda: self._connect(role)).pack(side=tk.LEFT)
        ttk.Button(row, text="Disconnect", command=lambda: self._disconnect(role)).pack(side=tk.LEFT, padx=6)

        status_row = ttk.Frame(parent)
        status_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(status_row, textvariable=self.status_vars[role]).pack(side=tk.LEFT, fill=tk.X, expand=True)

        id_row = ttk.Frame(parent)
        id_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(id_row, textvariable=self.identity_vars[role]).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(id_row, text="Identify", command=lambda: self._send(role, "BOARD_ID")).pack(side=tk.RIGHT)
        ttk.Button(id_row, text="HELP", command=lambda: self._send(role, "HELP")).pack(side=tk.RIGHT)
        ttk.Button(id_row, text="STATUS", command=lambda: self._send(role, "STATUS")).pack(side=tk.RIGHT, padx=6)

        rate_row = ttk.Frame(parent)
        rate_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(rate_row, textvariable=self.a2dp_rate_vars[role]).pack(side=tk.LEFT, padx=(0, 18))
        ttk.Label(rate_row, textvariable=self.hfp_rate_vars[role]).pack(side=tk.LEFT)

    def _build_hf_panel(self, parent):
        box = ttk.LabelFrame(parent, text="HF Side: connects to real phone", padding=8)
        box.pack(fill=tk.X, expand=True, pady=(0, 8))
        self._build_common_connection(box, "HF")

        controls = ttk.Frame(box)
        controls.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(controls, text="Discoverable", command=lambda: self._send("HF", "DISCOVERABLE")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Connectable", command=lambda: self._send("HF", "CONNECTABLE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Hidden", command=lambda: self._send("HF", "HIDDEN")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Clear Pairing", command=lambda: self._send("HF", "CLEAR_PAIRING")).pack(side=tk.LEFT, padx=6)

        media = ttk.Frame(box)
        media.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(media, text="Phone controls").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(media, text="Play", command=lambda: self._send("HF", "AVRCP_PLAY")).pack(side=tk.LEFT)
        ttk.Button(media, text="Pause", command=lambda: self._send("HF", "AVRCP_PAUSE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Prev", command=lambda: self._send("HF", "AVRCP_PREVIOUS")).pack(side=tk.LEFT)
        ttk.Button(media, text="Next", command=lambda: self._send("HF", "AVRCP_NEXT")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Answer", command=lambda: self._send("HF", "HF_ANSWER")).pack(side=tk.LEFT)
        ttk.Button(media, text="End", command=lambda: self._send("HF", "HF_HANGUP")).pack(side=tk.LEFT, padx=6)

    def _build_ag_panel(self, parent):
        box = ttk.LabelFrame(parent, text="AG Side: connects to real car", padding=8)
        box.pack(fill=tk.X, expand=True)
        self._build_common_connection(box, "AG")

        scan = ttk.Frame(box)
        scan.pack(fill=tk.X, pady=(0, 6))
        scan_button = ttk.Button(scan, text="Scan", command=lambda: self._scan("AG"))
        scan_button.pack(side=tk.LEFT)
        stop_button = ttk.Button(scan, text="Stop Scan", command=lambda: self._stop_scan("AG"), state=tk.DISABLED)
        stop_button.pack(side=tk.LEFT, padx=6)
        device_combo = ttk.Combobox(scan, textvariable=self.device_vars["AG"], width=42, state="readonly")
        device_combo.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        ttk.Button(scan, text="Connect Car", command=lambda: self._connect_device("AG")).pack(side=tk.LEFT)
        self.ag_scan_button = scan_button
        self.ag_stop_scan_button = stop_button
        self.ag_device_combo = device_combo

        controls = ttk.Frame(box)
        controls.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(controls, text="Clear Pairing", command=lambda: self._send("AG", "CLEAR_PAIRING")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Passive Reconnect", command=lambda: self._send("AG", "PASSIVE_RECONNECT")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Active Reconnect", command=lambda: self._send("AG", "ACTIVE_RECONNECT")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Pause Reconnect", command=lambda: self._send("AG", "PAUSE_RECONNECT")).pack(side=tk.LEFT, padx=6)

        media = ttk.Frame(box)
        media.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(media, text="Car media").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(media, text="Play", command=lambda: self._send("AG", "PLAY")).pack(side=tk.LEFT)
        ttk.Button(media, text="Pause", command=lambda: self._send("AG", "PAUSE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Stop", command=lambda: self._send("AG", "STOP")).pack(side=tk.LEFT)
        ttk.Button(media, text="Prev", command=lambda: self._send("AG", "PREVIOUS")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Next", command=lambda: self._send("AG", "NEXT")).pack(side=tk.LEFT)

        calls = ttk.LabelFrame(box, text="AG Call Table", padding=6)
        calls.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(calls, textvariable=self.ag_call_summary_var).pack(anchor=tk.W, pady=(0, 4))
        self.ag_call_tree = ttk.Treeview(
            calls,
            columns=("state", "number", "direction"),
            show="tree headings",
            height=4,
            selectmode="none",
        )
        self.ag_call_tree.heading("#0", text="Call")
        self.ag_call_tree.heading("state", text="State")
        self.ag_call_tree.heading("number", text="Number")
        self.ag_call_tree.heading("direction", text="Dir")
        self.ag_call_tree.column("#0", width=80, stretch=False)
        self.ag_call_tree.column("state", width=130, stretch=False)
        self.ag_call_tree.column("number", width=220, stretch=True)
        self.ag_call_tree.column("direction", width=60, stretch=False)
        self.ag_call_tree.tag_configure("active", background="#cfe8ff")
        self.ag_call_tree.tag_configure("held", background="#eeeeee")
        self.ag_call_tree.tag_configure("incoming", background="#fff2b8")
        self.ag_call_tree.tag_configure("waiting", background="#ffe0c2")
        self.ag_call_tree.pack(fill=tk.X)

    def _refresh_ports(self):
        if list_ports is None:
            messagebox.showerror("Missing dependency", "Install pyserial: python3 -m pip install pyserial")
            return
        ports = []
        self.port_label_to_device.clear()
        for port in list_ports.comports():
            if not port.device.startswith("/dev/cu."):
                continue
            if "Bluetooth" in port.device:
                continue
            parts = [os.path.basename(port.device)]
            details = []
            if port.serial_number:
                details.append(f"SER={port.serial_number}")
            if port.location:
                details.append(f"LOC={port.location}")
            if port.description and port.description != "n/a":
                details.append(port.description)
            if details:
                parts.append("(" + ", ".join(details) + ")")
            label = " ".join(parts)
            ports.append(label)
            self.port_label_to_device[label] = port.device
        for role in ("HF", "AG"):
            combo = getattr(self, f"{role.lower()}_port_combo", None)
            if combo:
                combo["values"] = ports
            current = self.port_vars[role].get()
            if current not in ports:
                default_index = 0 if role == "HF" else min(1, len(ports) - 1)
                self.port_vars[role].set(ports[default_index] if ports else "")

    def _connect(self, role):
        if serial is None:
            messagebox.showerror("Missing dependency", "Install pyserial: python3 -m pip install pyserial")
            return
        if self.endpoints[role]:
            return
        selected = self.port_vars[role].get()
        port = self.port_label_to_device.get(selected, selected)
        if not port:
            messagebox.showwarning("No port", f"Select a serial port for {role}.")
            return
        other_role = "AG" if role == "HF" else "HF"
        other = self.endpoints.get(other_role)
        if other and other.port == port:
            messagebox.showerror("Port already in use", f"{port} is already connected as {other_role}. Select the other USB serial port.")
            return
        self.want_connected[role] = True
        worker = SerialEndpoint(role, port, int(self.baud_vars[role].get()), self.events)
        self.endpoints[role] = worker
        worker.start()

    def _disconnect(self, role):
        self.want_connected[role] = False
        worker = self.endpoints[role]
        if worker:
            worker.stop()
            self.endpoints[role] = None

    def _send(self, role, cmd, log=True):
        worker = self.endpoints[role]
        if not worker:
            self._append_log(role, "TX_BLOCKED", f"not connected:{cmd}", error=True)
            self.status_vars[role].set("Connect serial first")
            return False
        if log:
            self._append_log(role, "TX", cmd)
        worker.send(cmd)
        return True

    def _send_raw(self, role):
        cmd = self.raw_vars[role].get().strip()
        if not cmd:
            return
        self.raw_vars[role].set("")
        self._send(role, cmd)

    def _scan(self, role):
        self.devices[role].clear()
        self.label_to_bda[role].clear()
        self.device_vars[role].set("")
        if role == "AG":
            self.ag_device_combo["values"] = []
        self._send(role, "SCAN")

    def _stop_scan(self, role):
        self._send(role, "STOP_SCAN")

    def _connect_device(self, role):
        label = self.device_vars[role].get()
        bda = self.label_to_bda[role].get(label)
        if not bda:
            messagebox.showwarning("No device", "Select a discovered Bluetooth device first.")
            return
        self._send(role, f"CONNECT:{bda}")

    def _append_log(self, role, direction, text, error=False):
        stamp = time.strftime("%H:%M:%S")
        line = f"{stamp} [{role} {direction}] {text}\n"
        tag = "ERROR" if error else role
        self.log_text.insert(tk.END, line, tag)
        self.log_text.see(tk.END)

    def _append_bridge_log(self, source, target, payload, forwarded):
        stamp = time.strftime("%H:%M:%S")
        status = "FORWARDED" if forwarded else "BLOCKED"
        self.log_text.insert(tk.END, f"{stamp} [{source}->{target} {status}] {payload}\n", "BRIDGE")
        self.log_text.see(tk.END)

    def _message_from_line(self, line):
        match = LOG_RE.match(line) or DISPLAY_LOG_RE.match(line)
        if match:
            return match.group("msg")
        return line

    def _handle_line(self, role, line):
        self._append_log(role, "RX", line)
        message = self._message_from_line(line)
        self._detect_identity(role, message)
        self._handle_discovery(role, message)
        self._handle_status(role, message)
        self._handle_audio_rate(role, message)
        self._handle_ag_call_table(role, message)
        self._handle_bridge_payload(role, message)

    def _detect_identity(self, role, message):
        board_match = BOARD_ID_RE.search(message)
        if board_match:
            board_role = board_match.group("role")
            name = board_match.group("name")
            build = board_match.group("build")
            profiles = board_match.group("profiles")
            detected = f"Detected: {board_role} board ({name})"
            if build:
                detected += f" build={build}"
            detected += f" {profiles}"
            if board_role != role:
                detected += f" - WRONG PORT, expected {role}"
            self.identity_vars[role].set(detected)
            return

        if "ROLE:A2DP_SINK HFP_HF AVRCP_CT" in message or "ESP-A2DP-HF" in message:
            detected = "Detected: HF board"
            if role != "HF":
                detected += " - WRONG PORT, expected AG"
            self.identity_vars[role].set(detected)
        elif "ROLE:A2DP_SOURCE" in message or "ESP-A2DP-Source" in message or "ESP_A2DP:" in message:
            detected = "Detected: AG board"
            if role != "AG":
                detected += " - WRONG PORT, expected HF"
            self.identity_vars[role].set(detected)

    def _handle_status(self, role, message):
        if "BT_READY:" in message:
            self.status_vars[role].set(message)
        elif "CONNECTING:" in message:
            self.status_vars[role].set(message)
        elif "A2DP_CONNECTION:" in message:
            self.status_vars[role].set(message)
        elif "HFP_HF_CONNECTION:" in message or "HFP_AG_CONNECTION:" in message:
            self.status_vars[role].set(message)
        elif "BT_SCAN_MODE:" in message:
            self.status_vars[role].set(message)
        elif "SCAN_FAILED:" in message:
            self.status_vars[role].set(message)

    def _format_rate(self, prefix, codec, rate, channels):
        rate = int(rate)
        if rate <= 0 or codec == "none":
            return f"{prefix}: --"
        text = f"{prefix}: {codec} {rate // 1000} kHz {channels}"
        return text

    def _handle_audio_rate(self, role, message):
        a2dp_match = A2DP_RATE_RE.search(message)
        if a2dp_match:
            text = self._format_rate(
                "A2DP",
                a2dp_match.group("codec"),
                a2dp_match.group("rate"),
                a2dp_match.group("channels"),
            )
            if a2dp_match.group("bitpool"):
                text += f" bitpool {a2dp_match.group('bitpool')}"
            self.a2dp_rate_vars[role].set(text)
            return

        hfp_match = HFP_RATE_RE.search(message)
        if hfp_match:
            text = self._format_rate(
                "HFP",
                hfp_match.group("codec"),
                hfp_match.group("rate"),
                hfp_match.group("channels"),
            )
            if int(hfp_match.group("rate")) > 0:
                text += f" frame {hfp_match.group('frame')}"
            self.hfp_rate_vars[role].set(text)
            return

    def _handle_ag_call_table(self, role, message):
        if role != "AG":
            return

        call_match = HFP_AG_CALL_RE.search(message)
        if call_match:
            self.ag_call_summary_var.set(
                "AG calls: "
                f"{call_match.group('source')} "
                f"count={call_match.group('count')} "
                f"active={call_match.group('active')} "
                f"held={call_match.group('held')} "
                f"setup={call_match.group('setup')}"
            )
            self.ag_call_rows.clear()
            self._refresh_ag_call_table()
            return

        slot_match = HFP_AG_CALL_SLOT_RE.search(message)
        if slot_match:
            idx = int(slot_match.group("idx"))
            mode = int(slot_match.group("mode"))
            direction = "in" if slot_match.group("dir") == "1" else "out"
            state = AG_CALL_MODE_NAMES.get(mode, f"Mode {mode}")
            self.ag_call_rows[idx] = {
                "idx": str(idx),
                "state": state,
                "status": self._ag_mode_status(mode),
                "dir": direction,
                "mpty": slot_match.group("mpty"),
                "number": slot_match.group("num"),
            }
            self._refresh_ag_call_table()
            return

        clcc_match = HFP_AG_CLCC_RE.search(message)
        if clcc_match:
            idx = int(clcc_match.group("idx"))
            if idx in self.ag_call_rows:
                return
            status = int(clcc_match.group("status"))
            direction = "in" if clcc_match.group("dir") == "1" else "out"
            self.ag_call_rows[idx] = {
                "idx": str(idx),
                "state": CLCC_STATUS_NAMES.get(status, f"Status {status}"),
                "status": status,
                "dir": direction,
                "mpty": clcc_match.group("mpty"),
                "number": clcc_match.group("num"),
            }
            self._refresh_ag_call_table()

    def _refresh_ag_call_table(self):
        for item in self.ag_call_tree.get_children():
            self.ag_call_tree.delete(item)
        for idx in sorted(self.ag_call_rows):
            row = self.ag_call_rows[idx]
            self.ag_call_tree.insert(
                "",
                tk.END,
                text=f"Call {idx}",
                values=(
                    row["state"],
                    row["number"],
                    row["dir"],
                ),
                tags=(self._ag_call_row_tag(row.get("status")),),
            )

    def _ag_mode_status(self, mode):
        if mode in (3, 5):
            return 0
        if mode == 4:
            return 1
        if mode == 2:
            return 2
        if mode == 1:
            return 4
        return -1

    def _ag_call_row_tag(self, status):
        if status == 0:
            return "active"
        if status == 1:
            return "held"
        if status == 4:
            return "incoming"
        if status == 5:
            return "waiting"
        return ""

    def _handle_discovery(self, role, message):
        if "SCAN_STARTED" in message or "SCAN:STARTED" in message:
            self.scan_in_progress[role] = True
            self._set_scan_controls(role)
            return
        if "SCAN_FAILED:" in message or "SCAN_ALREADY_STOPPED" in message:
            self.scan_in_progress[role] = False
            self._set_scan_controls(role)
            return
        if "SCAN_ALREADY_ACTIVE" in message:
            self.scan_in_progress[role] = True
            self._set_scan_controls(role)
            return
        if "SCAN_DONE:" in message or "SCAN:STOPPED" in message or "SCAN_STOP:" in message:
            self.scan_in_progress[role] = False
            self._set_scan_controls(role)
            return
        if "DISCOVERED:" in message:
            match = DISCOVERED_RE.search(message)
            if match:
                self._upsert_device(role, match.group("bda").lower(), match.group("name").strip() or "unnamed")
            return
        if "DEVICE_NAME:" in message or "DEVICE:" in message:
            match = DEVICE_NAME_RE.search(message) or DEVICE_RE.search(message)
            if match:
                self._upsert_device(role, match.group("bda").lower(), match.group("name").strip() or "unnamed")
            return
        if "BT_FOUND:" in message:
            bda_match = BT_FOUND_BDA_RE.search(message)
            if not bda_match:
                return
            bd_name_match = BT_FOUND_BD_NAME_RE.search(message)
            eir_name_match = BT_FOUND_EIR_NAME_RE.search(message)
            rssi_match = BT_FOUND_RSSI_RE.search(message)
            name = ""
            if bd_name_match:
                name = bd_name_match.group("name").strip()
            if not name and eir_name_match:
                name = eir_name_match.group("name").strip()
            rssi = int(rssi_match.group("rssi")) if rssi_match else None
            self._upsert_device(role, bda_match.group("bda").lower(), name or "unnamed", rssi)

    def _handle_bridge_payload(self, role, message):
        if BRIDGE_TX_PREFIX not in message:
            return
        payload = message.split(BRIDGE_TX_PREFIX, 1)[1]
        target = "AG" if role == "HF" else "HF"
        if not self.auto_forward_var.get():
            self._append_bridge_log(role, target, payload, False)
            return
        forwarded = self._send(target, BRIDGE_RX_PREFIX + payload, log=False)
        self._append_bridge_log(role, target, payload, forwarded)

    def _upsert_device(self, role, bda, name, rssi=None):
        existing = self.devices[role].get(bda, {})
        preferred_name = existing.get("name") or name or "unnamed"
        if name and name != "unnamed":
            preferred_name = name
        best_rssi = existing.get("rssi")
        if rssi is not None and (best_rssi is None or rssi > best_rssi):
            best_rssi = rssi
        self.devices[role][bda] = {"name": preferred_name, "rssi": best_rssi}
        self._refresh_device_dropdown(role, selected_bda=bda)

    def _refresh_device_dropdown(self, role, selected_bda=None):
        labels = []
        selected_label = ""
        self.label_to_bda[role].clear()
        for bda, info in sorted(self.devices[role].items(), key=lambda item: item[1].get("name", "")):
            rssi = info.get("rssi")
            suffix = f", {rssi} dBm" if rssi is not None else ""
            label = f"{info.get('name', 'unnamed')} ({bda}{suffix})"
            labels.append(label)
            self.label_to_bda[role][label] = bda
            if selected_bda == bda:
                selected_label = label
        if role == "AG":
            self.ag_device_combo.configure(values=labels)
        if selected_label:
            self.device_vars[role].set(selected_label)
        elif self.device_vars[role].get() not in labels:
            self.device_vars[role].set(labels[0] if labels else "")

    def _set_scan_controls(self, role):
        if role != "AG":
            return
        if self.scan_in_progress[role]:
            self.ag_scan_button.configure(state=tk.DISABLED)
            self.ag_stop_scan_button.configure(state=tk.NORMAL)
        else:
            self.ag_scan_button.configure(state=tk.NORMAL)
            self.ag_stop_scan_button.configure(state=tk.DISABLED)

    def _poll_events(self):
        while True:
            try:
                event = self.events.get_nowait()
            except queue.Empty:
                break
            if len(event) == 4:
                role, kind, payload, source = event
            else:
                role, kind, payload = event
                source = self.endpoints.get(role)
            if kind == "line":
                self._handle_line(role, payload)
            elif kind == "state":
                if payload == "Disconnected" and self.endpoints.get(role) is not source:
                    continue
                self.status_vars[role].set(payload)
                self._append_log(role, "STATE", payload)
                if payload.startswith("Connected to "):
                    self._send(role, "BOARD_ID")
                    self._send(role, "STATUS")
            elif kind == "error":
                if self.endpoints.get(role) is not source:
                    continue
                self.status_vars[role].set("Serial error")
                self._append_log(role, "ERROR", payload, error=True)
            elif kind == "closed":
                if self.endpoints.get(role) is not source:
                    continue
                self.endpoints[role] = None
                self._refresh_ports()
                if self.want_connected[role]:
                    self.status_vars[role].set("Serial dropped - retrying")
                    self.after(1000, lambda role=role: self._connect(role))
        self.after(100, self._poll_events)

    def destroy(self):
        for role in ("HF", "AG"):
            self._disconnect(role)
        super().destroy()


if __name__ == "__main__":
    BridgeApp().mainloop()
