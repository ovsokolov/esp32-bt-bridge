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
DISCOVERED_RE = re.compile(r'DISCOVERED:bda=(?P<bda>(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}) name="(?P<name>[^"]*)"')
AVRCP_STATUS_RE = re.compile(r"AVRCP_STATUS:len=(?P<length>\d+) pos=(?P<pos>\d+) status=(?P<status>\d+)")
AVRCP_NOTIFY_RE = re.compile(r"AVRCP_NOTIFY:event=0x(?P<event>[0-9a-fA-F]+)(?:\([^)]*\))?(?P<fields>.*)")
AVRCP_METADATA_RE = re.compile(r'AVRCP_METADATA:attr=(?P<attr>\d+) len=\d+ text="(?P<text>.*)"')
AVRCP_NOTIFY_FIELD_RE = re.compile(r"\b(?P<key>playback|pos|volume|battery|system)=(?P<value>\d+)")
A2DP_CONNECTION_RE = re.compile(r"A2DP_CONNECTION:state=(?P<state>\d+)")
AVRCP_CONNECTION_RE = re.compile(r"AVRCP_CT_CONNECTION:connected=(?P<connected>\d+)")
HFP_CONNECTION_RE = re.compile(r"HFP_HF_CONNECTION:state=(?P<state>\d+)")
A2DP_RATE_RE = re.compile(
    r"A2DP_AUDIO_RATE:codec=(?P<codec>\S+) sample_rate=(?P<rate>\d+) channels=(?P<channels>\S+)"
    r"(?: bitpool=(?P<bitpool>\S+) blocks=(?P<blocks>\d+) subbands=(?P<subbands>\d+) alloc=(?P<alloc>\S+))?"
)
HFP_RATE_RE = re.compile(
    r"HFP_HF_AUDIO_RATE:codec=(?P<codec>\S+) sample_rate=(?P<rate>\d+) channels=(?P<channels>\S+) frame=(?P<frame>\d+)"
)
HFP_CIND_CALL_RE = re.compile(r"HFP_HF_CIND_CALL:(?P<status>\d+)")
HFP_CIND_SETUP_RE = re.compile(r"HFP_HF_CIND_CALLSETUP:(?P<status>\d+)")
HFP_CIND_HELD_RE = re.compile(r"HFP_HF_CIND_CALLHELD:(?P<status>\d+)")
HFP_NUMBER_RE = re.compile(r'HFP_HF_(?:CLIP|CCWA):number="(?P<number>[^"]*)"')
HFP_CLCC_RE = re.compile(
    r'HFP_HF_CLCC:idx=(?P<idx>\d+) dir=(?P<dir>\d+) status=(?P<status>\d+) mpty=(?P<mpty>\d+) num="(?P<num>[^"]*)"'
)

METADATA_ATTRS = {
    1: "title",
    2: "artist",
    4: "album",
    8: "track",
    16: "track_count",
    32: "genre",
    64: "playing_time",
}

UNKNOWN_AVRCP_TIME = 0xFFFFFFFF

CALL_SETUP_NAMES = {
    0: "No incoming call",
    1: "Incoming call",
    2: "Dialing",
    3: "Remote ringing",
}

CALL_HELD_NAMES = {
    0: "No held call",
    1: "Active and held calls",
    2: "Held call only",
}

CLCC_STATUS_NAMES = {
    0: "Active",
    1: "Held",
    2: "Dialing",
    3: "Alerting",
    4: "Incoming",
    5: "Waiting",
    6: "Held by response",
}


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
            self.events.put(("closed", None))
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
        self.events.put(("closed", None))

    def send(self, cmd):
        self.tx.put(cmd)

    def stop(self):
        self.stop_event.set()


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP-A2DP-HF Phone Control")
        self.geometry("1120x700")
        self.events = queue.Queue()
        self.worker = None
        self.want_serial_connected = False
        self.devices = {}
        self.label_to_bda = {}
        self.scan_in_progress = False
        self.audio_mode = "STOPPED"

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.device_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Disconnected")
        self.play_pause_var = tk.StringVar(value="Play")
        self.playback_status = None
        self.track_length_ms = None
        self.track_position_ms = None
        self.screen_title_var = tk.StringVar(value="No track")
        self.screen_artist_var = tk.StringVar(value="Unknown artist")
        self.screen_album_var = tk.StringVar(value="Unknown album")
        self.screen_state_var = tk.StringVar(value="Stopped")
        self.screen_progress_var = tk.StringVar(value="0:00 / 0:00")
        self.screen_track_var = tk.StringVar(value="Track --")
        self.screen_volume_var = tk.StringVar(value="Volume --")
        self.screen_link_var = tk.StringVar(value="Phone: disconnected | Audio: disconnected | AVRCP: disconnected")
        self.a2dp_rate_var = tk.StringVar(value="A2DP --")
        self.hfp_rate_var = tk.StringVar(value="HFP --")
        self.call_status_var = tk.StringVar(value="No call")
        self.call_number_var = tk.StringVar(value="Number --")
        self.call_held_var = tk.StringVar(value="No held call")
        self.call_action_var = tk.StringVar(value="Idle")
        self.call_focus_var = tk.StringVar(value="")
        self.hfp_call = 0
        self.hfp_call_setup = 0
        self.hfp_call_held = 0
        self.hfp_last_number = ""
        self.hfp_waiting_number = ""
        self.hfp_other_number = ""
        self.hfp_clcc = {}
        self.hfp_clcc_pending = {}
        self.hfp_clcc_collecting = False
        self.hfp_clcc_after_id = None
        self.a2dp_connected = False
        self.avrcp_connected = False
        self.hfp_connected = False

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

        discovery = ttk.Frame(self, padding=(8, 0, 8, 8))
        discovery.pack(fill=tk.X)
        self.scan_button = ttk.Button(discovery, text="Scan", command=self._scan)
        self.scan_button.pack(side=tk.LEFT)
        self.stop_scan_button = ttk.Button(discovery, text="Stop Scan", command=self._stop_scan, state=tk.DISABLED)
        self.stop_scan_button.pack(side=tk.LEFT, padx=6)
        self.device_combo = ttk.Combobox(discovery, textvariable=self.device_var, width=34, state="readonly")
        self.device_combo.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(discovery, text="Connect Device", command=self._connect_device).pack(side=tk.LEFT)
        ttk.Separator(discovery, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)
        ttk.Button(discovery, text="Discoverable", command=lambda: self._send("DISCOVERABLE")).pack(side=tk.LEFT)
        ttk.Button(discovery, text="Connectable", command=lambda: self._send("CONNECTABLE")).pack(side=tk.LEFT, padx=6)
        ttk.Button(discovery, text="Hidden", command=lambda: self._send("HIDDEN")).pack(side=tk.LEFT)
        ttk.Button(discovery, text="Clear Pairing", command=lambda: self._send("CLEAR_PAIRING")).pack(side=tk.LEFT, padx=6)
        ttk.Button(discovery, text="Status", command=lambda: self._send("STATUS")).pack(side=tk.LEFT)

        media = ttk.Frame(self, padding=(8, 0, 8, 8))
        media.pack(fill=tk.X)
        ttk.Label(media, text="Media").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(media, text="Previous", command=lambda: self._send("AVRCP_PREVIOUS")).pack(side=tk.LEFT)
        self.play_pause_button = ttk.Button(
            media,
            textvariable=self.play_pause_var,
            command=self._toggle_play_pause,
            width=10,
        )
        self.play_pause_button.pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Next", command=lambda: self._send("AVRCP_NEXT")).pack(side=tk.LEFT)
        ttk.Button(media, text="Stop", command=lambda: self._send("AVRCP_STOP")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="Metadata", command=lambda: self._send("AVRCP_METADATA")).pack(side=tk.LEFT)
        ttk.Button(media, text="AVRCP RN", command=lambda: self._send("AVRCP_RN")).pack(side=tk.LEFT, padx=6)
        ttk.Button(media, text="AVRCP Status", command=lambda: self._send("AVRCP_STATUS")).pack(side=tk.LEFT)

        hfp = ttk.Frame(self, padding=(8, 0, 8, 8))
        hfp.pack(fill=tk.X)
        ttk.Label(hfp, text="Phone").pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(hfp, text="Answer", command=lambda: self._send("HF_ANSWER")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Reject/End", command=self._reject_or_end_call).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="CLCC", command=lambda: self._send("HF_CLCC")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Swap/Hold", command=lambda: self._send("HF_CHLD:2")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Merge", command=lambda: self._send("HF_CHLD:3")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Release Waiting/Held", command=lambda: self._send("HF_CHLD:0")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Release Active", command=lambda: self._send("HF_CHLD:1")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="Audio Connect", command=lambda: self._send("HF_AUDIO_CONNECT")).pack(side=tk.LEFT, padx=6)
        ttk.Button(hfp, text="Audio Disconnect", command=lambda: self._send("HF_AUDIO_DISCONNECT")).pack(side=tk.LEFT)
        ttk.Button(hfp, text="NREC", command=lambda: self._send("HF_NREC")).pack(side=tk.LEFT, padx=6)

        self._build_car_screen()

        self.log_text = scrolledtext.ScrolledText(self, wrap=tk.NONE, height=24, undo=False)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        bottom = ttk.Frame(self, padding=(8, 0, 8, 8))
        bottom.pack(fill=tk.X)
        ttk.Button(bottom, text="Clear Logs", command=lambda: self.log_text.delete("1.0", tk.END)).pack(side=tk.LEFT)

    def _build_car_screen(self):
        self.screen_area = ttk.Frame(self)
        self.screen_area.pack(fill=tk.X, padx=8, pady=(0, 8))
        self.media_screen = ttk.LabelFrame(self.screen_area, text="Car Screen", padding=10)
        self.call_screen = ttk.LabelFrame(self.screen_area, text="Phone Screen", padding=10)

        screen = self.media_screen
        screen.columnconfigure(0, weight=1)

        ttk.Label(screen, textvariable=self.screen_title_var, font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, sticky=tk.W
        )
        ttk.Label(screen, textvariable=self.screen_state_var, font=("Helvetica", 12, "bold")).grid(
            row=0, column=1, sticky=tk.E, padx=(12, 0)
        )
        ttk.Label(screen, textvariable=self.screen_artist_var, font=("Helvetica", 13)).grid(
            row=1, column=0, sticky=tk.W, pady=(2, 0)
        )
        ttk.Label(screen, textvariable=self.screen_album_var).grid(row=2, column=0, sticky=tk.W)
        ttk.Label(screen, textvariable=self.screen_progress_var).grid(row=1, column=1, sticky=tk.E, padx=(12, 0))

        self.screen_progress_bar = ttk.Progressbar(screen, mode="determinate", maximum=1000)
        self.screen_progress_bar.grid(row=3, column=0, columnspan=2, sticky=tk.EW, pady=(8, 4))

        controls = ttk.Frame(screen)
        controls.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=(4, 2))
        ttk.Button(controls, text="Previous", command=lambda: self._send("AVRCP_PREVIOUS")).pack(side=tk.LEFT)
        ttk.Button(controls, textvariable=self.play_pause_var, command=self._toggle_play_pause, width=10).pack(
            side=tk.LEFT, padx=6
        )
        ttk.Button(controls, text="Next", command=lambda: self._send("AVRCP_NEXT")).pack(side=tk.LEFT)

        details = ttk.Frame(screen)
        details.grid(row=5, column=0, columnspan=2, sticky=tk.EW)
        ttk.Label(details, textvariable=self.screen_track_var).pack(side=tk.LEFT)
        ttk.Label(details, textvariable=self.screen_volume_var).pack(side=tk.LEFT, padx=16)
        ttk.Label(details, textvariable=self.screen_link_var).pack(side=tk.RIGHT)

        rates = ttk.Frame(screen)
        rates.grid(row=6, column=0, columnspan=2, sticky=tk.EW, pady=(4, 0))
        ttk.Label(rates, textvariable=self.a2dp_rate_var).pack(side=tk.LEFT)
        ttk.Label(rates, textvariable=self.hfp_rate_var).pack(side=tk.LEFT, padx=16)

        self._build_call_screen()
        self._show_media_screen()

    def _build_call_screen(self):
        screen = self.call_screen
        screen.columnconfigure(0, weight=1)

        ttk.Label(screen, textvariable=self.call_status_var, font=("Helvetica", 18, "bold")).grid(
            row=0, column=0, sticky=tk.W
        )
        ttk.Label(screen, textvariable=self.call_action_var, font=("Helvetica", 12, "bold")).grid(
            row=0, column=1, sticky=tk.E, padx=(12, 0)
        )
        ttk.Label(screen, textvariable=self.call_number_var, font=("Helvetica", 13)).grid(
            row=1, column=0, sticky=tk.W, pady=(2, 0)
        )
        ttk.Label(screen, textvariable=self.call_held_var).grid(row=2, column=0, sticky=tk.W)
        ttk.Label(screen, textvariable=self.call_focus_var, font=("Helvetica", 12, "bold")).grid(
            row=2, column=1, sticky=tk.E, padx=(12, 0)
        )
        ttk.Label(screen, textvariable=self.hfp_rate_var).grid(row=1, column=1, sticky=tk.E, padx=(12, 0))

        controls = ttk.Frame(screen)
        controls.grid(row=3, column=0, columnspan=2, sticky=tk.W, pady=(8, 4))
        ttk.Button(controls, text="Accept", command=lambda: self._send("HF_ANSWER")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Reject/End", command=self._reject_or_end_call).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Swap/Hold", command=lambda: self._send("HF_CHLD:2")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Merge", command=lambda: self._send("HF_CHLD:3")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Release Waiting/Held", command=lambda: self._send("HF_CHLD:0")).pack(side=tk.LEFT)
        ttk.Button(controls, text="Release Active", command=lambda: self._send("HF_CHLD:1")).pack(side=tk.LEFT, padx=6)
        ttk.Button(controls, text="Audio", command=lambda: self._send("HF_AUDIO_CONNECT")).pack(side=tk.LEFT)
        ttk.Button(controls, text="CLCC", command=lambda: self._send("HF_CLCC")).pack(side=tk.LEFT, padx=6)

        self.call_list = ttk.Treeview(
            screen,
            columns=("state", "number", "direction"),
            show="tree headings",
            height=4,
            selectmode="none",
        )
        self.call_list.heading("#0", text="Call")
        self.call_list.heading("state", text="State")
        self.call_list.heading("number", text="Number")
        self.call_list.heading("direction", text="Dir")
        self.call_list.column("#0", width=80, stretch=False)
        self.call_list.column("state", width=130, stretch=False)
        self.call_list.column("number", width=220, stretch=True)
        self.call_list.column("direction", width=60, stretch=False)
        self.call_list.tag_configure("active", background="#cfe8ff")
        self.call_list.tag_configure("held", background="#eeeeee")
        self.call_list.tag_configure("incoming", background="#fff2b8")
        self.call_list.tag_configure("waiting", background="#ffe0c2")
        self.call_list.grid(row=4, column=0, columnspan=2, sticky=tk.EW, pady=(4, 0))

    def _show_media_screen(self):
        self.call_screen.pack_forget()
        self.media_screen.pack(fill=tk.X)

    def _show_call_screen(self):
        self.media_screen.pack_forget()
        self.call_screen.pack(fill=tk.X)

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
        self.want_serial_connected = True
        self.worker = SerialWorker(self.port_var.get(), int(self.baud_var.get()), self.events)
        self.worker.start()

    def _disconnect(self):
        self.want_serial_connected = False
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
        if self.playback_status == 1:
            self._send("AVRCP_PAUSE")
        else:
            self._send("AVRCP_PLAY")
        self._send("AVRCP_STATUS")

    def _reject_or_end_call(self):
        rows = self._call_display_rows()
        has_waiting_or_incoming = any(row["status"] in (4, 5) for row in rows)
        has_active = self.hfp_call != 0 or any(row["status"] == 0 for row in rows)

        if has_waiting_or_incoming and has_active:
            self._send("HF_CHLD:0")
        else:
            self._send("HF_HANGUP")
        self._schedule_hf_clcc()

    def _set_audio_mode(self, mode):
        mode = mode.strip().upper()
        if not mode:
            return
        self.audio_mode = mode
        self.play_pause_var.set("Pause" if mode in ("I2S", "SIMULATE") else "Play")

    def _set_playback_status(self, status):
        self.playback_status = status
        self.play_pause_var.set("Pause" if status == 1 else "Play")
        self.screen_state_var.set(self._playback_name(status))

    def _playback_name(self, status):
        return {
            0: "Stopped",
            1: "Playing",
            2: "Paused",
            3: "Seeking >>",
            4: "Seeking <<",
            255: "Error",
        }.get(status, f"Status {status}")

    def _format_ms(self, value):
        if value is None or value <= 0 or value == UNKNOWN_AVRCP_TIME:
            return "0:00"
        seconds = value // 1000
        return f"{seconds // 60}:{seconds % 60:02d}"

    def _update_progress(self, pos=None, length=None):
        if pos is not None and pos != UNKNOWN_AVRCP_TIME:
            self.track_position_ms = pos
        if length is not None and length != UNKNOWN_AVRCP_TIME:
            self.track_length_ms = length

        pos_text = self._format_ms(self.track_position_ms)
        length_text = self._format_ms(self.track_length_ms)
        self.screen_progress_var.set(f"{pos_text} / {length_text}")

        if self.track_length_ms and self.track_length_ms > 0 and self.track_position_ms is not None:
            value = max(0, min(1000, int((self.track_position_ms / self.track_length_ms) * 1000)))
        else:
            value = 0
        self.screen_progress_bar.configure(value=value)

    def _update_link_summary(self):
        phone = "connected" if self.hfp_connected else "disconnected"
        audio = "connected" if self.a2dp_connected else "disconnected"
        avrcp = "connected" if self.avrcp_connected else "disconnected"
        self.screen_link_var.set(f"Phone: {phone} | Audio: {audio} | AVRCP: {avrcp}")

    def _parse_notify_fields(self, fields):
        return {match.group("key"): int(match.group("value")) for match in AVRCP_NOTIFY_FIELD_RE.finditer(fields)}

    def _handle_metadata(self, message):
        match = AVRCP_METADATA_RE.search(message)
        if not match:
            return
        attr = METADATA_ATTRS.get(int(match.group("attr")))
        text = match.group("text").strip()
        if not attr:
            return
        if attr == "title":
            self.screen_title_var.set(text or "No track")
            self.track_position_ms = 0
            self.track_length_ms = None
            self._update_progress()
        elif attr == "artist":
            self.screen_artist_var.set(text or "Unknown artist")
        elif attr == "album":
            self.screen_album_var.set(text or "Unknown album")
        elif attr == "track":
            self.screen_track_var.set(f"Track {text or '--'}")
        elif attr == "track_count":
            current = self.screen_track_var.get()
            if current == "Track --":
                self.screen_track_var.set(f"Track -- / {text}")
            elif " / " not in current:
                self.screen_track_var.set(f"{current} / {text}")
        elif attr == "playing_time":
            try:
                self._update_progress(length=int(text))
            except ValueError:
                pass

    def _handle_avrcp_status(self, message):
        status_match = AVRCP_STATUS_RE.search(message)
        if not status_match:
            return
        self._set_playback_status(int(status_match.group("status")))
        self._update_progress(
            pos=int(status_match.group("pos")),
            length=int(status_match.group("length")),
        )

    def _handle_avrcp_notify(self, message):
        notify_match = AVRCP_NOTIFY_RE.search(message)
        if not notify_match:
            return
        event = int(notify_match.group("event"), 16)
        fields = self._parse_notify_fields(notify_match.group("fields"))
        if event == 1 and "playback" in fields:
            self._set_playback_status(fields["playback"])
        elif event == 5 and "pos" in fields:
            self._update_progress(pos=fields["pos"])
        elif event == 13 and "volume" in fields:
            self.screen_volume_var.set(f"Volume {fields['volume']}")

    def _handle_connection_state(self, message):
        a2dp_match = A2DP_CONNECTION_RE.search(message)
        if a2dp_match:
            self.a2dp_connected = int(a2dp_match.group("state")) == 2
            if not self.a2dp_connected:
                self.a2dp_rate_var.set("A2DP --")
            self._update_link_summary()
            return

        avrcp_match = AVRCP_CONNECTION_RE.search(message)
        if avrcp_match:
            self.avrcp_connected = int(avrcp_match.group("connected")) == 1
            self._update_link_summary()
            return

        hfp_match = HFP_CONNECTION_RE.search(message)
        if hfp_match:
            self.hfp_connected = int(hfp_match.group("state")) == 3
            self._update_link_summary()
            if not self.hfp_connected:
                self.hfp_call = 0
                self.hfp_call_setup = 0
                self.hfp_call_held = 0
                self.hfp_waiting_number = ""
                self.hfp_other_number = ""
                self.hfp_clcc.clear()
                self.hfp_clcc_pending.clear()
                self.hfp_clcc_collecting = False
            self._refresh_call_screen()

    def _handle_audio_rate(self, message):
        a2dp_match = A2DP_RATE_RE.search(message)
        if a2dp_match:
            codec = a2dp_match.group("codec")
            rate = int(a2dp_match.group("rate"))
            channels = a2dp_match.group("channels")
            text = f"A2DP {codec} {rate // 1000 if rate else 0} kHz {channels}"
            if a2dp_match.group("bitpool"):
                text += f" bitpool {a2dp_match.group('bitpool')}"
            self.a2dp_rate_var.set(text)
            return

        hfp_match = HFP_RATE_RE.search(message)
        if hfp_match:
            codec = hfp_match.group("codec")
            rate = int(hfp_match.group("rate"))
            frame = int(hfp_match.group("frame"))
            if rate:
                self.hfp_rate_var.set(f"HFP {codec} {rate // 1000} kHz mono frame {frame}")
            else:
                self.hfp_rate_var.set("HFP --")

    def _schedule_hf_clcc(self):
        if self.hfp_clcc_after_id is not None:
            self.after_cancel(self.hfp_clcc_after_id)
        self.hfp_clcc_after_id = self.after(250, self._auto_hf_clcc)

    def _auto_hf_clcc(self):
        self.hfp_clcc_after_id = None
        if self.worker and self.hfp_connected and self._call_activity_active():
            self._send("HF_CLCC")

    def _begin_clcc_update(self):
        self.hfp_clcc_pending = {}
        self.hfp_clcc_collecting = True

    def _finish_clcc_update(self):
        if not self.hfp_clcc_collecting:
            return
        self.hfp_clcc = dict(self.hfp_clcc_pending)
        self.hfp_clcc_pending.clear()
        self.hfp_clcc_collecting = False
        self._refresh_call_screen()

    def _call_activity_active(self):
        return self.hfp_call != 0 or self.hfp_call_setup != 0 or self.hfp_call_held != 0 or bool(self.hfp_clcc)

    def _call_status_name(self):
        if self.hfp_call_setup == 1:
            return "Incoming call"
        if self.hfp_call_setup == 2:
            return "Dialing"
        if self.hfp_call_setup == 3:
            return "Remote ringing"
        if self.hfp_call and self.hfp_call_held:
            return "Call active + held"
        if self.hfp_call:
            return "Call active"
        if self.hfp_call_held:
            return "Call held"
        return "No call"

    def _clcc_row_tag(self, status):
        if status == 0:
            return "active"
        if status == 1:
            return "held"
        if status == 4:
            return "incoming"
        if status == 5:
            return "waiting"
        return ""

    def _clcc_state_label(self, call):
        status = call["status"]
        label = CLCC_STATUS_NAMES.get(status, f"Status {status}")
        if call["mpty"]:
            label = f"{label} conf"
        return label

    def _call_display_rows(self):
        rows = []
        for idx, call in sorted(self.hfp_clcc.items()):
            rows.append(
                {
                    "label": f"Call {idx}",
                    "dir": "out" if call["dir"] == 0 else "in",
                    "status": call["status"],
                    "state": self._clcc_state_label(call),
                    "num": call["num"] or "--",
                    "synthetic": False,
                }
            )

        has_active = any(row["status"] == 0 for row in rows)
        has_held = any(row["status"] == 1 for row in rows)
        has_incoming = any(row["status"] in (4, 5) for row in rows)

        if self.hfp_call and not has_active and self.hfp_call_held != 2:
            rows.append(
                {
                    "label": "Call A",
                    "dir": "in",
                    "status": 0,
                    "state": "Active",
                    "num": self.hfp_last_number or "--",
                    "synthetic": True,
                }
            )

        if self.hfp_call_held and not has_held:
            rows.append(
                {
                    "label": "Call H",
                    "dir": "in",
                    "status": 1,
                    "state": "Held",
                    "num": self.hfp_other_number or self.hfp_waiting_number or self.hfp_last_number or "--",
                    "synthetic": True,
                }
            )

        if self.hfp_call_setup == 1 and not has_incoming:
            rows.append(
                {
                    "label": "Call W" if self.hfp_call else "Call",
                    "dir": "in",
                    "status": 5 if self.hfp_call else 4,
                    "state": "Waiting" if self.hfp_call else "Incoming",
                    "num": self.hfp_waiting_number or self.hfp_other_number or "--",
                    "synthetic": True,
                }
            )

        return rows

    def _refresh_call_screen(self):
        self.call_status_var.set(self._call_status_name())
        self.call_held_var.set(CALL_HELD_NAMES.get(self.hfp_call_held, f"Held status {self.hfp_call_held}"))
        rows = self._call_display_rows()

        number = self.hfp_waiting_number or self.hfp_last_number
        if not number and rows:
            active = next((row for row in rows if row["status"] in (0, 4, 5)), None)
            number = active["num"] if active else rows[0]["num"]
        self.call_number_var.set(f"Number {number}" if number else "Number --")

        if self.hfp_call_setup == 1:
            self.call_action_var.set("Accept or reject")
        elif self.hfp_call and self.hfp_call_held:
            self.call_action_var.set("Swap, merge, or end")
        elif self.hfp_call:
            self.call_action_var.set("End or hold")
        elif self.hfp_call_held:
            self.call_action_var.set("Resume or end")
        else:
            self.call_action_var.set("Idle")

        active_calls = [row["label"] for row in rows if row["status"] == 0]
        held_calls = [row["label"] for row in rows if row["status"] == 1]
        waiting_calls = [row["label"] for row in rows if row["status"] == 5]
        focus_parts = []
        if active_calls:
            focus_parts.append("Active: " + ", ".join(active_calls))
        if held_calls:
            focus_parts.append("Held: " + ", ".join(held_calls))
        if waiting_calls:
            focus_parts.append("Waiting: " + ", ".join(waiting_calls))
        self.call_focus_var.set(" | ".join(focus_parts))

        for item in self.call_list.get_children():
            self.call_list.delete(item)

        if rows:
            for row in rows:
                tag = self._clcc_row_tag(row["status"])
                self.call_list.insert(
                    "",
                    tk.END,
                    text=row["label"],
                    values=(row["state"], row["num"], row["dir"]),
                    tags=(tag,) if tag else (),
                )
        else:
            setup = CALL_SETUP_NAMES.get(self.hfp_call_setup, f"Setup {self.hfp_call_setup}")
            tag = "incoming" if self.hfp_call_setup == 1 else ""
            self.call_list.insert(
                "",
                tk.END,
                text="Call",
                values=(setup, self.hfp_last_number or self.hfp_waiting_number or "--", "--"),
                tags=(tag,) if tag else (),
            )

        if self._call_activity_active():
            self._show_call_screen()
        else:
            self._show_media_screen()

    def _handle_hfp_event(self, message):
        call_match = HFP_CIND_CALL_RE.search(message)
        if call_match:
            self.hfp_call = int(call_match.group("status"))
            if self.hfp_call == 0 and self.hfp_call_setup == 0 and self.hfp_call_held == 0:
                self.hfp_clcc.clear()
                self.hfp_clcc_pending.clear()
                self.hfp_clcc_collecting = False
                self.hfp_waiting_number = ""
                self.hfp_other_number = ""
            self._refresh_call_screen()
            self._schedule_hf_clcc()
            return

        setup_match = HFP_CIND_SETUP_RE.search(message)
        if setup_match:
            self.hfp_call_setup = int(setup_match.group("status"))
            if self.hfp_call_setup == 0 and self.hfp_call_held == 0:
                self.hfp_waiting_number = ""
            self._refresh_call_screen()
            self._schedule_hf_clcc()
            return

        held_match = HFP_CIND_HELD_RE.search(message)
        if held_match:
            self.hfp_call_held = int(held_match.group("status"))
            if self.hfp_call == 0 and self.hfp_call_setup == 0 and self.hfp_call_held == 0:
                self.hfp_clcc.clear()
                self.hfp_clcc_pending.clear()
                self.hfp_clcc_collecting = False
                self.hfp_waiting_number = ""
                self.hfp_other_number = ""
            self._refresh_call_screen()
            self._schedule_hf_clcc()
            return

        number_match = HFP_NUMBER_RE.search(message)
        if number_match:
            number = number_match.group("number").strip()
            if number:
                if "HFP_HF_CCWA:" in message:
                    self.hfp_waiting_number = number
                    self.hfp_other_number = number
                else:
                    self.hfp_last_number = number
            self._refresh_call_screen()
            self._schedule_hf_clcc()
            return

        clcc_match = HFP_CLCC_RE.search(message)
        if clcc_match:
            if not self.hfp_clcc_collecting:
                self._begin_clcc_update()
            idx = int(clcc_match.group("idx"))
            self.hfp_clcc_pending[idx] = {
                "dir": int(clcc_match.group("dir")),
                "status": int(clcc_match.group("status")),
                "mpty": int(clcc_match.group("mpty")),
                "num": clcc_match.group("num").strip(),
            }
            self.hfp_clcc = dict(self.hfp_clcc_pending)
            if self.hfp_clcc_pending:
                self.hfp_last_number = next(
                    (
                        call["num"]
                        for call in self.hfp_clcc_pending.values()
                        if call["status"] == 0 and call["num"]
                    ),
                    self.hfp_last_number,
                )
            self._refresh_call_screen()
            return

        if "HFP_HF_AT_RESPONSE:" in message:
            self._finish_clcc_update()
            return

        if "HFP_HF_RING" in message:
            self.hfp_call_setup = 1
            self._refresh_call_screen()
            self._schedule_hf_clcc()

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
        elif "DISCOVERED:" in message:
            self._add_discovered_device(message)
        elif "BT_FOUND:" in message:
            self._add_bt_found_device(message)
        elif "DEVICE_NAME:" in message or "DEVICE:" in message:
            self._add_device_from_line(message)
        elif "SCAN_STARTED" in message or "SCAN:STARTED" in message:
            self.scan_in_progress = True
            self._set_scan_controls()
            self.status_var.set("Scanning")
        elif "SCAN_START:" in message:
            self.status_var.set(message)
        elif "SCAN_STOP:" in message:
            self.scan_in_progress = False
            self._set_scan_controls()
            self.status_var.set(message)
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
            self._handle_connection_state(message)
            self.status_var.set(message.split("A2DP_CONNECTION:", 1)[1])
        elif "AVRCP_CT_CONNECTION:" in message:
            self._handle_connection_state(message)
            self.status_var.set(message)
        elif "HFP_HF_CONNECTION:" in message:
            self._handle_connection_state(message)
            self.status_var.set(message)
        elif "BT_SCAN_MODE:" in message:
            self.status_var.set(message)
        elif "BT_BOND_CLEAR:" in message or "BT_BOND_REMOVED:" in message:
            self.status_var.set(message)
        elif "A2DP_AUDIO_RATE:" in message or "HFP_HF_AUDIO_RATE:" in message:
            self._handle_audio_rate(message)
            self.status_var.set(message)
        elif message == "HOST_TX: HF_CLCC" or message == "UART_RX:HF_CLCC":
            self._begin_clcc_update()
            self.status_var.set(message)
        elif "HFP_HF_CLCC_AUTO_QUERY:err=0x0" in message:
            self._begin_clcc_update()
            self.status_var.set(message)
        elif "AVRCP_METADATA:" in message:
            self._handle_metadata(message)
            self.status_var.set(message)
        elif "AVRCP_STATUS:" in message:
            self._handle_avrcp_status(message)
            self.status_var.set(message)
        elif "AVRCP_NOTIFY:" in message:
            self._handle_avrcp_notify(message)
            self.status_var.set(message)
        elif "AUDIO_MODE:" in message:
            mode = message.split("AUDIO_MODE:", 1)[1]
            self._set_audio_mode(mode)
            self.status_var.set(mode)
        elif "HFP_AG_CALL:" in message:
            self.status_var.set(message.split("HFP_AG_CALL:", 1)[1])
        elif "HFP_HF_" in message:
            self._handle_hfp_event(message)
            self.status_var.set(message)

    def _add_discovered_device(self, line):
        match = DISCOVERED_RE.search(line)
        if not match:
            return
        self._upsert_device(match.group("bda").lower(), match.group("name").strip() or "unnamed")

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
                self.status_var.set("Serial error - reconnect")
            elif kind == "closed":
                self.worker = None
                self._refresh_ports()
                if self.want_serial_connected:
                    self.status_var.set("Serial dropped - retrying")
                    self.after(1000, self._connect)
        self.after(100, self._poll_events)

    def destroy(self):
        self._disconnect()
        super().destroy()


if __name__ == "__main__":
    App().mainloop()
