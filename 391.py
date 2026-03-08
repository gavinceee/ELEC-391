import sys
import time
import threading
import queue
import re
from collections import deque

# Forward to VOFA
import socket

import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ----------------------------
# Config (edit these)
# ----------------------------
DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200
READ_TIMEOUT_S = 0.2

# Plot history
HIST_N = 1000
UI_POLL_MS = 30

# VOFA
VOFA_IP      = "127.0.0.1"
VOFA_TX_PORT = 9000
VOFA_RX_PORT = 1346

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ----------------------------
# Physical key → motor degree mapping
# Measured: E3 = -1457 deg, E5 = -208 deg  (24 semitone steps, linear)
# ----------------------------
_KEY_DEGREES = {
    "E3":  -1457.00,
    "F3":  -1404.96,
    "F#3": -1352.92,
    "G3":  -1300.88,
    "G#3": -1248.83,
    "A3":  -1196.79,
    "A#3": -1144.75,
    "B3":  -1092.71,
    "C4":  -1040.67,
    "C#4":  -988.62,
    "D4":   -936.58,
    "D#4":  -884.54,
    "E4":   -832.50,
    "F4":   -780.46,
    "F#4":  -728.42,
    "G4":   -676.38,
    "G#4":  -624.33,
    "A4":   -572.29,
    "A#4":  -520.25,
    "B4":   -468.21,
    "C5":   -416.17,
    "C#5":  -364.12,
    "D5":   -312.08,
    "D#5":  -260.04,
    "E5":   -208.00,
}

# Ordered list of valid note names (E3 → E5)
_KEY_ORDER = list(_KEY_DEGREES.keys())

# Flat → sharp normalisation
_FLAT_MAP = {
    "DB": "C#", "EB": "D#", "GB": "F#", "AB": "G#", "BB": "A#",
    "DB3": "C#3", "EB3": "D#3", "GB3": "F#3", "AB3": "G#3", "BB3": "A#3",
    "DB4": "C#4", "EB4": "D#4", "GB4": "F#4", "AB4": "G#4", "BB4": "A#4",
    "DB5": "C#5", "EB5": "D#5", "GB5": "F#5", "AB5": "G#5", "BB5": "A#5",
}


def _normalise_note(note_name: str) -> str:
    """Upper-case and convert flats to sharps."""
    n = note_name.strip().upper()
    return _FLAT_MAP.get(n, n)


def note_to_angle(note_name: str) -> float:
    """Return motor degrees for a note in E3–E5.  Raises ValueError if out of range."""
    n = _normalise_note(note_name)
    if n not in _KEY_DEGREES:
        raise ValueError(f"Note {note_name!r} is outside the E3–E5 range. "
                         f"Valid keys: {', '.join(_KEY_ORDER)}")
    return _KEY_DEGREES[n]


def angle_to_note(degrees: float) -> str:
    """Return the closest note name for a given motor position."""
    closest = min(_KEY_DEGREES.items(), key=lambda kv: abs(kv[1] - degrees))
    return closest[0]


# ----------------------------
# Parsing  (updated format includes 'sw' field at the end)
# ----------------------------
LINE_RE = re.compile(
    r"Desired,\s*Actual,\s*Duty,\s*u,\s*P,\s*I,\s*D,\s*tau,\s*dir"
    r"(?:,\s*sw)?\s*:\s*"
    r"([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*"
    r"([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*([-\d]+)"
    r"(?:,\s*([-\d]+))?"
)


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# ─────────────────────────────────────────────────────────────────────────────
# Built-in songs  (all notes must be within E3–E5)
# ─────────────────────────────────────────────────────────────────────────────
SONGS = {
    "scale_up": [
        ("E3", 2), ("E4", 2),("E3", 2), ("E4", 2),("E3", 2), ("E4", 2),("E3", 2), ("E4", 2),("E3", 2), ("E4", 2),("E3", 2), ("E4", 2),
    ],
}


# ─────────────────────────────────────────────────────────────────────────────
# Sequencer thread
# ─────────────────────────────────────────────────────────────────────────────
class Sequencer(threading.Thread):
    def __init__(self, song, reader, out_q, stop_evt):
        super().__init__(daemon=True)
        self.song     = song
        self.reader   = reader
        self.out_q    = out_q
        self.stop_evt = stop_evt

    def run(self):
        total = len(self.song)
        for idx, (note, duration) in enumerate(self.song):
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", None))
                return

            try:
                angle = note_to_angle(note)
            except ValueError as e:
                self.out_q.put(("error", f"[Sequencer] {e}"))
                continue

            cmd = f"SP={angle:.2f}!"
            try:
                self.reader.write(cmd)
            except Exception as e:
                self.out_q.put(("error", f"[Sequencer] Serial write failed: {e}"))
                self.out_q.put(("seq_stop", None))
                return

            self.out_q.put(("seq_note", (idx, note, angle, total)))

            t_end = time.perf_counter() + duration
            sleep_bulk = duration - 0.005
            if sleep_bulk > 0:
                time.sleep(sleep_bulk)
            while time.perf_counter() < t_end:
                pass

        self.out_q.put(("seq_done", None))


# ─────────────────────────────────────────────────────────────────────────────
# Serial reader thread
# ─────────────────────────────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_q, stop_evt):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.out_q, self.stop_evt = out_q, stop_evt
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baud,
                                 timeout=READ_TIMEOUT_S,
                                 write_timeout=READ_TIMEOUT_S)
        time.sleep(0.2)

    def run(self):
        try:
            self.connect()
            self.out_q.put(("status", f"Connected to {self.port} @ {self.baud}\n"))
        except Exception as e:
            self.out_q.put(("error", f"Failed to open {self.port}: {e}\n"))
            return

        buf = b""
        while not self.stop_evt.is_set():
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip(b"\r")
                    try:
                        text = line.decode("utf-8", errors="replace")
                        tx_sock.sendto((text + "\n").encode(), (VOFA_IP, VOFA_TX_PORT))
                    except Exception:
                        text = str(line)
                    self.out_q.put(("line", text))
            except Exception as e:
                self.out_q.put(("error", f"Serial read error: {e}\n"))
                break

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.out_q.put(("status", "Disconnected.\n"))

    def write(self, s: str):
        if self.ser and self.ser.is_open:
            self.ser.write(s.encode("utf-8"))


# ─────────────────────────────────────────────────────────────────────────────
# VOFA listener thread
# ─────────────────────────────────────────────────────────────────────────────
class VOFAListener(threading.Thread):
    def __init__(self, out_q, stop_evt):
        super().__init__(daemon=True)
        self.out_q, self.stop_evt = out_q, stop_evt
        self.reader_ref = None
        self._sock = None

    def run(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind(("0.0.0.0", VOFA_RX_PORT))
            self._sock.settimeout(0.5)
            self.out_q.put(("status", f"VOFA listener started on UDP:{VOFA_RX_PORT}\n"))
        except Exception as e:
            self.out_q.put(("error", f"VOFA listener failed to bind UDP:{VOFA_RX_PORT}: {e}\n"))
            return

        while not self.stop_evt.is_set():
            try:
                data, addr = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_evt.is_set():
                    self.out_q.put(("error", f"VOFA listener recv error: {e}\n"))
                break

            try:
                text = data.decode("utf-8", errors="replace").strip()
            except Exception:
                text = str(data)

            if not text:
                continue

            if self.reader_ref and self.reader_ref.is_alive():
                try:
                    cmd = text if (text.endswith("!") or text.endswith("\n")) else text + "\n"
                    self.reader_ref.write(cmd)
                    self.out_q.put(("vofa_cmd", f"[VOFA→MCU] {text}"))
                except Exception as e:
                    self.out_q.put(("error", f"Failed to forward VOFA cmd to MCU: {e}\n"))
            else:
                self.out_q.put(("vofa_cmd", f"[VOFA→MCU] (no serial) {text}"))

        try:
            self._sock.close()
        except Exception:
            pass

    def stop(self):
        self.stop_evt.set()


# ─────────────────────────────────────────────────────────────────────────────
# Main App
# ─────────────────────────────────────────────────────────────────────────────
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("UART Monitor + Piano Sequencer")
        self.geometry("1200x750")

        self.q        = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader   = None

        self._seq_thread   = None
        self._seq_stop_evt = threading.Event()

        self.t    = deque(maxlen=HIST_N)
        self.des  = deque(maxlen=HIST_N)
        self.act  = deque(maxlen=HIST_N)
        self.duty = deque(maxlen=HIST_N)
        self.u    = deque(maxlen=HIST_N)
        self.t0   = time.time()

        self._build_ui()

        self.vofa_listener = VOFAListener(self.q, threading.Event())
        self.vofa_listener.start()

        self.after(UI_POLL_MS, self._poll_queue)

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=12,
                                       values=list_serial_ports())
        self.port_combo.pack(side=tk.LEFT, padx=4)

        ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(10, 0))
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        ttk.Entry(top, textvariable=self.baud_var, width=10).pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="Refresh ports", command=self._refresh_ports).pack(side=tk.LEFT, padx=(10, 0))
        self.btn_connect = ttk.Button(top, text="Connect", command=self._connect)
        self.btn_connect.pack(side=tk.LEFT, padx=6)
        self.btn_disconnect = ttk.Button(top, text="Disconnect", command=self._disconnect, state=tk.DISABLED)
        self.btn_disconnect.pack(side=tk.LEFT, padx=6)

        ttk.Label(top, text="Send:").pack(side=tk.LEFT, padx=(16, 0))
        self.send_var = tk.StringVar()
        self.send_entry = ttk.Entry(top, textvariable=self.send_var, width=24)
        self.send_entry.pack(side=tk.LEFT, padx=4)
        self.send_entry.bind("<Return>", lambda e: self._send())
        self.btn_send = ttk.Button(top, text="Send", command=self._send, state=tk.DISABLED)
        self.btn_send.pack(side=tk.LEFT, padx=4)

        self.auto_newline = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Append newline", variable=self.auto_newline).pack(side=tk.LEFT, padx=(10, 0))

        self.status_var = tk.StringVar(value="Idle.")
        ttk.Label(self, textvariable=self.status_var, anchor="w").pack(
            side=tk.BOTTOM, fill=tk.X, padx=8, pady=4)

        nb = ttk.Notebook(self)
        nb.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=4)

        monitor_tab   = ttk.Frame(nb)
        sequencer_tab = ttk.Frame(nb)
        nb.add(monitor_tab,   text="  Monitor  ")
        nb.add(sequencer_tab, text="  Sequencer  ")

        self._build_monitor_tab(monitor_tab)
        self._build_sequencer_tab(sequencer_tab)

    def _build_monitor_tab(self, parent):
        main = ttk.Panedwindow(parent, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        main.add(left, weight=1)

        note_frame = ttk.Labelframe(left, text="Current Note")
        note_frame.pack(side=tk.TOP, fill=tk.X, padx=2, pady=(0, 4))

        self.note_var  = tk.StringVar(value="—")
        self.angle_var = tk.StringVar(value="angle: —")

        tk.Label(note_frame, textvariable=self.note_var,
                 font=("Helvetica", 56, "bold"), fg="#1a1a2e", bg="#e8f4f8",
                 width=6, anchor="center", relief=tk.FLAT, pady=10).pack(fill=tk.X, padx=6, pady=(6, 2))
        tk.Label(note_frame, textvariable=self.angle_var,
                 font=("Helvetica", 11), fg="#555555", bg="#e8f4f8",
                 anchor="center").pack(fill=tk.X, padx=6, pady=(0, 8))

        table_frame = ttk.Labelframe(left, text="Live Values")
        table_frame.pack(side=tk.TOP, fill=tk.X, padx=2, pady=(0, 4))

        _FIELDS = ["Desired", "Actual", "Duty", "u", "P", "I", "D", "tau", "dir", "sw"]
        self._table_vars = {}
        for i, field in enumerate(_FIELDS):
            ttk.Label(table_frame, text=field, width=8, anchor="e",
                      font=("Helvetica", 9, "bold")).grid(row=i, column=0, sticky="e", padx=(6, 2), pady=1)
            var = tk.StringVar(value="--")
            self._table_vars[field] = var
            ttk.Label(table_frame, textvariable=var, width=12, anchor="w",
                      font=("Courier", 10), foreground="#003366").grid(row=i, column=1, sticky="w", padx=(2, 6), pady=1)

        log_frame = ttk.Labelframe(left, text="Raw UART")
        log_frame.pack(side=tk.TOP, fill=tk.X, padx=2)
        self.log = tk.Text(log_frame, wrap=tk.NONE, height=3,
                           font=("Courier", 8), foreground="#444444")
        self.log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.log.tag_configure("vofa", foreground="#0088cc")

        plot_frame = ttk.Labelframe(main, text="Live Plot")
        main.add(plot_frame, weight=1)

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax1 = self.fig.add_subplot(111)
        self.ax1.set_title("Angle")
        self.ax1.set_xlabel("t (s)")
        self.ax1.set_ylabel("deg")
        self.l_des, = self.ax1.plot([], [], label="Desired")
        self.l_act, = self.ax1.plot([], [], label="Actual")
        self.ax1.legend(loc="upper right")
        self.ax1.set_ylim(-1500, 0)   # physical range E3..E5
        self.ax1.autoscale(enable=False, axis="y")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_sequencer_tab(self, parent):
        ctrl = ttk.Frame(parent)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=12, pady=10)

        ttk.Label(ctrl, text="Song", font=("Helvetica", 11, "bold")).pack(anchor="w")
        self.song_var = tk.StringVar(value=list(SONGS.keys())[0])
        song_menu = ttk.Combobox(ctrl, textvariable=self.song_var,
                                 values=list(SONGS.keys()), state="readonly", width=22)
        song_menu.pack(anchor="w", pady=(2, 10))
        song_menu.bind("<<ComboboxSelected>>", lambda e: self._seq_load_song())

        ttk.Label(ctrl, text="Tempo scale", font=("Helvetica", 10)).pack(anchor="w")
        self.tempo_scale = tk.DoubleVar(value=1.0)
        tempo_slider = ttk.Scale(ctrl, variable=self.tempo_scale, from_=0.25, to=3.0,
                                 orient=tk.HORIZONTAL, length=180)
        tempo_slider.pack(anchor="w", pady=(2, 0))
        self.tempo_label = ttk.Label(ctrl, text="1.00×  (100%)")
        self.tempo_label.pack(anchor="w", pady=(0, 10))
        tempo_slider.config(command=self._on_tempo_change)

        btn_row = ttk.Frame(ctrl)
        btn_row.pack(anchor="w", pady=6)
        self.btn_play = ttk.Button(btn_row, text="▶  Play", width=12, command=self._seq_play)
        self.btn_play.pack(side=tk.LEFT, padx=(0, 6))
        self.btn_stop_seq = ttk.Button(btn_row, text="■  Stop", width=12,
                                       command=self._seq_stop, state=tk.DISABLED)
        self.btn_stop_seq.pack(side=tk.LEFT)

        ttk.Label(ctrl, text="Progress", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(12, 2))
        self.seq_progress = ttk.Progressbar(ctrl, length=180, mode="determinate")
        self.seq_progress.pack(anchor="w")
        self.seq_progress_lbl = ttk.Label(ctrl, text="—", font=("Courier", 9))
        self.seq_progress_lbl.pack(anchor="w", pady=(2, 10))

        self.seq_note_var = tk.StringVar(value="—")
        ttk.Label(ctrl, text="Playing", font=("Helvetica", 10, "bold")).pack(anchor="w")
        tk.Label(ctrl, textvariable=self.seq_note_var,
                 font=("Helvetica", 48, "bold"), fg="#1a2e5a", bg="#ddeeff",
                 width=6, anchor="center", relief=tk.GROOVE, pady=8).pack(fill=tk.X, pady=4)

        # Key range label
        ttk.Label(ctrl, text="Range: E3 – E5  (25 keys)",
                  font=("Helvetica", 8), foreground="#666666").pack(anchor="w", pady=(8, 0))

        score_frame = ttk.Labelframe(parent, text="Song Score")
        score_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8, pady=10)

        sb = ttk.Scrollbar(score_frame)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.score_list = tk.Listbox(score_frame, yscrollcommand=sb.set,
                                     font=("Courier", 10), selectmode=tk.BROWSE,
                                     activestyle="dotbox", width=40)
        self.score_list.pack(fill=tk.BOTH, expand=True)
        sb.config(command=self.score_list.yview)

        edit_frame = ttk.Labelframe(parent, text="Custom Song Editor")
        edit_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8), pady=10)

        ttk.Label(edit_frame,
                  text="One note per line:  NOTE  DURATION_S\n"
                       "Valid range: E3 to E5\n"
                       "Example:   C4  0.5\n"
                       "           G4  1.0\n"
                       "           E4  0.5",
                  font=("Courier", 9), foreground="#555555",
                  justify=tk.LEFT).pack(anchor="w", padx=6, pady=(4, 2))

        self.custom_text = tk.Text(edit_frame, font=("Courier", 10), height=20, width=24)
        self.custom_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=4)

        btn_custom_row = ttk.Frame(edit_frame)
        btn_custom_row.pack(anchor="w", padx=6, pady=(0, 6))
        ttk.Button(btn_custom_row, text="Load Custom", command=self._seq_load_custom).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(btn_custom_row, text="Clear", command=lambda: self.custom_text.delete("1.0", tk.END)).pack(side=tk.LEFT)

        self._seq_load_song()

    # ── Song loading ──────────────────────────────────────────────────────────
    def _seq_load_song(self):
        name = self.song_var.get()
        self._current_song = SONGS[name]
        self._populate_score(self._current_song)

    def _seq_load_custom(self):
        raw = self.custom_text.get("1.0", tk.END).strip()
        if not raw:
            messagebox.showwarning("Empty", "Type some notes first.")
            return
        song = []
        errors = []
        for lineno, line in enumerate(raw.splitlines(), 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 2:
                errors.append(f"Line {lineno}: expected NOTE DURATION, got {line!r}")
                continue
            note, dur_str = parts
            try:
                note_to_angle(note)
                dur = float(dur_str)
                song.append((note, dur))
            except Exception as e:
                errors.append(f"Line {lineno}: {e}")

        if errors:
            messagebox.showerror("Parse errors", "\n".join(errors))
            return
        if not song:
            messagebox.showwarning("Empty", "No valid notes found.")
            return

        self._current_song = song
        self._populate_score(song)
        self.song_var.set("(custom)")
        self.status_var.set(f"Custom song loaded: {len(song)} notes.")

    def _populate_score(self, song):
        self.score_list.delete(0, tk.END)
        for i, (note, dur) in enumerate(song):
            try:
                angle = note_to_angle(note)
                self.score_list.insert(tk.END, f"  {i+1:>3}.  {note:<6}  {dur:.2f}s   ({angle:+.1f}°)")
            except Exception:
                self.score_list.insert(tk.END, f"  {i+1:>3}.  {note:<6}  {dur:.2f}s   (??)")

    # ── Tempo ─────────────────────────────────────────────────────────────────
    def _on_tempo_change(self, _=None):
        v = self.tempo_scale.get()
        self.tempo_label.config(text=f"{v:.2f}×  ({int(v*100)}%)")

    def _scaled_song(self):
        scale = self.tempo_scale.get()
        return [(note, dur / scale) for note, dur in self._current_song]

    # ── Play / Stop ───────────────────────────────────────────────────────────
    def _seq_play(self):
        if not (self.reader and self.reader.is_alive()):
            messagebox.showwarning("Not connected", "Connect to serial port first.")
            return
        if self._seq_thread and self._seq_thread.is_alive():
            return
        self._seq_stop_evt.clear()
        song = self._scaled_song()
        self._seq_thread = Sequencer(song, self.reader, self.q, self._seq_stop_evt)
        self._seq_thread.start()
        self.btn_play.config(state=tk.DISABLED)
        self.btn_stop_seq.config(state=tk.NORMAL)
        self.seq_progress["maximum"] = len(song)
        self.seq_progress["value"] = 0
        self.seq_progress_lbl.config(text=f"0 / {len(song)}")
        self.seq_note_var.set("…")
        self.status_var.set(f"▶ Playing: {self.song_var.get()}")

    def _seq_stop(self):
        self._seq_stop_evt.set()
        self.btn_stop_seq.config(state=tk.DISABLED)
        self.btn_play.config(state=tk.NORMAL)
        self.seq_note_var.set("—")
        self.status_var.set("■ Stopped.")

    # ── Serial ────────────────────────────────────────────────────────────────
    def _refresh_ports(self):
        self.port_combo["values"] = list_serial_ports()

    def _connect(self):
        if self.reader and self.reader.is_alive():
            return
        port = self.port_var.get().strip()
        baud = int(self.baud_var.get())
        self.stop_evt.clear()
        self.reader = SerialReader(port, baud, self.q, self.stop_evt)
        self.reader.start()
        self.vofa_listener.reader_ref = self.reader
        self.btn_connect.config(state=tk.DISABLED)
        self.btn_disconnect.config(state=tk.NORMAL)
        self.btn_send.config(state=tk.NORMAL)

    def _disconnect(self):
        self._seq_stop()
        self.stop_evt.set()
        self.vofa_listener.reader_ref = None
        self.btn_disconnect.config(state=tk.DISABLED)
        self.btn_send.config(state=tk.DISABLED)
        self.btn_connect.config(state=tk.NORMAL)

    def _send(self):
        if not (self.reader and self.reader.is_alive()):
            return
        s = self.send_var.get()
        if not s:
            return
        if self.auto_newline.get() and not s.endswith("\n"):
            s += "\n"
        try:
            self.reader.write(s)
            self._append_log(f">> {s.strip()}\n")
            self.send_var.set("")
        except Exception as e:
            messagebox.showerror("Send failed", str(e))

    # ── Log ───────────────────────────────────────────────────────────────────
    def _append_log(self, s, tag=None):
        if tag:
            self.log.insert(tk.END, s, tag)
        else:
            self.log.insert(tk.END, s)
        lines = int(self.log.index(tk.END).split(".")[0]) - 1
        if lines > 3:
            self.log.delete("1.0", f"{lines - 3}.0")
        self.log.see(tk.END)

    # ── Queue polling ─────────────────────────────────────────────────────────
    def _handle_line(self, line):
        self._append_log(line + "\n")
        m = LINE_RE.search(line)
        if not m:
            return
        desired = float(m.group(1))
        actual  = float(m.group(2))
        duty    = float(m.group(3))
        u_val   = float(m.group(4))
        p_val   = float(m.group(5))
        i_val   = float(m.group(6))
        d_val   = float(m.group(7))
        tau_val = float(m.group(8))
        dir_val = int(m.group(9))
        sw_val  = int(m.group(10)) if m.group(10) is not None else 0

        t_now = time.time() - self.t0
        self.t.append(t_now)
        self.des.append(desired)
        self.act.append(actual)
        self.duty.append(duty)
        self.u.append(u_val)

        for field, val in zip(
            ["Desired", "Actual", "Duty", "u", "P", "I", "D", "tau", "dir", "sw"],
            [f"{desired:.2f}", f"{actual:.2f}", f"{duty:.3f}", f"{u_val:.3f}",
             f"{p_val:.5f}", f"{i_val:.5f}", f"{d_val:.5f}", f"{tau_val:.5f}",
             str(dir_val), str(sw_val)]
        ):
            self._table_vars[field].set(val)

        note = angle_to_note(actual)
        if note != self.note_var.get():
            self.note_var.set(note)
        self.angle_var.set(f"angle: {actual:.1f}°")

    def _update_plot(self):
        if len(self.t) < 2:
            return
        t = list(self.t)
        self.l_des.set_data(t, list(self.des))
        self.l_act.set_data(t, list(self.act))
        self.ax1.relim()
        self.ax1.autoscale_view(scaley=False)
        self.canvas.draw_idle()

    def _poll_queue(self):
        updated = False
        try:
            while True:
                kind, payload = self.q.get_nowait()

                if kind == "line":
                    self._handle_line(payload)
                    updated = True
                elif kind == "vofa_cmd":
                    self._append_log(payload + "\n", tag="vofa")
                elif kind == "status":
                    self.status_var.set(payload.strip())
                    self._append_log(payload)
                elif kind == "error":
                    self.status_var.set(payload.strip())
                    self._append_log(payload)
                elif kind == "seq_note":
                    idx, note, angle, total = payload
                    self.seq_note_var.set(note)
                    self.seq_progress["value"] = idx + 1
                    self.seq_progress_lbl.config(text=f"{idx+1} / {total}  ({note}  {angle:+.1f}°)")
                    self.score_list.selection_clear(0, tk.END)
                    self.score_list.selection_set(idx)
                    self.score_list.see(idx)
                elif kind == "seq_done":
                    self.btn_play.config(state=tk.NORMAL)
                    self.btn_stop_seq.config(state=tk.DISABLED)
                    self.seq_note_var.set("✓")
                    self.status_var.set("Song finished.")
                elif kind == "seq_stop":
                    self.btn_play.config(state=tk.NORMAL)
                    self.btn_stop_seq.config(state=tk.DISABLED)

        except queue.Empty:
            pass

        if updated:
            self._update_plot()

        self.after(UI_POLL_MS, self._poll_queue)

    def on_close(self):
        self._seq_stop()
        self.stop_evt.set()
        if self.vofa_listener:
            self.vofa_listener.stop()
        self.destroy()


if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
