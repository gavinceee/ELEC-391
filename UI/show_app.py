import os
import sys
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import filedialog
from Song.midi_loader import load_song_from_midi

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Communication.uart import SerialReader, VOFAListener, list_serial_ports, LINE_RE
from Core_function.note import angle_to_note
from Core_function.sequencer import *
from Song.song import SONGS
from UI.piano_roll import PianoRollView

UI_POLL_MS = 30
DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200


class ShowApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Automatic Piano Player - Show UI")
        self.geometry("1500x900")
        self.minsize(1280, 780)
        self.configure(bg="#0b0f14")

        self.q = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader = None

        self.vofa_listener = VOFAListener(self.q, threading.Event())
        self.vofa_listener.start()

        self._seq_thread = None
        self._seq_stop_evt = threading.Event()
        self._seq_pause_evt = threading.Event()

        self.latest_actual = None
        self.latest_desired = None
        self.latest_sw = 0
        self.latest_dir = 0

        self._current_song = []
        self._current_song_name = list(SONGS.keys())[0] if SONGS else "(empty)"
        self._play_state = "Stopped"
        self._current_note_name = "—"
        self._current_time_s = 0.0
        self._song_total_s = 1.0
        self._connected = False
        self._pressed_note = None

        self._build_ui()
        self._seq_load_song()

        self.after(UI_POLL_MS, self._poll_queue)

    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass

        root = tk.Frame(self, bg="#0b0f14")
        root.pack(fill=tk.BOTH, expand=True)

        top = tk.Frame(root, bg="#0b0f14")
        top.pack(side=tk.TOP, fill=tk.X, padx=14, pady=(12, 8))

        conn_box = tk.Frame(top, bg="#11161e")
        conn_box.pack(side=tk.LEFT, padx=(0, 12), ipadx=8, ipady=8)

        self.conn_dot = tk.Canvas(conn_box, width=18, height=18, bg="#11161e", highlightthickness=0)
        self.conn_dot.pack(side=tk.LEFT, padx=(4, 6))
        self._draw_conn_dot("#ff5f56")

        self.conn_text_var = tk.StringVar(value="Disconnected")
        tk.Label(conn_box, textvariable=self.conn_text_var, fg="#dfe7f2", bg="#11161e",
                 font=("Segoe UI", 11, "bold")).pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(conn_box, text="Port", fg="#9eb0c6", bg="#11161e", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        self.port_combo = ttk.Combobox(conn_box, textvariable=self.port_var, values=list_serial_ports(), width=10)
        self.port_combo.pack(side=tk.LEFT, padx=(6, 10))

        tk.Label(conn_box, text="Baud", fg="#9eb0c6", bg="#11161e", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.baud_var = tk.IntVar(value=DEFAULT_BAUD)
        self.baud_entry = ttk.Entry(conn_box, textvariable=self.baud_var, width=10)
        self.baud_entry.pack(side=tk.LEFT, padx=(6, 10))

        ttk.Button(conn_box, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT, padx=4)
        self.btn_connect = ttk.Button(conn_box, text="Connect", command=self._connect)
        self.btn_connect.pack(side=tk.LEFT, padx=4)
        self.btn_disconnect = ttk.Button(conn_box, text="Disconnect", command=self._disconnect, state=tk.DISABLED)
        self.btn_disconnect.pack(side=tk.LEFT, padx=4)

        song_box = tk.Frame(top, bg="#11161e")
        song_box.pack(side=tk.LEFT, padx=(0, 12), ipadx=8, ipady=8)

        tk.Label(song_box, text="Song", fg="#9eb0c6", bg="#11161e", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.song_var = tk.StringVar(value=self._current_song_name)
        self.song_menu = ttk.Combobox(
            song_box,
            textvariable=self.song_var,
            values=list(SONGS.keys()),
            state="readonly",
            width=24
        )
        self.song_menu.pack(side=tk.LEFT, padx=(6, 10))
        self.song_menu.bind("<<ComboboxSelected>>", lambda e: self._seq_load_song())

        tk.Label(song_box, text="Tempo", fg="#9eb0c6", bg="#11161e", font=("Segoe UI", 10)).pack(side=tk.LEFT)
        self.tempo_var = tk.DoubleVar(value=1.0)
        self.tempo_scale = ttk.Scale(
            song_box, variable=self.tempo_var, from_=0.25, to=3.0,
            orient=tk.HORIZONTAL, length=130, command=self._on_tempo_change
        )
        self.tempo_scale.pack(side=tk.LEFT, padx=(6, 8))

        self.tempo_lbl = tk.Label(song_box, text="1.00×", fg="#dfe7f2", bg="#11161e", font=("Consolas", 10))
        self.tempo_lbl.pack(side=tk.LEFT)

        playback_box = tk.Frame(top, bg="#11161e")
        playback_box.pack(side=tk.LEFT, padx=(0, 12), ipadx=8, ipady=8)

        self.btn_play = ttk.Button(playback_box, text="Play", command=self._seq_play, width=10)
        self.btn_play.pack(side=tk.LEFT, padx=4)

        self.btn_pause = ttk.Button(playback_box, text="Pause", command=self._seq_pause_toggle, width=10, state=tk.DISABLED)
        self.btn_pause.pack(side=tk.LEFT, padx=4)

        self.btn_stop = ttk.Button(playback_box, text="Stop", command=self._seq_stop, width=10, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=4)

        right_box = tk.Frame(top, bg="#11161e")
        right_box.pack(side=tk.RIGHT, ipadx=10, ipady=8)

        self.status_var = tk.StringVar(value="Stopped")
        self.now_song_var = tk.StringVar(value="Song: —")
        self.now_note_var = tk.StringVar(value="Note: —")
        self.now_time_var = tk.StringVar(value="00:00.00 / 00:00.00")

        tk.Label(right_box, textvariable=self.status_var, fg="#89f0dd", bg="#11161e",
                 font=("Segoe UI", 12, "bold")).pack(anchor="e")
        tk.Label(right_box, textvariable=self.now_song_var, fg="#dfe7f2", bg="#11161e",
                 font=("Segoe UI", 10)).pack(anchor="e")
        tk.Label(right_box, textvariable=self.now_note_var, fg="#dfe7f2", bg="#11161e",
                 font=("Segoe UI", 10)).pack(anchor="e")
        tk.Label(right_box, textvariable=self.now_time_var, fg="#9eb0c6", bg="#11161e",
                 font=("Consolas", 10)).pack(anchor="e")

        progress_wrap = tk.Frame(root, bg="#0b0f14")
        progress_wrap.pack(fill=tk.X, padx=14, pady=(0, 8))

        self.progress = ttk.Progressbar(progress_wrap, mode="determinate")
        self.progress.pack(fill=tk.X, expand=True)

        content = tk.Frame(root, bg="#0b0f14")
        content.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 14))

        left = tk.Frame(content, bg="#0b0f14")
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.piano_view = PianoRollView(left, start_note="C3", octaves=4)
        self.piano_view.pack(fill=tk.BOTH, expand=True)

        right = tk.Frame(content, bg="#11161e", width=290)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        right.pack_propagate(False)

        tk.Label(right, text="Machine State", bg="#11161e", fg="#dfe7f2",
                 font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=14, pady=(16, 12))

        self.machine_vars = {}
        fields = [
            ("Desired", "—"),
            ("Actual", "—"),
            ("Direction", "—"),
            ("Switch", "—"),
            ("Arm Note", "—"),
            ("Current Song", "—"),
            ("Play State", "Stopped"),
        ]
        for key, default in fields:
            row = tk.Frame(right, bg="#11161e")
            row.pack(fill=tk.X, padx=14, pady=4)

            tk.Label(row, text=key, width=12, anchor="w",
                     bg="#11161e", fg="#9eb0c6", font=("Segoe UI", 10)).pack(side=tk.LEFT)

            var = tk.StringVar(value=default)
            self.machine_vars[key] = var
            tk.Label(row, textvariable=var, anchor="e",
                     bg="#11161e", fg="#dfe7f2", font=("Consolas", 10, "bold")).pack(side=tk.RIGHT)

        editor_frame = tk.Frame(right, bg="#11161e")
        editor_frame.pack(fill=tk.BOTH, expand=True, padx=14, pady=(16, 16))

        tk.Label(editor_frame, text="Custom Song Editor", bg="#11161e", fg="#dfe7f2",
                 font=("Segoe UI", 12, "bold")).pack(anchor="w")

        tk.Label(
            editor_frame,
            text="One note per line: NOTE DURATION\nExample:\nC4 0.5\nG4 1.0\nREST 0.2",
            justify=tk.LEFT,
            bg="#11161e", fg="#9eb0c6", font=("Consolas", 9)
        ).pack(anchor="w", pady=(6, 8))

        self.custom_text = tk.Text(
            editor_frame,
            height=16,
            bg="#0c1016",
            fg="#dfe7f2",
            insertbackground="#ffffff",
            relief=tk.FLAT,
            font=("Consolas", 10)
        )
        self.custom_text.pack(fill=tk.BOTH, expand=True)

        btns = tk.Frame(editor_frame, bg="#11161e")
        btns.pack(fill=tk.X, pady=(8, 0))

        ttk.Button(btns, text="Load Custom", command=self._seq_load_custom).pack(side=tk.LEFT)
        ttk.Button(btns, text="Load MIDI", command=self._load_midi_file).pack(side=tk.LEFT, padx=8)
        ttk.Button(btns, text="Clear", command=lambda: self.custom_text.delete("1.0", tk.END)).pack(side=tk.LEFT)

    def _draw_conn_dot(self, color):
        self.conn_dot.delete("all")
        self.conn_dot.create_oval(3, 3, 15, 15, fill=color, outline="")

    def _fmt_time(self, s: float) -> str:
        s = max(0.0, float(s))
        m = int(s // 60)
        sec = s - m * 60
        return f"{m:02d}:{sec:05.2f}"

    def _set_play_state(self, text: str):
        self._play_state = text
        self.status_var.set(text)
        self.machine_vars["Play State"].set(text)
        self.piano_view.set_status(text, paused=self._seq_pause_evt.is_set())

    def _update_header_labels(self):
        self.now_song_var.set(f"Song: {self._current_song_name}")
        self.now_note_var.set(f"Note: {self._current_note_name}")
        self.now_time_var.set(f"{self._fmt_time(self._current_time_s)} / {self._fmt_time(self._song_total_s)}")
        self.machine_vars["Current Song"].set(self._current_song_name)

        ratio = 0.0 if self._song_total_s <= 1e-9 else min(1.0, self._current_time_s / self._song_total_s)
        self.progress["maximum"] = 1000
        self.progress["value"] = ratio * 1000

    def _update_machine_note(self):
        if self.latest_actual is None:
            arm_note = "—"
        else:
            try:
                arm_note = angle_to_note(self.latest_actual)
            except Exception:
                arm_note = "—"

        self.machine_vars["Arm Note"].set(arm_note)
        self.piano_view.set_arm_note(arm_note if arm_note != "—" else None)

    def _seq_load_song(self):
        name = self.song_var.get()
        if name not in SONGS:
            return

        self._current_song_name = name
        self._current_song = [(str(n).strip().upper(), float(d)) for n, d in SONGS[name]]
        self._song_total_s = sum(max(0.01, float(dur)) for _, dur in self._current_song)
        self._current_time_s = 0.0
        self._current_note_name = "—"

        self.piano_view.set_song(self._current_song_name, self._current_song)
        self._update_header_labels()
        self._set_play_state("Stopped")

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
                errors.append(f"Line {lineno}: expected NOTE DURATION")
                continue

            note, dur_str = parts
            try:
                dur = float(dur_str)
                song.append((note.strip().upper(), dur))
            except Exception as e:
                errors.append(f"Line {lineno}: {e}")

        if errors:
            messagebox.showerror("Parse errors", "\n".join(errors))
            return

        if not song:
            messagebox.showwarning("Empty", "No valid notes found.")
            return

        self._current_song_name = "(custom)"
        self._current_song = song
        self._song_total_s = sum(max(0.01, float(d)) for _, d in self._current_song)
        self._current_time_s = 0.0
        self._current_note_name = "—"

        self.piano_view.set_song(self._current_song_name, self._current_song)
        self._update_header_labels()
        self._set_play_state("Stopped")

    def _scaled_song(self):
        scale = max(0.25, float(self.tempo_var.get()))
        return [(note, dur / scale) for note, dur in self._current_song]

    def _on_tempo_change(self, _=None):
        self.tempo_lbl.config(text=f"{self.tempo_var.get():.2f}×")

    def _seq_play(self):
        if not (self.reader and self.reader.is_alive()):
            messagebox.showwarning("Not connected", "Connect to serial port first.")
            return

        if self._seq_thread and self._seq_thread.is_alive() and self._seq_pause_evt.is_set():
            self._seq_pause_evt.clear()
            self.btn_pause.config(text="Pause")
            self._set_play_state("Playing")
            return

        if self._seq_thread and self._seq_thread.is_alive():
            return

        if not self._current_song:
            messagebox.showwarning("No song", "Load a song first.")
            return

        self._seq_stop_evt.clear()
        self._seq_pause_evt.clear()
        scaled_song = self._scaled_song()

        self._song_total_s = sum(max(0.01, float(d)) for _, d in scaled_song)
        self._current_time_s = 0.0
        self._current_note_name = "—"
        self._pressed_note = None

        self.piano_view.set_song(self._current_song_name, scaled_song)
        self._update_header_labels()

        self._seq_thread = Sequencer(
            song=scaled_song,
            reader=self.reader,
            out_q=self.q,
            stop_evt=self._seq_stop_evt,
            pause_evt=self._seq_pause_evt,
            get_actual_fn=lambda: self.latest_actual,
        )
        self._seq_thread.start()

        self.btn_pause.config(state=tk.NORMAL, text="Pause")
        self.btn_stop.config(state=tk.NORMAL)
        self._set_play_state("Playing")

    def _seq_pause_toggle(self):
        if not (self._seq_thread and self._seq_thread.is_alive()):
            return

        if self._seq_pause_evt.is_set():
            self._seq_pause_evt.clear()
            self.btn_pause.config(text="Pause")
            self._set_play_state("Playing")
        else:
            self._seq_pause_evt.set()
            self.btn_pause.config(text="Resume")
            self._set_play_state("Paused")

    def _seq_stop(self):
        self._seq_stop_evt.set()
        self._seq_pause_evt.clear()

        try:
            if self.reader and self.reader.is_alive():
                self.reader.write("SL=0!")
        except Exception:
            pass

        self._pressed_note = None
        self.piano_view.set_pressed_note(None)

        self.btn_pause.config(text="Pause", state=tk.DISABLED)
        self.btn_stop.config(state=tk.DISABLED)
        self._set_play_state("Stopped")

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

    def _disconnect(self):
        self._seq_stop()
        self.stop_evt.set()
        self.vofa_listener.reader_ref = None
        self.btn_disconnect.config(state=tk.DISABLED)
        self.btn_connect.config(state=tk.NORMAL)

        self._connected = False
        self.conn_text_var.set("Disconnected")
        self._draw_conn_dot("#ff5f56")

    def _handle_line(self, line):
        m = LINE_RE.search(line)
        if not m:
            return

        desired = float(m.group(1))
        actual = float(m.group(2))
        dir_val = int(m.group(9))
        sw_val = int(m.group(10)) if m.group(10) is not None else 0

        self.latest_desired = desired
        self.latest_actual = actual
        self.latest_dir = dir_val
        self.latest_sw = sw_val

        self.machine_vars["Desired"].set(f"{desired:.2f}")
        self.machine_vars["Actual"].set(f"{actual:.2f}")
        self.machine_vars["Direction"].set(str(dir_val))
        self.machine_vars["Switch"].set(str(sw_val))

        self._update_machine_note()

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()

                if kind == "line":
                    self._handle_line(payload)

                elif kind == "status":
                    txt = str(payload).strip()
                    if txt.startswith("Connected to"):
                        self._connected = True
                        self.conn_text_var.set("Connected")
                        self._draw_conn_dot("#2dd36f")
                    elif txt.startswith("Disconnected"):
                        self._connected = False
                        self.conn_text_var.set("Disconnected")
                        self._draw_conn_dot("#ff5f56")

                elif kind == "seq_started":
                    self._set_play_state("Playing")

                elif kind == "seq_note":
                    idx, note, angle, total = payload
                    self._current_note_name = note
                    self.piano_view.set_current_note(note if note != "REST" else None)
                    self._update_header_labels()

                elif kind == "seq_press":
                    self._pressed_note = payload
                    self.piano_view.set_pressed_note(payload)

                elif kind == "seq_release":
                    if self._pressed_note == payload:
                        self._pressed_note = None
                        self.piano_view.set_pressed_note(None)

                elif kind == "seq_time":
                    self._current_time_s = float(payload)
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                elif kind == "seq_paused":
                    paused = bool(payload)
                    if paused:
                        self._set_play_state("Paused")
                    else:
                        self._set_play_state("Playing")

                elif kind == "seq_done":
                    self._current_time_s = self._song_total_s
                    self._pressed_note = None
                    self.piano_view.set_pressed_note(None)
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                    self.btn_pause.config(text="Pause", state=tk.DISABLED)
                    self.btn_stop.config(state=tk.DISABLED)
                    self._set_play_state("Finished")

                elif kind == "seq_stop":
                    self._pressed_note = None
                    self.piano_view.set_pressed_note(None)
                    self.btn_pause.config(text="Pause", state=tk.DISABLED)
                    self.btn_stop.config(state=tk.DISABLED)
                    if self._play_state != "Finished":
                        self._set_play_state("Stopped")

        except queue.Empty:
            pass

        self.after(UI_POLL_MS, self._poll_queue)

    def on_close(self):
        self._seq_stop()
        self.stop_evt.set()
        if self.vofa_listener:
            self.vofa_listener.stop()
        self.destroy()

    def _load_midi_file(self):
        path = filedialog.askopenfilename(
            title="Select MIDI file",
            filetypes=[
                ("MIDI files", "*.mid *.midi"),
                ("All files", "*.*"),
            ]
        )
        if not path:
            return

        try:
            song = load_song_from_midi(path)
        except Exception as e:
            messagebox.showerror("MIDI load failed", str(e))
            return

        if not song:
            messagebox.showwarning("Empty MIDI", "No playable notes were extracted from this MIDI.")
            return

        self._current_song_name = os.path.basename(path)
        self._current_song = [(str(n).strip().upper(), float(d)) for n, d in song]
        self._song_total_s = sum(max(0.01, float(d)) for _, d in self._current_song)
        self._current_time_s = 0.0
        self._current_note_name = "—"

        self.piano_view.set_song(self._current_song_name, self._current_song)
        self._update_header_labels()
        self._set_play_state("Stopped")

        # 可选：把 MIDI 解析结果也显示到右侧文本框里，方便检查
        self.custom_text.delete("1.0", tk.END)
        for note, dur in self._current_song:
            self.custom_text.insert(tk.END, f"{note} {dur:.3f}\n")