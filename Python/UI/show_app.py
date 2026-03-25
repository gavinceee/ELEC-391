import os
import sys
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from Song.midi_loader import load_song_from_midi

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from Communication.uart import SerialReader, VOFAListener, list_serial_ports, parse_telemetry_line
from Core_function.note import angle_to_note
from Core_function.sequencer import Sequencer
from Song.song import SONGS
from UI.piano_roll import PianoRollView

UI_POLL_MS = 30
DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200
RESET_HOME_CMD = "RE=1!"


class ShowApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Automatic Piano Player - Show UI")
        self.geometry("1200x680")
        self.minsize(1000, 600)
        self.configure(bg="#0b0f14")

        self.q = queue.Queue()
        self.stop_evt = threading.Event()
        self.reader = None

        self.vofa_stop_evt = threading.Event()
        self.vofa_listener = VOFAListener(self.q, self.vofa_stop_evt)
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

        self._planned_total_s = 1.0
        self._song_total_s = 1.0

        self._connected = False
        self._pressed_note = None

        self._pause_song_time = None
        self._pause_started_at = None

        self._right_panel_mode = "state"
        self._live_log_max_lines = 200

        self._build_ui()
        self._seq_load_song()

        self.after(UI_POLL_MS, self._poll_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

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
        tk.Label(
            conn_box,
            textvariable=self.conn_text_var,
            fg="#dfe7f2",
            bg="#11161e",
            font=("Segoe UI", 11, "bold")
        ).pack(side=tk.LEFT, padx=(0, 10))

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
            song_box,
            variable=self.tempo_var,
            from_=0.25,
            to=3.0,
            orient=tk.HORIZONTAL,
            length=110,
            command=self._on_tempo_change
        )
        self.tempo_scale.pack(side=tk.LEFT, padx=(6, 8))

        self.tempo_lbl = tk.Label(song_box, text="1.00×", fg="#dfe7f2", bg="#11161e", font=("Consolas", 10))
        self.tempo_lbl.pack(side=tk.LEFT)

        playback_box = tk.Frame(top, bg="#11161e")
        playback_box.pack(side=tk.LEFT, padx=(0, 12), ipadx=8, ipady=8)

        self.btn_play = ttk.Button(playback_box, text="Play", command=self._seq_play, width=10)
        self.btn_play.pack(side=tk.LEFT, padx=4)

        self.btn_pause_resume = ttk.Button(
            playback_box,
            text="Pause",
            command=self._seq_toggle_pause,
            width=10,
            state=tk.DISABLED
        )
        self.btn_pause_resume.pack(side=tk.LEFT, padx=4)

        self.btn_reset = ttk.Button(playback_box, text="Reset", command=self._seq_reset, width=10)
        self.btn_reset.pack(side=tk.LEFT, padx=4)

        right_box = tk.Frame(top, bg="#11161e")
        right_box.pack(side=tk.RIGHT, ipadx=10, ipady=8)

        self.status_var = tk.StringVar(value="Stopped")
        self.now_song_var = tk.StringVar(value="Song: —")
        self.now_note_var = tk.StringVar(value="Note: —")
        self.now_time_var = tk.StringVar(value="00:00.00 / 00:00.00")

        tk.Label(
            right_box,
            textvariable=self.status_var,
            fg="#89f0dd",
            bg="#11161e",
            font=("Segoe UI", 12, "bold")
        ).pack(anchor="e")
        tk.Label(
            right_box,
            textvariable=self.now_song_var,
            fg="#dfe7f2",
            bg="#11161e",
            font=("Segoe UI", 10)
        ).pack(anchor="e")
        tk.Label(
            right_box,
            textvariable=self.now_note_var,
            fg="#dfe7f2",
            bg="#11161e",
            font=("Segoe UI", 10)
        ).pack(anchor="e")
        tk.Label(
            right_box,
            textvariable=self.now_time_var,
            fg="#9eb0c6",
            bg="#11161e",
            font=("Consolas", 10)
        ).pack(anchor="e")

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

        right = tk.Frame(content, bg="#11161e", width=380)
        right.pack(side=tk.RIGHT, fill=tk.Y, padx=(12, 0))
        right.pack_propagate(False)

        toggle_bar = tk.Frame(right, bg="#11161e")
        toggle_bar.pack(fill=tk.X, padx=14, pady=(16, 8))

        self.btn_state_panel = tk.Button(
            toggle_bar,
            text="Machine State",
            command=self._show_state_panel,
            bg="#1b2430",
            fg="#dfe7f2",
            activebackground="#253244",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            font=("Segoe UI", 10, "bold"),
            padx=10,
            pady=6,
            cursor="hand2"
        )
        self.btn_state_panel.pack(side=tk.LEFT, padx=(0, 6))

        self.btn_log_panel = tk.Button(
            toggle_bar,
            text="Live Log",
            command=self._show_log_panel,
            bg="#11161e",
            fg="#9eb0c6",
            activebackground="#253244",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            font=("Segoe UI", 10, "bold"),
            padx=10,
            pady=6,
            cursor="hand2"
        )
        self.btn_log_panel.pack(side=tk.LEFT)

        self.top_panel_host = tk.Frame(right, bg="#11161e", height=300)
        self.top_panel_host.pack(fill=tk.X, padx=14, pady=(0, 10))
        self.top_panel_host.pack_propagate(False)

        self.state_panel = tk.Frame(self.top_panel_host, bg="#11161e")
        self.state_panel.place(relx=0, rely=0, relwidth=1, relheight=1)

        self.machine_vars = {}
        fields = [
            ("Desired", "—"),
            ("Actual", "—"),
            ("Direction", "—"),
            ("Solenoid", "—"),
            ("Arm Note", "—"),
            ("Current Song", "—"),
            ("Play State", "Stopped"),
            ("Move Time", "—"),
            ("Move Reached", "—"),
            ("Move Error", "—"),
        ]
        for key, default in fields:
            row = tk.Frame(self.state_panel, bg="#11161e")
            row.pack(fill=tk.X, pady=4)

            tk.Label(
                row,
                text=key,
                width=12,
                anchor="w",
                bg="#11161e",
                fg="#9eb0c6",
                font=("Segoe UI", 10)
            ).pack(side=tk.LEFT)

            var = tk.StringVar(value=default)
            self.machine_vars[key] = var
            tk.Label(
                row,
                textvariable=var,
                anchor="e",
                bg="#11161e",
                fg="#dfe7f2",
                font=("Consolas", 10, "bold")
            ).pack(side=tk.RIGHT)

        self.log_panel = tk.Frame(self.top_panel_host, bg="#11161e")
        self.log_panel.place(relx=0, rely=0, relwidth=1, relheight=1)

        tk.Label(
            self.log_panel,
            text="Live Log",
            bg="#11161e",
            fg="#dfe7f2",
            font=("Segoe UI", 14, "bold")
        ).pack(anchor="w", pady=(0, 12))

        log_box_frame = tk.Frame(self.log_panel, bg="#11161e")
        log_box_frame.pack(fill=tk.BOTH, expand=True)

        self.live_log = tk.Text(
            log_box_frame,
            bg="#0c1016",
            fg="#dfe7f2",
            insertbackground="#ffffff",
            relief=tk.FLAT,
            wrap=tk.WORD,
            font=("Consolas", 9),
            height=14
        )
        self.live_log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        live_log_scroll = ttk.Scrollbar(log_box_frame, orient="vertical", command=self.live_log.yview)
        live_log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.live_log.configure(yscrollcommand=live_log_scroll.set)

        self._show_state_panel()

        editor_frame = tk.Frame(right, bg="#11161e")
        editor_frame.pack(fill=tk.BOTH, expand=True, padx=14, pady=(0, 16))

        tk.Label(
            editor_frame,
            text="Custom Song Editor",
            bg="#11161e",
            fg="#dfe7f2",
            font=("Segoe UI", 12, "bold")
        ).pack(anchor="w")

        tk.Label(
            editor_frame,
            text="One note per line: NOTE DURATION\nExample:\nC4 0.5\nG4 1.0\nREST 0.2",
            justify=tk.LEFT,
            bg="#11161e",
            fg="#9eb0c6",
            font=("Consolas", 9)
        ).pack(anchor="w", pady=(6, 8))

        self.custom_text = tk.Text(
            editor_frame,
            height=8,
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

    def _show_state_panel(self):
        self._right_panel_mode = "state"
        self.state_panel.lift()
        self.btn_state_panel.config(bg="#1b2430", fg="#dfe7f2")
        self.btn_log_panel.config(bg="#11161e", fg="#9eb0c6")

    def _show_log_panel(self):
        self._right_panel_mode = "log"
        self.log_panel.lift()
        self.btn_state_panel.config(bg="#11161e", fg="#9eb0c6")
        self.btn_log_panel.config(bg="#1b2430", fg="#dfe7f2")

    def _append_live_log(self, text: str, blank_after: bool = False):
        if not hasattr(self, "live_log"):
            return

        self.live_log.insert(tk.END, text.rstrip() + "\n")
        if blank_after:
            self.live_log.insert(tk.END, "\n")

        line_count = int(self.live_log.index("end-1c").split(".")[0])
        if line_count > self._live_log_max_lines:
            remove_to = line_count - self._live_log_max_lines
            self.live_log.delete("1.0", f"{remove_to + 1}.0")

        self.live_log.see(tk.END)

    def _log_time_prefix(self):
        return time.strftime("[%H:%M:%S]")

    def _fmt_time(self, s: float) -> str:
        s = max(0.0, float(s))
        m = int(s // 60)
        sec = s - m * 60
        return f"{m:02d}:{sec:05.2f}"

    def _set_play_state(self, text: str):
        self._play_state = text
        self.status_var.set(text)
        self.machine_vars["Play State"].set(text)
        self.piano_view.set_status(text)

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

    def _clear_play_ui_state(self):
        self._current_time_s = 0.0
        self._current_note_name = "—"
        self._pressed_note = None

        self.machine_vars["Move Time"].set("—")
        self.machine_vars["Move Reached"].set("—")
        self.machine_vars["Move Error"].set("—")

        self.piano_view.set_current_note(None)
        self.piano_view.set_pressed_note(None)
        self.piano_view.set_play_time(0.0)
        self.piano_view.set_move_info(None, None)

        self._song_total_s = self._planned_total_s
        self._update_header_labels()

        if hasattr(self, "live_log"):
            self.live_log.delete("1.0", tk.END)

    def _seq_load_song(self):
        name = self.song_var.get()
        if name not in SONGS:
            return

        self._current_song_name = name
        self._current_song = [(str(n).strip().upper(), float(d)) for n, d in SONGS[name]]
        self._planned_total_s = sum(max(0.01, float(dur)) for _, dur in self._current_song)
        self._song_total_s = self._planned_total_s

        self._clear_play_ui_state()

        self.piano_view.set_song(self._current_song_name, self._current_song)
        self._planned_total_s = self.piano_view.song_total_duration
        self._song_total_s = self._planned_total_s
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
        self._planned_total_s = sum(max(0.01, float(d)) for _, d in self._current_song)
        self._song_total_s = self._planned_total_s

        self._clear_play_ui_state()

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

        if self._seq_thread and self._seq_thread.is_alive():
            return

        if not self._current_song:
            messagebox.showwarning("No song", "Load a song first.")
            return

        self._seq_stop_evt.clear()
        self._seq_pause_evt.clear()
        scaled_song = self._scaled_song()

        self._planned_total_s = sum(max(0.01, float(d)) for _, d in scaled_song)
        self._song_total_s = self._planned_total_s

        self._clear_play_ui_state()
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

        self.btn_play.config(state=tk.DISABLED)
        self.btn_pause_resume.config(state=tk.NORMAL, text="Pause")
        self._set_play_state("Playing")

    def _seq_pause(self):
        if not (self._seq_thread and self._seq_thread.is_alive()):
            return
        if self._seq_pause_evt.is_set():
            return

        self._seq_pause_evt.set()
        self.btn_pause_resume.config(text="Resume", state=tk.NORMAL)

    def _seq_resume(self):
        if not (self._seq_thread and self._seq_thread.is_alive()):
            return
        if not self._seq_pause_evt.is_set():
            return

        self._seq_pause_evt.clear()
        self.btn_pause_resume.config(text="Pause", state=tk.NORMAL)

    def _seq_toggle_pause(self):
        if not (self._seq_thread and self._seq_thread.is_alive()):
            return

        if self._seq_pause_evt.is_set():
            self._seq_resume()
        else:
            self._seq_pause()

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

        self.btn_play.config(state=tk.NORMAL)
        self.btn_pause_resume.config(state=tk.DISABLED, text="Pause")

        self._append_live_log(f"{self._log_time_prefix()} Stop pressed")

    def _seq_reset(self):
    # fully terminate current playback
        self._seq_stop_evt.set()
        self._seq_pause_evt.clear()

        try:
            if self.reader and self.reader.is_alive():
                self.reader.write("SL=0!")
                self.reader.write("RE=1!")
        except Exception:
            pass

        self._pause_started_at = None
        self._pause_song_time = None

        self._pressed_note = None
        self.piano_view.set_pressed_note(None)

        self.btn_play.config(state=tk.NORMAL)
        self.btn_pause_resume.config(state=tk.DISABLED, text="Pause")

        self._clear_play_ui_state()
        self.piano_view.reset_view()
        self.piano_view.set_song(self._current_song_name, self._scaled_song())
        self._planned_total_s = self.piano_view.song_total_duration
        self._song_total_s = self._planned_total_s
        self._update_header_labels()
        self._set_play_state("Homing...")

        self._append_live_log(f"{self._log_time_prefix()} Reset / Homing")

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
        data = parse_telemetry_line(line)
        if not data:
            return

        desired = data["desired"]
        actual = data["actual"]
        direction = data["dir"]
        switch = data["sw"]

        self.latest_desired = desired
        self.latest_actual = actual
        self.latest_dir = direction
        self.latest_sw = switch

        self.machine_vars["Desired"].set(f"{desired:.2f}")
        self.machine_vars["Actual"].set(f"{actual:.2f}")
        self.machine_vars["Direction"].set(str(direction))
        self.machine_vars["Solenoid"].set(str(switch))

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
                    self._append_live_log(
                        f"{self._log_time_prefix()} SONG start: {self._current_song_name}"
                    )

                elif kind == "seq_home_start":
                    self._append_live_log(f"{self._log_time_prefix()} HOME start")

                elif kind == "seq_home_done":
                    home_elapsed = float(payload.get("elapsed", 0.0)) if isinstance(payload, dict) else 0.0
                    self._append_live_log(
                        f"{self._log_time_prefix()} HOME done ({home_elapsed:.2f}s)",
                        blank_after=True
                    )

                elif kind == "seq_note":
                    idx = int(payload.get("idx", 0))
                    total = int(payload.get("total", 0))
                    note = str(payload.get("note", "—"))
                    angle = payload.get("angle", None)

                    self._current_note_name = note
                    self.piano_view.set_current_note(note if note != "REST" else None)
                    self._update_header_labels()

                    if angle is None:
                        self._append_live_log(
                            f"{self._log_time_prefix()} NOTE {idx+1}/{total} {note}"
                        )
                    else:
                        self._append_live_log(
                            f"{self._log_time_prefix()} NOTE {idx+1}/{total} {note} -> target {angle:.1f}°"
                        )
                        self._append_live_log(f"{self._log_time_prefix()} MOVE start")

                elif kind == "seq_move_time":
                    idx = payload.get("idx", -1)
                    move_time = float(payload.get("move_time", 0.0))
                    reached = bool(payload.get("reached", False))
                    note = str(payload.get("note", "?"))

                    actual_angle = payload.get("actual_angle", None)
                    angle_error = payload.get("angle_error", None)

                    self.machine_vars["Move Time"].set(f"{move_time:.3f}s")
                    self.machine_vars["Move Reached"].set("YES" if reached else "NO")

                    if angle_error is None:
                        self.machine_vars["Move Error"].set("—")
                    else:
                        self.machine_vars["Move Error"].set(f"{angle_error:+.1f}°")

                    self.piano_view.set_move_info(move_time, reached)
                    self.piano_view.set_note_move_time(idx, move_time)

                    if note != "REST":
                        extra_parts = []

                        if actual_angle is not None:
                            extra_parts.append(f"actual={float(actual_angle):.1f}°")

                        if angle_error is not None:
                            extra_parts.append(f"err={float(angle_error):+.1f}°")

                        extra_text = ""
                        if extra_parts:
                            extra_text = " " + " ".join(extra_parts)

                        if reached:
                            self._append_live_log(
                                f"{self._log_time_prefix()} reached ({move_time:.3f}s){extra_text}"
                            )
                        else:
                            self._append_live_log(
                                f"{self._log_time_prefix()} timeout ({move_time:.3f}s){extra_text}"
                            )

                elif kind == "seq_press":
                    idx = int(payload.get("idx", -1))
                    note = str(payload.get("note", "—"))
                    elapsed = float(payload.get("elapsed", self._current_time_s))

                    self._pressed_note = note
                    self.piano_view.set_pressed_note(note)
                    self.piano_view.set_note_actual_press(idx, elapsed)
                    self._append_live_log(f"{self._log_time_prefix()} PRESS {note}")

                elif kind == "seq_release":
                    idx = int(payload.get("idx", -1))
                    note = str(payload.get("note", "—"))
                    elapsed = float(payload.get("elapsed", self._current_time_s))

                    if self._pressed_note == note:
                        self._pressed_note = None
                        self.piano_view.set_pressed_note(None)

                    self.piano_view.set_note_actual_release(idx, elapsed)
                    self._append_live_log(
                        f"{self._log_time_prefix()} RELEASE {note}",
                        blank_after=True
                    )

                elif kind == "seq_paused":
                    paused_elapsed = float(payload.get("elapsed", self._current_time_s))
                    self._current_time_s = paused_elapsed
                    self._song_total_s = max(self._song_total_s, self._current_time_s)
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                    self._pause_song_time = paused_elapsed
                    self._pause_started_at = time.perf_counter()

                    self._pressed_note = None
                    self.piano_view.set_pressed_note(None)

                    self._set_play_state("Paused")
                    self._append_live_log(
                        f"{self._log_time_prefix()} Paused at song t={paused_elapsed:.2f}s"
    )

                elif kind == "seq_resumed":
                    resumed_elapsed = float(payload.get("elapsed", self._current_time_s))
                    paused_for = payload.get("paused_for", None)

                    if paused_for is None:
                        if self._pause_started_at is not None:
                            paused_for = time.perf_counter() - self._pause_started_at
                        else:
                            paused_for = 0.0
                    else:
                        paused_for = float(paused_for)

                    paused_song_t = self._pause_song_time
                    if paused_song_t is None:
                        paused_song_t = resumed_elapsed

                    self._current_time_s = resumed_elapsed
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                    self._pause_started_at = None
                    self._pause_song_time = None

                    self._set_play_state("Playing")
                    self._append_live_log(
                        f"{self._log_time_prefix()} Resumed at song t={paused_song_t:.2f}s after {paused_for:.2f}s pause"
                    )

                elif kind == "seq_time":
                    self._current_time_s = float(payload)
                    self._song_total_s = max(self._planned_total_s, self._current_time_s)
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                elif kind == "seq_done":
                    final_elapsed = (
                        float(payload.get("elapsed", self._current_time_s))
                        if isinstance(payload, dict)
                        else float(self._current_time_s)
                    )

                    self._current_time_s = final_elapsed
                    self._song_total_s = final_elapsed

                    self._pressed_note = None
                    self.piano_view.set_pressed_note(None)
                    self.piano_view.set_play_time(self._current_time_s)
                    self._update_header_labels()

                    self.btn_play.config(state=tk.NORMAL)
                    self.btn_pause_resume.config(state=tk.DISABLED, text="Pause")

                    self._set_play_state("Finished")
                    self._append_live_log(
                        f"{self._log_time_prefix()} Done ({final_elapsed:.2f}s)"
                    )

                elif kind == "seq_stop":
                    if isinstance(payload, dict):
                        stop_elapsed = float(payload.get("elapsed", self._current_time_s))
                        self._current_time_s = stop_elapsed

                    self._pressed_note = None
                    self.piano_view.set_pressed_note(None)

                    self.btn_play.config(state=tk.NORMAL)
                    self.btn_pause_resume.config(state=tk.DISABLED, text="Pause")  

                    self.piano_view.set_play_time(self._current_time_s)
                    self._song_total_s = max(self._song_total_s, self._current_time_s)
                    self._update_header_labels()

                    if self._play_state != "Finished":
                        self._set_play_state("Stopped")
                        self._append_live_log(
                            f"{self._log_time_prefix()} Stopped ({self._current_time_s:.2f}s)"
                        )

                elif kind == "error":
                    print(payload)
                    self._append_live_log(f"{self._log_time_prefix()} ERROR {payload}")

        except queue.Empty:
            pass

        self.after(UI_POLL_MS, self._poll_queue)

    def on_close(self):
        self._seq_stop()
        self.stop_evt.set()
        self.vofa_stop_evt.set()
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
        self._planned_total_s = sum(max(0.01, float(d)) for _, d in self._current_song)
        self._song_total_s = self._planned_total_s

        self._clear_play_ui_state()

        self.piano_view.set_song(self._current_song_name, self._current_song)
        self._update_header_labels()
        self._set_play_state("Stopped")

        self.custom_text.delete("1.0", tk.END)
        for note, dur in self._current_song:
            self.custom_text.insert(tk.END, f"{note} {dur:.3f}\n")