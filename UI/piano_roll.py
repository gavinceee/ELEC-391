import tkinter as tk

NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']


def midi_to_name(note_num: int) -> str:
    pitch = NOTE_NAMES[note_num % 12]
    octave = (note_num // 12) - 1
    return f"{pitch}{octave}"


def name_to_midi(note_name: str) -> int:
    s = note_name.strip().upper()
    if len(s) < 2:
        raise ValueError(f"Bad note: {note_name}")

    if s[1] == "#":
        pitch = s[:2]
        octave = int(s[2:])
    else:
        pitch = s[:1]
        octave = int(s[1:])

    return NOTE_NAMES.index(pitch) + (octave + 1) * 12


def is_black(note_name: str) -> bool:
    return "#" in note_name


class PianoRollView(tk.Canvas):
    BG = "#04070b"
    GRID = "#122033"

    NOTE_FILL = "#8ef5e4"
    NOTE_ACTIVE = "#d9fff8"
    NOTE_GLOW = "#5ddfcb"
    NOTE_GLOW_ACTIVE = "#c8fff6"

    KEY_WHITE = "#f3f3f5"
    KEY_BLACK = "#0f1115"
    KEY_BORDER = "#28313a"
    KEY_ACTIVE = "#72f2df"
    KEY_ACTIVE_SOFT = "#b6fff4"
    KEY_ARM = "#6aa8ff"

    TEXT = "#dce7f7"
    SUBTEXT = "#8ea0b8"

    PLAYHEAD = "#ffffff"
    PLAYHEAD_GLOW = "#96fff1"
    COLUMN_HILITE = "#0b2a35"

    def __init__(self, master, start_note="C3", octaves=4, **kwargs):
        super().__init__(
            master,
            bg=self.BG,
            highlightthickness=0,
            **kwargs
        )
        self.start_note = start_note
        self.octaves = octaves
        self.total_keys = octaves * 12
        self.key_notes = [
            midi_to_name(name_to_midi(start_note) + i)
            for i in range(self.total_keys)
        ]

        self.song = []
        self.song_events = []
        self.song_total_duration = 1.0

        self.play_time = 0.0
        self.current_note = None
        self.pressed_note = None
        self.arm_note = None
        self.song_name = ""
        self.status_text = "IDLE"
        self.paused = False

        self.keyboard_h = 180
        self.padding_x = 20
        self.lane_top = 48
        self.note_hit_y_gap = 12
        self.seconds_visible = 6.0

        self.bind("<Configure>", lambda e: self.redraw())

    # -------------------------
    # Public API
    # -------------------------
    def set_song(self, song_name, song):
        self.song_name = song_name
        self.song = list(song)
        self.song_events = self._build_song_events(song)
        self.song_total_duration = max(
            0.001,
            self.song_events[-1]["end"] if self.song_events else 1.0
        )
        self.play_time = 0.0
        self.current_note = None
        self.pressed_note = None
        self.redraw()

    def set_play_time(self, seconds: float):
        self.play_time = max(0.0, float(seconds))
        self.redraw()

    def set_current_note(self, note):
        self.current_note = note
        self.redraw()

    def set_pressed_note(self, note):
        self.pressed_note = note
        self.redraw()

    def set_arm_note(self, note):
        self.arm_note = note
        self.redraw()

    def set_status(self, status_text: str, paused: bool = False):
        self.status_text = status_text
        self.paused = paused
        self.redraw()

    # -------------------------
    # Geometry helpers
    # -------------------------
    def _lane_w(self):
        w = max(400, self.winfo_width())
        return (w - 2 * self.padding_x) / self.total_keys

    def _white_key_height(self):
        return self.keyboard_h

    def _black_key_height(self):
        return int(self.keyboard_h * 0.62)

    def _x_for_note(self, note_name: str):
        if note_name not in self.key_notes:
            return None
        idx = self.key_notes.index(note_name)
        lane_w = self._lane_w()
        x0 = self.padding_x + idx * lane_w
        x1 = x0 + lane_w
        return x0, x1

    def _keyboard_top(self):
        h = max(500, self.winfo_height())
        return h - self.keyboard_h

    def _lane_bottom(self):
        return self._keyboard_top() - self.note_hit_y_gap

    def _lane_height(self):
        return self._lane_bottom() - self.lane_top

    def _time_to_y(self, event_time_sec: float):
        dt = event_time_sec - self.play_time
        px_per_sec = self._lane_height() / self.seconds_visible
        return self._lane_bottom() - dt * px_per_sec

    def _build_song_events(self, song):
        t = 0.0
        events = []
        for idx, (note, duration) in enumerate(song):
            dur = max(0.01, float(duration))
            start = t
            end = t + dur
            events.append({
                "index": idx,
                "note": str(note).strip().upper(),
                "start": start,
                "end": end,
                "duration": dur,
            })
            t = end
        return events

    # -------------------------
    # Draw
    # -------------------------
    def redraw(self):
        self.delete("all")
        w = max(400, self.winfo_width())
        h = max(500, self.winfo_height())

        self.create_rectangle(0, 0, w, h, fill=self.BG, outline="")

        self._draw_active_column()
        self._draw_grid()
        self._draw_falling_notes()
        self._draw_hit_line()
        self._draw_keyboard()
        self._draw_overlay_text()

    def _draw_active_column(self):
        note = self.pressed_note or self.current_note
        if not note or note not in self.key_notes:
            return

        x = self._x_for_note(note)
        if x is None:
            return

        x0, x1 = x
        self.create_rectangle(
            x0 + 2, self.lane_top, x1 - 2, self._lane_bottom(),
            fill=self.COLUMN_HILITE,
            outline=""
        )

    def _draw_grid(self):
        lane_w = self._lane_w()
        for i in range(self.total_keys + 1):
            x = self.padding_x + i * lane_w
            self.create_line(x, self.lane_top, x, self._lane_bottom(), fill=self.GRID, width=1)

    def _draw_single_note_block(self, x0, x1, y0, y1, active_now=False):
        glow = self.NOTE_GLOW_ACTIVE if active_now else self.NOTE_GLOW
        fill = self.NOTE_ACTIVE if active_now else self.NOTE_FILL

        # outer glow
        self.create_rectangle(
            x0 + 1, y0 - 2, x1 - 1, y1 + 2,
            fill=glow, outline=""
        )
        self.create_rectangle(
            x0 + 3, y0, x1 - 3, y1,
            fill=fill, outline=""
        )

        # top highlight strip
        self.create_rectangle(
            x0 + 4, y0, x1 - 4, min(y0 + 6, y1),
            fill="#f4fffd" if active_now else "#c8fff6",
            outline=""
        )

    def _draw_falling_notes(self):
        for ev in self.song_events:
            note = ev["note"]
            if note == "REST":
                continue
            if note not in self.key_notes:
                continue

            x = self._x_for_note(note)
            if x is None:
                continue

            x0, x1 = x
            y0 = self._time_to_y(ev["start"])
            y1 = self._time_to_y(ev["end"])

            if y1 < self.lane_top or y0 > self._lane_bottom():
                continue

            y0 = max(self.lane_top, y0)
            y1 = min(self._lane_bottom(), y1)

            active_now = ev["start"] <= self.play_time <= ev["end"]
            self._draw_single_note_block(x0 + 3, x1 - 3, y0, y1, active_now=active_now)

    def _draw_hit_line(self):
        y = self._lane_bottom()

        # glow
        self.create_line(
            self.padding_x, y - 1,
            self.winfo_width() - self.padding_x, y - 1,
            fill=self.PLAYHEAD_GLOW,
            width=5
        )

        # core line
        self.create_line(
            self.padding_x, y,
            self.winfo_width() - self.padding_x, y,
            fill=self.PLAYHEAD,
            width=2
        )

    def _draw_keyboard(self):
        top = self._keyboard_top()
        bottom = top + self._white_key_height()
        lane_w = self._lane_w()

        # white keys
        for note in self.key_notes:
            if is_black(note):
                continue

            x0, x1 = self._x_for_note(note)
            fill = self.KEY_WHITE
            outline = self.KEY_BORDER
            width = 1

            is_pressed = (note == self.pressed_note)
            is_current = (note == self.current_note)
            is_arm = (note == self.arm_note)

            if is_pressed:
                fill = self.KEY_ACTIVE
            elif is_current:
                fill = self.KEY_ACTIVE_SOFT

            if is_arm:
                outline = self.KEY_ARM
                width = 3

            self.create_rectangle(x0, top, x1, bottom, fill=fill, outline=outline, width=width)

            # top reflection
            if is_pressed or is_current:
                self.create_rectangle(
                    x0 + 2, top + 2, x1 - 2, top + 10,
                    fill="#f6fffd",
                    outline=""
                )

            if len(note) == 2:
                self.create_text(
                    (x0 + x1) / 2,
                    bottom - 18,
                    text=note,
                    fill="#444",
                    font=("Segoe UI", 9)
                )

        # black keys
        bh = self._black_key_height()
        for note in self.key_notes:
            if not is_black(note):
                continue

            x0, x1 = self._x_for_note(note)
            cx = (x0 + x1) / 2
            bw = lane_w * 0.68
            bx0 = cx - bw / 2
            bx1 = cx + bw / 2

            fill = self.KEY_BLACK
            outline = "#000000"
            width = 1

            is_pressed = (note == self.pressed_note)
            is_current = (note == self.current_note)
            is_arm = (note == self.arm_note)

            if is_pressed:
                fill = self.KEY_ACTIVE
                outline = self.KEY_ACTIVE
            elif is_current:
                fill = "#3bcaba"
                outline = "#3bcaba"

            if is_arm:
                outline = self.KEY_ARM
                width = 3

            self.create_rectangle(
                bx0, top, bx1, top + bh,
                fill=fill, outline=outline, width=width
            )

            if is_pressed or is_current:
                self.create_rectangle(
                    bx0 + 2, top + 2, bx1 - 2, top + 8,
                    fill="#cffff9",
                    outline=""
                )

    def _draw_overlay_text(self):
        w = self.winfo_width()

        left_text = self.song_name if self.song_name else "No Song Loaded"
        status = self.status_text
        if self.paused:
            status += " (Paused)"

        self.create_text(
            18, 16,
            text=left_text,
            anchor="nw",
            fill=self.TEXT,
            font=("Segoe UI", 15, "bold")
        )

        self.create_text(
            w - 18, 16,
            text=status,
            anchor="ne",
            fill=self.SUBTEXT,
            font=("Segoe UI", 11, "bold")
        )