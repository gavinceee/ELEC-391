import tkinter as tk

NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

try:
    from Core_function.note import note_to_angle
except Exception:
    note_to_angle = None


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
    WARN = "#ffb86b"

    PLAYHEAD = "#ffffff"
    PLAYHEAD_GLOW = "#96fff1"

    COLUMN_ARM = "#071722"
    COLUMN_CURRENT = "#0b2430"
    COLUMN_PRESS = "#113844"

    NOTE_SIDE_PAD = 3

    # 预测 move_time 的 clamp
    MIN_MOVE_S = 0.10
    MAX_MOVE_S = 1.20

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
        self.status_text = "Stopped"

        self.current_move_time = None
        self.current_move_reached = None

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
        self.current_move_time = None
        self.current_move_reached = None
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

    def set_status(self, status_text: str):
        self.status_text = status_text
        self.redraw()

    def set_move_info(self, move_time=None, reached=None):
        self.current_move_time = move_time
        self.current_move_reached = reached
        self.redraw()

    def set_note_move_time(self, idx: int, move_time: float):
        if idx < 0 or idx >= len(self.song_events):
            return
        try:
            move_time = max(0.0, float(move_time))
        except (TypeError, ValueError):
            return

        ev = self.song_events[idx]
        ev["move_time"] = move_time
        ev["preview_start"] = ev["start"] - move_time
        self.redraw()

    def set_note_actual_press(self, idx: int, elapsed: float):
        if idx < 0 or idx >= len(self.song_events):
            return
        try:
            elapsed = float(elapsed)
        except (TypeError, ValueError):
            return
        self.song_events[idx]["actual_start"] = elapsed
        self.redraw()

    def set_note_actual_release(self, idx: int, elapsed: float):
        if idx < 0 or idx >= len(self.song_events):
            return
        try:
            elapsed = float(elapsed)
        except (TypeError, ValueError):
            return
        self.song_events[idx]["actual_end"] = elapsed
        self.redraw()

    def reset_view(self):
        self.play_time = 0.0
        self.current_note = None
        self.pressed_note = None
        self.current_move_time = None
        self.current_move_reached = None
        self.status_text = "Stopped"

        for ev in self.song_events:
            ev["actual_start"] = None
            ev["actual_end"] = None

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

    # -------------------------
    # Timing model
    # -------------------------
    def _estimate_move_time(self, prev_playable_note, note):
        if note == "REST":
            return 0.0

        # 优先按真实角度差来估计
        if note_to_angle is not None:
            try:
                cur_angle = float(note_to_angle(note))
                if prev_playable_note is None:
                    mt = 0.18 + 0.00055 * abs(cur_angle)
                else:
                    prev_angle = float(note_to_angle(prev_playable_note))
                    mt = 0.08 + 0.00100 * abs(cur_angle - prev_angle)
                return max(self.MIN_MOVE_S, min(self.MAX_MOVE_S, mt))
            except Exception:
                pass

        # 兜底：按 MIDI 音高差估计
        try:
            if prev_playable_note is None:
                interval = abs(name_to_midi(note) - name_to_midi(self.start_note))
                mt = 0.22 + 0.035 * interval
            else:
                interval = abs(name_to_midi(note) - name_to_midi(prev_playable_note))
                mt = 0.08 + 0.03 * interval
        except Exception:
            mt = 0.45 if prev_playable_note is None else 0.18

        return max(self.MIN_MOVE_S, min(self.MAX_MOVE_S, mt))

    def _build_song_events(self, song):
        raw_events = []
        t = 0.0
        prev_playable_note = None
        global_offset = 0.0

        # 第一遍：先算 raw start 和每颗音预测 move_time
        for idx, (note, duration) in enumerate(song):
            note = str(note).strip().upper()
            dur = max(0.01, float(duration))
            raw_start = t
            raw_end = t + dur

            move_time = self._estimate_move_time(prev_playable_note, note)

            if note != "REST":
                global_offset = max(global_offset, move_time - raw_start)
                prev_playable_note = note

            raw_events.append({
                "index": idx,
                "note": note,
                "raw_start": raw_start,
                "raw_end": raw_end,
                "duration": dur,
                "move_time": move_time,
            })
            t = raw_end

        # 第二遍：整体右移，保证不会一开场就出现“条已经在底线下面”
        events = []
        for ev in raw_events:
            start = ev["raw_start"] + global_offset
            end = ev["raw_end"] + global_offset

            events.append({
                "index": ev["index"],
                "note": ev["note"],
                "start": start,                     # 计划 hit_time
                "end": end,                         # 计划 slot end
                "duration": ev["duration"],
                "move_time": ev["move_time"],
                "preview_start": start - ev["move_time"],
                "actual_start": None,
                "actual_end": None,
            })

        return events

    # -------------------------
    # Draw
    # -------------------------
    def redraw(self):
        self.delete("all")
        w = max(400, self.winfo_width())
        h = max(500, self.winfo_height())

        self.create_rectangle(0, 0, w, h, fill=self.BG, outline="")

        self._draw_active_columns()
        self._draw_grid()
        self._draw_falling_notes()
        self._draw_hit_line()
        self._draw_keyboard()
        self._draw_overlay_text()

    def _draw_one_column(self, note, fill_color, inset=2):
        if not note or note not in self.key_notes:
            return
        x = self._x_for_note(note)
        if x is None:
            return
        x0, x1 = x
        self.create_rectangle(
            x0 + inset, self.lane_top, x1 - inset, self._lane_bottom(),
            fill=fill_color,
            outline=""
        )

    def _draw_active_columns(self):
        if self.arm_note and self.arm_note != self.current_note and self.arm_note != self.pressed_note:
            self._draw_one_column(self.arm_note, self.COLUMN_ARM, inset=4)

        if self.current_note and self.current_note != self.pressed_note:
            self._draw_one_column(self.current_note, self.COLUMN_CURRENT, inset=3)

        if self.pressed_note:
            self._draw_one_column(self.pressed_note, self.COLUMN_PRESS, inset=2)

    def _draw_grid(self):
        lane_w = self._lane_w()
        for i in range(self.total_keys + 1):
            x = self.padding_x + i * lane_w
            self.create_line(x, self.lane_top, x, self._lane_bottom(), fill=self.GRID, width=1)

    def _draw_single_note_block(self, x0, x1, y0, y1, active_now=False):
        glow = self.NOTE_GLOW_ACTIVE if active_now else self.NOTE_GLOW
        fill = self.NOTE_ACTIVE if active_now else self.NOTE_FILL

        self.create_rectangle(
            x0 + 1, y0 - 2, x1 - 1, y1 + 2,
            fill=glow, outline=""
        )
        self.create_rectangle(
            x0 + 3, y0, x1 - 3, y1,
            fill=fill, outline=""
        )
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

            preview_start = float(ev.get("preview_start", ev["start"]))
            # 一旦真实按下，就立即消失；否则按计划 hit_time 落到底线
            hit_time = ev["actual_start"] if ev.get("actual_start") is not None else ev["start"]

            # 已经 press 过了，就不再画这个预告条
            if self.play_time >= hit_time:
                continue

            y0 = self._time_to_y(preview_start)
            y1 = self._time_to_y(hit_time)

            top = min(y0, y1)
            bottom = max(y0, y1)

            if bottom < self.lane_top or top > self._lane_bottom():
                continue

            top = max(self.lane_top, top)
            bottom = min(self._lane_bottom(), bottom)

            if bottom - top < 2:
                continue

            active_now = (self.current_note == note)

            self._draw_single_note_block(
                x0 + self.NOTE_SIDE_PAD,
                x1 - self.NOTE_SIDE_PAD,
                top,
                bottom,
                active_now=active_now
            )

    def _draw_hit_line(self):
        y = self._lane_bottom()

        self.create_line(
            self.padding_x, y - 1,
            self.winfo_width() - self.padding_x, y - 1,
            fill=self.PLAYHEAD_GLOW,
            width=5
        )

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

        self.create_text(
            18, 16,
            text=left_text,
            anchor="nw",
            fill=self.TEXT,
            font=("Segoe UI", 15, "bold")
        )

        self.create_text(
            w - 18, 16,
            text=self.status_text,
            anchor="ne",
            fill=self.SUBTEXT,
            font=("Segoe UI", 11, "bold")
        )

        if self.current_move_time is not None:
            reached_txt = "OK" if self.current_move_reached else "WAIT/TIMEOUT"
            reached_color = "#89f0dd" if self.current_move_reached else self.WARN
            self.create_text(
                18, 42,
                text=f"Move: {self.current_move_time:.3f}s",
                anchor="nw",
                fill=self.SUBTEXT,
                font=("Consolas", 10, "bold")
            )
            self.create_text(
                150, 42,
                text=f"[{reached_txt}]",
                anchor="nw",
                fill=reached_color,
                font=("Consolas", 10, "bold")
            )