# ─────────────────────────────────────────────────────────────────────────────
# Sequencer thread
# Logic:
#   1) Send SP=<angle>!
#   2) Wait until actual is near target (or timeout)
#   3) Send SL=1!
#   4) Hold for SOLENOID_ON_S
#   5) Send SL=0!
#   6) Wait until note duration ends
# ─────────────────────────────────────────────────────────────────────────────
# Core_function/sequencer.py
import threading
import time
from Core_function.note import note_to_angle

POSITION_TOL_DEG = 15.0
POSITION_WAIT_S = 1.0
SOLENOID_ON_S = 0.12


class Sequencer(threading.Thread):
    """
    Song format:
        [(note_name, duration_sec), ...]

    UI messages pushed to out_q:
        ("seq_started", {"total": N, "song": [...]})
        ("seq_note", (idx, note, angle_or_none, total))
        ("seq_press", note)
        ("seq_release", note)
        ("seq_paused", True/False)
        ("seq_time", elapsed_sec)
        ("seq_done", None)
        ("seq_stop", None)
        ("error", "...")

    Notes:
    - "REST" is supported for UI timing only; no motor command will be sent.
    - Notes outside hardware range are skipped for hardware, but still appear in UI.
    """

    def __init__(
        self,
        song,
        reader,
        out_q,
        stop_evt,
        pause_evt,
        get_actual_fn,
    ):
        super().__init__(daemon=True)
        self.song = song
        self.reader = reader
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.pause_evt = pause_evt
        self.get_actual_fn = get_actual_fn

    def _write_cmd(self, cmd: str):
        self.reader.write(cmd)

    def _wait_while_paused(self):
        was_paused = False
        while self.pause_evt.is_set() and not self.stop_evt.is_set():
            if not was_paused:
                self.out_q.put(("seq_paused", True))
                was_paused = True
            time.sleep(0.01)
        if was_paused and not self.stop_evt.is_set():
            self.out_q.put(("seq_paused", False))

    def _sleep_with_pause(self, seconds: float, song_t0: float):
        end_t = time.perf_counter() + max(0.0, seconds)
        while not self.stop_evt.is_set():
            self._wait_while_paused()
            if self.stop_evt.is_set():
                return False

            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            if now >= end_t:
                return True

            time.sleep(0.005)
        return False

    def _wait_until_position(self, target_angle: float, tol_deg: float, timeout_s: float, song_t0: float) -> bool:
        t0 = time.perf_counter()
        while not self.stop_evt.is_set():
            self._wait_while_paused()
            if self.stop_evt.is_set():
                return False

            actual = self.get_actual_fn()
            if actual is not None and abs(actual - target_angle) <= tol_deg:
                return True

            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            if now - t0 >= timeout_s:
                return False
            time.sleep(0.005)
        return False

    def run(self):
        total = len(self.song)
        self.out_q.put(("seq_started", {"total": total, "song": self.song}))

        song_t0 = time.perf_counter()

        for idx, (note, duration) in enumerate(self.song):
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", None))
                return

            self._wait_while_paused()
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", None))
                return

            duration = max(0.01, float(duration))
            is_rest = str(note).strip().upper() == "REST"

            angle = None
            if not is_rest:
                try:
                    angle = note_to_angle(note)
                except ValueError:
                    # 超出硬件范围时，不给机械臂发命令，但 UI 仍继续
                    angle = None

            self.out_q.put(("seq_note", (idx, note, angle, total)))

            note_start = time.perf_counter()

            # 1) Move to key (only if hardware-supported note)
            if angle is not None:
                try:
                    self._write_cmd(f"SP={angle:.2f}!")
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Serial write failed: {e}"))
                    self.out_q.put(("seq_stop", None))
                    return

                reached = self._wait_until_position(
                    target_angle=angle,
                    tol_deg=POSITION_TOL_DEG,
                    timeout_s=POSITION_WAIT_S,
                    song_t0=song_t0,
                )

                if self.stop_evt.is_set():
                    self.out_q.put(("seq_stop", None))
                    return

                if not reached:
                    self.out_q.put(("error", f"[Sequencer] Position timeout on {note}"))

                # 2) Press
                try:
                    self._write_cmd("SL=1!")
                    self.out_q.put(("seq_press", note))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=1!: {e}"))
                    self.out_q.put(("seq_stop", None))
                    return

                # 3) Hold
                hold_s = min(SOLENOID_ON_S, max(0.03, duration * 0.35))
                if not self._sleep_with_pause(hold_s, song_t0):
                    self.out_q.put(("seq_stop", None))
                    return

                # 4) Release
                try:
                    self._write_cmd("SL=0!")
                    self.out_q.put(("seq_release", note))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=0!: {e}"))
                    self.out_q.put(("seq_stop", None))
                    return

            # 5) Wait until full note duration ends
            elapsed = time.perf_counter() - note_start
            remain = max(0.0, duration - elapsed)
            if not self._sleep_with_pause(remain, song_t0):
                self.out_q.put(("seq_stop", None))
                return

        self.out_q.put(("seq_done", None))