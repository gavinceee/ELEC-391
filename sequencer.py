# ─────────────────────────────────────────────────────────────────────────────
# Sequencer thread
# Logic:
#   1) Send SP=<angle>!
#   2) Wait until actual is near target (or timeout)
#   3) Send SL=1!
#   4) Hold for note duration
#   5) Send SL=0!
#   6) Move on to next note
# ─────────────────────────────────────────────────────────────────────────────
# Core_function/sequencer.py

import threading
import time
from Core_function.note import note_to_angle

POSITION_TOL_DEG = 15.0
POSITION_WAIT_S = 1.0
MIN_RELEASE_GAP_S = 0.03


class Sequencer(threading.Thread):
    """
    Song format:
        [(note_name, duration_sec), ...]

    UI messages pushed to out_q:
        ("seq_started", {"total": N, "song": [...]})
        ("seq_note", (idx, note, angle_or_none, total))
        ("seq_move_time", {"idx": idx, "note": note, "move_time": t, "reached": bool})
        ("seq_press", note)
        ("seq_release", note)
        ("seq_time", elapsed_sec)
        ("seq_done", None)
        ("seq_stop", None)
        ("error", "...")

    Notes:
    - "REST" is supported for UI timing only; no motor command will be sent.
    - Notes outside hardware range are skipped for hardware, but still consume duration.
    - In this version, movement time is NOT included in note hold duration.
    """

    def __init__(
        self,
        song,
        reader,
        out_q,
        stop_evt,
        get_actual_fn,
    ):
        super().__init__(daemon=True)
        self.song = song
        self.reader = reader
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.get_actual_fn = get_actual_fn

    def _write_cmd(self, cmd: str):
        self.reader.write(cmd)

    def _sleep_for(self, seconds: float, song_t0: float) -> bool:
        """
        Sleep for 'seconds' while:
        - allowing stop to interrupt immediately
        - continuously updating UI elapsed time
        """
        seconds = max(0.0, float(seconds))
        end_t = time.perf_counter() + seconds

        while not self.stop_evt.is_set():
            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            if now >= end_t:
                return True

            time.sleep(0.005)

        return False

    def _wait_until_position(
        self,
        target_angle: float,
        tol_deg: float,
        timeout_s: float,
        song_t0: float,
    ) -> bool:
        """
        Wait until actual position is close enough to target.
        Returns True if reached, False if timeout / stopped.
        """
        timeout_s = max(0.0, float(timeout_s))
        t0 = time.perf_counter()

        while not self.stop_evt.is_set():
            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            actual = self.get_actual_fn()
            if actual is not None and abs(actual - target_angle) <= tol_deg:
                return True

            if now - t0 >= timeout_s:
                return False

            time.sleep(0.005)

        return False

    def _safe_duration(self, duration) -> float:
        try:
            d = float(duration)
        except (TypeError, ValueError):
            d = 0.0
        return max(0.01, d)

    def run(self):
        total = len(self.song)
        self.out_q.put(("seq_started", {"total": total, "song": self.song}))

        song_t0 = time.perf_counter()

        for idx, (note, duration) in enumerate(self.song):
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", None))
                return

            duration = self._safe_duration(duration)
            is_rest = str(note).strip().upper() == "REST"

            angle = None
            if not is_rest:
                try:
                    angle = note_to_angle(note)
                except ValueError:
                    angle = None

            self.out_q.put(("seq_note", (idx, note, angle, total)))

            note_start = time.perf_counter()

            # Case 1: normal hardware-supported note
            if angle is not None:
                # 1) Move to key
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

                move_time = time.perf_counter() - note_start
                self.out_q.put(("seq_move_time", {
                    "idx": idx,
                    "note": note,
                    "move_time": move_time,
                    "reached": reached,
                }))

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

                # 3) Hold for note duration
                if not self._sleep_for(duration, song_t0):
                    try:
                        self._write_cmd("SL=0!")
                    except Exception:
                        pass
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
                
                if not self._sleep_for(MIN_RELEASE_GAP_S, song_t0):
                    self.out_q.put(("seq_stop", None))
                    return

            # Case 2: REST or invalid / unsupported note
            else:
                self.out_q.put(("seq_move_time", {
                    "idx": idx,
                    "note": note,
                    "move_time": 0.0,
                    "reached": False,
                }))

                if not self._sleep_for(duration, song_t0):
                    self.out_q.put(("seq_stop", None))
                    return

        self.out_q.put(("seq_done", None))