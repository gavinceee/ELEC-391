# Core_function/sequencer.py
import threading
import time
from Core_function.note import note_to_angle

POSITION_TOL_DEG = 15.0
POSITION_WAIT_S = 1.2
MIN_RELEASE_GAP_S = 0.03
HOME_WAIT_S = 4.5


class Sequencer(threading.Thread):
    """
    Song format:
        [(note_name, duration_sec), ...]

    UI messages pushed to out_q:
        ("seq_started", {"total": N, "song": [...]})

        ("seq_home_start", None)
        ("seq_home_done", {"elapsed": home_elapsed})

        ("seq_note", {
            "idx": idx,
            "total": total,
            "note": note_name,
            "angle": target_angle_or_none,
        })

        ("seq_move_time", {
            "idx": idx,
            "note": note_name,
            "move_time": t,
            "reached": bool,
            "target_angle": target_angle_or_none,
            "actual_angle": actual_angle_or_none,
            "angle_error": err_or_none,
        })

        ("seq_press", {"idx": idx, "note": note_name, "elapsed": t})
        q("seq_release", {"idx": idx, "note": note_name, "elapsed": t})
        ("seq_time", elapsed_sec)

        ("seq_done", {"elapsed": final_elapsed})
        ("seq_stop", {"elapsed": elapsed_when_stopped})

        ("error", "...")
        ("status", "...")
    """

    def __init__(self, song, reader, out_q, stop_evt, get_actual_fn):
        super().__init__(daemon=True)
        self.song = song
        self.reader = reader
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.get_actual_fn = get_actual_fn

    def _write_cmd(self, cmd: str):
        self.reader.write(cmd)

    def _safe_duration(self, duration) -> float:
        try:
            d = float(duration)
        except (TypeError, ValueError):
            d = 0.0
        return max(0.01, d)

    def _elapsed(self, song_t0: float) -> float:
        return max(0.0, time.perf_counter() - song_t0)

    def _sleep_for(self, seconds: float, song_t0: float) -> bool:
        try:
            seconds = float(seconds)
        except (TypeError, ValueError):
            seconds = 0.0

        seconds = max(0.0, seconds)
        end_t = time.perf_counter() + seconds

        while not self.stop_evt.is_set():
            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            if now >= end_t:
                return True

            time.sleep(0.005)

        return False

    def _wait_until_position(self, target_angle: float, tol_deg: float, timeout_s: float, song_t0: float):
        """
        Wait until actual position is close enough to target.
        Returns:
            (reached: bool, actual_angle: float|None)
        """
        try:
            timeout_s = float(timeout_s)
        except (TypeError, ValueError):
            timeout_s = 0.0

        timeout_s = max(0.0, timeout_s)
        t0 = time.perf_counter()
        last_actual = None

        while not self.stop_evt.is_set():
            now = time.perf_counter()
            self.out_q.put(("seq_time", now - song_t0))

            actual = self.get_actual_fn()
            if actual is not None:
                last_actual = actual
                if abs(actual - target_angle) <= tol_deg:
                    return True, actual

            if now - t0 >= timeout_s:
                return False, last_actual

            time.sleep(0.005)

        return False, last_actual

    def run(self):
        total = len(self.song)
        self.out_q.put(("seq_started", {"total": total, "song": self.song}))

        # -------------------------
        # Auto-home before playback
        # -------------------------
        try:
            self.out_q.put(("seq_home_start", None))
            self.out_q.put(("status", "[Sequencer] Auto homing..."))
            self._write_cmd("RE=1!")
        except Exception as e:
            self.out_q.put(("error", f"[Sequencer] Failed to send RE=1!: {e}"))
            self.out_q.put(("seq_stop", {"elapsed": 0.0}))
            return

        home_t0 = time.perf_counter()
        while not self.stop_evt.is_set():
            if time.perf_counter() - home_t0 >= HOME_WAIT_S:
                break
            time.sleep(0.01)

        if self.stop_evt.is_set():
            self.out_q.put(("seq_stop", {"elapsed": 0.0}))
            return

        home_elapsed = max(0.0, time.perf_counter() - home_t0)
        self.out_q.put(("seq_home_done", {"elapsed": home_elapsed}))

        # -------------------------
        # Real song timing starts here
        # -------------------------
        song_t0 = time.perf_counter()

        for idx, (note, duration) in enumerate(self.song):
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                return

            duration = self._safe_duration(duration)
            note_str = str(note).strip().upper()
            is_rest = note_str == "REST"

            angle = None
            if not is_rest:
                try:
                    angle = note_to_angle(note_str)
                except ValueError:
                    angle = None

            self.out_q.put(("seq_note", {
                "idx": idx,
                "total": total,
                "note": note_str,
                "angle": angle,
            }))

            note_start = time.perf_counter()

            # ----------------------------------------
            # Valid hardware-supported note
            # ----------------------------------------
            if angle is not None:
                try:
                    self._write_cmd(f"SP={angle:.2f}!")
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Serial write failed: {e}"))
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

                reached, actual_after_move = self._wait_until_position(
                    target_angle=angle,
                    tol_deg=POSITION_TOL_DEG,
                    timeout_s=POSITION_WAIT_S,
                    song_t0=song_t0,
                )

                move_time = time.perf_counter() - note_start
                angle_error = None
                if actual_after_move is not None:
                    angle_error = actual_after_move - angle

                self.out_q.put(("seq_move_time", {
                    "idx": idx,
                    "note": note_str,
                    "move_time": move_time,
                    "reached": reached,
                    "target_angle": angle,
                    "actual_angle": actual_after_move,
                    "angle_error": angle_error,
                }))

                if self.stop_evt.is_set():
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

                if not reached:
                    self.out_q.put(("error", f"[Sequencer] Position timeout on {note_str}"))

               # Press
                try:
                    self._write_cmd("SL=1!")
                    press_elapsed = self._elapsed(song_t0)
                    self.out_q.put(("seq_press", {
                        "idx": idx,
                        "note": note_str,
                        "elapsed": press_elapsed,
                    }))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=1!: {e}"))
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

                # Hold
                if not self._sleep_for(duration, song_t0):
                    try:
                        self._write_cmd("SL=0!")
                    except Exception:
                        pass
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

                # Release
                try:
                    self._write_cmd("SL=0!")
                    release_elapsed = self._elapsed(song_t0)
                    self.out_q.put(("seq_release", {
                        "idx": idx,
                        "note": note_str,
                        "elapsed": release_elapsed,
                    }))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=0!: {e}"))
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

                # Small release gap
                if not self._sleep_for(MIN_RELEASE_GAP_S, song_t0):
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

            # ----------------------------------------
            # REST or invalid / unsupported note
            # ----------------------------------------
            else:
                self.out_q.put(("seq_move_time", {
                    "idx": idx,
                    "note": note_str,
                    "move_time": 0.0,
                    "reached": False,
                    "target_angle": None,
                    "actual_angle": None,
                    "angle_error": None,
                }))

                if not self._sleep_for(duration, song_t0):
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0)}))
                    return

        final_elapsed = self._elapsed(song_t0)
        self.out_q.put(("seq_time", final_elapsed))
        self.out_q.put(("seq_done", {"elapsed": final_elapsed}))