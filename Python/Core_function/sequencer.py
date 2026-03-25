# Core_function/sequencer.py
import threading
import time
from Core_function.note import note_to_angle

POSITION_TOL_DEG = 15.0
POSITION_WAIT_S = 1.25
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
        ("seq_release", {"idx": idx, "note": note_name, "elapsed": t})
        ("seq_time", elapsed_sec)

        ("seq_paused", {"elapsed": paused_at})
        ("seq_resumed", {"elapsed": resumed_at})

        ("seq_done", {"elapsed": final_elapsed})
        ("seq_stop", {"elapsed": elapsed_when_stopped})

        ("error", "...")
        ("status", "...")
    """

    def __init__(self, song, reader, out_q, stop_evt, pause_evt, get_actual_fn):
        super().__init__(daemon=True)
        self.song = song
        self.reader = reader
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.pause_evt = pause_evt
        self.get_actual_fn = get_actual_fn

        self._pause_notified = False
        self._pause_release_sent = False

    def _write_cmd(self, cmd: str):
        self.reader.write(cmd)

    def _safe_duration(self, duration) -> float:
        try:
            d = float(duration)
        except (TypeError, ValueError):
            d = 0.0
        return max(0.01, d)

    def _elapsed(self, song_t0: float, paused_accum: float) -> float:
        return max(0.0, time.perf_counter() - song_t0 - paused_accum)

    def _handle_pause(self, song_t0: float, paused_accum: float) -> float | None:
        """
        While paused:
        - freeze sequence time
        - send seq_paused / seq_resumed
        - send SL=0! once for safety
        Returns updated paused_accum, or None if stopped.
        """
        if not self.pause_evt.is_set():
            self._pause_notified = False
            self._pause_release_sent = False
            return paused_accum

        pause_start = time.perf_counter()

        if not self._pause_notified:
            paused_at = self._elapsed(song_t0, paused_accum)
            self.out_q.put(("seq_paused", {"elapsed": paused_at}))
            self._pause_notified = True

        if not self._pause_release_sent:
            try:
                self._write_cmd("SL=0!")
            except Exception:
                pass
            self._pause_release_sent = True

        frozen_elapsed = self._elapsed(song_t0, paused_accum)

        while self.pause_evt.is_set() and not self.stop_evt.is_set():
            self.out_q.put(("seq_time", frozen_elapsed))
            time.sleep(0.02)

        if self.stop_evt.is_set():
            return None

        paused_for = time.perf_counter() - pause_start
        paused_accum += paused_for
        resumed_at = self._elapsed(song_t0, paused_accum)

        self.out_q.put(("seq_resumed", {
            "elapsed": resumed_at,
            "paused_for": paused_for,
        }))

        self._pause_notified = False
        self._pause_release_sent = False
        return paused_accum

    def _sleep_for(self, seconds: float, song_t0: float, paused_accum: float):
        """
        Pause-aware sleep.
        Returns:
            (ok: bool, paused_accum: float)
        """
        try:
            seconds = float(seconds)
        except (TypeError, ValueError):
            seconds = 0.0

        seconds = max(0.0, seconds)
        active_elapsed = 0.0
        t0 = time.perf_counter()

        while not self.stop_evt.is_set():
            updated = self._handle_pause(song_t0, paused_accum)
            if updated is None:
                return False, paused_accum
            paused_accum = updated

            now = time.perf_counter()
            dt = now - t0
            t0 = now
            active_elapsed += dt

            self.out_q.put(("seq_time", self._elapsed(song_t0, paused_accum)))

            if active_elapsed >= seconds:
                return True, paused_accum

            time.sleep(0.005)

        return False, paused_accum

    def _wait_until_position(self, target_angle: float, tol_deg: float, timeout_s: float, song_t0: float, paused_accum: float):
        """
        Pause-aware wait until target position.
        Returns:
            (reached: bool, actual_angle: float|None, paused_accum: float)
        """
        try:
            timeout_s = float(timeout_s)
        except (TypeError, ValueError):
            timeout_s = 0.0

        timeout_s = max(0.0, timeout_s)
        active_wait = 0.0
        t0 = time.perf_counter()
        last_actual = None

        while not self.stop_evt.is_set():
            updated = self._handle_pause(song_t0, paused_accum)
            if updated is None:
                return False, last_actual, paused_accum
            paused_accum = updated

            now = time.perf_counter()
            dt = now - t0
            t0 = now
            active_wait += dt

            self.out_q.put(("seq_time", self._elapsed(song_t0, paused_accum)))

            actual = self.get_actual_fn()
            if actual is not None:
                last_actual = actual
                if abs(actual - target_angle) <= tol_deg:
                    return True, actual, paused_accum

            if active_wait >= timeout_s:
                return False, last_actual, paused_accum

            time.sleep(0.005)

        return False, last_actual, paused_accum

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
        paused_accum = 0.0

        for idx, (note, duration) in enumerate(self.song):
            if self.stop_evt.is_set():
                self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                return

            updated = self._handle_pause(song_t0, paused_accum)
            if updated is None:
                self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                return
            paused_accum = updated

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
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

                reached, actual_after_move, paused_accum = self._wait_until_position(
                    target_angle=angle,
                    tol_deg=POSITION_TOL_DEG,
                    timeout_s=POSITION_WAIT_S,
                    song_t0=song_t0,
                    paused_accum=paused_accum,
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
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

                if not reached:
                    self.out_q.put(("error", f"[Sequencer] Position timeout on {note_str}"))

                # Press
                try:
                    self._write_cmd("SL=1!")
                    press_elapsed = self._elapsed(song_t0, paused_accum)
                    self.out_q.put(("seq_press", {
                        "idx": idx,
                        "note": note_str,
                        "elapsed": press_elapsed,
                    }))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=1!: {e}"))
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

                # Hold
                ok, paused_accum = self._sleep_for(duration, song_t0, paused_accum)
                if not ok:
                    try:
                        self._write_cmd("SL=0!")
                    except Exception:
                        pass
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

                # Release
                try:
                    self._write_cmd("SL=0!")
                    release_elapsed = self._elapsed(song_t0, paused_accum)
                    self.out_q.put(("seq_release", {
                        "idx": idx,
                        "note": note_str,
                        "elapsed": release_elapsed,
                    }))
                except Exception as e:
                    self.out_q.put(("error", f"[Sequencer] Failed to send SL=0!: {e}"))
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

                # Small release gap
                ok, paused_accum = self._sleep_for(MIN_RELEASE_GAP_S, song_t0, paused_accum)
                if not ok:
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
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

                ok, paused_accum = self._sleep_for(duration, song_t0, paused_accum)
                if not ok:
                    self.out_q.put(("seq_stop", {"elapsed": self._elapsed(song_t0, paused_accum)}))
                    return

        final_elapsed = self._elapsed(song_t0, paused_accum)
        self.out_q.put(("seq_time", final_elapsed))
        self.out_q.put(("seq_done", {"elapsed": final_elapsed}))