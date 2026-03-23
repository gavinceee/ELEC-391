import threading
import time
import socket

import serial
import serial.tools.list_ports

# ----------------------------
# Config
# ----------------------------
DEFAULT_PORT = "COM5"
DEFAULT_BAUD = 115200
READ_TIMEOUT_S = 0.2

# VOFA
VOFA_IP = "127.0.0.1"
VOFA_TX_PORT = 9000
VOFA_RX_PORT = 1346

tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def parse_telemetry_line(line: str):
    """
    Parse STM32 telemetry line with fixed-order numeric fields.

    Expected format:
        desired, actual, duty, u, P, I, D, tau, dir, sw

    Example:
        60.000, -832.500, 0.320, 3.200, 0.19120, 0.00231, -0.00045, 0.09130, 1, 0

    Returns:
        dict on success, or None on failure.
    """
    if not line:
        return None

    try:
        parts = [x.strip() for x in line.split(",")]
        if len(parts) < 10:
            return None

        return {
            "desired": float(parts[0]),
            "actual": float(parts[1]),
            "duty": float(parts[2]),
            "u": float(parts[3]),
            "p": float(parts[4]),
            "i": float(parts[5]),
            "d": float(parts[6]),
            "tau": float(parts[7]),
            "dir": int(parts[8]),
            "sw": int(parts[9]),
        }

    except (ValueError, TypeError, IndexError):
        return None


class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_q, stop_evt):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=READ_TIMEOUT_S,
            write_timeout=READ_TIMEOUT_S
        )
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
                    except Exception:
                        text = str(line)

                    try:
                        tx_sock.sendto((text + "\n").encode("utf-8"), (VOFA_IP, VOFA_TX_PORT))
                    except Exception:
                        pass

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


class VOFAListener(threading.Thread):
    def __init__(self, out_q, stop_evt):
        super().__init__(daemon=True)
        self.out_q = out_q
        self.stop_evt = stop_evt
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
            if self._sock:
                self._sock.close()
        except Exception:
            pass

    def stop(self):
        self.stop_evt.set()