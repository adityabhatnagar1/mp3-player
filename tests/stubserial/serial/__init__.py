"""Tiny stand-in for pyserial (not installable offline) that talks to a pty. Same API subset as play_mp3.py uses."""
import fcntl, os, select, termios, time, tty, struct

class SerialException(OSError): pass
class SerialTimeoutException(SerialException): pass

class Serial:
    def __init__(self):
        self.port = None; self.baudrate = 9600; self.timeout = None; self.write_timeout = None
        self.rtscts = self.dsrdtr = self.xonxoff = False; self.dtr = self.rts = True
        self.fd = None
    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
    @property
    def in_waiting(self):
        buf = fcntl.ioctl(self.fd, termios.FIONREAD, b"\0\0\0\0")
        return struct.unpack("i", buf)[0]
    def read(self, n):
        try: return os.read(self.fd, n)
        except BlockingIOError: return b""
    def write(self, data):
        deadline = time.monotonic() + (self.write_timeout or 1e9)
        view = memoryview(data)
        while len(view):
            try:
                k = os.write(self.fd, view); view = view[k:]
            except BlockingIOError:
                if time.monotonic() > deadline: raise SerialTimeoutException("write timeout")
                select.select([], [self.fd], [], 0.01)
        return len(data)
    def reset_input_buffer(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)
    def close(self):
        if self.fd is not None: os.close(self.fd); self.fd = None
