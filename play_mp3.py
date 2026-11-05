#!/usr/bin/env python3
"""
play_mp3.py - stream an MP3 (or a generated test tone) to the ESP32 + PCM5102A over USB serial.

    python play_mp3.py --port COM5 --file "C:\\Music\\song.mp3"
    python play_mp3.py --port COM5 --test-tone            # no FFmpeg needed
    python play_mp3.py --list-ports

Pipeline:  FFmpeg (MP3 -> s16le PCM)  ->  framed + CRC'd packets  ->  USB serial  ->  ESP32 ring buffer -> I2S

The ESP32 reports its buffer fill ~50 times a second; this script only sends while the
ring has room (credit-based flow control), so playback is locked to the ESP32's own clock.

Protocol (must match stream_proto.h):
  frame = AA 55 | type u8 | seq u8 | len u16le | payload | crc16le   (CRC-16/CCITT-FALSE over type..payload)
"""

import argparse
import binascii
import collections
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import threading
import time
from array import array

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is not installed.  Run:  python -m pip install pyserial")

# ----------------------------------------------------------------------------- protocol

MAGIC = b"\xAA\x55"
T_PING, T_HELLO, T_DATA, T_END, T_ABORT = 0x01, 0x02, 0x03, 0x04, 0x05
T_STATUS = 0x81
MAX_PAYLOAD = 1024
PROTO_VERSION = 1

ST_IDLE, ST_BUFFERING, ST_PLAYING, ST_DRAINING = 0, 1, 2, 3
STATE_NAMES = {ST_IDLE: "idle", ST_BUFFERING: "buffering", ST_PLAYING: "playing", ST_DRAINING: "draining"}

ERR_NAMES = {0: "none", 1: "bad HELLO parameters", 2: "bad/corrupt data frame",
             3: "ring overflow (flow-control violated)", 4: "session timed out"}

STATUS_FMT = "<BBBBHHHII"          # version state error pad session underruns errors fill capacity
STATUS_LEN = struct.calcsize(STATUS_FMT)   # 18


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    return binascii.crc_hqx(data, crc)


def encode_frame(ftype, seq, payload=b""):
    body = struct.pack("<BBH", ftype, seq & 0xFF, len(payload)) + payload
    return MAGIC + body + struct.pack("<H", crc16(body))


class Status:
    __slots__ = ("version", "state", "error", "session", "underruns", "errors", "fill", "capacity", "t")

    def __init__(self, payload):
        (self.version, self.state, self.error, _pad, self.session,
         self.underruns, self.errors, self.fill, self.capacity) = struct.unpack(STATUS_FMT, payload[:STATUS_LEN])
        self.t = time.monotonic()


class FrameReader:
    """Extracts STATUS frames from the ESP32's output, which also contains plain-text log lines.
    Anything that is not a valid frame is treated as text and skipped (and kept for --verbose)."""

    def __init__(self):
        self.buf = bytearray()
        self.text = bytearray()

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # keep a trailing 0xAA: it may be the first half of a magic split across reads
                keep = 1 if self.buf.endswith(b"\xAA") else 0
                self.text += self.buf[:len(self.buf) - keep]
                del self.buf[:len(self.buf) - keep]
                break
            if i > 0:
                self.text += self.buf[:i]
                del self.buf[:i]
            if len(self.buf) < 6:
                break
            ftype, seq, ln = struct.unpack_from("<BBH", self.buf, 2)
            if ln > 64:                                   # device frames are tiny: this was not a frame
                self.text += self.buf[:1]
                del self.buf[:1]
                continue
            total = 6 + ln + 2
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:6 + ln])
            (rx_crc,) = struct.unpack_from("<H", self.buf, 6 + ln)
            if crc16(body) != rx_crc:
                self.text += self.buf[:1]
                del self.buf[:1]
                continue
            frames.append((ftype, seq, bytes(self.buf[6:6 + ln])))
            del self.buf[:total]
        return frames


# ----------------------------------------------------------------------------- PCM sources

class FFmpegSource:
    """Decodes a file to interleaved s16le PCM with FFmpeg. read(n) returns b'' at EOF."""

    def __init__(self, ffmpeg, path, rate, channels, volume, swap_lr):
        filters = []
        if swap_lr and channels == 2:
            filters.append("pan=stereo|c0=c1|c1=c0")
        if volume != 1.0:
            filters.append("volume=%.4f" % volume)
        cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin", "-i", path, "-vn"]
        if filters:
            cmd += ["-af", ",".join(filters)]
        cmd += ["-ac", str(channels), "-ar", str(rate), "-f", "s16le", "-acodec", "pcm_s16le", "pipe:1"]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE)
        self.stderr_lines = []
        self._t = threading.Thread(target=self._drain_stderr, daemon=True)
        self._t.start()

    def _drain_stderr(self):
        for line in self.proc.stderr:
            self.stderr_lines.append(line.decode("utf-8", "replace").rstrip())

    def read(self, n):
        return self.proc.stdout.read(n)

    def close(self):
        try:
            if self.proc.poll() is None:
                self.proc.kill()
            self.proc.wait(timeout=2)
        except Exception:
            pass

    def error_text(self):
        self._t.join(timeout=1)
        return "\n".join(self.stderr_lines)

    def returncode(self):
        return self.proc.poll()


class ToneSource:
    """Generated sine test tone (stage-2 test). Needs no FFmpeg."""

    def __init__(self, rate, channels, seconds, volume, freq=440.0, which="both"):
        self.rate, self.channels = rate, channels
        self.remaining = int(seconds * rate)          # frames
        self.step = 2.0 * math.pi * freq / rate
        self.phase = 0.0
        self.amp = 0.5 * volume * 32767.0
        self.which = which

    def read(self, nbytes):
        frame_bytes = 2 * self.channels
        frames = min(self.remaining, nbytes // frame_bytes)
        if frames <= 0:
            return b""
        out = array("h")
        for _ in range(frames):
            v = int(self.amp * math.sin(self.phase))
            self.phase += self.step
            if self.channels == 1:
                out.append(v)
            else:
                out.append(v if self.which in ("both", "left") else 0)
                out.append(v if self.which in ("both", "right") else 0)
        if self.phase > 1e6:
            self.phase %= 2.0 * math.pi
        self.remaining -= frames
        if sys.byteorder == "big":
            out.byteswap()
        return out.tobytes()

    def close(self):
        pass


# ----------------------------------------------------------------------------- helpers

def fmt_time(sec):
    sec = max(0, int(sec))
    return "%d:%02d" % (sec // 60, sec % 60)


def probe_duration(path):
    ffprobe = shutil.which("ffprobe")
    if not ffprobe:
        return None
    try:
        out = subprocess.run([ffprobe, "-v", "error", "-show_entries", "format=duration",
                              "-of", "default=nw=1:nk=1", path],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        return float(out)
    except Exception:
        return None


class DeviceError(Exception):
    pass


class Link:
    """Serial link: framed TX, STATUS RX."""

    def __init__(self, ser, verbose=False):
        self.ser = ser
        self.reader = FrameReader()
        self.seq = 0
        self.status = None
        self.last_tx = time.monotonic()
        self.verbose = verbose

    def send(self, ftype, payload=b""):
        frame = encode_frame(ftype, self.seq, payload)
        self.seq = (self.seq + 1) & 0xFF
        try:
            self.ser.write(frame)
        except serial.SerialTimeoutException:
            raise DeviceError("serial write timed out - the ESP32 stopped reading "
                              "(reset, crashed, or USB cable unplugged?)")
        except (serial.SerialException, OSError) as e:
            raise DeviceError("serial write failed: %s" % e)
        self.last_tx = time.monotonic()

    def poll(self):
        """Read whatever arrived, return the list of new STATUS objects (also stored in self.status)."""
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b""
        except (serial.SerialException, OSError) as e:
            raise DeviceError("serial read failed: %s" % e)
        new = []
        if data:
            for ftype, _seq, payload in self.reader.feed(data):
                if ftype == T_STATUS and len(payload) >= STATUS_LEN:
                    st = Status(payload)
                    if st.version == PROTO_VERSION:
                        self.status = st
                        new.append(st)
            if self.verbose and self.reader.text:
                sys.stderr.write(self.reader.text.decode("utf-8", "replace"))
                sys.stderr.flush()
            self.reader.text.clear()
        return new


def open_serial(port, baud):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0
    ser.write_timeout = 3.0
    ser.rtscts = False
    ser.dsrdtr = False
    ser.xonxoff = False
    ser.dtr = False        # try not to reset the board when the port opens (many boards reset anyway;
    ser.rts = False        # the handshake below copes with both cases)
    ser.open()
    return ser


def handshake(link, timeout, verbose):
    deadline = time.monotonic() + timeout
    link.status = None
    while time.monotonic() < deadline:
        link.send(T_PING)
        t_end = time.monotonic() + 0.25
        while time.monotonic() < t_end:
            if link.poll():
                return link.status
            time.sleep(0.005)
    return None


def start_session(link, rate, channels, timeout=5.0):
    session = random.randint(1, 0xFFFF)
    hello = struct.pack("<IBBH", rate, channels, 16, session)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        link.send(T_HELLO, hello)
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            for st in link.poll():
                if st.session == session and st.state in (ST_BUFFERING, ST_PLAYING, ST_DRAINING):
                    return session, st
                if st.error == 1:
                    raise DeviceError("ESP32 rejected the stream parameters (rate=%d, channels=%d)"
                                      % (rate, channels))
            time.sleep(0.005)
    raise DeviceError("ESP32 did not accept the stream session (no matching STATUS after HELLO)")


# ----------------------------------------------------------------------------- main streaming loop

def stream(link, source, rate, channels, chunk, total_seconds, high_frac, quiet):
    bytes_per_sec = rate * channels * 2
    st = link.status
    capacity = st.capacity
    high = int(capacity * high_frac)
    est_fill = st.fill
    sent = 0
    t_start = time.monotonic()
    last_print = 0.0
    last_status_seen = time.monotonic()
    frame_bytes = 2 * channels

    recent = collections.deque()          # (time, bytes) of DATA frames sent recently
    LATENCY = 0.06                        # a STATUS may not yet include data sent in the last ~60 ms

    def absorb(new):
        """Update the fill estimate from the newest STATUS. Conservative: add what we sent
        within the last LATENCY seconds, which the ESP32 may not have counted yet."""
        nonlocal est_fill, last_status_seen
        if not new:
            return
        now = time.monotonic()
        while recent and now - recent[0][0] > LATENCY:
            recent.popleft()
        est_fill = new[-1].fill + sum(n for _, n in recent)
        last_status_seen = now

    def progress(final=False):
        nonlocal last_print
        now = time.monotonic()
        if quiet or (not final and now - last_print < 0.25):
            return
        last_print = now
        s = link.status
        played = max(0, sent - est_fill) / bytes_per_sec
        total = ("/ " + fmt_time(total_seconds)) if total_seconds else ""
        sys.stdout.write("\r  %s %s  buffer %3d%% (%4.2fs)  state=%-9s underruns=%d errors=%d   " % (
            fmt_time(played), total, 100 * est_fill // max(1, capacity), est_fill / bytes_per_sec,
            STATE_NAMES.get(s.state, "?"), s.underruns, s.errors))
        sys.stdout.flush()

    eof = False
    while not eof:
        data = source.read(chunk)
        if not data:
            break
        data = data[:len(data) - (len(data) % frame_bytes)]      # never split a sample frame
        if not data:
            break

        # Wait for room in the ESP32 ring (credit-based flow control).
        while est_fill + len(data) > high:
            absorb(link.poll())
            if link.status.state == ST_IDLE:
                raise DeviceError("ESP32 ended the session (%s)" % ERR_NAMES.get(link.status.error, "?"))
            if time.monotonic() - last_status_seen > 1.5:
                raise DeviceError("ESP32 stopped reporting status (reset or unplugged?)")
            if time.monotonic() - link.last_tx > 0.25:
                link.send(T_PING)                                  # keep-alive; also elicits a STATUS
            progress()
            time.sleep(0.002)

        link.send(T_DATA, data)
        recent.append((time.monotonic(), len(data)))
        est_fill += len(data)
        sent += len(data)
        absorb(link.poll())
        progress()

    # ---- end of data: tell the ESP32 to play out what it has, then wait for IDLE ----
    link.send(T_END)
    last_end = time.monotonic()
    deadline = time.monotonic() + est_fill / bytes_per_sec + 6.0
    while time.monotonic() < deadline:
        absorb(link.poll())
        s = link.status
        if s.state == ST_IDLE:
            break
        if s.state != ST_DRAINING and time.monotonic() - last_end > 0.5:
            link.send(T_END)                                       # END frame may have been lost
            last_end = time.monotonic()
        if time.monotonic() - last_status_seen > 2.0:
            raise DeviceError("ESP32 stopped reporting status while draining")
        progress()
        time.sleep(0.005)
    else:
        raise DeviceError("timed out waiting for the ESP32 to finish playing")

    est_fill = 0
    progress(final=True)
    return sent, time.monotonic() - t_start


# ----------------------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser(description="Stream an MP3 to the ESP32 + PCM5102A over USB serial.")
    ap.add_argument("--port", help="serial port, e.g. COM5 (Windows) or /dev/ttyUSB0")
    ap.add_argument("--file", help="MP3 (or any FFmpeg-readable audio) file")
    ap.add_argument("--test-tone", action="store_true", help="stream a generated 440 Hz tone instead of a file")
    ap.add_argument("--tone-seconds", type=float, default=5.0)
    ap.add_argument("--tone-channel", choices=("both", "left", "right"), default="both",
                    help="with --channels 2: which channel carries the tone (L/R wiring check)")
    ap.add_argument("--list-ports", action="store_true", help="list serial ports and exit")
    ap.add_argument("--ping-only", action="store_true", help="only test the serial handshake, then exit")
    ap.add_argument("--baud", type=int, default=921600, help="must match stream::BAUD in the sketch")
    ap.add_argument("--rate", type=int, default=32000, help="PCM sample rate sent to the ESP32 (default 32000)")
    ap.add_argument("--channels", type=int, choices=(1, 2), default=1, help="1 = mono (default), 2 = stereo")
    ap.add_argument("--volume", type=float, default=0.5,
                    help="linear gain applied by FFmpeg, default 0.5 (-6 dB). Values >1 can clip.")
    ap.add_argument("--swap-lr", action="store_true", help="swap left/right (stereo only)")
    ap.add_argument("--chunk", type=int, default=1024, help="PCM bytes per DATA frame (<=1024)")
    ap.add_argument("--ffmpeg", default="ffmpeg", help="path to ffmpeg executable")
    ap.add_argument("--boot-wait", type=float, default=2.0,
                    help="seconds to wait after opening the port (the board may reset), default 2")
    ap.add_argument("--connect-timeout", type=float, default=15.0)
    ap.add_argument("--high-water", type=float, default=0.75,
                    help="stop sending when the ESP32 ring is this full (0.3 - 0.9)")
    ap.add_argument("--force", action="store_true", help="skip the serial-bandwidth sanity check")
    ap.add_argument("--verbose", action="store_true", help="show the ESP32's text log lines")
    ap.add_argument("--quiet", action="store_true", help="no progress line")
    args = ap.parse_args()

    if args.list_ports:
        ports = list(list_ports.comports())
        if not ports:
            print("No serial ports found.")
        for p in ports:
            print("%-12s %s" % (p.device, p.description))
        return 0

    if not args.port:
        ap.error("--port is required (use --list-ports to see what is available)")
    if not args.test_tone and not args.file and not args.ping_only:
        ap.error("give --file PATH (or --test-tone, or --ping-only)")
    if args.ping_only:
        args.test_tone, args.tone_seconds = True, 0.0
    if not (8000 <= args.rate <= 48000):
        ap.error("--rate must be between 8000 and 48000")
    if not (1 <= args.chunk <= MAX_PAYLOAD):
        ap.error("--chunk must be 1..%d" % MAX_PAYLOAD)
    if not (0.3 <= args.high_water <= 0.9):
        ap.error("--high-water must be 0.3..0.9")
    if not (0.0 < args.volume <= 4.0):
        ap.error("--volume must be > 0 and <= 4")
    args.chunk -= args.chunk % (2 * args.channels)

    # ---- serial bandwidth sanity check ----
    need = args.rate * args.channels * 2 * (1 + 8.0 / args.chunk)      # payload + 8 bytes framing per chunk
    avail = args.baud / 10.0                                           # 8N1 = 10 bits per byte
    print("Stream: %d Hz, %s, 16-bit  = %.1f KB/s of %.1f KB/s serial capacity (%.0f%%)" % (
        args.rate, "mono" if args.channels == 1 else "stereo", need / 1000, avail / 1000, 100 * need / avail))
    if need > 0.90 * avail and not args.force:
        print("ERROR: that leaves no headroom on the serial link. Use --channels 1 and/or a lower --rate "
              "(e.g. --rate 32000), or raise the baud rate on BOTH sides (stream::BAUD in the sketch and "
              "--baud here). Override with --force.", file=sys.stderr)
        return 2

    # ---- source ----
    duration = None
    if args.test_tone:
        source = ToneSource(args.rate, args.channels, args.tone_seconds, args.volume, which=args.tone_channel)
        duration = args.tone_seconds
        print("Source: generated 440 Hz test tone, %.1f s" % args.tone_seconds)
    else:
        if not os.path.isfile(args.file):
            print("ERROR: file not found: %s" % args.file, file=sys.stderr)
            return 2
        if shutil.which(args.ffmpeg) is None and not os.path.isfile(args.ffmpeg):
            print("ERROR: FFmpeg not found ('%s'). Install it and make sure 'ffmpeg -version' works in a new "
                  "terminal, or pass --ffmpeg C:\\path\\to\\ffmpeg.exe" % args.ffmpeg, file=sys.stderr)
            return 2
        duration = probe_duration(args.file)
        source = FFmpegSource(args.ffmpeg, args.file, args.rate, args.channels, args.volume, args.swap_lr)
        print("Source: %s%s" % (args.file, (" (%s)" % fmt_time(duration)) if duration else ""))

    # ---- serial ----
    try:
        ser = open_serial(args.port, args.baud)
    except (serial.SerialException, OSError) as e:
        print("ERROR: cannot open %s: %s\nClose the Arduino Serial Monitor/Plotter (it holds the port) and "
              "check --list-ports." % (args.port, e), file=sys.stderr)
        source.close()
        return 2

    link = Link(ser, args.verbose)
    rc = 0
    try:
        if args.boot_wait > 0:
            time.sleep(args.boot_wait)             # board may be rebooting because the port opened
        ser.reset_input_buffer()

        st = handshake(link, args.connect_timeout, args.verbose)
        if st is None:
            raise DeviceError("no reply from the ESP32 on %s at %d baud. Check: right COM port, sketch flashed "
                              "(Serial Monitor must be closed), baud rate matches stream::BAUD, USB cable carries data."
                              % (args.port, args.baud))
        print("ESP32 connected: protocol v%d, ring %d bytes, state %s"
              % (st.version, st.capacity, STATE_NAMES.get(st.state, "?")))

        if args.ping_only:
            print("Handshake OK.")
            return 0

        session, st = start_session(link, args.rate, args.channels)
        if not args.quiet:
            print("Playing (Ctrl+C to stop)...")
        sent, secs = stream(link, source, args.rate, args.channels, args.chunk, duration,
                            args.high_water, args.quiet)
        s = link.status
        if not args.test_tone and source.returncode() not in (None, 0):
            print("\nWARNING: FFmpeg exited with code %s:\n%s" % (source.returncode(), source.error_text()),
                  file=sys.stderr)
        print("\nDone: %.1f s of audio sent in %.1f s. Underruns: %d, frame errors: %d."
              % (sent / (args.rate * args.channels * 2.0), secs, s.underruns, s.errors))
        if s.underruns or s.errors:
            print("Note: underruns mean the laptop could not keep the buffer filled (CPU/USB load); "
                  "errors mean frames were lost or corrupted on the link.")

    except KeyboardInterrupt:
        print("\nStopping...")
        try:
            link.send(T_ABORT)
            time.sleep(0.2)
        except Exception:
            pass
        rc = 130
    except DeviceError as e:
        print("\nERROR: %s" % e, file=sys.stderr)
        rc = 1
    finally:
        source.close()
        try:
            ser.close()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
