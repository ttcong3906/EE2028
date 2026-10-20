#!/usr/bin/env python3
"""
EE2028 "The Last of EE2028" - Command Centre dashboard (spec v3.2, section 17).

Replaces an on-board display: reads the drone over USB (ST-LINK virtual COM
port, 115200 8N1), draws its state, and relays the pilot controller (board 2,
ROLE CONTROLLER) to the drone as ordinary UART commands.

    pip install pyserial            (Tkinter ships with Python on Windows)
    python dashboard.py             pick the COM ports in the top bar
    python dashboard.py --drone COM5 --ctl COM7
    python dashboard.py --demo      built-in fake drone, no hardware needed

Run it with Windows Python (not inside WSL): the COM ports live on Windows.
Only one program can open a COM port, so close TeraTerm first. The Console
tab shows every raw line and lets you type commands like TeraTerm.

Keyboard (when the command box does not have focus):
    arrows = AIM, L = LOCK, P = POWER NEXT, space = CHARGE, R = RESCUE
"""
import argparse
import collections
import math
import queue
import random
import re
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:            # dashboard still opens (demo mode works)
    serial = None

BAUD = 115200
MODES = ("STANDBY", "BATTLE", "ATTACKED", "TERMINATED")
MODE_COLOR = {"STANDBY": "#2e7d32", "BATTLE": "#ef6c00", "ATTACKED": "#c62828",
              "TERMINATED": "#616161", "NO SIGNAL": "#37474f"}
SENSORS = [("T", "Temperature", "C"), ("P", "Pressure", "hPa"), ("H", "Humidity", "%RH"),
           ("A", "Acceleration", "m/s^2"), ("G", "Angular rate", "dps"), ("M", "Magnetic field", "uT")]
THRESH = {"T": (10, 35), "P": (950, 1040), "H": (20, 80), "A": (3, 15), "G": (None, 300), "M": (10, 100)}
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
DRONE_TIMEOUT_S = 2.5
CTL_TIMEOUT_S = 2.5

BG, PANEL, FG, DIM = "#101418", "#1b2128", "#e8eaed", "#9aa0a6"


# =============================================================================
# Transport: one serial port read by a background thread
# =============================================================================
class SerialLink:
    def __init__(self, name, q):
        self.name, self.q, self.ser, self.alive = name, q, None, False

    def open(self, port):
        if serial is None:
            raise RuntimeError("pyserial is not installed: pip install pyserial")
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.alive = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while self.alive:
            try:
                data = self.ser.read(512)
            except Exception as e:                     # unplugged
                self.q.put((self.name, "__error__", str(e)))
                self.alive = False
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self.q.put((self.name, "line", clean_line(raw.decode("ascii", "replace"))))

    def send(self, text):
        if self.ser and self.alive:
            try:
                self.ser.write((text + "\r").encode("ascii", "replace"))
            except Exception as e:
                self.q.put((self.name, "__error__", str(e)))

    def close(self):
        self.alive = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


def clean_line(s):
    """Undo the drone's console redraw: keep what is after the last CR, drop VT100."""
    s = s.rstrip("\r")
    if "\r" in s:
        s = s.rsplit("\r", 1)[1]
    return ANSI.sub("", s).strip()


# =============================================================================
# Demo: a small fake drone that speaks the same protocol (no hardware)
# =============================================================================
class FakeDrone:
    def __init__(self, name, q):
        self.name, self.q, self.alive = name, q, False
        self.lock = threading.Lock()
        self.mode, self.dash = "STANDBY", False
        self.e, self.sel, self.p, self.cd_until = 0, False, 1, 0.0
        self.map, self.ax, self.ay, self.wave = [0] * 8, 3, 3, 0
        self.down = self.shots = 0
        self.alt = 0.0
        self._scout_new()
        self.t_attack = 0.0
        self.s = {"T": 26.0, "P": 1008.5, "H": 58.0, "A": 9.81, "G": 1.2, "M": 42.0}

    def out(self, line):
        self.q.put((self.name, "line", line))

    def open(self, port):
        self.alive = True
        threading.Thread(target=self._run, daemon=True).start()

    def close(self):
        self.alive = False

    def _cells(self):
        return sum(bin(r).count("1") for r in self.map)

    def _run(self):
        self.out("=== Drone #3 online === mount: chip-up, silent: OFF")
        self.out("[EVT] System check: sensors OK, LED matrix OK, 5-way OK, IR on A0. Type HELP for commands.")
        self.out("Entering STANDBY MODE")
        t_tlm = t_hud = t_spread = t_att = time.time()
        while self.alive:
            time.sleep(0.05)
            now = time.time()
            with self.lock:
                if self.mode == "STANDBY" and now - t_tlm >= 1:
                    t_tlm = now
                    for k, step in (("T", .05), ("P", .02), ("H", .3), ("A", .05), ("G", .3), ("M", .4)):
                        self.s[k] = max(0.0, self.s[k] + random.uniform(-step, step))
                    s = self.s
                    if s["H"] > 80:
                        self.out(f"[WARN] H: {s['H']:.2f} %RH, exceeds threshold of 80.00 %RH")
                    else:
                        self.out(f"[TLM] T: {s['T']:.2f} C, P: {s['P']:.2f} hPa, H: {s['H']:.2f} %RH, "
                                 f"A: {s['A']:.2f} m/s^2, G: {s['G']:.2f} dps, M: {s['M']:.2f} uT")
                if self.mode == "BATTLE" and now - t_spread >= 2.5 and self._cells():
                    t_spread = now
                    cand = [(x, y) for y in range(8) for x in range(8) if not self.map[y] >> (7 - x) & 1
                            and any(0 <= x + dx < 8 and 0 <= y + dy < 8 and self.map[y + dy] >> (7 - x - dx) & 1
                                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)))]
                    if cand:
                        x, y = random.choice(cand)
                        self.map[y] |= 0x80 >> x
                    self._map()
                if self.mode == "BATTLE" and self.cd_until and now >= self.cd_until:
                    self.cd_until = 0
                    self.out("[EVT] Laser ready")
                    self._try_fire()
                if self.mode == "ATTACKED":
                    left = 10 - (now - self.t_attack)
                    if left <= 0:
                        self.out("[EVT] Drone #3: no rescue. Black box saved. Self-destructing. Cutting all communication.")
                        self.mode = "TERMINATED"
                        self.alive = False
                        break
                    if now - t_att >= 1:
                        t_att = now
                        self.out(f"[ATTACK] Drone #3 Was Attacked! Rescue in {math.ceil(left)} s")
                if self.dash and now - t_hud >= 0.25:
                    t_hud = now
                    self._hud(now)

    def _hud(self, now):
        cd = max(0.0, self.cd_until - now) if self.cd_until else 0.0
        resc = math.ceil(10 - (now - self.t_attack)) if self.mode == "ATTACKED" else 0
        hab = 1 if 20 <= self.s["T"] <= 32 and 40 <= self.s["H"] <= 70 else 0
        self.out(f"[HUD] ID 3 MODE {self.mode} E {self.e} P {self.p if self.sel else 5} SEL {int(self.sel)} "
                 f"CD {cd:.1f} WAVE {self.wave} TGT {self._cells() if self.wave else 0} DOWN {self.down} "
                 f"SHOTS {self.shots} ALT {self.alt:.1f} HAB {hab} RESC {resc} IR 0")
        if self.mode == "STANDBY":
            self._scout_send()

    def _scout_new(self):
        self.fog, self.found = [0xFF] * 8, [0] * 8
        self.nests = [0] * 8
        while sum(bin(r).count("1") for r in self.nests) < 3:
            self.nests[random.randrange(6)] |= 0x80 >> random.randrange(8)
        self.sx, self.sy = 3, 7
        self._scan()

    def _scan(self):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                x, y = self.sx + dx, self.sy + dy
                if 0 <= x < 8 and 0 <= y < 8:
                    self.fog[y] &= ~(0x80 >> x) & 0xFF
                    if self.nests[y] & (0x80 >> x) and not self.found[y] & (0x80 >> x):
                        self.found[y] |= 0x80 >> x
                        n = sum(bin(r).count("1") for r in self.found)
                        self.out(f"[EVT] Scouting: nest found at ({x},{y}) ({n}/3)")

    def _scout_send(self):
        n = sum(bin(r).count("1") for r in self.found)
        scan = 64 - sum(bin(r).count("1") for r in self.fog)
        self.out("[SCOUT] FOG " + "".join(f"{r:02X}" for r in self.fog) + " NEST " +
                 "".join(f"{r:02X}" for r in self.found) + f" POS {self.sx} {self.sy} FOUND {n} 3 SCAN {scan}")

    def _map(self):
        if self.dash:
            self.out("[MAP] " + "".join(f"{r:02X}" for r in self.map) + f" AIM {self.ax} {self.ay}")

    def _wave(self, n):
        self.wave, self.map = n, [0] * 8
        if n == 1:                               # scouted nests are bigger (plus of 5)
            for y in range(8):
                for x in range(8):
                    if self.nests[y] & (0x80 >> x):
                        cells = [(x, y)] + ([(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
                                            if self.found[y] & (0x80 >> x) else [])
                        for cx, cy in cells:
                            if 0 <= cx < 8 and 0 <= cy < 8:
                                self.map[cy] |= 0x80 >> cx
            self.out("[EVT] Wave 1 incoming: 3 outbreaks")
            self._map()
            return
        for _ in range(2 + n):
            self.map[random.randrange(5)] |= 0x80 >> random.randrange(8)
        self.out(f"[EVT] Wave {n} incoming: {2 + n} outbreaks")
        self._map()

    def _pw(self):
        return self.p if self.sel else 5

    def _try_fire(self):
        p = self._pw()
        if self.mode != "BATTLE" or self.cd_until or self.e < p:
            return
        before = self.e
        self.e -= p
        self.shots += 1
        cleared = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                inside = p >= 4 or (dx == 0 and dy == 0) or (p >= 2 and (dx == 0 or dy == 0))
                x, y = self.ax + dx, self.ay + dy
                if inside and 0 <= x < 8 and 0 <= y < 8 and self.map[y] >> (7 - x) & 1:
                    self.map[y] &= ~(0x80 >> x)
                    cleared += 1
        self.down += cleared
        self.out(f"[EVT] LASER FIRED ({p}/10) at ({self.ax},{self.ay}): {cleared} cells cleared. "
                 f"Energy {before}/10 -> {self.e}/10. Zombies down: {self.down}")
        if self.sel and p >= 2:
            self.cd_until = time.time() + p
        self._map()
        if not self._cells():
            self.out(f"[EVT] Wave {self.wave} cleared! AREA CLEARED")
            self._wave(self.wave + 1)

    def send(self, text):
        with self.lock:
            self.out("> " + text)
            v = text.upper().split()
            if not v:
                return
            c, a = v[0], v[1:]
            if c == "DASH":
                self.dash = a[:1] == ["ON"]
                self.out(f"[CMD] OK DASH {'ON' if self.dash else 'OFF'}")
            elif c == "MODE" and a == ["BATTLE"] and self.mode == "STANDBY":
                self.mode, self.ax, self.ay = "BATTLE", 3, 3
                self.out("Entering BATTLE MODE")
                self._wave(1)
            elif c == "AIM" and self.mode == "STANDBY" and len(a) == 2:
                sgn = lambda v: (int(v) > 0) - (int(v) < 0)
                self.sx = min(7, max(0, self.sx + sgn(a[0])))
                self.sy = min(7, max(0, self.sy + sgn(a[1])))
                self._scan()
            elif c == "MODE" and a == ["STANDBY"] and self.mode == "BATTLE":
                self.mode, self.wave, self.map = "STANDBY", 0, [0] * 8
                self._scout_new()
                self.out("Entering STANDBY MODE")
            elif c == "CHARGE" and self.mode == "BATTLE":
                b = self.e
                self.e = min(10, self.e + 3)
                self.out(f"[EVT] LASER charged: {b}/10 -> {self.e}/10")
                self._try_fire()
            elif c == "AIM" and self.mode == "BATTLE" and len(a) == 2:
                sgn = lambda s: (int(s) > 0) - (int(s) < 0)
                self.ax = min(7, max(0, self.ax + sgn(a[0])))
                self.ay = min(7, max(0, self.ay + sgn(a[1])))
                self._map()
            elif c == "LOCK" and self.mode == "BATTLE":
                cells = [(x, y) for y in range(8) for x in range(8) if self.map[y] >> (7 - x) & 1]
                if cells:
                    self.ax, self.ay = min(cells, key=lambda t: abs(t[0] - self.ax) + abs(t[1] - self.ay))
                self.out(f"[EVT] Target locked at ({self.ax},{self.ay})")
                self._map()
            elif c == "POWER" and self.mode == "BATTLE":
                if not self.sel:
                    self.sel, self.p = True, 1
                elif self.p < 5:
                    self.p += 1
                else:
                    self.sel, self.cd_until = False, 0
                self.out(f"[EVT] Weapon: power {self._pw()}/10" if self.sel else "[EVT] Weapon: rule mode 5/10, no cooldown")
                self._try_fire()
            elif c == "FLIP" and self.mode == "BATTLE":          # demo only
                self.mode, self.t_attack = "ATTACKED", time.time()
                self.out("[EVT] Drone #3 flipped! The Last of EE2028")
            elif c == "RESCUE" and self.mode == "ATTACKED":
                self.mode = "BATTLE"
                self.out("[EVT] Drone #3 rescued! Returning to BATTLE MODE")
            elif c == "BLACKBOX" and self.mode == "STANDBY":
                self.out("[BBOX] Last sortie (RAM), drone #3, 3 records")
                self.out(f"[BBOX] LAST MISSION: waves cleared {max(0, self.wave - 1)}, shots fired {self.shots}, "
                         f"cells cleared {self.down}, flight 42 s, END: RETURNED")
                self.out("[BBOX] t=  0.0s BATTLE    SORTIE START energy 0/10, power 5/10")
                self.out("[BBOX] t=  0.0s BATTLE    WAVE START 1, 3 outbreaks")
                self.out("[BBOX] t= 42.0s BATTLE    BACK TO STANDBY")
            else:
                self.out(f"[CMD] demo drone ignores '{text}' in {self.mode}")


# =============================================================================
# Drone state, rebuilt from the text lines
# =============================================================================
class DroneModel:
    def __init__(self):
        self.reset()

    def reset(self):
        self.id = None
        self.mode = None
        self.hud = {}
        self.sensors = {}
        self.warn_t = {}
        self.hist = {k: collections.deque(maxlen=60) for k, _, _ in SENSORS}
        self.rows, self.aim = [0] * 8, (3, 3)
        self.fog, self.nests, self.scout = [0xFF] * 8, [0] * 8, (3, 7)
        self.found, self.total, self.scanned = 0, 3, 0
        self.rescue_left = 10
        self.last_rx = 0.0
        self.lost = False

    def feed(self, line, now):
        """Update from one line. Returns a tag for the UI: 'bbox', 'event', 'banner' or None."""
        self.last_rx = now
        if line.startswith("[HUD]"):
            t = line.split()[1:]
            self.hud = {t[i]: t[i + 1] for i in range(0, len(t) - 1, 2)}
            if self.hud.get("MODE") in MODES:
                self.mode = self.hud["MODE"]
            if self.hud.get("ID", "").isdigit():
                self.id = int(self.hud["ID"])
            if self.mode == "ATTACKED":
                self.rescue_left = int(self.hud.get("RESC", self.rescue_left))
            return None
        if line.startswith("[SCOUT]"):
            t = line.split()
            try:
                k = {t[i]: i for i in range(1, len(t))}
                fog, nest = t[k["FOG"] + 1], t[k["NEST"] + 1]
                self.fog = [int(fog[i:i + 2], 16) for i in range(0, 16, 2)]
                self.nests = [int(nest[i:i + 2], 16) for i in range(0, 16, 2)]
                self.scout = (int(t[k["POS"] + 1]), int(t[k["POS"] + 2]))
                self.found, self.total = int(t[k["FOUND"] + 1]), int(t[k["FOUND"] + 2])
                self.scanned = int(t[k["SCAN"] + 1])
            except (KeyError, IndexError, ValueError):
                pass
            return None
        if line.startswith("[MAP]"):
            t = line.split()
            if len(t) >= 5 and len(t[1]) == 16:
                self.rows = [int(t[1][i:i + 2], 16) for i in range(0, 16, 2)]
                self.aim = (int(t[3]), int(t[4]))
            return None
        if line.startswith("[TLM]"):
            for tag, val in re.findall(r"\b([TPHAGM]): (-?[\d.]+)", line):
                self._sensor(tag, float(val))
            self.warn_t.clear()
            return None
        if line.startswith("[WARN]"):
            m = re.match(r"\[WARN\] ([TPHAGM]): (-?[\d.]+)", line)
            if m:
                self._sensor(m.group(1), float(m.group(2)))
                self.warn_t[m.group(1)] = now
            return None
        m = re.search(r"=== Drone #(\d) online", line)
        if m:
            self.reset()
            self.last_rx, self.id, self.mode = now, int(m.group(1)), "STANDBY"
            return "banner"
        if "Entering STANDBY MODE" in line:
            self.mode = "STANDBY"
        elif "Entering BATTLE MODE" in line or "rescued!" in line:
            self.mode = "BATTLE"
        elif "flipped!" in line:
            self.mode, self.rescue_left = "ATTACKED", 10
        elif "Cutting all communication" in line:
            self.mode = "TERMINATED"
        m = re.search(r"Rescue in (\d+) s", line)
        if m:
            self.rescue_left = int(m.group(1))
        if line.startswith("[BBOX]"):
            return "bbox"
        if line.startswith(("[EVT]", "[ATTACK]", "[REPORT]", "[CMD]", "[STAT]", "Entering")):
            return "event"
        return None

    def _sensor(self, tag, val):
        self.sensors[tag] = val
        self.hist[tag].append(val)

    def warned(self, tag, now):
        if now - self.warn_t.get(tag, -99) < 1.6:
            return True
        lo, hi = THRESH[tag]
        v = self.sensors.get(tag)
        return v is not None and ((lo is not None and v < lo) or (hi is not None and v > hi))

    def h(self, key, default=0):
        try:
            return float(self.hud.get(key, default))
        except ValueError:
            return default


# =============================================================================
# User interface
# =============================================================================
class Dashboard:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.q = queue.Queue()
        self.model = DroneModel()
        self.drone = None
        self.ctl = None
        self.ctl_last = 0.0
        self.led_on, self.led_t = False, 0.0
        self.blink = False

        root.title("EE2028 Command Centre")
        root.configure(bg=BG)
        root.geometry("1280x820")
        root.minsize(1100, 720)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=FG, padding=(14, 6))
        style.map("TNotebook.Tab", background=[("selected", "#2b3440")])
        style.configure("Treeview", background=PANEL, fieldbackground=PANEL, foreground=FG, rowheight=22)
        style.configure("Treeview.Heading", background="#2b3440", foreground=FG)

        self._build_top()
        nb = ttk.Notebook(root)
        nb.pack(fill="both", expand=True, padx=8, pady=(0, 4))
        self.live = tk.Frame(nb, bg=BG)
        self.bbox_tab = tk.Frame(nb, bg=BG)
        self.console_tab = tk.Frame(nb, bg=BG)
        nb.add(self.live, text="  Live  ")
        nb.add(self.bbox_tab, text="  Black box  ")
        nb.add(self.console_tab, text="  Console  ")
        self._build_live()
        self._build_bbox()
        self._build_console()
        self._build_cmdbar()

        for key, cmd in (("<Left>", "AIM -1 0"), ("<Right>", "AIM 1 0"), ("<Up>", "AIM 0 -1"),
                         ("<Down>", "AIM 0 1"), ("l", "LOCK"), ("p", "POWER NEXT"),
                         ("<space>", "CHARGE"), ("r", "RESCUE")):
            root.bind(key, lambda e, c=cmd: self._hotkey(c))
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.refresh_ports()
        if args.demo:
            self.connect_drone(demo=True)
        elif args.drone:
            self.drone_port.set(args.drone)
            self.connect_drone()
        if args.ctl:
            self.ctl_port.set(args.ctl)
            self.connect_ctl()
        self.root.after(30, self.poll)
        self.root.after(250, self.tick_link)
        self.root.after(100, self.tick_ui)

    # ---------------------------------------------------------------- layout
    def _label(self, parent, text="", size=11, bold=False, fg=FG, bg=PANEL, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=bg,
                        font=("Segoe UI", size, "bold" if bold else "normal"), **kw)

    def _build_top(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=8, pady=8)
        self.id_lbl = self._label(top, "DRONE #-", 22, True, bg=BG)
        self.id_lbl.pack(side="left", padx=(4, 16))
        self.mode_lbl = tk.Label(top, text="NO SIGNAL", fg="white", bg=MODE_COLOR["NO SIGNAL"],
                                 font=("Segoe UI", 22, "bold"), width=12)
        self.mode_lbl.pack(side="left")
        self.led_cv = tk.Canvas(top, width=44, height=44, bg=BG, highlightthickness=0)
        self.led_cv.pack(side="left", padx=12)
        self.led_dot = self.led_cv.create_oval(8, 8, 36, 36, fill="#1b3a1b", outline="#444")
        self._label(top, "LED2", 9, fg=DIM, bg=BG).pack(side="left")

        right = tk.Frame(top, bg=BG)
        right.pack(side="right")
        self.drone_port = tk.StringVar()
        self.ctl_port = tk.StringVar()
        for row, (name, var, cmd) in enumerate((("Drone", self.drone_port, self.connect_drone),
                                                ("Controller", self.ctl_port, self.connect_ctl))):
            self._label(right, name, 10, fg=DIM, bg=BG).grid(row=row, column=0, sticky="e", padx=4)
            cb = ttk.Combobox(right, textvariable=var, width=12)
            cb.grid(row=row, column=1, padx=4, pady=1)
            setattr(self, f"{name.lower()}_cb", cb)
            btn = tk.Button(right, text="Connect", command=cmd, width=10, bg="#2b3440", fg=FG,
                            activebackground="#3a4654", relief="flat")
            btn.grid(row=row, column=2, padx=4, pady=1)
            setattr(self, f"{name.lower()}_btn", btn)
            st = self._label(right, "offline", 10, fg=DIM, bg=BG, width=14, anchor="w")
            st.grid(row=row, column=3, padx=4)
            setattr(self, f"{name.lower()}_st", st)
        tk.Button(right, text="Rescan", command=self.refresh_ports, bg="#2b3440", fg=FG,
                  relief="flat").grid(row=0, column=4, rowspan=2, padx=6, sticky="ns")

    def _build_live(self):
        body = tk.Frame(self.live, bg=BG)
        body.pack(fill="both", expand=True, pady=6)
        self.stage = tk.Frame(body, bg=BG)
        self.stage.pack(side="left", fill="both", expand=True)
        side = tk.Frame(body, bg=PANEL, width=380)
        side.pack(side="right", fill="y", padx=(8, 0))
        side.pack_propagate(False)
        self._label(side, "EVENTS", 10, True, fg=DIM).pack(anchor="w", padx=10, pady=(8, 2))
        self.events = tk.Text(side, bg=PANEL, fg=FG, relief="flat", wrap="word", font=("Consolas", 9),
                              state="disabled")
        self.events.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        for tag, col in (("ATTACK", "#ff6b6b"), ("EVT", FG), ("CMD", "#8ab4f8"), ("REPORT", "#81c995"),
                         ("STAT", DIM), ("MODE", "#fdd663")):
            self.events.tag_configure(tag, foreground=col)

        self.frames = {}
        for name in ("NO SIGNAL",) + MODES:
            f = tk.Frame(self.stage, bg=BG)
            f.place(relx=0, rely=0, relwidth=1, relheight=1)
            self.frames[name] = f
        self._build_nosignal(self.frames["NO SIGNAL"])
        self._build_standby(self.frames["STANDBY"])
        self._build_battle(self.frames["BATTLE"])
        self._build_attacked(self.frames["ATTACKED"])
        self._build_terminated(self.frames["TERMINATED"])
        self.shown = None

    def _build_nosignal(self, f):
        self._label(f, "NO SIGNAL", 40, True, fg="#5f6b76", bg=BG).pack(expand=True, pady=(120, 0))
        self._label(f, "Choose the drone's COM port and press Connect (or run with --demo).",
                    12, fg=DIM, bg=BG).pack(expand=True, pady=(0, 160))

    def _build_standby(self, f):
        nav = tk.Frame(f, bg=PANEL, width=290)
        nav.pack(side="right", fill="y", padx=(5, 0), pady=5)
        nav.pack_propagate(False)
        grid = tk.Frame(f, bg=BG)
        grid.pack(side="left", fill="both", expand=True)
        self.tiles = {}
        for i, (tag, name, unit) in enumerate(SENSORS):
            t = tk.Frame(grid, bg=PANEL, highlightthickness=2, highlightbackground=PANEL)
            t.grid(row=i // 2, column=i % 2, sticky="nsew", padx=5, pady=5)
            lo, hi = THRESH[tag]
            rng = f"{'-' if lo is None else lo} .. {hi} {unit}"
            self._label(t, f"{tag}  {name}", 10, fg=DIM).pack(anchor="w", padx=10, pady=(8, 0))
            val = self._label(t, "--", 26, True)
            val.pack(anchor="w", padx=10)
            self._label(t, f"{unit}    safe {rng}", 9, fg=DIM).pack(anchor="w", padx=10)
            cv = tk.Canvas(t, width=120, height=46, bg=PANEL, highlightthickness=0)
            cv.pack(fill="x", padx=10, pady=(4, 8))
            self.tiles[tag] = (t, val, cv)
        for c in range(2):
            grid.columnconfigure(c, weight=1)
        for r in range(3):
            grid.rowconfigure(r, weight=1)

        self._label(nav, "SCOUTING MAP", 10, True, fg=DIM).pack(anchor="w", padx=12, pady=(10, 0))
        self.SC = 30
        self.scout_cv = tk.Canvas(nav, width=8 * self.SC + 2, height=8 * self.SC + 2, bg=PANEL, highlightthickness=0)
        self.scout_cv.pack(pady=6)
        self.scout_rects = [[self.scout_cv.create_rectangle(1 + x * self.SC + 2, 1 + y * self.SC + 2,
                                                            1 + (x + 1) * self.SC - 2, 1 + (y + 1) * self.SC - 2,
                                                            fill="#3a424c", outline="")
                             for x in range(8)] for y in range(8)]
        self._label(nav, "grey fog   dark scanned   red nest   blue drone", 8, fg=DIM).pack()
        self.nav_txt = self._label(nav, "", 11, justify="left")
        self.nav_txt.pack(anchor="w", padx=14, pady=(8, 0))
        self.scout_txt = self._label(nav, "", 11, justify="left")
        self.scout_txt.pack(anchor="w", padx=14, pady=(6, 0))
        self._label(nav, "5-way / tilt / arrow keys: fly the scout\nSingle press: scout report   Double: BATTLE\n"
                         "5-way centre 1 s: altitude zero",
                    9, fg=DIM, justify="left").pack(side="bottom", anchor="w", padx=14, pady=10)

    def _build_battle(self, f):
        self.CELL = 56
        mp = tk.Frame(f, bg=PANEL)
        mp.pack(side="left", padx=5, pady=5, fill="y")
        self._label(mp, "INFECTION MAP  (drone at the bottom edge)", 10, True, fg=DIM).pack(anchor="w", padx=10, pady=(8, 0))
        self.map_cv = tk.Canvas(mp, width=8 * self.CELL + 2, height=8 * self.CELL + 26, bg=PANEL, highlightthickness=0)
        self.map_cv.pack(padx=12, pady=8)
        self.map_rects = [[self.map_cv.create_rectangle(1 + x * self.CELL + 3, 1 + y * self.CELL + 3,
                                                        1 + (x + 1) * self.CELL - 3, 1 + (y + 1) * self.CELL - 3,
                                                        fill="#262d36", outline="")
                           for x in range(8)] for y in range(8)]
        self.aim_rect = self.map_cv.create_rectangle(0, 0, 0, 0, outline="#fdd663", width=4)
        c = 8 * self.CELL
        self.map_cv.create_polygon(c / 2 - 14, c + 24, c / 2 + 14, c + 24, c / 2, c + 6, fill="#8ab4f8")

        info = tk.Frame(f, bg=PANEL)
        info.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        self._label(info, "LASER", 10, True, fg=DIM).pack(anchor="w", padx=14, pady=(10, 0))
        self.energy_cv = tk.Canvas(info, width=200, height=46, bg=PANEL, highlightthickness=0)
        self.energy_cv.pack(fill="x", padx=14, pady=4)
        self.energy_lbl = self._label(info, "0/10", 26, True)
        self.energy_lbl.pack(anchor="w", padx=14)
        self.weapon_lbl = self._label(info, "", 13, justify="left", anchor="w", wraplength=330)
        self.weapon_lbl.pack(anchor="w", padx=14, pady=(4, 0))
        self.cool_lbl = self._label(info, "", 14, True, fg="#8ab4f8")
        self.cool_lbl.pack(anchor="w", padx=14)
        self._label(info, "MISSION", 10, True, fg=DIM).pack(anchor="w", padx=14, pady=(18, 0))
        self.wave_lbl = self._label(info, "", 16, justify="left")
        self.wave_lbl.pack(anchor="w", padx=14)
        self.radar_lbl = tk.Label(info, text="RADAR CLEAR", fg="white", bg="#2e3b2e",
                                  font=("Segoe UI", 16, "bold"), width=22, pady=6)
        self.radar_lbl.pack(anchor="w", padx=14, pady=18)
        self._label(info, "Single press: charge +3/10   Double press: STANDBY\n"
                          "5-way / controller tilt: aim    centre or controller 1x: lock\n"
                          "5-way centre 1 s / controller 2x: weapon power\n"
                          "Keys: arrows aim, L lock, P power, space charge",
                    9, fg=DIM, justify="left").pack(side="bottom", anchor="w", padx=14, pady=10)

    def _build_attacked(self, f):
        f.configure(bg="#7f0000")
        self.att_title = tk.Label(f, text="DRONE UNDER ATTACK", fg="white", bg="#7f0000",
                                  font=("Segoe UI", 34, "bold"))
        self.att_title.pack(pady=(50, 0))
        tk.Label(f, text="The Last of EE2028", fg="#ffcdd2", bg="#7f0000", font=("Segoe UI", 16, "italic")).pack()
        self.att_count = tk.Label(f, text="10", fg="white", bg="#7f0000", font=("Segoe UI", 160, "bold"))
        self.att_count.pack()
        tk.Label(f, text="DOUBLE-PRESS TO RESCUE", fg="white", bg="#7f0000", font=("Segoe UI", 26, "bold")).pack()
        tk.Label(f, text="drone button, controller double press, or RESCUE    |    SOS beacon on",
                 fg="#ffcdd2", bg="#7f0000", font=("Segoe UI", 12)).pack(pady=8)

    def _build_terminated(self, f):
        f.configure(bg="#1a1a1a")
        self.term_title = tk.Label(f, text="DRONE LOST", fg="#e0e0e0", bg="#1a1a1a", font=("Segoe UI", 46, "bold"))
        self.term_title.pack(pady=(140, 10))
        tk.Label(f, text="COMMS OFFLINE", fg="#ff6b6b", bg="#1a1a1a", font=("Segoe UI", 26, "bold")).pack()
        tk.Label(f, text="Black box saved to flash. Press RESET on the drone, then read it in the Black box tab.",
                 fg="#9e9e9e", bg="#1a1a1a", font=("Segoe UI", 12)).pack(pady=20)

    def _build_bbox(self):
        bar = tk.Frame(self.bbox_tab, bg=BG)
        bar.pack(fill="x", pady=8)
        tk.Button(bar, text="Read black box (STANDBY)", command=lambda: self.send("BLACKBOX"),
                  bg="#2b3440", fg=FG, relief="flat", padx=12).pack(side="left", padx=4)
        self.bbox_src = self._label(bar, "No black box read yet.", 11, fg=DIM, bg=BG)
        self.bbox_src.pack(side="left", padx=12)
        self.bbox_sum = self._label(self.bbox_tab, "", 13, True, bg=BG, anchor="w", justify="left")
        self.bbox_sum.pack(fill="x", padx=8)
        self.tree = ttk.Treeview(self.bbox_tab, columns=("t", "mode", "event"), show="headings")
        for col, w, txt in (("t", 90, "time (s)"), ("mode", 120, "mode"), ("event", 800, "event")):
            self.tree.heading(col, text=txt)
            self.tree.column(col, width=w, anchor="w")
        self.tree.pack(fill="both", expand=True, padx=8, pady=8)

    def _build_console(self):
        self.console = tk.Text(self.console_tab, bg="#0b0e11", fg="#c8d0d8", font=("Consolas", 9),
                               relief="flat", state="disabled")
        self.console.pack(fill="both", expand=True, padx=4, pady=4)
        self.console.tag_configure("ctl", foreground="#8ab4f8")
        self.console.tag_configure("tx", foreground="#fdd663")

    def _build_cmdbar(self):
        bar = tk.Frame(self.root, bg=BG)
        bar.pack(fill="x", padx=8, pady=(0, 8))
        self._label(bar, "Command to drone:", 10, fg=DIM, bg=BG).pack(side="left")
        self.cmd = tk.Entry(bar, bg=PANEL, fg=FG, insertbackground=FG, relief="flat", font=("Consolas", 11))
        self.cmd.pack(side="left", fill="x", expand=True, padx=8, ipady=4)
        self.cmd.bind("<Return>", lambda e: self._send_entry())
        tk.Button(bar, text="Send", command=self._send_entry, bg="#2b3440", fg=FG, relief="flat",
                  padx=12).pack(side="left")
        for txt in ("HELP", "STATUS", "MODE BATTLE", "MODE STANDBY", "CHARGE"):
            tk.Button(bar, text=txt, command=lambda t=txt: self.send(t), bg=PANEL, fg=DIM,
                      relief="flat").pack(side="left", padx=2)
        if self.args.demo:
            tk.Button(bar, text="FLIP (demo)", command=lambda: self.send("FLIP"), bg="#5c1a1a", fg=FG,
                      relief="flat").pack(side="left", padx=2)

    # ------------------------------------------------------------ connections
    def refresh_ports(self):
        ports = []
        if serial is not None:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        self.drone_cb["values"] = ports
        self.controller_cb["values"] = ports

    def connect_drone(self, demo=False):
        if self.drone:
            self.send("DASH OFF")
            self.drone.close()
            self.drone = None
            self.drone_btn.configure(text="Connect")
            self.drone_st.configure(text="offline", fg=DIM)
            self.model.reset()
            return
        link = FakeDrone("drone", self.q) if demo else SerialLink("drone", self.q)
        try:
            link.open(self.drone_port.get())
        except Exception as e:
            self.drone_st.configure(text="error", fg="#ff6b6b")
            self._log_console(f"!! cannot open drone port: {e}", "tx")
            return
        self.drone = link
        self.drone_btn.configure(text="Disconnect")
        self.drone_st.configure(text="demo" if demo else "connecting", fg="#fdd663")
        self.root.after(300, lambda: self.send("DASH ON"))

    def connect_ctl(self):
        if self.ctl:
            self.ctl.close()
            self.ctl = None
            self.controller_btn.configure(text="Connect")
            self.controller_st.configure(text="offline", fg=DIM)
            return
        link = SerialLink("ctl", self.q)
        try:
            link.open(self.ctl_port.get())
        except Exception as e:
            self.controller_st.configure(text="error", fg="#ff6b6b")
            self._log_console(f"!! cannot open controller port: {e}", "tx")
            return
        self.ctl = link
        self.controller_btn.configure(text="Disconnect")
        self.controller_st.configure(text="connecting", fg="#fdd663")

    def send(self, text):
        if self.drone:
            self.drone.send(text)
            self._log_console(f">> {text}", "tx")

    def _send_entry(self):
        t = self.cmd.get().strip()
        if t:
            self.send(t)
            self.cmd.delete(0, "end")

    def _hotkey(self, cmd):
        if self.root.focus_get() is self.cmd:
            return
        if cmd.startswith("AIM") and self.model.mode not in ("BATTLE", "STANDBY"):
            return
        if cmd.startswith(("LOCK", "POWER", "CHARGE")) and self.model.mode != "BATTLE":
            return
        if cmd == "RESCUE" and self.model.mode != "ATTACKED":
            return
        self.send(cmd)

    def on_close(self):
        if self.drone:
            self.send("DASH OFF")
            time.sleep(0.05)
            self.drone.close()
        if self.ctl:
            self.ctl.close()
        self.root.destroy()

    # ------------------------------------------------------------ data flow
    def poll(self):
        now = time.time()
        try:
            while True:
                src, kind, data = self.q.get_nowait()
                if kind == "__error__":
                    self._log_console(f"!! {src} port error: {data}", "tx")
                    if src == "drone" and self.drone:
                        self.connect_drone()
                    elif src == "ctl" and self.ctl:
                        self.connect_ctl()
                    continue
                if not data:
                    continue
                if src == "drone":
                    self._on_drone(data, now)
                else:
                    self._on_ctl(data, now)
        except queue.Empty:
            pass
        self.root.after(30, self.poll)

    def _on_drone(self, line, now):
        if not line.startswith(("[HUD]", "[MAP]")):
            self._log_console(line)
        if line.startswith(">"):
            return                                  # echo of our own command
        kind = self.model.feed(line, now)
        if kind == "banner":
            self.root.after(300, lambda: self.send("DASH ON"))   # drone was reset
            self._log_event("=== drone online ===", "MODE")
        elif kind == "event":
            tag = "ATTACK" if line.startswith("[ATTACK]") else line[1:line.find("]")] if line.startswith("[") else "MODE"
            self._log_event(line, tag if tag in ("EVT", "CMD", "REPORT", "STAT", "ATTACK") else "MODE")
        elif kind == "bbox":
            self._on_bbox(line)

    def _on_ctl(self, line, now):
        self._log_console("CTL " + line, "ctl")
        if line.startswith(">"):
            return
        if line.startswith("[CTL]") or "Pilot controller online" in line:
            self.ctl_last = now
        mode = self.model.mode
        t = line.split()
        if line.startswith("[CTL] TILT") and len(t) == 4 and mode in ("BATTLE", "STANDBY"):
            self.send(f"AIM {t[2]} {t[3]}")
        elif line == "[CTL] BTN SINGLE" and mode == "BATTLE":
            self.send("LOCK")
        elif line == "[CTL] BTN DOUBLE":
            if mode == "BATTLE":
                self.send("POWER NEXT")
            elif mode == "ATTACKED":
                self.send("RESCUE")
                self._log_event("Lab rescue team dispatched (controller)", "CMD")

    def tick_link(self):
        """Tell the controller what the drone is doing (its LED mirrors the mode)."""
        now = time.time()
        if self.ctl and self.model.mode and now - self.model.last_rx < DRONE_TIMEOUT_S:
            self.ctl.send(f"LINK {self.model.mode}")
        self.root.after(250, self.tick_link)

    def _on_bbox(self, line):
        body = line[6:].strip()
        if body.startswith("empty"):
            self.bbox_src.configure(text="Black box is empty (no sortie yet).")
            self.tree.delete(*self.tree.get_children())
            self.bbox_sum.configure(text="")
            return
        if "records" in body and "drone #" in body:
            self.bbox_src.configure(text=body)
            self.tree.delete(*self.tree.get_children())
            return
        if body.startswith("LAST MISSION"):
            self.bbox_sum.configure(text=body.replace("LAST MISSION: ", "Last mission:  "))
            return
        m = re.match(r"t=\s*([\d.]+)s\s+(\S+)\s+(.*)", body)
        if m:
            self.tree.insert("", "end", values=m.groups())

    # ----------------------------------------------------------------- views
    def tick_ui(self):
        now = time.time()
        m = self.model
        drone_alive = self.drone is not None and now - m.last_rx < DRONE_TIMEOUT_S
        mode = m.mode
        if self.drone is None or mode is None:
            mode = "NO SIGNAL"
        elif not drone_alive and mode in ("ATTACKED", "TERMINATED"):
            mode = "TERMINATED"                     # silence after the last message
        elif not drone_alive and not isinstance(self.drone, FakeDrone):
            mode = "NO SIGNAL"

        if self.drone is not None:
            self.drone_st.configure(text="demo" if isinstance(self.drone, FakeDrone) and drone_alive
                                    else ("online" if drone_alive else "no data"),
                                    fg="#81c995" if drone_alive else "#fdd663")
        if self.ctl is not None:
            ok = now - self.ctl_last < CTL_TIMEOUT_S
            self.controller_st.configure(text="online" if ok else "no data", fg="#81c995" if ok else "#fdd663")

        self.id_lbl.configure(text=f"DRONE #{m.id if m.id else '-'}")
        self.mode_lbl.configure(text=mode, bg=MODE_COLOR[mode])
        if mode != self.shown:
            self.frames[mode].tkraise()
            self.shown = mode

        # LED2 mirror: solid / 1 Hz / 2 Hz / off
        half = {"BATTLE": 0.5, "ATTACKED": 0.25}.get(mode)
        if half and now - self.led_t >= half:
            self.led_t, self.led_on = now, not self.led_on
        elif not half:
            self.led_on = mode == "STANDBY"
        self.led_cv.itemconfigure(self.led_dot, fill="#39d353" if self.led_on else "#1b3a1b")
        self.blink = (int(now * 4) % 2) == 0

        if mode == "STANDBY":
            self._draw_standby(now)
        elif mode == "BATTLE":
            self._draw_battle()
        elif mode == "ATTACKED":
            self.att_title.configure(text=f"DRONE #{m.id or '-'} UNDER ATTACK")
            self.att_count.configure(text=str(max(0, m.rescue_left)))
        elif mode == "TERMINATED":
            self.term_title.configure(text=f"DRONE #{m.id or '-'} LOST")
        self.root.after(100, self.tick_ui)

    def _draw_standby(self, now):
        m = self.model
        for tag, _, unit in SENSORS:
            frame, val, cv = self.tiles[tag]
            v = m.sensors.get(tag)
            bad = m.warned(tag, now)
            val.configure(text="--" if v is None else f"{v:.2f}", fg="#ff6b6b" if bad else FG)
            frame.configure(highlightbackground="#c62828" if bad else PANEL)
            self._spark(cv, list(m.hist[tag]), "#ff6b6b" if bad else "#8ab4f8")
        self._draw_scout()
        done = "   ALL FOUND" if m.found >= m.total else ""
        self.nav_txt.configure(text=f"Nests found {m.found}/{m.total}{done}\nScanned     {m.scanned}/64\n"
                                    f"Altitude    {m.h('ALT'):+.1f} m")
        hab = "FOUND" if m.h("HAB") else "searching (20-32 C, 40-70 %RH)"
        ir = "OBSTACLE AHEAD" if m.h("IR") else "clear"
        self.scout_txt.configure(text=f"Habitat   {hab}\nIR        {ir}\n"
                                      f"Zombies down {m.h('DOWN'):.0f}   shots {m.h('SHOTS'):.0f}")

    def _spark(self, cv, data, color):
        cv.delete("all")
        w, h = max(cv.winfo_width(), 50), 46
        if len(data) < 2:
            return
        lo, hi = min(data), max(data)
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        pts = []
        for i, v in enumerate(data):
            pts += [i * (w - 2) / 59 + 1, h - 3 - (v - lo) / (hi - lo) * (h - 6)]
        cv.create_line(*pts, fill=color, width=2)

    def _draw_scout(self):
        m = self.model
        sx, sy = m.scout
        for y in range(8):
            for x in range(8):
                bit = 0x80 >> x
                if (x, y) == (sx, sy):
                    col = "#8ab4f8"
                elif m.nests[y] & bit:
                    col = "#e53935" if self.blink else "#8e2420"
                elif m.fog[y] & bit:
                    col = "#3a424c"
                else:
                    col = "#151a20"
                self.scout_cv.itemconfigure(self.scout_rects[y][x], fill=col)

    def _draw_battle(self):
        m = self.model
        for y in range(8):
            for x in range(8):
                on = (m.rows[y] >> (7 - x)) & 1
                self.map_cv.itemconfigure(self.map_rects[y][x], fill="#e53935" if on else "#262d36")
        ax, ay = m.aim
        C = self.CELL
        self.map_cv.coords(self.aim_rect, 1 + ax * C + 1, 1 + ay * C + 1, 1 + (ax + 1) * C - 1, 1 + (ay + 1) * C - 1)
        self.map_cv.itemconfigure(self.aim_rect, outline="#fdd663" if self.blink else "#a88a20")

        e = int(m.h("E"))
        cv = self.energy_cv
        cv.delete("all")
        w = max(cv.winfo_width(), 200)
        seg = (w - 4) / 10
        p = int(m.h("P", 5))
        for i in range(10):
            col = ("#fdd663" if i < p else "#8ab4f8") if i < e else "#262d36"
            cv.create_rectangle(2 + i * seg + 2, 4, 2 + (i + 1) * seg - 2, 42, fill=col, outline="")
        cv.create_line(2 + p * seg, 0, 2 + p * seg, 46, fill="#ff6b6b", width=2)
        self.energy_lbl.configure(text=f"{e}/10")
        name = "PIU" if p <= 1 else ("PEW" if p <= 3 else "BOOM")
        if m.h("SEL"):
            self.weapon_lbl.configure(text=f"Weapon  {p}/10 {name}   cooldown {p if p >= 2 else 0} s")
        else:
            self.weapon_lbl.configure(text="Weapon  RULE 5/10   no cooldown")
        cd = m.h("CD")
        self.cool_lbl.configure(text=f"COOLDOWN {cd:.1f} s" if cd > 0 else ("READY" if e >= p else "charging"))
        tgt = int(m.h("TGT"))
        over = "   OVERRUN!" if tgt >= 48 else ""
        self.wave_lbl.configure(text=f"Wave {m.h('WAVE'):.0f}     targets {tgt}/64{over}\n"
                                     f"Zombies down {m.h('DOWN'):.0f}     shots {m.h('SHOTS'):.0f}")
        if m.h("IR"):
            self.radar_lbl.configure(text="ZOMBIE AT CLOSE RANGE!", bg="#c62828" if self.blink else "#7f0000")
        else:
            self.radar_lbl.configure(text="RADAR CLEAR", bg="#2e3b2e")

    # ----------------------------------------------------------------- logs
    def _append(self, widget, text, tag=None, limit=2000):
        widget.configure(state="normal")
        widget.insert("end", text + "\n", tag)
        if int(widget.index("end-1c").split(".")[0]) > limit:
            widget.delete("1.0", "200.0")
        widget.see("end")
        widget.configure(state="disabled")

    def _log_event(self, line, tag):
        self._append(self.events, time.strftime("%H:%M:%S ") + line, tag, 600)

    def _log_console(self, line, tag=None):
        self._append(self.console, line, tag)


def main():
    ap = argparse.ArgumentParser(description="EE2028 Command Centre dashboard")
    ap.add_argument("--drone", help="COM port of the drone, e.g. COM5")
    ap.add_argument("--ctl", help="COM port of the pilot controller (optional)")
    ap.add_argument("--demo", action="store_true", help="use a built-in fake drone")
    args = ap.parse_args()
    root = tk.Tk()
    Dashboard(root, args)
    root.mainloop()


if __name__ == "__main__":
    main()
