#!/usr/bin/env python3
"""
6-Strip RGBW DMX Controller  (uDMX / Anyma USB interface)

Install:
    python -m pip install --upgrade pyusb libusb-package

Windows: the uDMX must use the WinUSB (libusb) driver. If the device is not
found, install it with Zadig (select the uDMX, VID 16C0 / PID 05DC, driver WinUSB).

Layout: 6 strips x 4 channels (R,G,B,W) = 24 channels, starting at the
"Start CH" address (default 1) -- set it to your decoder's DMX address.
"""

import colorsys
import json
import math
import threading
import time
import tkinter as tk
from contextlib import contextmanager
from tkinter import ttk, colorchooser, filedialog, messagebox

import usb.core
import usb.util

try:
    import libusb_package
except ImportError:          # pyusb will fall back to its default backend search
    libusb_package = None


# ============================================================
# CONSTANTS
# ============================================================

VID = 0x16C0
PID = 0x05DC

NUM_STRIPS = 6
CH_PER_STRIP = 4
TOTAL_CHANNELS = NUM_STRIPS * CH_PER_STRIP
MAX_START = 512 - TOTAL_CHANNELS + 1

PRESETS = {
    "Red":        (255, 0, 0, 0),
    "Orange":     (255, 120, 0, 0),
    "Yellow":     (255, 220, 0, 0),
    "Green":      (0, 255, 0, 0),
    "Cyan":       (0, 255, 255, 0),
    "Blue":       (0, 0, 255, 0),
    "Purple":     (160, 0, 255, 0),
    "Pink":       (255, 20, 147, 0),
    "White":      (0, 0, 0, 255),
    "Warm White": (255, 120, 40, 200),
    "Full White": (255, 255, 255, 255),
    "Off":        (0, 0, 0, 0),
}

EFFECTS = ("Rainbow", "Chase", "Breathe")

# ---- theme ----
BG = "#12141a"
PANEL = "#1a1d26"
CARD = "#1f2330"
BORDER = "#2c3142"
TRACK = "#0e1015"
BTN = "#2a2f40"
BTN_HOVER = "#363d54"
FG = "#e8ebf2"
MUTED = "#8b93a7"
ACCENT = "#4f8cff"
GOOD = "#35c76b"
DANGER = "#e5484d"
OFF_BG = "#262a37"

FONT = ("Segoe UI", 10)
FONT_B = ("Segoe UI", 10, "bold")
FONT_S = ("Segoe UI", 8)
FONT_T = ("Segoe UI", 20, "bold")
FONT_MONO = ("Consolas", 9)


# ============================================================
# COLOR HELPERS
# ============================================================

def display_rgb(r, g, b, w):
    """Approximate what the strip looks like (white LED is slightly warm)."""
    return (
        min(255, int(r + w)),
        min(255, int(g + w * 0.96)),
        min(255, int(b + w * 0.88)),
    )


def to_hex(rgb):
    return "#%02x%02x%02x" % rgb


def text_on(rgb):
    lum = 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2]
    return "#000000" if lum > 150 else "#ffffff"


# ============================================================
# uDMX USB DEVICE  (non-blocking: USB writes happen in a worker thread)
# ============================================================

class UDMX:

    def __init__(self):
        self.device = None
        self.last_error = ""

        self._usb_lock = threading.Lock()
        self._pending = {}                      # start_channel -> bytes
        self._pending_lock = threading.Lock()
        self._wake = threading.Event()
        self._running = True
        self._last_reconnect = 0.0

        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ---------------- connection ----------------

    @property
    def connected(self):
        return self.device is not None

    def connect(self):
        """Find and open the uDMX. Raises RuntimeError with a helpful message."""
        with self._usb_lock:
            self._release()

            try:
                backend = (
                    libusb_package.get_libusb1_backend()
                    if libusb_package else None
                )
                device = usb.core.find(
                    backend=backend, idVendor=VID, idProduct=PID
                )
            except usb.core.NoBackendError:
                raise RuntimeError(
                    "No libusb backend found.\n"
                    "Run: python -m pip install --upgrade pyusb libusb-package"
                )
            except usb.core.USBError as error:
                raise RuntimeError(f"USB error while searching: {error}")

            if device is None:
                raise RuntimeError(
                    "uDMX not found (VID 16C0, PID 05DC). Check the USB cable "
                    "and that the WinUSB/libusb driver is installed (Zadig)."
                )

            try:
                device.set_configuration()
            except usb.core.USBError:
                pass            # already configured

            self.device = device
            self.last_error = ""

    def _release(self):
        """Drop the device handle. Caller must hold _usb_lock (or be __init__)."""
        if self.device is not None:
            try:
                usb.util.dispose_resources(self.device)
            except Exception:
                pass
        self.device = None

    def close(self):
        self._running = False
        self._wake.set()
        self._thread.join(timeout=1.0)
        with self._usb_lock:
            self._release()

    # ---------------- sending ----------------

    def queue(self, start_channel, values):
        """Queue a frame. Only the newest frame per start address is kept."""
        data = bytes(int(v) for v in values)

        if start_channel < 1 or start_channel + len(data) - 1 > 512:
            raise ValueError("DMX data must fit inside channels 1-512.")
        if any(v < 0 or v > 255 for v in values):
            raise ValueError("DMX values must be between 0 and 255.")

        with self._pending_lock:
            self._pending[start_channel] = data
        self._wake.set()

    def send_now(self, start_channel, values):
        """Synchronous send (used on shutdown). Never raises."""
        try:
            self._write(start_channel, bytes(int(v) for v in values))
        except Exception:
            pass

    def _write(self, start_channel, data):
        if self.device is None:
            if time.time() - self._last_reconnect < 2.0:
                raise RuntimeError("uDMX not connected")
            self._last_reconnect = time.time()
            self.connect()

        with self._usb_lock:
            if self.device is None:
                raise RuntimeError("uDMX not connected")
            try:
                # Anyma uDMX: request 0x02 = set channel range,
                # wValue = channel count, wIndex = zero-based first channel.
                self.device.ctrl_transfer(
                    0x40, 0x02, len(data), start_channel - 1, data,
                    timeout=1000,
                )
            except usb.core.USBError as error:
                self._release()
                raise RuntimeError(f"USB write failed: {error}")

    def _worker(self):
        while self._running:
            self._wake.wait(timeout=1.0)
            self._wake.clear()

            with self._pending_lock:
                items = list(self._pending.items())
                self._pending.clear()

            for start, data in items:
                try:
                    self._write(start, data)
                    self.last_error = ""
                except Exception as error:
                    self.last_error = str(error)
                    with self._pending_lock:      # retry later, keep newest
                        self._pending.setdefault(start, data)

            time.sleep(0.01)                      # cap at ~100 frames/s


# ============================================================
# CUSTOM WIDGETS
# ============================================================

class FlatButton(tk.Button):

    KINDS = {
        "normal": (BTN, FG, BTN_HOVER),
        "accent": (ACCENT, "#ffffff", "#6aa0ff"),
        "good":   (GOOD, "#08140c", "#5be08a"),
        "danger": (DANGER, "#ffffff", "#ff6b70"),
    }

    def __init__(self, parent, text, command=None, kind="normal", **kw):
        bg, fg, hover = self.KINDS[kind]
        self._k_bg, self._k_hover = bg, hover
        super().__init__(
            parent, text=text, command=command, bg=bg, fg=fg,
            activebackground=hover, activeforeground=fg,
            relief="flat", bd=0, highlightthickness=0,
            padx=12, pady=6, cursor="hand2", font=FONT_B, **kw
        )
        self.bind("<Enter>", lambda e: self.config(bg=self._k_hover))
        self.bind("<Leave>", lambda e: self.config(bg=self._k_bg))

    def set_kind(self, kind, text=None):
        bg, fg, hover = self.KINDS[kind]
        self._k_bg, self._k_hover = bg, hover
        self.config(
            bg=bg, fg=fg, activebackground=hover, activeforeground=fg
        )
        if text is not None:
            self.config(text=text)


class VSlider(tk.Canvas):
    """Vertical 0-255 slider bound to an IntVar."""

    PAD = 12

    def __init__(self, parent, variable, color, width=34, height=170, bg=CARD):
        super().__init__(
            parent, width=width, height=height, bg=bg,
            highlightthickness=0, cursor="hand2"
        )
        self.var = variable
        self.color = color
        self.sw = width
        self.sh = height

        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        self.bind("<MouseWheel>", self._wheel)
        self.bind("<Button-4>", lambda e: self._step(5))     # Linux
        self.bind("<Button-5>", lambda e: self._step(-5))

        self.var.trace_add("write", lambda *a: self.redraw())
        self.redraw()

    def _drag(self, event):
        top, bottom = self.PAD, self.sh - self.PAD
        frac = 1.0 - (event.y - top) / (bottom - top)
        self.var.set(int(round(max(0.0, min(1.0, frac)) * 255)))

    def _wheel(self, event):
        self._step(5 if event.delta > 0 else -5)

    def _step(self, delta):
        self.var.set(max(0, min(255, self.var.get() + delta)))

    def redraw(self):
        self.delete("all")
        cx = self.sw // 2
        top, bottom = self.PAD, self.sh - self.PAD
        value = self.var.get()
        y = bottom - (bottom - top) * value / 255.0

        self.create_line(cx, top, cx, bottom, width=8, fill=TRACK,
                         capstyle="round")
        if value > 0:
            self.create_line(cx, y, cx, bottom, width=8, fill=self.color,
                             capstyle="round")
        self.create_oval(cx - 10, y - 10, cx + 10, y + 10,
                         fill="#f2f4f8", outline=self.color, width=3)


# ============================================================
# ONE STRIP CARD
# ============================================================

class StripCard(tk.Frame):

    CHANNELS = (
        ("R", "#ff4d4d"),
        ("G", "#3ddc84"),
        ("B", "#4f8cff"),
        ("W", "#f2f4f8"),
    )

    def __init__(self, parent, app, index):
        super().__init__(
            parent, bg=CARD, highlightthickness=1, highlightbackground=BORDER
        )
        self.app = app
        self.index = index

        self.vars = {name: tk.IntVar(value=0) for name, _ in self.CHANNELS}
        self.on = tk.BooleanVar(value=True)

        inner = tk.Frame(self, bg=CARD)
        inner.pack(fill="both", expand=True, padx=10, pady=10)

        # ---- header ----
        head = tk.Frame(inner, bg=CARD)
        head.pack(fill="x")
        tk.Label(head, text=f"STRIP {index + 1}", bg=CARD, fg=FG,
                 font=FONT_B).pack(side="left")
        self.ch_label = tk.Label(head, text="", bg=CARD, fg=MUTED,
                                 font=FONT_S)
        self.ch_label.pack(side="right")

        # ---- preview swatch ----
        self.swatch = tk.Label(inner, text="", height=2, bd=0,
                               font=FONT_MONO)
        self.swatch.pack(fill="x", pady=(8, 8))

        # ---- sliders ----
        sliders = tk.Frame(inner, bg=CARD)
        sliders.pack()
        for col, (name, color) in enumerate(self.CHANNELS):
            tk.Label(sliders, text=name, bg=CARD, fg=color,
                     font=FONT_B).grid(row=0, column=col)
            VSlider(sliders, self.vars[name], color).grid(row=1, column=col)
            tk.Label(sliders, textvariable=self.vars[name], bg=CARD, fg=MUTED,
                     font=FONT_MONO, width=4).grid(row=2, column=col)

        # ---- controls ----
        row = tk.Frame(inner, bg=CARD)
        row.pack(fill="x", pady=(8, 4))
        self.on_btn = FlatButton(row, "ON", self.toggle_on, "good")
        self.on_btn.pack(side="left", expand=True, fill="x", padx=(0, 3))
        FlatButton(row, "Color…", self.pick_color).pack(
            side="left", expand=True, fill="x", padx=(3, 0))

        self.preset = ttk.Combobox(
            inner, values=list(PRESETS.keys()), state="readonly",
            style="Dark.TCombobox", width=12
        )
        self.preset.set("Preset…")
        self.preset.pack(fill="x", pady=4)
        self.preset.bind("<<ComboboxSelected>>", self._preset_selected)

        FlatButton(inner, "Copy to all", self.copy_to_all).pack(fill="x")

        # ---- change tracking ----
        for var in self.vars.values():
            var.trace_add("write", self._changed)
        self.on.trace_add("write", self._changed)

    # ---------------- helpers ----------------

    def values(self):
        return tuple(self.vars[n].get() for n, _ in self.CHANNELS)

    def set_values(self, r, g, b, w):
        for (name, _), value in zip(self.CHANNELS, (r, g, b, w)):
            self.vars[name].set(int(max(0, min(255, value))))

    def set_channel_label(self, start_channel):
        first = start_channel + self.index * CH_PER_STRIP
        self.ch_label.config(text=f"CH {first}–{first + CH_PER_STRIP - 1}")

    def update_view(self, output):
        """output = final (r,g,b,w) actually being sent."""
        if not self.on.get():
            self.swatch.config(bg=OFF_BG, fg=MUTED, text="OFF")
            self.on_btn.set_kind("normal", "OFF")
            return

        rgb = display_rgb(*output)
        self.swatch.config(
            bg=to_hex(rgb), fg=text_on(rgb),
            text="  ".join(f"{v:3d}" for v in output)
        )
        self.on_btn.set_kind("good", "ON")

    # ---------------- callbacks ----------------

    def _changed(self, *args):
        self.app.on_card_changed(self.index)

    def toggle_on(self):
        self.on.set(not self.on.get())

    def _preset_selected(self, event):
        name = self.preset.get()
        self.preset.set("Preset…")
        self.app.apply_preset(self.index, name)

    def pick_color(self):
        r, g, b, w = self.values()
        rgb, _ = colorchooser.askcolor(
            color=to_hex((r, g, b)), parent=self.app.root,
            title=f"Strip {self.index + 1} color"
        )
        if rgb:
            with self.app.batch():
                self.set_values(rgb[0], rgb[1], rgb[2], 0)
                self.on.set(True)

    def copy_to_all(self):
        self.app.copy_to_all(self.index)


# ============================================================
# MAIN APPLICATION
# ============================================================

class DMXApp:

    def __init__(self, root, dmx):
        self.root = root
        self.dmx = dmx

        root.title("6-Strip RGBW DMX Controller")
        root.geometry("1240x820")
        root.minsize(1100, 740)
        root.configure(bg=BG)

        self._bulk = 0
        self.blackout_on = False
        self.start_channel = 1

        self.fx_running = False
        self.fx_job = None
        self.fx_t = 0.0
        self.fx_last = 0.0
        self.fx_snapshot = None
        self.fx_mult = [1.0] * NUM_STRIPS

        self.master = tk.DoubleVar(value=255)
        self.live = tk.BooleanVar(value=True)
        self.start_var = tk.StringVar(value="1")
        self.fx_var = tk.StringVar(value=EFFECTS[0])
        self.fx_speed = tk.DoubleVar(value=1.0)
        self.status = tk.StringVar(value="Ready")
        self._shown_error = None

        self.setup_style()
        self.build_header()
        self.build_master()
        self.build_cards()
        self.build_statusbar()

        self.master.trace_add("write", self.on_master_changed)
        self.start_var.trace_add("write", self.on_start_changed)

        root.bind("<Escape>", lambda e: self.toggle_blackout())
        root.protocol("WM_DELETE_WINDOW", self.close)

        for card in self.cards:
            card.set_channel_label(self.start_channel)

        self.refresh_all()
        self.poll_connection()

    # ========================================================
    # STYLE
    # ========================================================

    def setup_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        style.configure(
            "Dark.TCombobox", fieldbackground=BTN, background=BTN,
            foreground=FG, arrowcolor=FG, bordercolor=BORDER,
            lightcolor=BTN, darkcolor=BTN,
        )
        style.map(
            "Dark.TCombobox",
            fieldbackground=[("readonly", BTN)],
            foreground=[("readonly", FG)],
            selectbackground=[("readonly", BTN)],
            selectforeground=[("readonly", FG)],
        )
        style.configure(
            "Dark.Horizontal.TScale", background=PANEL, troughcolor=TRACK,
            bordercolor=PANEL, lightcolor=ACCENT, darkcolor=ACCENT,
        )

        self.root.option_add("*TCombobox*Listbox.background", BTN)
        self.root.option_add("*TCombobox*Listbox.foreground", FG)
        self.root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        self.root.option_add("*TCombobox*Listbox.selectForeground", "#ffffff")

    # ========================================================
    # HEADER
    # ========================================================

    def build_header(self):
        header = tk.Frame(self.root, bg=BG)
        header.pack(fill="x", padx=20, pady=(14, 6))

        left = tk.Frame(header, bg=BG)
        left.pack(side="left")
        tk.Label(left, text="6-STRIP RGBW DMX CONTROLLER", bg=BG, fg=FG,
                 font=FONT_T).pack(anchor="w")
        tk.Label(left, text="uDMX  •  24 DMX channels  •  RGBW  •  "
                            "Esc = blackout",
                 bg=BG, fg=MUTED, font=FONT).pack(anchor="w")

        right = tk.Frame(header, bg=BG)
        right.pack(side="right")
        self.conn_label = tk.Label(right, text="● connecting…", bg=BG,
                                   fg=MUTED, font=FONT_B)
        self.conn_label.pack(side="left", padx=10)
        FlatButton(right, "Reconnect", self.reconnect).pack(side="left")

    # ========================================================
    # MASTER PANEL
    # ========================================================

    def section(self, parent, title, column):
        frame = tk.Frame(parent, bg=PANEL)
        frame.grid(row=0, column=column, sticky="nsew", padx=(0, 24))
        tk.Label(frame, text=title, bg=PANEL, fg=MUTED,
                 font=FONT_S).pack(anchor="w", pady=(0, 6))
        body = tk.Frame(frame, bg=PANEL)
        body.pack(anchor="w")
        return body

    def build_master(self):
        panel = tk.Frame(self.root, bg=PANEL, highlightthickness=1,
                         highlightbackground=BORDER)
        panel.pack(fill="x", padx=20, pady=8)

        wrap = tk.Frame(panel, bg=PANEL)
        wrap.pack(fill="x", padx=16, pady=12)

        # ---------- OUTPUT ----------
        out = self.section(wrap, "OUTPUT", 0)

        tk.Label(out, text="Master", bg=PANEL, fg=FG, font=FONT).grid(
            row=0, column=0, sticky="w")
        ttk.Scale(out, from_=0, to=255, orient="horizontal",
                  variable=self.master, length=200,
                  style="Dark.Horizontal.TScale").grid(
            row=0, column=1, padx=8)
        self.master_label = tk.Label(out, text="255", bg=PANEL, fg=FG,
                                     font=FONT_MONO, width=4)
        self.master_label.grid(row=0, column=2)

        self.blackout_btn = FlatButton(out, "BLACKOUT", self.toggle_blackout,
                                       "danger")
        self.blackout_btn.grid(row=1, column=0, columnspan=2, sticky="ew",
                               pady=(8, 0))

        opts = tk.Frame(out, bg=PANEL)
        opts.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))
        tk.Checkbutton(
            opts, text="Live", variable=self.live, bg=PANEL, fg=FG,
            selectcolor=BTN, activebackground=PANEL, activeforeground=FG,
            font=FONT, command=self.on_live_toggled, bd=0,
            highlightthickness=0,
        ).pack(side="left")
        FlatButton(opts, "SEND", self.send_frame, "accent").pack(
            side="left", padx=8)
        tk.Label(opts, text="Start CH", bg=PANEL, fg=MUTED,
                 font=FONT).pack(side="left", padx=(8, 4))
        tk.Spinbox(
            opts, from_=1, to=MAX_START, textvariable=self.start_var,
            width=5, bg=BTN, fg=FG, buttonbackground=BTN,
            insertbackground=FG, relief="flat", font=FONT_MONO,
            highlightthickness=0,
        ).pack(side="left")

        # ---------- QUICK COLORS ----------
        quick = self.section(wrap, "SET ALL STRIPS", 1)
        chips = tk.Frame(quick, bg=PANEL)
        chips.pack(anchor="w")
        for i, (name, color) in enumerate(PRESETS.items()):
            rgb = display_rgb(*color) if name != "Off" else (40, 44, 56)
            chip = tk.Button(
                chips, text="", width=3, height=1, bg=to_hex(rgb),
                activebackground=to_hex(rgb), relief="flat", bd=0,
                highlightthickness=0, cursor="hand2",
                command=lambda n=name: self.set_all_preset(n),
            )
            chip.grid(row=i // 6, column=i % 6, padx=3, pady=3)
            chip.bind("<Enter>", lambda e, n=name: self.status.set(
                f"Set all strips to {n}"))
        FlatButton(quick, "Custom color…", self.pick_all_color).pack(
            anchor="w", pady=(8, 0))

        # ---------- EFFECTS ----------
        fx = self.section(wrap, "EFFECTS", 2)
        ttk.Combobox(fx, values=EFFECTS, textvariable=self.fx_var,
                     state="readonly", style="Dark.TCombobox",
                     width=12).grid(row=0, column=0, columnspan=2,
                                    sticky="w")
        tk.Label(fx, text="Speed", bg=PANEL, fg=FG, font=FONT).grid(
            row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Scale(fx, from_=0.2, to=3.0, orient="horizontal",
                  variable=self.fx_speed, length=130,
                  style="Dark.Horizontal.TScale").grid(
            row=1, column=1, padx=8, pady=(8, 0))
        self.fx_btn = FlatButton(fx, "▶ START", self.toggle_effect, "accent")
        self.fx_btn.grid(row=2, column=0, columnspan=2, sticky="ew",
                         pady=(8, 0))

        # ---------- SCENE ----------
        scene = self.section(wrap, "SCENE", 3)
        FlatButton(scene, "Save…", self.save_scene).pack(fill="x")
        FlatButton(scene, "Load…", self.load_scene).pack(fill="x", pady=6)
        FlatButton(scene, "Reset", self.reset_all).pack(fill="x")

    # ========================================================
    # STRIP CARDS / STATUS BAR
    # ========================================================

    def build_cards(self):
        container = tk.Frame(self.root, bg=BG)
        container.pack(fill="both", expand=True, padx=16, pady=4)

        self.cards = []
        for i in range(NUM_STRIPS):
            container.columnconfigure(i, weight=1, uniform="cards")
            card = StripCard(container, self, i)
            card.grid(row=0, column=i, sticky="nsew", padx=4)
            self.cards.append(card)

    def build_statusbar(self):
        bar = tk.Frame(self.root, bg=PANEL)
        bar.pack(fill="x", side="bottom")
        tk.Label(bar, textvariable=self.status, bg=PANEL, fg=MUTED,
                 font=FONT, anchor="w").pack(fill="x", padx=16, pady=6)

    # ========================================================
    # OUTPUT COMPUTATION
    # ========================================================

    def master_value(self):
        return int(round(self.master.get()))

    def strip_output(self, index):
        """Final (r,g,b,w) for a strip after on/off, master, effects, blackout."""
        card = self.cards[index]
        if self.blackout_on or not card.on.get():
            return (0, 0, 0, 0)

        scale = (self.master_value() / 255.0) * self.fx_mult[index]
        return tuple(
            int(max(0, min(255, round(v * scale)))) for v in card.values()
        )

    def build_frame(self):
        frame = []
        for i in range(NUM_STRIPS):
            frame.extend(self.strip_output(i))
        return frame

    def send_frame(self):
        try:
            self.dmx.queue(self.start_channel, self.build_frame())
            if not self.live.get():
                self.status.set("Frame sent")
        except Exception as error:
            self.status.set(f"Error: {error}")

    def refresh_all(self):
        for i, card in enumerate(self.cards):
            card.update_view(self.strip_output(i))
        if self.live.get():
            self.send_frame()

    @contextmanager
    def batch(self):
        """Group many changes into a single refresh / USB frame."""
        self._bulk += 1
        try:
            yield
        finally:
            self._bulk -= 1
            if self._bulk == 0:
                self.refresh_all()

    # ========================================================
    # CHANGE HANDLERS
    # ========================================================

    def on_card_changed(self, index):
        if self._bulk:
            return
        self.cards[index].update_view(self.strip_output(index))
        if self.live.get():
            self.send_frame()
        else:
            self.status.set("Changes pending – press SEND")

    def on_master_changed(self, *args):
        self.master_label.config(text=str(self.master_value()))
        if self._bulk:
            return
        self.refresh_all()
        if not self.live.get():
            self.status.set("Changes pending – press SEND")

    def on_live_toggled(self):
        if self.live.get():
            self.refresh_all()
            self.status.set("Live mode on")
        else:
            self.status.set("Live mode off – use SEND to apply changes")

    def on_start_changed(self, *args):
        try:
            new = int(self.start_var.get())
        except ValueError:
            return
        if new < 1 or new > MAX_START or new == self.start_channel:
            return

        old = self.start_channel
        self.start_channel = new
        self.dmx.queue(old, [0] * TOTAL_CHANNELS)       # clear old address
        for card in self.cards:
            card.set_channel_label(new)
        self.send_frame()
        self.status.set(f"Start channel set to {new}")

    # ========================================================
    # ACTIONS
    # ========================================================

    def toggle_blackout(self):
        self.blackout_on = not self.blackout_on
        if self.blackout_on:
            self.blackout_btn.set_kind("good", "RESTORE")
            self.status.set("BLACKOUT – all channels at 0")
        else:
            self.blackout_btn.set_kind("danger", "BLACKOUT")
            self.status.set("Output restored")

        for i, card in enumerate(self.cards):
            card.update_view(self.strip_output(i))
        self.send_frame()                                # always immediate

    def apply_preset(self, index, name):
        with self.batch():
            self.cards[index].set_values(*PRESETS[name])
            self.cards[index].on.set(True)
        self.status.set(f"Strip {index + 1}: {name}")

    def set_all_preset(self, name):
        with self.batch():
            for card in self.cards:
                card.set_values(*PRESETS[name])
                card.on.set(True)
        self.status.set(f"All strips: {name}")

    def pick_all_color(self):
        rgb, _ = colorchooser.askcolor(parent=self.root,
                                       title="Color for all strips")
        if rgb:
            with self.batch():
                for card in self.cards:
                    card.set_values(rgb[0], rgb[1], rgb[2], 0)
                    card.on.set(True)

    def copy_to_all(self, index):
        source = self.cards[index]
        values, on = source.values(), source.on.get()
        with self.batch():
            for card in self.cards:
                card.set_values(*values)
                card.on.set(on)
        self.status.set(f"Copied strip {index + 1} to all strips")

    def reset_all(self):
        self.stop_effect(restore=False)
        self.blackout_on = False
        self.blackout_btn.set_kind("danger", "BLACKOUT")
        with self.batch():
            self.master.set(255)
            for card in self.cards:
                card.set_values(0, 0, 0, 0)
                card.on.set(True)
        self.status.set("Reset")

    # ========================================================
    # SCENES
    # ========================================================

    def save_scene(self):
        path = filedialog.asksaveasfilename(
            parent=self.root, defaultextension=".json",
            filetypes=[("DMX scene", "*.json")], title="Save scene")
        if not path:
            return
        data = {
            "version": 1,
            "master": self.master_value(),
            "start_channel": self.start_channel,
            "strips": [
                dict(zip("rgbw", c.values()), on=c.on.get())
                for c in self.cards
            ],
        }
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            self.status.set(f"Scene saved: {path}")
        except OSError as error:
            messagebox.showerror("Save failed", str(error))

    def load_scene(self):
        path = filedialog.askopenfilename(
            parent=self.root, filetypes=[("DMX scene", "*.json")],
            title="Load scene")
        if not path:
            return
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            strips = data["strips"]
            if len(strips) != NUM_STRIPS:
                raise ValueError(f"Scene must contain {NUM_STRIPS} strips.")

            self.stop_effect(restore=False)
            with self.batch():
                self.master.set(int(data.get("master", 255)))
                for card, s in zip(self.cards, strips):
                    card.set_values(s["r"], s["g"], s["b"], s["w"])
                    card.on.set(bool(s.get("on", True)))
            self.status.set(f"Scene loaded: {path}")
        except (OSError, ValueError, KeyError, TypeError) as error:
            messagebox.showerror("Load failed", f"Invalid scene file:\n{error}")

    # ========================================================
    # EFFECTS
    # ========================================================

    def toggle_effect(self):
        if self.fx_running:
            self.stop_effect()
        else:
            self.start_effect()

    def start_effect(self):
        name = self.fx_var.get()
        if name not in EFFECTS:
            return

        self.fx_name = name
        self.fx_t = 0.0
        self.fx_last = time.time()
        self.fx_snapshot = None
        if name == "Rainbow":     # Rainbow overwrites colors; restore on stop
            self.fx_snapshot = [c.values() for c in self.cards]
        elif all(not any(c.values()) for c in self.cards):
            self.status.set(f"{name} uses the strip colors – set some colors first")

        self.fx_running = True
        self.fx_btn.set_kind("danger", "■ STOP")
        self.fx_tick()

    def stop_effect(self, restore=True):
        if not self.fx_running:
            return
        self.fx_running = False
        if self.fx_job is not None:
            self.root.after_cancel(self.fx_job)
            self.fx_job = None
        self.fx_btn.set_kind("accent", "▶ START")

        self.fx_mult = [1.0] * NUM_STRIPS
        with self.batch():
            if restore and self.fx_snapshot:
                for card, values in zip(self.cards, self.fx_snapshot):
                    card.set_values(*values)
        self.fx_snapshot = None

    def fx_tick(self):
        if not self.fx_running:
            return

        now = time.time()
        self.fx_t += (now - self.fx_last) * self.fx_speed.get()
        self.fx_last = now
        t = self.fx_t

        with self.batch():
            if self.fx_name == "Rainbow":
                for i, card in enumerate(self.cards):
                    h = (t * 0.08 + i / NUM_STRIPS) % 1.0
                    r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
                    card.set_values(int(r * 255), int(g * 255),
                                    int(b * 255), 0)

            elif self.fx_name == "Chase":
                pos = (t * 1.5) % NUM_STRIPS
                for i in range(NUM_STRIPS):
                    d = abs(i - pos)
                    d = min(d, NUM_STRIPS - d)
                    self.fx_mult[i] = max(0.0, 1.0 - d / 1.6)

            elif self.fx_name == "Breathe":
                level = (1.0 - math.cos(2 * math.pi * 0.3 * t)) / 2.0
                self.fx_mult = [0.04 + 0.96 * level] * NUM_STRIPS

        self.fx_job = self.root.after(40, self.fx_tick)

    # ========================================================
    # CONNECTION
    # ========================================================

    def reconnect(self):
        try:
            self.dmx.connect()
            self.status.set("uDMX connected")
            self.send_frame()
        except RuntimeError as error:
            self.status.set(str(error).replace("\n", " "))

    def poll_connection(self):
        if self.dmx.connected:
            self.conn_label.config(text="● uDMX connected", fg=GOOD)
        else:
            self.conn_label.config(text="● uDMX disconnected", fg=DANGER)

        error = self.dmx.last_error
        if error and error != self._shown_error:
            self.status.set(error.replace("\n", " "))
        self._shown_error = error

        self.root.after(300, self.poll_connection)

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):
        self.stop_effect(restore=False)
        self.dmx.send_now(self.start_channel, [0] * TOTAL_CHANNELS)
        self.dmx.close()
        self.root.destroy()


# ============================================================
# MAIN
# ============================================================

def main():
    try:                                    # crisp text on Windows HiDPI
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    dmx = UDMX()
    try:
        dmx.connect()
        print("uDMX connected.")
    except RuntimeError as error:
        # The GUI still opens so you can reconnect without restarting.
        dmx.last_error = str(error)
        print(f"WARNING: {error}")

    root = tk.Tk()
    DMXApp(root, dmx)
    root.mainloop()


if __name__ == "__main__":
    main()