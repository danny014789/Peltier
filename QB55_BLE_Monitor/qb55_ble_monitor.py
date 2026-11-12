"""QB55 BLE UART Monitor.

Connects to the board's Nordic UART Service (NUS). Shows whatever the board sends on the
TX characteristic (6E400003-...) and writes what you type to the RX characteristic
(6E400002-...). The board sends nothing on its own: it echoes each received line back as
"echo: <line>" (lines are limited to 64 characters), and understands these commands (any case):
    led on | led off        switch LD3, confirmed with "LED is ON" / "LED is OFF"
    IN+ <volts> | IN- <volts>   set an analog output, e.g. "IN+ 2V", "IN- 1.5", "IN+ 800mV";
                            0 to 3.3 V, 0 V at power-up; "IN+" alone reports the present level

    python qb55_ble_monitor.py                 open the window
    python qb55_ble_monitor.py --cli           no window, print lines to the console
    python qb55_ble_monitor.py --cli --seconds 10 --out log.txt     scripted check
    python qb55_ble_monitor.py --cli --seconds 8 --send "ping"      send a line, see the echo
    python qb55_ble_monitor.py --cli --scan                          list nearby BLE devices

Requires: pip install bleak   (uses the built-in Windows Bluetooth stack, no driver needed)
"""

import argparse
import asyncio
import ctypes
import queue
import sys
import threading
import time
from datetime import datetime

from bleak import BleakClient, BleakScanner

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # phone/PC -> board (write)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # board -> phone/PC (notify)
NUS_CHUNK = 20  # the firmware's RX characteristic holds at most 20 bytes per write

EOL_CHOICES = {"LF": b"\n", "CR+LF": b"\r\n", "None": b""}

DEFAULT_NAME_PREFIX = "QB55"
SCAN_TIMEOUT_S = 20.0      # how long Connect waits for the chosen device to be heard
SCAN_LIST_SECONDS = 5.0    # how long the Scan button listens

# The firmware stops advertising 60 s after boot and after every disconnect.
NOT_FOUND_HINT = (
    "Board not found. The firmware stops advertising 60 s after power-up or after a "
    "disconnect - press the RESET button on the board and click Connect again."
)


def stamp() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class BleUart:
    """Owns an asyncio loop on a worker thread; reports everything through `events`.

    Event tuples put on `events`:
        ("status", text)            progress line
        ("connected", name, addr, rssi)
        ("data", bytes)             one notification payload
        ("sent", bytes)             a message was fully written to the board
        ("senderror", text)
        ("scandone", [(name, address, rssi), ...])   result of scan()
        ("scanerror", text)
        ("notfound",)               the chosen device was not heard in time
        ("error", text)
        ("disconnected",)           always the last event of a session
    """

    def __init__(self) -> None:
        self.events: "queue.Queue[tuple]" = queue.Queue()
        self._loop = asyncio.new_event_loop()
        self._stop: "asyncio.Event | None" = None
        self._session_task: "asyncio.Task | None" = None
        self._client: "BleakClient | None" = None   # set only while a session is connected
        self._send_lock = asyncio.Lock()            # keeps chunks of concurrent sends in order
        self._thread = threading.Thread(target=self._run, name="ble-loop", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    # ---- public, thread-safe -------------------------------------------------
    def connect(self, name_prefix: str, address: str = "") -> None:
        asyncio.run_coroutine_threadsafe(self._start(name_prefix, address), self._loop)

    def disconnect(self) -> None:
        self._loop.call_soon_threadsafe(self._request_stop)

    def send(self, data: bytes) -> None:
        """Write `data` to the board's RX characteristic (split into 20-byte writes)."""
        asyncio.run_coroutine_threadsafe(self._send(data), self._loop)

    def scan(self, seconds: float = SCAN_LIST_SECONDS) -> None:
        """List nearby BLE devices; result arrives as ("scandone", [(name, address, rssi), ...])."""
        asyncio.run_coroutine_threadsafe(self._scan(seconds), self._loop)

    def shutdown(self) -> None:
        self.disconnect()
        deadline = time.time() + 3.0
        while self.busy() and time.time() < deadline:
            time.sleep(0.05)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)

    def busy(self) -> bool:
        task = self._session_task
        return task is not None and not task.done()

    # ---- loop-side -------------------------------------------------------------
    def _request_stop(self) -> None:
        if self._stop is not None:
            self._stop.set()

    async def _start(self, name_prefix: str, address: str) -> None:
        if self.busy():
            return
        self._stop = asyncio.Event()
        self._session_task = asyncio.ensure_future(self._session(name_prefix, address))

    def _emit(self, *event) -> None:
        self.events.put(event)

    async def _scan(self, seconds: float) -> None:
        try:
            found = await BleakScanner.discover(timeout=seconds, return_adv=True)
            items = [(adv.local_name or dev.name or "", dev.address, int(adv.rssi))
                     for dev, adv in found.values()]
            self._emit("scandone", items)
        except Exception as exc:  # noqa: BLE001 - e.g. Bluetooth switched off in Windows
            self._emit("scanerror", f"Scan failed: {type(exc).__name__}: {exc}. "
                                    "Is Bluetooth turned on in Windows?")

    async def _send(self, data: bytes) -> None:
        client = self._client
        if client is None or not client.is_connected:
            self._emit("senderror", "Not connected - nothing was sent.")
            return
        try:
            async with self._send_lock:
                # Reported before the writes finish: the board's echo can arrive before the last
                # write is acknowledged, and the log must show what you typed first. A failure
                # still produces a "senderror" line right after.
                self._emit("sent", data)
                # response=True: each write is acknowledged, so nothing is lost or reordered
                # if the board is busy. Throughput is irrelevant for typing text.
                for i in range(0, len(data), NUS_CHUNK):
                    await client.write_gatt_char(NUS_RX, data[i:i + NUS_CHUNK], response=True)
        except Exception as exc:  # noqa: BLE001
            self._emit("senderror", f"Send failed: {type(exc).__name__}: {exc}")

    async def _find_device(self, name_prefix: str, address: str):
        """Scan until the board shows up, the user cancels, or SCAN_TIMEOUT_S elapses."""
        seen_rssi: dict = {}
        want_addr = address.strip().upper()
        want_name = name_prefix.strip().lower()

        def match(device, adv) -> bool:
            name = (adv.local_name or device.name or "").lower()
            ok = (device.address.upper() == want_addr) if want_addr else (
                bool(want_name) and name.startswith(want_name))
            if ok:
                seen_rssi[device.address] = adv.rssi
            return ok

        scan = asyncio.ensure_future(
            BleakScanner.find_device_by_filter(match, timeout=SCAN_TIMEOUT_S))
        cancel = asyncio.ensure_future(self._stop.wait())
        done, _ = await asyncio.wait({scan, cancel}, return_when=asyncio.FIRST_COMPLETED)
        if cancel in done:
            scan.cancel()
            return None, None
        cancel.cancel()
        device = scan.result()
        return device, (seen_rssi.get(device.address) if device else None)

    async def _session(self, name_prefix: str, address: str) -> None:
        try:
            self._emit("status", "Scanning for the board...")
            device, rssi = await self._find_device(name_prefix, address)
            if self._stop.is_set():
                return
            if device is None:
                self._emit("notfound")
                return

            self._emit("status", f"Found {device.name or 'device'} ({device.address}), connecting...")
            lost = asyncio.Event()
            loop = self._loop

            def on_disconnect(_client) -> None:
                loop.call_soon_threadsafe(lost.set)

            # use_cached_services=False: never trust a GATT table Windows cached from earlier firmware.
            async with BleakClient(device, disconnected_callback=on_disconnect, timeout=15.0,
                                   winrt={"use_cached_services": False}) as client:
                if client.services.get_service(NUS_SERVICE) is None:
                    self._emit("error", "Connected, but the Nordic UART service (6E400001-...) is "
                                        "not on this device.")
                    return
                await client.start_notify(NUS_TX, lambda _c, data: self._emit("data", bytes(data)))
                self._client = client
                self._emit("connected", device.name or "", device.address, rssi)

                stop_wait = asyncio.ensure_future(self._stop.wait())
                lost_wait = asyncio.ensure_future(lost.wait())
                await asyncio.wait({stop_wait, lost_wait}, return_when=asyncio.FIRST_COMPLETED)
                for w in (stop_wait, lost_wait):
                    w.cancel()
                if not lost.is_set():
                    try:
                        await client.stop_notify(NUS_TX)
                    except Exception:
                        pass
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - surface anything the OS Bluetooth stack throws
            self._emit("error", f"{type(exc).__name__}: {exc}")
        finally:
            self._client = None
            self._emit("disconnected")


class LineAssembler:
    """Turns arbitrary notification chunks back into text lines (a line may span chunks)."""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list:
        self._buf.extend(chunk)
        lines = []
        while True:
            i = self._buf.find(b"\n")
            if i < 0:
                break
            lines.append(bytes(self._buf[:i]).decode("utf-8", "replace").rstrip("\r"))
            del self._buf[: i + 1]
        return lines

    def flush(self) -> str:
        text = bytes(self._buf).decode("utf-8", "replace").rstrip("\r")
        self._buf.clear()
        return text

    def pending(self) -> bool:
        return bool(self._buf)


def sent_text(data: bytes) -> str:
    """How a transmitted message is shown in the log (line ending stripped)."""
    return data.decode("utf-8", "replace").rstrip("\r\n")


def hex_line(chunk: bytes) -> str:
    text = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
    return f"{chunk.hex(' ').upper():<60} |{text}|"


# ============================== console mode =====================================
def run_cli(args) -> int:
    if sys.stdout is not None and hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    out = open(args.out, "w", encoding="utf-8") if args.out else None

    def say(text: str) -> None:
        line = f"{stamp()}  {text}"
        if sys.stdout is not None:
            print(line, flush=True)
        if out:
            out.write(line + "\n")
            out.flush()

    ble = BleUart()
    if args.scan:
        say(f"-- scanning for {SCAN_LIST_SECONDS:.0f} s ...")
        ble.scan(SCAN_LIST_SECONDS)
        ev = ble.events.get(timeout=SCAN_LIST_SECONDS + 15)
        if ev[0] != "scandone":
            say(f"-- ERROR {ev[1]}")
            ble.shutdown()
            return 1
        for n, a, r in sorted(ev[1], key=lambda d: (not d[0], -d[2])):
            say(f"{n or '(unnamed)':<24} {a}  {r:>4} dBm")
        say(f"-- {len(ev[1])} device(s), {sum(1 for d in ev[1] if d[0])} named")
        ble.shutdown()
        return 0 if ev[1] else 1

    asm = LineAssembler()
    received = 0
    deadline = time.time() + args.seconds if args.seconds else None
    ble.connect(args.name, args.address)
    try:
        while True:
            if deadline and time.time() > deadline:
                say("-- time limit reached")
                break
            try:
                ev = ble.events.get(timeout=0.2)
            except queue.Empty:
                continue
            kind = ev[0]
            if kind == "status":
                say(f"-- {ev[1]}")
            elif kind == "connected":
                say(f"-- CONNECTED to {ev[1] or '?'} [{ev[2]}] rssi={ev[3]} dBm")
                for text in args.send:
                    ble.send(text.encode("utf-8") + EOL_CHOICES[args.eol])
            elif kind == "sent":
                say(f"> {sent_text(ev[1])}")
            elif kind == "senderror":
                say(f"-- ERROR {ev[1]}")
            elif kind == "data":
                if args.hex:
                    say(hex_line(ev[1]))
                    received += 1
                else:
                    for line in asm.feed(ev[1]):
                        say(line)
                        received += 1
            elif kind == "notfound":
                say(f"-- {NOT_FOUND_HINT}")
                break
            elif kind == "error":
                say(f"-- ERROR {ev[1]}")
                break
            elif kind == "disconnected":
                say("-- disconnected")
                break
    except KeyboardInterrupt:
        pass
    finally:
        ble.shutdown()
        if out:
            out.close()
    return 0 if received else 1


# ================================ GUI mode =======================================
def run_gui(autoconnect: bool = False, name: str = DEFAULT_NAME_PREFIX,
            selftest: float = 0.0, dump: str = "", selftest_send: str = "") -> None:
    import tkinter as tk
    from tkinter import ttk
    from tkinter.scrolledtext import ScrolledText

    try:  # crisp text on high-DPI screens
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    root = tk.Tk()
    root.title("QB55 BLE UART Monitor")

    ble = BleUart()
    asm = LineAssembler()
    prefer = name.strip().lower()  # device whose name starts with this is pre-selected after a scan
    state = {"connected": False, "busy": False, "scanning": False,
             "auto": bool(autoconnect or selftest),  # connect by itself once the first scan ends
             "lines": 0, "bytes": 0, "tx": 0, "flush_job": None,
             "devices": [], "labels": {}}

    dev_var = tk.StringVar()
    unnamed_var = tk.BooleanVar(value=False)
    send_var = tk.StringVar()
    eol_var = tk.StringVar(value="LF")
    hex_var = tk.BooleanVar(value=False)
    scroll_var = tk.BooleanVar(value=True)
    status_var = tk.StringVar(value="Not connected")
    count_var = tk.StringVar(value="0 lines / 0 bytes received | 0 bytes sent")

    # Layout: the rows that must always be visible are packed first (top rows to the top, the
    # send and status rows to the bottom); the log is packed LAST so it is the only widget that
    # shrinks when space runs short. Packing the log first would push the send row out of view.
    top = ttk.Frame(root, padding=(10, 10, 10, 2))
    top.pack(side="top", fill="x")
    ttk.Label(top, text="Device:").pack(side="left")
    dev_box = ttk.Combobox(top, textvariable=dev_var, state="readonly", width=48)
    dev_box.pack(side="left", fill="x", expand=True, padx=(6, 8))
    scan_btn = ttk.Button(top, text="Scan", width=11)
    scan_btn.pack(side="left", padx=(0, 6))
    btn = ttk.Button(top, text="Connect", width=12)
    btn.pack(side="left")

    stat = ttk.Frame(root, padding=(10, 2, 10, 4))
    stat.pack(side="top", fill="x")
    dot = tk.Canvas(stat, width=14, height=14, highlightthickness=0)
    dot.pack(side="left", padx=(0, 6))
    dot_id = dot.create_oval(2, 2, 12, 12, fill="#9aa0a6", outline="")
    ttk.Label(stat, textvariable=status_var).pack(side="left")
    unnamed_chk = ttk.Checkbutton(stat, text="Show unnamed devices", variable=unnamed_var)
    unnamed_chk.pack(side="right")

    bottom = ttk.Frame(root, padding=(10, 0, 10, 10))
    bottom.pack(side="bottom", fill="x")
    ttk.Checkbutton(bottom, text="Hex view", variable=hex_var).pack(side="left")
    ttk.Checkbutton(bottom, text="Auto-scroll", variable=scroll_var).pack(side="left", padx=10)
    ttk.Label(bottom, textvariable=count_var, foreground="#555555").pack(side="right")
    clear_btn = ttk.Button(bottom, text="Clear", width=8)
    clear_btn.pack(side="right", padx=10)

    sendf = ttk.Frame(root, padding=(10, 4, 10, 6))
    sendf.pack(side="bottom", fill="x")
    ttk.Label(sendf, text="Send:").pack(side="left")
    send_entry = ttk.Entry(sendf, textvariable=send_var, state="disabled")
    send_entry.pack(side="left", fill="x", expand=True, padx=(6, 8))
    ttk.Label(sendf, text="Ends with:").pack(side="left")
    eol_box = ttk.Combobox(sendf, textvariable=eol_var, values=list(EOL_CHOICES), width=6,
                           state="disabled")
    eol_box.pack(side="left", padx=(4, 8))
    send_btn = ttk.Button(sendf, text="Send", width=8, state="disabled")
    send_btn.pack(side="left")

    box = ScrolledText(root, font=("Consolas", 11), height=8, state="disabled", wrap="word",
                       borderwidth=1, relief="solid")
    box.pack(side="top", fill="both", expand=True, padx=10, pady=4)
    box.tag_configure("ts", foreground="#7a7f87")
    box.tag_configure("data", foreground="#111111")
    box.tag_configure("sys", foreground="#1a5fb4", font=("Consolas", 10, "italic"))
    box.tag_configure("err", foreground="#c01c28")
    box.tag_configure("tx", foreground="#0b6e4f")

    def set_dot(color: str) -> None:
        dot.itemconfigure(dot_id, fill=color)

    def write(text: str, tag: str, with_stamp: bool = True) -> None:
        box.configure(state="normal")
        if with_stamp:
            box.insert("end", stamp() + "  ", "ts")
        box.insert("end", text + "\n", tag)
        box.configure(state="disabled")
        if scroll_var.get():
            box.see("end")

    def update_counts() -> None:
        count_var.set(f"{state['lines']} lines / {state['bytes']} bytes received | "
                      f"{state['tx']} bytes sent")

    def selected_address() -> str:
        return state["labels"].get(dev_var.get(), "")

    def update_buttons() -> None:
        live = state["connected"] or state["busy"]
        idle = not (live or state["scanning"])
        scan_btn.configure(text="Scanning..." if state["scanning"] else "Scan",
                           state="normal" if idle else "disabled")
        dev_box.configure(state="readonly" if idle else "disabled")
        unnamed_chk.configure(state="normal" if idle else "disabled")
        if live:
            btn.configure(text="Disconnect", state="normal")
        else:
            btn.configure(text="Connect",
                          state="normal" if (selected_address() and not state["scanning"]) else "disabled")
        ready = "normal" if state["connected"] else "disabled"
        send_entry.configure(state=ready)
        send_btn.configure(state=ready)
        eol_box.configure(state="readonly" if state["connected"] else "disabled")

    def refresh_list() -> None:
        """Rebuild the dropdown from the last scan, keeping the current choice if it is still there."""
        keep = selected_address()
        items = [d for d in state["devices"] if d[0] or unnamed_var.get()]
        items.sort(key=lambda d: (not d[0], -d[2]))  # named first, strongest signal first
        labels = {f"{n or '(unnamed)'}   [{a}]   {r} dBm": a for n, a, r in items}
        names = {a: n for n, a, _ in items}
        state["labels"] = labels
        dev_box.configure(values=list(labels))
        choice = next((lab for lab, a in labels.items() if a == keep), "")
        if not choice and prefer:
            choice = next((lab for lab, a in labels.items()
                           if names[a].lower().startswith(prefer)), "")
        dev_var.set(choice)
        update_buttons()

    def start_scan() -> None:
        if state["connected"] or state["busy"] or state["scanning"]:
            return
        state["scanning"] = True
        set_dot("#e5a50a")
        status_var.set(f"Scanning for Bluetooth devices ({int(SCAN_LIST_SECONDS)} s)...")
        update_buttons()
        ble.scan(SCAN_LIST_SECONDS)

    def on_button() -> None:
        if state["connected"] or state["busy"]:
            status_var.set("Disconnecting...")
            ble.disconnect()
            return
        addr = selected_address()
        if not addr:
            write("Pick a device from the list first (press Scan if it is empty).", "err")
            return
        state["busy"] = True
        set_dot("#e5a50a")
        status_var.set("Connecting...")
        update_buttons()
        ble.connect("", addr)

    def do_send(_event=None) -> None:
        if not state["connected"]:
            return
        ending = EOL_CHOICES[eol_var.get()]
        text = send_var.get()
        if not text and not ending:
            return
        ble.send(text.encode("utf-8") + ending)
        send_var.set("")

    def flush_partial() -> None:
        state["flush_job"] = None
        if asm.pending():
            text = asm.flush()
            if text:
                write(text, "data")
                state["lines"] += 1
                update_counts()

    def on_data(chunk: bytes) -> None:
        state["bytes"] += len(chunk)
        if hex_var.get():
            write(hex_line(chunk), "data")
            state["lines"] += 1
        else:
            for line in asm.feed(chunk):
                write(line, "data")
                state["lines"] += 1
            if asm.pending():  # unterminated text: show it if nothing completes it shortly
                if state["flush_job"]:
                    root.after_cancel(state["flush_job"])
                state["flush_job"] = root.after(300, flush_partial)
        update_counts()

    def on_scan_done(items: list) -> None:
        state["scanning"] = False
        state["devices"] = items
        named = sum(1 for d in items if d[0])
        write(f"Scan finished: {len(items)} device(s) heard, {named} with a name.", "sys")
        refresh_list()
        set_dot("#9aa0a6")
        if selected_address():
            status_var.set("Ready - press Connect")
            if state["auto"]:
                on_button()
        else:
            status_var.set("Pick a device, then press Connect")
            if prefer:
                write(f"No device named '{name}...' was heard. The board only advertises for 60 s "
                      "after power-up or a disconnect: press RESET on it and click Scan again.", "err")
        state["auto"] = False

    def poll() -> None:
        try:
            while True:
                ev = ble.events.get_nowait()
                kind = ev[0]
                if kind == "scandone":
                    on_scan_done(ev[1])
                elif kind == "scanerror":
                    state["scanning"] = False
                    status_var.set("Scan failed")
                    set_dot("#9aa0a6")
                    write(ev[1], "err")
                    update_buttons()
                elif kind == "status":
                    status_var.set(ev[1])
                    write(ev[1], "sys")
                elif kind == "connected":
                    state["connected"], state["busy"] = True, False
                    set_dot("#26a269")
                    rssi = f", {ev[3]} dBm" if ev[3] is not None else ""
                    status_var.set(f"Connected to {ev[1] or ev[2]}{rssi}")
                    write(f"Connected to {ev[1] or '?'} [{ev[2]}]{rssi}. "
                          "Type  help  for the commands: led on/off, IN+ 2V, IN- 1.5V, and "
                          "temperature control (set 40, pid on, status).", "sys")
                    update_buttons()
                    send_entry.focus_set()
                    if selftest_send:  # test hook: type-and-send through the real code path
                        root.after(600, lambda: (send_var.set(selftest_send), do_send()))
                elif kind == "sent":
                    state["tx"] += len(ev[1])
                    write("> " + sent_text(ev[1]), "tx")
                    update_counts()
                elif kind == "senderror":
                    write(ev[1], "err")
                elif kind == "data":
                    on_data(ev[1])
                elif kind == "notfound":
                    status_var.set("Board not found")
                    write(NOT_FOUND_HINT, "err")
                elif kind == "error":
                    status_var.set("Error")
                    write(ev[1], "err")
                elif kind == "disconnected":
                    flush_partial()
                    if status_var.get() not in ("Board not found", "Error"):
                        status_var.set("Disconnected")
                        write("Disconnected.", "sys")
                    state["connected"], state["busy"] = False, False
                    set_dot("#9aa0a6")
                    update_buttons()
        except queue.Empty:
            pass
        root.after(30, poll)

    def clear() -> None:
        box.configure(state="normal")
        box.delete("1.0", "end")
        box.configure(state="disabled")
        state["lines"] = state["bytes"] = state["tx"] = 0
        update_counts()

    def on_close() -> None:
        ble.shutdown()
        root.destroy()

    btn.configure(command=on_button)
    scan_btn.configure(command=start_scan)
    send_btn.configure(command=do_send)
    send_entry.bind("<Return>", do_send)
    clear_btn.configure(command=clear)
    dev_box.bind("<<ComboboxSelected>>", lambda _e: update_buttons())
    unnamed_chk.configure(command=refresh_list)
    root.protocol("WM_DELETE_WINDOW", on_close)

    # Size the window from what its widgets actually need (scales with Windows display scaling),
    # never below that, and keep the fixed rows visible however small the log gets.
    root.update_idletasks()
    fixed = sum(f.winfo_reqheight() for f in (top, stat, sendf, bottom))
    root.geometry(f"{max(780, root.winfo_reqwidth())}x{max(500, root.winfo_reqheight())}")
    root.minsize(max(640, top.winfo_reqwidth()), fixed + 110)

    write("Scanning on start-up. Pick your board in the list and press Connect. "
          "If it is missing, press RESET on the board and click Scan.", "sys", with_stamp=False)
    update_buttons()
    root.after(30, poll)
    root.after(300, start_scan)

    if selftest:  # test hook: run for N seconds, report layout + what the window shows, then close
        def finish() -> None:
            root.update_idletasks()
            if dump:
                rows = []
                if not root.winfo_ismapped():  # minimized/hidden windows report every widget unmapped
                    rows.append("not measured: the window is minimized or hidden")
                else:
                    for label, w in (("top", top), ("status", stat), ("log", box.frame),
                                     ("send", sendf), ("bottom", bottom)):
                        y, h = w.winfo_y(), w.winfo_height()
                        clipped = (y + h > root.winfo_height()) or h < 10
                        rows.append(f"{label}: y={y} h={h}{'  <-- CLIPPED' if clipped else ''}")
                with open(dump, "w", encoding="utf-8") as fh:
                    fh.write(box.get("1.0", "end"))
                    fh.write(f"--- window {root.winfo_width()}x{root.winfo_height()} | "
                             f"dpi {root.winfo_fpixels('1i'):.0f}\n")
                    fh.write("--- layout: " + " | ".join(rows) + "\n")
                    fh.write(f"--- dropdown ({len(dev_box['values'])}): {list(dev_box['values'])} "
                             f"| selected: {dev_var.get()!r}\n")
                    fh.write(f"--- status: {status_var.get()} | {count_var.get()} | "
                             f"button: {btn.cget('text')} | send entry: {send_entry.cget('state')}\n")
            on_close()
        root.after(int(selftest * 1000), finish)

    root.mainloop()


def main() -> int:
    ap = argparse.ArgumentParser(description="QB55 BLE UART monitor (Nordic UART Service).")
    ap.add_argument("--cli", action="store_true", help="console mode, no window")
    ap.add_argument("--name", default=DEFAULT_NAME_PREFIX, help="match devices whose name starts with this")
    ap.add_argument("--address", default="", help="connect to this Bluetooth address instead of matching a name")
    ap.add_argument("--seconds", type=float, default=0, help="CLI: stop after N seconds")
    ap.add_argument("--hex", action="store_true", help="CLI: show each notification as hex")
    ap.add_argument("--out", default="", help="CLI: also write the output to this file")
    ap.add_argument("--scan", action="store_true", help="CLI: list nearby BLE devices and exit")
    ap.add_argument("--send", action="append", default=[], metavar="TEXT",
                    help="CLI: send this text to the board once connected (repeatable)")
    ap.add_argument("--eol", choices=["LF", "CR+LF", "None"], default="LF",
                    help="line ending appended to --send text (default LF)")
    ap.add_argument("--autoconnect", action="store_true", help="GUI: start connecting on launch")
    ap.add_argument("--selftest", type=float, default=0, help=argparse.SUPPRESS)
    ap.add_argument("--dump", default="", help=argparse.SUPPRESS)
    ap.add_argument("--selftest-send", default="", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.cli:
        return run_cli(args)
    run_gui(args.autoconnect, args.name, args.selftest, args.dump, args.selftest_send)
    return 0


if __name__ == "__main__":
    sys.exit(main())
