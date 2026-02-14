#!/usr/bin/env python3
"""
DAQ GUI controller (v3)
- Single Responses pane (pretty JSON + concise summary)
- Removed raw transcript and SD file listbox
- LED slider retains inverted behavior (visual: right = dim, left = bright)
- Layout improved to scale with window resizing
Requires: pyserial
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, simpledialog
import serial
import serial.tools.list_ports
import threading
import json
import time
import sys
import re

BAUDRATE = 115200
READ_TIMEOUT = 5
CMD_TIMEOUT = 15

# ---------------- Serial comm helper ----------------
class SerialComm:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    def open(self, port, baud=BAUDRATE, timeout=READ_TIMEOUT):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
            time.sleep(0.2)
        return True

    def close(self):
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def is_open(self):
        with self.lock:
            return self.ser is not None and self.ser.is_open

    def send_cmd(self, cmd, timeout=READ_TIMEOUT):
        with self.lock:
            if not self.ser or not self.ser.is_open:
                raise RuntimeError("Serial port not open")
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

            cmd_line = cmd.strip() + "\n"
            self.ser.write(cmd_line.encode('utf-8'))

            old_timeout = self.ser.timeout
            self.ser.timeout = timeout
            try:
                raw = self.ser.readline()
            finally:
                self.ser.timeout = old_timeout

            if not raw:
                raise TimeoutError(f"No response within {timeout}s for command: {cmd}")

            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception as e:
                raise RuntimeError("Failed to decode response: " + str(e))

            # Try parse JSON or extract JSON substring
            parsed = None
            try:
                parsed = json.loads(line)
            except Exception:
                m = re.search(r'\{.*\}', line)
                if m:
                    try:
                        parsed = json.loads(m.group(0))
                    except Exception:
                        parsed = None

            return parsed, line

# ---------------- GUI app ----------------
class DAQGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DAQ Test & Calibration GUI")
        self.geometry("980x680")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.comm = SerialComm()
        self._create_widgets()
        self._layout_widgets()

        # Keep port list updated
        self.after(1000, self.update_serial_ports)

    def _create_widgets(self):
        # Connection frame
        self.frm_serial = ttk.LabelFrame(self, text="Connection")
        self.cmb_ports = ttk.Combobox(self.frm_serial, values=self._list_ports(), state="readonly", width=30)
        self.btn_refresh = ttk.Button(self.frm_serial, text="Refresh", command=self.update_serial_ports)
        self.ent_baud = ttk.Entry(self.frm_serial, width=8); self.ent_baud.insert(0, str(BAUDRATE))
        self.btn_connect = ttk.Button(self.frm_serial, text="Connect", command=self.connect_serial)
        self.lbl_status = ttk.Label(self.frm_serial, text="Not connected", foreground="red")

        # Controls (top)
        self.frm_controls = ttk.LabelFrame(self, text="Controls")
        self.btn_ping = ttk.Button(self.frm_controls, text="PING", command=lambda: self._run_cmd_async("PING"))
        self.btn_status = ttk.Button(self.frm_controls, text="STATUS", command=lambda: self._run_cmd_async("STATUS"))
        self.btn_read = ttk.Button(self.frm_controls, text="READ", command=lambda: self._run_cmd_async("READ"))
        self.btn_test_temp = ttk.Button(self.frm_controls, text="TEST_TEMP", command=lambda: self._run_cmd_async("TEST_TEMP"))
        self.btn_test_pressure = ttk.Button(self.frm_controls, text="TEST_PRESSURE", command=lambda: self._run_cmd_async("TEST_PRESSURE"))
        self.btn_list_files = ttk.Button(self.frm_controls, text="List Files (SD)", command=lambda: self._run_cmd_async("LIST_FILES"))
        self.btn_start = ttk.Button(self.frm_controls, text="Start Recording", command=lambda: self._run_cmd_async("START"))
        self.btn_stop = ttk.Button(self.frm_controls, text="Stop Recording", command=lambda: self._run_cmd_async("STOP"))

        # LED control (label simplified)
        self.frm_led = ttk.LabelFrame(self.frm_controls, text="LED")
        # inverted numeric range visually: from 255 -> 0 so sliding right decreases numeric value
        self.led_scale = ttk.Scale(self.frm_led, from_=255, to=0, orient=tk.HORIZONTAL)
        self.led_scale.set(255)  # default dim (right-most)
        self.btn_set_led = ttk.Button(self.frm_led, text="SET_LED", command=self.set_led_from_scale)
        self.btn_get_led = ttk.Button(self.frm_led, text="GET_LED", command=lambda: self._run_cmd_async("GET_LED"))

        # Calibration
        self.frm_cal = ttk.LabelFrame(self, text="Load Cell Calibration")
        self.btn_tare = ttk.Button(self.frm_cal, text="TARE", command=lambda: self._run_cmd_async("TARE"))
        self.btn_calibrate = ttk.Button(self.frm_cal, text="Start Calibration (guided)", command=self.calibration_walkthrough)
        self.lbl_cal_instructions = ttk.Label(self.frm_cal, text="Guided calibration: follow prompts to place known weight.")

        # Full test
        self.frm_test = ttk.LabelFrame(self, text="Full System Test")
        self.btn_full_test = ttk.Button(self.frm_test, text="Run Full Automated Test", command=self.full_system_test)
        self.chk_save_test = tk.BooleanVar(value=True)
        self.chk_save_file = ttk.Checkbutton(self.frm_test, text="Create test recording file on SD", variable=self.chk_save_test)

        # Responses pane (single)
        self.frm_responses = ttk.LabelFrame(self, text="Responses")
        self.txt_responses = scrolledtext.ScrolledText(self.frm_responses, wrap=tk.WORD)
        self.txt_responses.configure(state=tk.DISABLED)

        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(self, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)

    def _layout_widgets(self):
        # Grid layout with good scaling
        PAD = 8
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(3, weight=1)

        # Connection row
        self.frm_serial.grid(row=0, column=0, columnspan=2, sticky="ew", padx=PAD, pady=PAD)
        ttk.Label(self.frm_serial, text="Port:").grid(row=0, column=0, sticky="w", padx=(6,2))
        self.cmb_ports.grid(row=0, column=1, sticky="w")
        self.btn_refresh.grid(row=0, column=2, sticky="w", padx=4)
        ttk.Label(self.frm_serial, text="Baud:").grid(row=0, column=3, sticky="e", padx=(10,2))
        self.ent_baud.grid(row=0, column=4, sticky="w")
        self.btn_connect.grid(row=0, column=5, sticky="w", padx=6)
        self.lbl_status.grid(row=0, column=6, sticky="w", padx=8)

        # Controls area
        self.frm_controls.grid(row=1, column=0, columnspan=2, sticky="ew", padx=PAD, pady=(0,4))
        btns = [self.btn_ping, self.btn_status, self.btn_read, self.btn_test_temp, self.btn_test_pressure]
        for i,b in enumerate(btns):
            b.grid(row=0, column=i, padx=4, pady=4, sticky="ew")
            self.frm_controls.grid_columnconfigure(i, weight=1)
        # second row of control buttons
        self.btn_list_files.grid(row=1, column=0, padx=4, pady=4, sticky="ew")
        self.btn_start.grid(row=1, column=1, padx=4, pady=4, sticky="ew")
        self.btn_stop.grid(row=1, column=2, padx=4, pady=4, sticky="ew")
        self.frm_controls.grid_columnconfigure(0, weight=1)
        self.frm_controls.grid_columnconfigure(1, weight=1)
        self.frm_controls.grid_columnconfigure(2, weight=1)

        # LED subframe in controls (placed to the right)
        self.frm_led.grid(row=0, column=6, rowspan=2, padx=6, pady=4, sticky="nsew")
        self.frm_led.grid_columnconfigure(0, weight=1)
        self.led_scale.grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        self.btn_set_led.grid(row=0, column=1, padx=6, pady=6)
        self.btn_get_led.grid(row=0, column=2, padx=6, pady=6)

        # Calibration and full-test frames on right column
        self.frm_cal.grid(row=2, column=0, sticky="nsew", padx=PAD, pady=PAD)
        self.frm_test.grid(row=2, column=1, sticky="nsew", padx=PAD, pady=PAD)
        self.frm_cal.grid_columnconfigure(0, weight=1)
        self.frm_test.grid_columnconfigure(0, weight=1)
        self.lbl_cal_instructions.grid(row=0, column=0, columnspan=2, sticky="w", padx=6, pady=(4,2))
        self.btn_tare.grid(row=1, column=0, padx=6, pady=6, sticky="ew")
        self.btn_calibrate.grid(row=1, column=1, padx=6, pady=6, sticky="ew")
        self.btn_full_test.grid(row=0, column=0, padx=6, pady=6, sticky="ew")
        self.chk_save_file.grid(row=0, column=1, padx=6, pady=6)

        # Responses pane (fills remaining area)
        self.frm_responses.grid(row=3, column=0, columnspan=2, sticky="nsew", padx=PAD, pady=PAD)
        self.frm_responses.grid_columnconfigure(0, weight=1)
        self.frm_responses.grid_rowconfigure(0, weight=1)
        self.txt_responses.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        # Status bar
        self.status_bar.grid(row=4, column=0, columnspan=2, sticky="ew")

    # ---------------- port helpers ----------------
    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def update_serial_ports(self):
        ports = self._list_ports()
        self.cmb_ports['values'] = ports
        if not self.cmb_ports.get() and ports:
            self.cmb_ports.set(ports[0])
        self.after(2000, self.update_serial_ports)

    def connect_serial(self):
        if self.comm.is_open():
            self.comm.close()
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Not connected", foreground="red")
            self.status_var.set("Disconnected")
            return

        port = self.cmb_ports.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first")
            return
        try:
            baud = int(self.ent_baud.get())
        except Exception:
            baud = BAUDRATE
        try:
            self.comm.open(port, baud, timeout=READ_TIMEOUT)
        except Exception as e:
            messagebox.showerror("Connection error", f"Failed to open {port}:\n{e}")
            return

        self.btn_connect.config(text="Disconnect")
        self.lbl_status.config(text=f"Connected: {port}", foreground="green")
        self.status_var.set(f"Connected {port}")
        self._log_response_plain(f"Connected to {port} @ {baud}")

    def on_close(self):
        if self.comm.is_open():
            try:
                self.comm.close()
            except:
                pass
        self.destroy()

    # ---------------- responses logging ----------------
    def _log_response_plain(self, text):
        """Add a plain text line to responses (unstructured)."""
        self.txt_responses.configure(state=tk.NORMAL)
        self.txt_responses.insert(tk.END, f"{time.strftime('%Y-%m-%d %H:%M:%S')} | {text}\n")
        self.txt_responses.see(tk.END)
        self.txt_responses.configure(state=tk.DISABLED)

    def _log_response(self, raw, parsed=None):
        """Pretty-print parsed JSON (if available) and a concise summary line."""
        self.txt_responses.configure(state=tk.NORMAL)
        ts = time.strftime('%Y-%m-%d %H:%M:%S')
        if parsed and isinstance(parsed, dict):
            # Summary line
            ok = parsed.get("ok")
            cmd = parsed.get("cmd", "")
            msg = parsed.get("msg", "")
            summary = f"{ts} | {cmd} | ok={ok} | msg={msg}\n"
            pretty = json.dumps(parsed, indent=2)
            self.txt_responses.insert(tk.END, summary)
            self.txt_responses.insert(tk.END, pretty + "\n\n")
        else:
            # fallback to raw line
            self.txt_responses.insert(tk.END, f"{ts} | RAW: {raw}\n\n")
        self.txt_responses.see(tk.END)
        self.txt_responses.configure(state=tk.DISABLED)

    # ---------------- async command wrapper ----------------
    def _run_cmd_async(self, cmd, timeout=READ_TIMEOUT, callback=None):
        t = threading.Thread(target=self._cmd_worker, args=(cmd, timeout, callback), daemon=True)
        t.start()

    def _cmd_worker(self, cmd, timeout, callback):
        self.status_var.set(f"Sending: {cmd}")
        self._log_response_plain(f">>> {cmd}")
        try:
            parsed, raw = self.comm.send_cmd(cmd, timeout=timeout)
            self._log_response(raw, parsed)
            if callback:
                callback(True, parsed, raw)
        except Exception as e:
            err = str(e)
            self._log_response_plain(f"<<< ERROR: {err}")
            self._log_response(err, None)
            if callback:
                callback(False, None, err)
        finally:
            self.status_var.set("Ready")

    # ---------------- actions ----------------
    def set_led_from_scale(self):
        # slider already inverted: value = brightness to send
        val = int(self.led_scale.get())
        self._run_cmd_async(f"SET_LED {val}")

    def calibration_walkthrough(self):
        if not self.comm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first")
            return

        if messagebox.askyesno("Tare now?", "Issue a TARE before calibration?"):
            self._run_cmd_async("TARE")
            time.sleep(0.2)

        messagebox.showinfo("Place weight", "Place the known calibration weight on the scale, then click OK to continue.")
        weight_str = simpledialog.askstring("Known weight (g)", "Enter known weight in grams:", parent=self)
        if not weight_str:
            messagebox.showinfo("Cancelled", "Calibration cancelled (no weight provided).")
            return
        try:
            weight_val = float(weight_str)
            if weight_val <= 0:
                raise ValueError()
        except Exception:
            messagebox.showerror("Invalid value", "Please enter a valid positive number in grams.")
            return

        def cb(ok, resp, raw):
            if not ok:
                messagebox.showerror("Calibration failed", f"Error: {raw}")
                return
            if resp.get("ok"):
                data = resp.get("data")
                messagebox.showinfo("Calibration success", f"Calibration result:\n{json.dumps(data, indent=2)}")
            else:
                messagebox.showerror("Calibration error", f"{resp.get('msg')}\n{json.dumps(resp)}")

        self._run_cmd_async(f"CALIBRATE {weight_val}", timeout=CMD_TIMEOUT, callback=cb)

    def full_system_test(self):
        if not self.comm.is_open():
            messagebox.showwarning("Not connected", "Connect to the board first")
            return
        t = threading.Thread(target=self._full_test_worker, daemon=True)
        t.start()

    def _full_test_worker(self):
        self.status_var.set("Running full test...")
        steps = [
            ("PING", 3),
            ("STATUS", 3),
            ("TEST_TEMP", 3),
            ("TEST_PRESSURE", 3),
        ]
        for cmd, to in steps:
            try:
                parsed, raw = self.comm.send_cmd(cmd, timeout=to)
                self._log_response(raw, parsed)
            except Exception as e:
                self._log_response(str(e), None)
            time.sleep(0.2)

        # READ several times
        for i in range(4):
            try:
                parsed, raw = self.comm.send_cmd("READ", timeout=3)
                self._log_response(raw, parsed)
            except Exception as e:
                self._log_response(str(e), None)
            time.sleep(0.2)

        # optional recording test
        if self.chk_save_test.get():
            try:
                parsed, raw = self.comm.send_cmd("START", timeout=5)
                self._log_response(raw, parsed)
                for i in range(6):
                    parsed, raw = self.comm.send_cmd("READ", timeout=3)
                    self._log_response(raw, parsed)
                    time.sleep(0.2)
                parsed, raw = self.comm.send_cmd("STOP", timeout=5)
                self._log_response(raw, parsed)
            except Exception as e:
                self._log_response(str(e), None)
        else:
            self._log_response_plain("Skipping recording file test (user choice)")

        # LIST_FILES and show in responses
        try:
            parsed, raw = self.comm.send_cmd("LIST_FILES", timeout=5)
            self._log_response(raw, parsed)
        except Exception as e:
            self._log_response(str(e), None)

        self.status_var.set("Full test complete")

# ---------------- main ----------------
def main():
    app = DAQGui()
    app.mainloop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting")
        sys.exit(0)
