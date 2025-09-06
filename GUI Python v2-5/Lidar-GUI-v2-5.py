#!/usr/bin/env python3
"""
LiDAR Configuration GUI - Modern Bootstrap Styled Version
Complete GUI for configuring Arduino LiDAR system parameters with proper communication fixes
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, font
import ttkbootstrap as tb
from ttkbootstrap.constants import *
from ttkbootstrap.tooltip import ToolTip
import threading
import queue
import time
import logging
import struct
import json
from typing import Dict, Any, Optional, List
from enum import Enum
from dataclasses import dataclass
import serial
import serial.tools.list_ports

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger('LidarGUI')

# ===================================================================================
# ===== COMMUNICATION PROTOCOL CONSTANTS (Must match Arduino code) ==================
# ===================================================================================
GUI_PACKET_START_BYTE = 0x7E
RSP_ACK = 0x06
RSP_NAK = 0x15

NAK_ERRORS = {
    0x00: "No Error",
    0x01: "Bad Checksum",
    0x02: "Unknown Command",
    0x03: "Invalid Payload",
    0x04: "Execution Fail",
    0x05: "Timeout"
}

# Conversion constant
MPH_TO_CMS = 44.704

# ===== ARDUINO-COMPATIBLE PROTOCOL =====
class LidarProtocol:
    """Arduino-compatible protocol implementation"""
    
    START_BYTE = 0x7E
    RSP_ACK = 0x06
    RSP_NAK = 0x15
    MAX_PAYLOAD_SIZE = 64
    
    MIN_DISTANCE_CM = 7
    MAX_DISTANCE_CM = 1200
    MIN_VELOCITY_MPH = 2
    MAX_VELOCITY_MPH = 120
    
    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        return sum(data) & 0xFF
    
    @classmethod
    def create_packet(cls, command: str, payload: bytes = b'') -> bytes:
        if len(command) != 1:
            raise ValueError("Command must be single character")
        if len(payload) > cls.MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(payload)} > {cls.MAX_PAYLOAD_SIZE}")
        
        cmd_byte = ord(command)
        len_byte = len(payload)
        packet = bytes([cls.START_BYTE, cmd_byte, len_byte]) + payload
        checksum_data = packet[1:]
        checksum = cls.calculate_checksum(checksum_data)
        return packet + bytes([checksum])
    
    @classmethod
    def parse_packet(cls, data: bytes) -> Optional[Dict[str, Any]]:
        if len(data) < 4:
            return None
            
        if data[0] != cls.START_BYTE:
            return None
        
        try:
            cmd = data[1]
            length = data[2]
            expected_total_length = 4 + length
            
            if len(data) < expected_total_length:
                return None
            
            payload = data[3:3+length] if length > 0 else b''
            received_checksum = data[3+length]
            
            checksum_data = data[1:3+length]
            calculated_checksum = cls.calculate_checksum(checksum_data)
            
            if calculated_checksum != received_checksum:
                logger.error(f"Checksum mismatch: calculated=0x{calculated_checksum:02X}, received=0x{received_checksum:02X}")
                return None
            
            return {
                'command': cmd,
                'payload': payload,
                'length': length,
                'valid': True
            }
            
        except (IndexError, ValueError) as e:
            logger.error(f"Packet parsing error: {e}")
            return None

# ===== CONNECTION STATE =====
class ConnectionState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting" 
    CONNECTED = "connected"
    ERROR = "error"

# ===== SERIAL COMMUNICATION MANAGER =====
class SerialManager:
    """Handles all low-level serial communication in a dedicated thread with Arduino compatibility."""

    def __init__(self, incoming_queue, outgoing_queue):
        self.ser = None
        self.incoming_queue = incoming_queue
        self.outgoing_queue = outgoing_queue
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.connection_state = ConnectionState.DISCONNECTED

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.thread.join(timeout=1)

    def connect(self, port, baudrate):
        try:
            self.connection_state = ConnectionState.CONNECTING
            time.sleep(0.1)
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(0.1)
            
            # CRITICAL: Send immediate data to trigger Arduino CONFIG mode
            self._send_packet('S')  # Send status request to trigger config mode
            time.sleep(0.1)
            
            self.connection_state = ConnectionState.CONNECTED
            self.log_message(f"Connected to {port} at {baudrate} baud.")
            return True
        except serial.SerialException as e:
            self.connection_state = ConnectionState.ERROR
            self.log_message(f"Error connecting to {port}: {e}", "error")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connection_state = ConnectionState.DISCONNECTED
            self.log_message("Disconnected.")

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def log_message(self, msg, level="info"):
        self.incoming_queue.put(("log", f"[{level.upper()}] {msg}"))

    def _send_packet(self, command, payload=b''):
        if not self.is_connected():
            self.log_message("Cannot send packet, not connected.", "warning")
            return
        try:
            packet = LidarProtocol.create_packet(command, payload)
            self.ser.write(packet)
            self.log_message(f"Sent packet: {packet.hex().upper()}")
        except Exception as e:
            self.log_message(f"Error sending packet: {e}", "error")

    def _run(self):
        """Main loop for the serial thread with Arduino packet parsing."""
        parser_buffer = bytearray()
        
        while not self.stop_event.is_set():
            # Process outgoing commands
            try:
                command, payload = self.outgoing_queue.get_nowait()
                self._send_packet(command, payload)
                time.sleep(0.05) 
            except queue.Empty:
                pass 

            if not self.is_connected():
                time.sleep(0.1)
                continue

            # Process incoming data
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    parser_buffer.extend(data)
                    
                    # Parse complete packets
                    while len(parser_buffer) >= 4:
                        packet = self._try_parse_packet(parser_buffer)
                        if packet:
                            packet_length = 4 + packet['length']
                            parser_buffer = parser_buffer[packet_length:]
                            self.incoming_queue.put(("packet", packet))
                        else:
                            break

            except serial.SerialException as e:
                self.log_message(f"Serial error: {e}", "error")
                self.disconnect()
                self.incoming_queue.put(("disconnected", None))
            except Exception as e:
                self.log_message(f"Processing error: {e}", "error")
            
            time.sleep(0.01)

    def _try_parse_packet(self, buffer):
        """Try to parse a complete Arduino packet from buffer"""
        # Find start byte
        start_idx = -1
        for i, byte in enumerate(buffer):
            if byte == GUI_PACKET_START_BYTE:
                start_idx = i
                break
        
        if start_idx == -1:
            if len(buffer) > 100:
                buffer.clear()
            return None
        
        if start_idx > 0:
            # Remove leading garbage
            buffer[:] = buffer[start_idx:]
        
        if len(buffer) < 4:
            return None
        
        payload_length = buffer[2]
        total_length = 4 + payload_length
        
        if len(buffer) < total_length:
            return None
        
        packet_data = bytes(buffer[:total_length])
        return LidarProtocol.parse_packet(packet_data)

# ===================================================================================
# ===== MAIN APPLICATION CLASS ======================================================
# ===================================================================================
class LidarGui(tb.Window):
    def __init__(self):
        super().__init__(themename="yeti", title="LiDAR Configuration GUI", minsize=(1150, 800))

        self.serial_manager = None
        self.incoming_queue = queue.Queue()
        self.outgoing_queue = queue.Queue()

        # GUI Variables
        self.dist_vars = [tk.IntVar(value=0) for _ in range(8)]
        self.vel_min_vars = [tk.IntVar(value=0) for _ in range(8)]
        self.vel_max_vars = [tk.IntVar(value=0) for _ in range(8)]
        self.mode_var = tk.IntVar(value=1)
        self.debug_var = tk.IntVar(value=0)
        self.trigger_rule_vars = [[tk.IntVar(value=0) for _ in range(4)] for _ in range(8)]
        
        self.connected_indicator_on = False
        
        self._configure_styles()
        self._create_widgets()
        self._start_serial_manager()
        self.process_incoming_queue()

        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _configure_styles(self):
        """Configure modern Bootstrap styling"""
        style = tb.Style.get_instance()
        default_font = font.nametofont("TkDefaultFont")
        default_font.configure(family="Segoe UI", size=11)
        
        style.configure('TButton', font=('Segoe UI', 12), padding=(10, 5))
        style.configure('TLabelframe.Label', font=('Segoe UI', 14, 'bold'))
        style.configure('Title.TLabel', font=('Segoe UI', 24, 'bold'))
        style.configure('TLabel', font=('Segoe UI', 11))
        style.configure('TCombobox', font=('Segoe UI', 11))
        style.configure('TCheckbutton', font=('Segoe UI', 11))
        style.configure('TRadiobutton', font=('Segoe UI', 11))
        style.configure('Toolbutton', font=('Segoe UI', 11))
        style.configure('Note.TLabel', font=('Segoe UI', 12, 'italic'))
        style.configure('Fire.TLabel', font=('Segoe UI Emoji', 12), foreground='red')

    def _start_serial_manager(self):
        """Start the Arduino-compatible serial manager"""
        self.serial_manager = SerialManager(self.incoming_queue, self.outgoing_queue)
        self.serial_manager.start()

    def _create_widgets(self):
        """Create the main GUI with beautiful Bootstrap styling"""
        # Title with animated laser effect
        title_frame = tb.Frame(self)
        title_frame.pack(side=tk.TOP, fill=tk.X, pady=(10, 0))
        
        theme_button = tb.Button(
            title_frame, 
            text="🌓 Toggle Theme", 
            command=self.toggle_theme, 
            bootstyle=(SECONDARY, OUTLINE)
        )
        theme_button.pack(side=tk.RIGHT, padx=20, pady=5)
        ToolTip(theme_button, "Switch between light and dark mode")
        
        # Create canvas for laser animation
        self.laser_canvas = tk.Canvas(title_frame, height=60, bg=self.cget('bg'), highlightthickness=0)
        self.laser_canvas.pack(fill=tk.X, padx=20)
        
        # Title text
        title_label = tb.Label(title_frame, text="VectorFlux LiDAR GUI", style='Title.TLabel', anchor='center')
        title_label.pack()
        
        # Initialize laser animation
        self.laser_lines = []
        self.laser_animation_active = True
        self._setup_laser_animation()
        self._animate_lasers()

        # Connection Frame
        conn_frame = tb.Labelframe(self, text="Connection", padding=10)
        conn_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        self.port_combo = tb.Combobox(conn_frame, state="readonly", width=30, font=('Segoe UI', 11))
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        refresh_btn = tb.Button(conn_frame, text="Refresh", command=self.refresh_ports, bootstyle=SECONDARY)
        refresh_btn.pack(side=tk.LEFT, padx=5)
        ToolTip(refresh_btn, "Rescan for available serial ports")
        
        self.connect_btn = tb.Button(conn_frame, text="Connect", command=self.toggle_connection, bootstyle=SUCCESS)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        reset_btn = tb.Button(conn_frame, text="Reset Device", command=self.reset_device, bootstyle=WARNING)
        reset_btn.pack(side=tk.LEFT, padx=5)
        ToolTip(reset_btn, "Soft reset the connected device")
        
        # Status indicators
        status_note_frame = tb.Frame(conn_frame)
        status_note_frame.pack(side=tk.RIGHT, padx=10)
        
        status_line_frame = tb.Frame(status_note_frame)
        status_line_frame.pack()
        
        self.connected_indicator = tb.Label(status_line_frame, text="●", font=("Segoe UI", 18), bootstyle=SECONDARY)
        self.connected_indicator.pack(side=tk.LEFT, padx=(0,5))
        
        self.status_label = tb.Label(status_line_frame, text="DISCONNECTED", bootstyle=(INVERSE, DANGER), 
                                     width=15, anchor='center', font=('Segoe UI', 12, 'bold'))
        self.status_label.pack(side=tk.LEFT)
        
        tb.Label(status_note_frame, text="(Reset Arduino, then connect within 15 seconds!)", 
                 style='Note.TLabel').pack()
        
        self.refresh_ports()

        # Tabbed interface
        notebook = ttk.Notebook(self)
        notebook.pack(expand=True, fill=tk.BOTH, padx=10, pady=10)

        config_tab = tb.Frame(notebook, padding=10)
        device_tab = tb.Frame(notebook, padding=10)
        log_tab = tb.Frame(notebook, padding=10)

        notebook.add(config_tab, text="Configuration")
        notebook.add(device_tab, text="Device")
        notebook.add(log_tab, text="Log")

        self._create_config_tab(config_tab)
        self._create_device_tab(device_tab)
        self._create_log_tab(log_tab)

    def _create_config_tab(self, parent):
        """Create the main configuration tab with thresholds and trigger rules"""
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        
        # Left column - Thresholds and Settings
        config_left = tb.Frame(parent)
        config_left.grid(row=0, column=0, sticky='nsew', padx=(0, 10))

        # Threshold configuration
        threshold_outer_frame = tb.Labelframe(config_left, text="Thresholds", padding=10)
        threshold_outer_frame.pack(fill=tk.X, pady=5, expand=True)
        
        threshold_frame = tb.Frame(threshold_outer_frame)
        threshold_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Limits and instructions
        limits_note = ("Limits:\nDist: 7-1200 cm\nVel: +/- 2-120 mph\n\n"
                       "Direction:\n(-) Towards Sensor\n(+) Away from Sensor\n\n"
                       "Note on Velocity:\nMin must be a larger negative\n"
                       "number than Max (e.g., Min -50, Max -20)")
        tb.Label(threshold_outer_frame, text=limits_note, style='Note.TLabel', 
                 justify=LEFT).pack(side=tk.RIGHT, padx=10, anchor='n')

        # Headers
        tb.Label(threshold_frame, text="Switch").grid(row=0, column=0, padx=5, pady=5)
        tb.Label(threshold_frame, text="Distance (cm)").grid(row=0, column=1, padx=5, pady=5)
        tb.Label(threshold_frame, text="Vel Min (mph)").grid(row=0, column=2, padx=5, pady=5)
        tb.Label(threshold_frame, text="Vel Max (mph)").grid(row=0, column=3, padx=5, pady=5)

        # Threshold entries
        for i in range(8):
            tb.Label(threshold_frame, text=f"{i}").grid(row=i + 1, column=0, padx=5, pady=5)
            
            dist_entry = tb.Entry(threshold_frame, textvariable=self.dist_vars[i], width=8, font=('Segoe UI', 11))
            dist_entry.grid(row=i + 1, column=1, padx=5, pady=5)
            ToolTip(dist_entry, "Distance threshold in centimeters (7-1200)")
            
            vel_min_entry = tb.Entry(threshold_frame, textvariable=self.vel_min_vars[i], width=8, font=('Segoe UI', 11))
            vel_min_entry.grid(row=i + 1, column=2, padx=5, pady=5)
            ToolTip(vel_min_entry, "Minimum velocity in MPH (+/- 2-120)")

            vel_max_entry = tb.Entry(threshold_frame, textvariable=self.vel_max_vars[i], width=8, font=('Segoe UI', 11))
            vel_max_entry.grid(row=i + 1, column=3, padx=5, pady=5)
            ToolTip(vel_max_entry, "Maximum velocity in MPH (+/- 2-120)")

        # Threshold buttons
        threshold_btn_frame = tb.Frame(threshold_frame)
        threshold_btn_frame.grid(row=9, column=0, columnspan=4, pady=10)
        
        self.read_thresh_btn = tb.Button(threshold_btn_frame, text="Read Thresholds", 
                                         command=self.read_thresholds, bootstyle=PRIMARY)
        self.read_thresh_btn.pack(side=tk.LEFT, padx=5)
        
        self.write_thresh_btn = tb.Button(threshold_btn_frame, text="Write Thresholds", 
                                          command=self.write_thresholds, bootstyle=SUCCESS)
        self.write_thresh_btn.pack(side=tk.LEFT, padx=5)

        # General Settings
        settings_outer_frame = tb.Labelframe(config_left, text="General Settings", padding=10)
        settings_outer_frame.pack(fill=tk.X, pady=5)

        settings_frame = tb.Frame(settings_outer_frame)
        settings_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        tb.Checkbutton(settings_frame, text="Enable Debug Output", variable=self.debug_var, 
                       bootstyle="round-toggle").pack(anchor='w', pady=5)
        tb.Radiobutton(settings_frame, text="Distance Only Mode", variable=self.mode_var, 
                       value=1).pack(anchor='w', pady=5)
        tb.Radiobutton(settings_frame, text="Distance + Velocity Mode", variable=self.mode_var, 
                       value=2).pack(anchor='w', pady=5)
        
        settings_btn_frame = tb.Frame(settings_frame)
        settings_btn_frame.pack(pady=5)
        
        self.read_settings_btn = tb.Button(settings_btn_frame, text="Read", 
                                           command=self.read_settings, bootstyle=PRIMARY)
        self.read_settings_btn.pack(side=tk.LEFT, padx=5)
        
        self.write_settings_btn = tb.Button(settings_btn_frame, text="Write", 
                                            command=self.write_settings, bootstyle=SUCCESS)
        self.write_settings_btn.pack(side=tk.LEFT, padx=5)
        
        debug_note = "Note: Debug mode is for testing only and\nsignificantly slows responsiveness."
        tb.Label(settings_outer_frame, text=debug_note, style='Note.TLabel', 
                 justify=LEFT).pack(side=tk.RIGHT, padx=10, anchor='n')

        # Right column - Trigger Rules
        config_right = tb.Frame(parent)
        config_right.grid(row=0, column=1, sticky='nsew')
        
        rules_outer_frame = tb.Labelframe(config_right, text="Trigger Logic Rules", padding=10)
        rules_outer_frame.pack(fill=tk.BOTH, expand=True)

        rules_frame = tb.Frame(rules_outer_frame)
        rules_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        tb.Label(rules_outer_frame, text="Output/Trigger is low-active fire.", 
                 style='Note.TLabel').pack(side=tk.RIGHT, padx=10, anchor='n')
        
        # Trigger rule headers
        tb.Label(rules_frame, text="Switch #").grid(row=0, column=0, padx=10, pady=5)
        tb.Label(rules_frame, text="EXT-TRIG").grid(row=0, column=1, padx=10, pady=5)
        tb.Label(rules_frame, text="EXT-EN").grid(row=0, column=2, padx=10, pady=5)
        tb.Label(rules_frame, text="Lidar-Trig").grid(row=0, column=3, padx=10, pady=5)
        tb.Label(rules_frame, text="Trigger").grid(row=0, column=4, padx=10, pady=5)
        
        # Trigger rule controls
        self.explosion_labels = []
        for i in range(8):
            tb.Label(rules_frame, text=f"{i}").grid(row=i+1, column=0, sticky='e')
            for j in range(4):
                var = self.trigger_rule_vars[i][j]
                cb = tb.Checkbutton(rules_frame, variable=var, bootstyle="round-toggle")
                cb.grid(row=i+1, column=j+1, padx=10, pady=5)
                if j == 3:  # Trigger column
                    var.trace_add("write", lambda *args, index=i: self._update_explosion_indicator(index))

            # Visual trigger indicator
            trigger_indicator = tb.Label(rules_frame, text="", style='Fire.TLabel', width=2)
            trigger_indicator.grid(row=i + 1, column=5, padx=5)
            self.explosion_labels.append(trigger_indicator)

        self._update_all_explosion_indicators()

        # Trigger rule buttons
        rules_btn_frame = tb.Frame(rules_frame)
        rules_btn_frame.grid(row=9, column=0, columnspan=6, pady=10)
        
        self.read_rules_btn = tb.Button(rules_btn_frame, text="Read Rules", 
                                        command=self.read_trigger_rules, bootstyle=PRIMARY)
        self.read_rules_btn.pack(side=tk.LEFT, padx=5)
        
        self.write_rules_btn = tb.Button(rules_btn_frame, text="Write Rules", 
                                         command=self.write_all_trigger_rules, bootstyle=SUCCESS)
        self.write_rules_btn.pack(side=tk.LEFT, padx=5)

    def _create_device_tab(self, parent):
        """Create device control and status tab"""
        parent.columnconfigure(0, weight=1)

        # Device Control
        control_frame = tb.Labelframe(parent, text="Device Control", padding=10)
        control_frame.grid(row=0, column=0, sticky="ew", pady=5)
        
        write_all_btn = tb.Button(control_frame, text="Write All to Device", 
                                  command=self.write_all_to_device, bootstyle=SUCCESS)
        write_all_btn.pack(side=tk.LEFT, padx=5, pady=5)
        ToolTip(write_all_btn, "Write all settings from the GUI to the device's RAM")

        read_all_btn = tb.Button(control_frame, text="Read All from Device", 
                                 command=self.read_all_from_device, bootstyle=PRIMARY)
        read_all_btn.pack(side=tk.LEFT, padx=5, pady=5)
        ToolTip(read_all_btn, "Read all settings from the device into the GUI")

        save_flash_btn = tb.Button(control_frame, text="Save All to Device Flash", 
                                   command=self.save_to_flash, bootstyle=(SUCCESS, OUTLINE))
        save_flash_btn.pack(side=tk.LEFT, padx=10)
        ToolTip(save_flash_btn, "Permanently save the device's current RAM settings to its flash memory")

        factory_reset_btn = tb.Button(control_frame, text="Factory Reset Device", 
                                      command=self.factory_reset, bootstyle=(DANGER, OUTLINE))
        factory_reset_btn.pack(side=tk.LEFT, padx=10)
        ToolTip(factory_reset_btn, "WARNING: Erases all settings on the device and restores defaults")
        
        storage_note = "'Write' -> RAM (Temporary)\n'Save to Flash' -> Permanent"
        tb.Label(control_frame, text=storage_note, style='Note.TLabel', 
                 justify=LEFT).pack(side=tk.LEFT, padx=20)

        # Configuration File
        file_frame = tb.Labelframe(parent, text="Configuration File", padding=10)
        file_frame.grid(row=1, column=0, sticky="ew", pady=5)
        
        save_file_btn = tb.Button(file_frame, text="Save to File", 
                                  command=self.save_config_to_file, bootstyle=INFO)
        save_file_btn.pack(side=tk.LEFT, padx=5)
        ToolTip(save_file_btn, "Save the current GUI settings to a local JSON file")

        load_file_btn = tb.Button(file_frame, text="Load from File", 
                                  command=self.load_config_from_file, bootstyle=INFO)
        load_file_btn.pack(side=tk.LEFT, padx=5)
        ToolTip(load_file_btn, "Load settings from a local JSON file into the GUI")

        # Device Status
        status_frame = tb.Labelframe(parent, text="Device Status", padding=10)
        status_frame.grid(row=2, column=0, sticky="ew", pady=5)
        
        self.status_switch_var = tk.StringVar(value="Switch Code: N/A")
        self.status_frames_var = tk.StringVar(value="Frames Received: N/A")
        
        tb.Label(status_frame, textvariable=self.status_switch_var).pack(anchor='w')
        tb.Label(status_frame, textvariable=self.status_frames_var).pack(anchor='w')

        # Error flags display
        self.error_flags_frame = tb.Frame(status_frame)
        self.error_flags_frame.pack(anchor='w', pady=5)
        tb.Label(self.error_flags_frame, text="Error Flags:").pack(side=tk.LEFT, anchor='n')
        
        self.error_labels = {}
        for code, text in NAK_ERRORS.items():
            if code == 0x00: 
                continue
            frame = tb.Frame(self.error_flags_frame)
            frame.pack(side=tk.LEFT, padx=5)
            label = tb.Label(frame, text=text, bootstyle=SECONDARY)
            label.pack()
            self.error_labels[code] = label
            ToolTip(label, f"Error Code: 0x{code:02X}")

        update_status_btn = tb.Button(status_frame, text="Update Status", 
                                      command=self.get_status, bootstyle=INFO)
        update_status_btn.pack(pady=5)
        ToolTip(update_status_btn, "Request a real-time status update from the device")
        
    def _create_log_tab(self, parent):
        """Create communication log tab"""
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)
        
        log_frame = tb.Labelframe(parent, text="Communication Log", padding=10)
        log_frame.grid(row=0, column=0, sticky="nsew", pady=5)
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        
        self.log_text = tk.Text(log_frame, height=10, state="disabled", wrap="word", 
                                font=("Consolas", 10))
        log_scroll = tb.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.config(yscrollcommand=log_scroll.set)
        
        self.log_text.grid(row=0, column=0, sticky="nsew")
        log_scroll.grid(row=0, column=1, sticky="ns")
        
        # Log control buttons
        log_btn_frame = tb.Frame(log_frame)
        log_btn_frame.grid(row=1, column=0, columnspan=2, pady=8)
        
        clear_log_btn = tb.Button(log_btn_frame, text="Clear Log", 
                                  command=self.clear_log, bootstyle=SECONDARY)
        clear_log_btn.pack(side=tk.LEFT)
        ToolTip(clear_log_btn, "Clear the communication log window")

        save_log_btn = tb.Button(log_btn_frame, text="Save Log", 
                                 command=self.save_log_to_file, bootstyle=INFO)
        save_log_btn.pack(side=tk.LEFT, padx=10)
        ToolTip(save_log_btn, "Save the communication log to a text file")

    # ===== VISUAL FEEDBACK METHODS =====
    def _flash_button(self, button, original_style, flash_style, duration=1000):
        """Flash button to show action feedback"""
        button.config(bootstyle=flash_style)
        self.after(duration, lambda: button.config(bootstyle=original_style))

    def _toggle_connected_indicator(self):
        """Animate connection status indicator"""
        if not self.serial_manager.is_connected():
            self.connected_indicator.config(bootstyle=SECONDARY)
            return

        if self.connected_indicator_on:
            self.connected_indicator.config(bootstyle=SECONDARY)
        else:
            self.connected_indicator.config(bootstyle=SUCCESS)
        self.connected_indicator_on = not self.connected_indicator_on
        self.after(500, self._toggle_connected_indicator)

    def _update_explosion_indicator(self, index):
        """Update trigger visual indicator"""
        var = self.trigger_rule_vars[index][3]
        label = self.explosion_labels[index]
        if var.get() == 0:
            label.config(text="💥")
        else:
            label.config(text="")

    def _update_all_explosion_indicators(self):
        """Update all trigger indicators"""
        for i in range(8):
            self._update_explosion_indicator(i)

    def _setup_laser_animation(self):
        """Setup laser beam graphics"""
        self.laser_canvas.delete("all")
        self.laser_lines = []
        
        # Create 5 laser beam lines
        canvas_width = 400  # Approximate width
        canvas_height = 60
        y_center = canvas_height // 2
        
        # Position lines across the canvas
        x_positions = [50, 100, 150, 200, 250]
        
        for i, x in enumerate(x_positions):
            # Create main laser line
            line = self.laser_canvas.create_line(
                x, y_center - 15, x + 30, y_center - 5, 
                fill="#FFD700", width=3, state='hidden'
            )
            # Create secondary laser line for effect
            line2 = self.laser_canvas.create_line(
                x + 5, y_center, x + 35, y_center + 10,
                fill="#FFA500", width=2, state='hidden'
            )
            # Create third laser line
            line3 = self.laser_canvas.create_line(
                x + 2, y_center + 8, x + 32, y_center + 18,
                fill="#FFD700", width=2, state='hidden'
            )
            
            self.laser_lines.extend([line, line2, line3])
    
    def _animate_lasers(self):
        """Animate laser beams from right to left"""
        if not self.laser_animation_active:
            return
            
        # Hide all lines first
        for line in self.laser_lines:
            self.laser_canvas.itemconfig(line, state='hidden')
        
        # Animate each group of lines in sequence (right to left)
        def show_laser_group(group_index):
            if group_index >= 0:  # Count backwards (right to left)
                start_idx = group_index * 3
                for i in range(3):  # Show 3 lines per group
                    if start_idx + i < len(self.laser_lines):
                        self.laser_canvas.itemconfig(self.laser_lines[start_idx + i], state='normal')
                
                # Schedule next group
                self.after(150, lambda: show_laser_group(group_index - 1))
            else:
                # Animation complete, wait and restart
                self.after(1000, lambda: self.after(100, self._animate_lasers))
        
        # Start animation from rightmost group (index 4, since we have 5 groups 0-4)
        show_laser_group(4)

    # ===== CONNECTION METHODS =====
    def toggle_theme(self):
        """Toggle between light (yeti) and dark (darkly) themes."""
        style = tb.Style.get_instance()
        if style.theme.name == "yeti":
            style.theme_use("darkly")
        else:
            style.theme_use("yeti")
        
        # Re-apply ALL custom styles that were overwritten by the theme change
        style.configure('Title.TLabel', font=('Segoe UI', 24, 'bold'))
        style.configure('TLabelframe.Label', font=('Segoe UI', 14, 'bold'))
        
        # Update the canvas background to match the new theme
        self.laser_canvas.config(bg=self.cget('bg'))

    def refresh_ports(self):
        """Refresh available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])

    def improved_connection_sequence(self):
        """Enhanced connection sequence with better Arduino synchronization"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("Connection Error", "No serial port selected.")
            return False
        
        try:
            # Step 1: Close any existing connection
            if self.serial_manager.is_connected():
                self.serial_manager.disconnect()
                time.sleep(0.5)
            
            # Step 2: Connect with extended Arduino reset detection
            self.log_text_message("Attempting connection to trigger config mode...")
            
            # Open connection
            if not self.serial_manager.connect(port, 115200):
                return False
            
            # Step 3: Send multiple packets to ensure Arduino enters config mode
            for i in range(3):
                self.serial_manager._send_packet('S')  # Status request
                time.sleep(0.1)
            
            # Step 4: Wait longer for Arduino to stabilize
            self.log_text_message("Waiting for Arduino to enter configuration mode...")
            time.sleep(1.0)  # Increased wait time
            
            # Step 5: Send test command and wait for response
            test_timeout = time.time() + 5  # 5 second timeout
            response_received = False
            
            while time.time() < test_timeout and not response_received:
                self.serial_manager._send_packet('S')  # Send status request
                time.sleep(0.2)
                
                # Check for any response
                try:
                    msg_type, data = self.incoming_queue.get(timeout=0.1)
                    if msg_type == "packet":
                        response_received = True
                        self.log_text_message("Arduino responded - configuration mode active!")
                        break
                except:
                    continue
            
            if not response_received:
                self.log_text_message("Warning: No response from Arduino. Try resetting the device first.", "warning")
                # Don't fail connection, sometimes it works anyway
            
            return True
            
        except Exception as e:
            self.log_text_message(f"Connection error: {e}", "error")
            return False

    def toggle_connection(self):
        """Connect/disconnect to/from Arduino with improved synchronization"""
        if self.serial_manager.is_connected():
            self.serial_manager.disconnect()
            self.connect_btn.config(text="Connect", bootstyle=SUCCESS)
            ToolTip(self.connect_btn, "Click to connect to the selected port")
            self.status_label.config(text="DISCONNECTED", bootstyle=(INVERSE, DANGER))
            self.connected_indicator.config(bootstyle=SECONDARY)
        else:
            if self.improved_connection_sequence():
                self.connect_btn.config(text="Disconnect", bootstyle=DANGER)
                ToolTip(self.connect_btn, "Click to disconnect from the device")
                self.status_label.config(text="CONNECTED", bootstyle=(INVERSE, SUCCESS))
                self.log_text_message("Device connected. Reading initial configuration...")
                self._toggle_connected_indicator()
                # Give Arduino more time to stabilize before reading config
                self.after(1000, self.read_all_from_device)
            else:
                messagebox.showerror("Connection Error", 
                    "Failed to connect. Please:\n"
                    "1. Reset the Arduino using the reset button\n"
                    "2. Wait 2-3 seconds\n"
                    "3. Try connecting again within 15 seconds")

    def reset_device(self):
        """Send reset command to Arduino"""
        if messagebox.askyesno("Reset Device", "Are you sure you want to reset the device?"):
            self.outgoing_queue.put(('R', b''))

    def factory_reset(self):
        """Send factory reset command"""
        if messagebox.askyesno("Factory Reset", "WARNING: This will erase all settings on the device. Continue?"):
            self.outgoing_queue.put(('F', b''))
    
    def save_to_flash(self):
        """Save current configuration to Arduino flash"""
        if messagebox.askyesno("Save to Flash", "Save the current configuration to the device's permanent memory?"):
            self.outgoing_queue.put(('W', b''))

    # ===== ARDUINO COMMUNICATION METHODS =====
    def read_all_from_device(self):
        """Read all configuration from Arduino"""
        self.read_thresholds()
        self.read_trigger_rules()
        self.read_settings()

    def write_all_to_device(self):
        """Write all configuration to Arduino"""
        self.write_thresholds()
        self.write_all_trigger_rules()
        self.write_settings()

    def read_thresholds(self):
        """Read distance and velocity thresholds"""
        self.outgoing_queue.put(('D', b''))  # Read distances
        self.outgoing_queue.put(('V', b''))  # Read min velocities
        self.outgoing_queue.put(('v', b''))  # Read max velocities

    def write_thresholds(self):
        """Write distance and velocity thresholds"""
        self.write_all_distances()
        self.write_all_velocities()

    def write_all_distances(self):
        """Write all distance thresholds with validation"""
        all_valid = True
        
        for i in range(8):
            try:
                val = self.dist_vars[i].get()
                if not (LidarProtocol.MIN_DISTANCE_CM <= val <= LidarProtocol.MAX_DISTANCE_CM):
                    self.log_text_message(f"Distance for Switch {i} ({val}) is out of range. Command not sent.", "error")
                    all_valid = False
                    continue
                
                payload = struct.pack('<BH', i, val)
                self.outgoing_queue.put(('d', payload))
            except (ValueError, tk.TclError):
                self.log_text_message(f"Invalid distance value for Switch {i}", "error")
                all_valid = False
        
        if not all_valid:
            messagebox.showerror("Validation Error", "One or more distance values are invalid. See log for details.")

    def write_all_velocities(self):
        """Write all velocity thresholds with validation"""
        all_valid = True
        
        for i in range(8):
            try:
                min_mph = self.vel_min_vars[i].get()
                max_mph = self.vel_max_vars[i].get()

                if not (LidarProtocol.MIN_VELOCITY_MPH <= abs(min_mph) <= LidarProtocol.MAX_VELOCITY_MPH and 
                        LidarProtocol.MIN_VELOCITY_MPH <= abs(max_mph) <= LidarProtocol.MAX_VELOCITY_MPH):
                    self.log_text_message(f"Velocity for Switch {i} is out of range. Command not sent.", "error")
                    all_valid = False
                    continue
                
                min_cms = int(min_mph * MPH_TO_CMS)
                max_cms = int(max_mph * MPH_TO_CMS)

                payload_min = struct.pack('<cBh', b'm', i, min_cms)
                self.outgoing_queue.put(('w', payload_min))
                payload_max = struct.pack('<cBh', b'x', i, max_cms)
                self.outgoing_queue.put(('w', payload_max))
            except (ValueError, tk.TclError):
                self.log_text_message(f"Invalid velocity value for Switch {i}", "error")
                all_valid = False

        if not all_valid:
            messagebox.showerror("Validation Error", "One or more velocity values are invalid. See log for details.")

    def read_trigger_rules(self):
        """Read trigger rules from Arduino"""
        self.outgoing_queue.put(('T', b''))

    def write_all_trigger_rules(self):
        """Write all trigger rules to Arduino"""
        for i in range(8):
            try:
                rules = [self.trigger_rule_vars[i][j].get() for j in range(4)]
                payload = struct.pack('<BBBBB', i, rules[0], rules[1], rules[2], rules[3])
                self.outgoing_queue.put(('t', payload))
            except (ValueError, tk.TclError):
                self.log_text_message(f"Invalid trigger rule value for Switch {i}", "error")

    def read_settings(self):
        """Read general settings"""
        self.outgoing_queue.put(('G', b''))  # Read debug setting
        self.outgoing_queue.put(('M', b''))  # Read mode setting

    def write_settings(self):
        """Write general settings"""
        debug_val = self.debug_var.get()
        self.outgoing_queue.put(('g', struct.pack('<B', debug_val)))
        mode_val = self.mode_var.get()
        self.outgoing_queue.put(('m', struct.pack('<B', mode_val)))

    def get_status(self):
        """Request status from Arduino"""
        self.outgoing_queue.put(('S', b''))

    # ===== FILE OPERATIONS =====
    def save_config_to_file(self):
        """Save current GUI configuration to JSON file"""
        filepath = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
            title="Save Configuration"
        )
        if not filepath:
            return
        
        config_data = {
            "distances": [v.get() for v in self.dist_vars],
            "vel_min": [v.get() for v in self.vel_min_vars],
            "vel_max": [v.get() for v in self.vel_max_vars],
            "mode": self.mode_var.get(),
            "debug": self.debug_var.get(),
            "trigger_rules": [[v.get() for v in row] for row in self.trigger_rule_vars]
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(config_data, f, indent=4)
            self.log_text_message(f"Configuration saved to {filepath}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save file: {e}")

    def load_config_from_file(self):
        """Load configuration from JSON file"""
        filepath = filedialog.askopenfilename(
            filetypes=[("JSON Files", "*.json"), ("All Files", "*.*")],
            title="Load Configuration"
        )
        if not filepath:
            return
            
        try:
            with open(filepath, 'r') as f:
                config_data = json.load(f)
            
            for i in range(8):
                self.dist_vars[i].set(config_data["distances"][i])
                self.vel_min_vars[i].set(config_data["vel_min"][i])
                self.vel_max_vars[i].set(config_data["vel_max"][i])
                for j in range(4):
                    self.trigger_rule_vars[i][j].set(config_data["trigger_rules"][i][j])
            
            self.mode_var.set(config_data["mode"])
            self.debug_var.set(config_data["debug"])
            
            self.log_text_message(f"Configuration loaded from {filepath}")
            self._update_all_explosion_indicators() 
        except Exception as e:
            messagebox.showerror("Load Error", f"Failed to load or parse file: {e}")

    # ===== LOG MANAGEMENT =====
    def clear_log(self):
        """Clear the communication log"""
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state="disabled")

    def save_log_to_file(self):
        """Save communication log to text file"""
        filepath = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")],
            title="Save Communication Log"
        )
        if not filepath:
            return
        
        try:
            with open(filepath, 'w') as f:
                f.write(self.log_text.get(1.0, tk.END))
            self.log_text_message(f"Log saved to {filepath}")
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save log file: {e}")

    # ===== MESSAGE PROCESSING =====
    def process_incoming_queue(self):
        """Process messages from Arduino communication thread"""
        try:
            while True:
                msg_type, data = self.incoming_queue.get_nowait()
                if msg_type == "log":
                    self.log_text_message(data)
                elif msg_type == "packet":
                    self.handle_packet(data)
                elif msg_type == "disconnected":
                    self.connect_btn.config(text="Connect", bootstyle=SUCCESS)
                    self.status_label.config(text="DISCONNECTED", bootstyle=(INVERSE, DANGER))
        except queue.Empty:
            pass 
        
        self.after(100, self.process_incoming_queue)

    def handle_packet(self, packet):
        """Handle received Arduino packets"""
        cmd = chr(packet['command'])
        payload = packet['payload']
        self.log_text_message(f"Received packet: Cmd='{cmd}', Payload={payload.hex().upper()}")

        if cmd == 'D':  # Distance thresholds response
            if len(payload) == 16:
                values = struct.unpack('<' + 'H'*8, payload)
                for i in range(8):
                    self.dist_vars[i].set(values[i])
                self._flash_button(self.read_thresh_btn, PRIMARY, WARNING)
        elif cmd == 'V':  # Velocity min thresholds response
            if len(payload) == 16:
                values = struct.unpack('<' + 'h'*8, payload)
                for i in range(8):
                    self.vel_min_vars[i].set(round(values[i] / MPH_TO_CMS))
                self._flash_button(self.read_thresh_btn, PRIMARY, WARNING)
        elif cmd == 'v':  # Velocity max thresholds response
            if len(payload) == 16:
                values = struct.unpack('<' + 'h'*8, payload)
                for i in range(8):
                    self.vel_max_vars[i].set(round(values[i] / MPH_TO_CMS))
                self._flash_button(self.read_thresh_btn, PRIMARY, WARNING)
        elif cmd == 'T':  # Trigger rules response
            if len(payload) == 32:
                values = struct.unpack('<' + 'B'*32, payload)
                for i in range(8):
                    for j in range(4):
                        self.trigger_rule_vars[i][j].set(values[i*4 + j])
                self._update_all_explosion_indicators()
                self._flash_button(self.read_rules_btn, PRIMARY, WARNING)
        elif cmd == 'G':  # Debug setting response
            if len(payload) >= 1:
                self.debug_var.set(payload[0])
                self._flash_button(self.read_settings_btn, PRIMARY, WARNING)
        elif cmd == 'M':  # Mode setting response
            if len(payload) >= 1:
                self.mode_var.set(payload[0])
                self._flash_button(self.read_settings_btn, PRIMARY, WARNING)
        elif cmd == 'S':  # Status response
            if len(payload) == 9:
                switch, frames, errors = struct.unpack('<BII', payload)
                self.status_switch_var.set(f"Switch Code: {switch}")
                self.status_frames_var.set(f"Frames Received: {frames}")
                # Update error flag display
                for code, label in self.error_labels.items():
                    if errors & (1 << (code-1)):
                        label.config(bootstyle=DANGER)
                    else:
                        label.config(bootstyle=SECONDARY)
        elif packet['command'] == RSP_ACK:  # ACK response
            if len(payload) >= 1:
                original_cmd = chr(payload[0])
                self.log_text_message(f"ACK received for command '{original_cmd}'")
                # Flash appropriate buttons
                if original_cmd in ('d', 'w'):
                    self._flash_button(self.write_thresh_btn, SUCCESS, WARNING)
                elif original_cmd == 't':
                    self._flash_button(self.write_rules_btn, SUCCESS, WARNING)
                elif original_cmd in ('g', 'm'):
                    self._flash_button(self.write_settings_btn, SUCCESS, WARNING)
        elif packet['command'] == RSP_NAK:  # NAK response
            if len(payload) >= 1:
                error_code = payload[0]
                error_msg = NAK_ERRORS.get(error_code, "Unknown Error")
                self.log_text_message(f"NAK received! Error: {error_msg} (Code: 0x{error_code:02X})", "error")

    def log_text_message(self, message, level="info"):
        """Add message to communication log with styling"""
        self.log_text.config(state="normal")
        timestamp = time.strftime("%H:%M:%S")
        full_message = f"{timestamp} - {message}\n"
        
        self.log_text.insert(tk.END, full_message)
        start_index = self.log_text.index(f"end-{len(full_message)}c")
        end_index = self.log_text.index("end-1c")
        
        if level == "error":
            self.log_text.tag_add("error", start_index, end_index)
        elif level == "warning":
            self.log_text.tag_add("warning", start_index, end_index)
        
        self.log_text.see(tk.END)
        self.log_text.config(state="disabled")
        
        # Configure tag styles
        self.log_text.tag_config("error", foreground="#ff6666") 
        self.log_text.tag_config("warning", foreground="#ffcc66")

    def on_closing(self):
        """Handle application shutdown"""
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.serial_manager.stop()
            self.destroy()

if __name__ == "__main__":
    app = LidarGui()
    app.mainloop()