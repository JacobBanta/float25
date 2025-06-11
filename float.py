#!/usr/bin/env python3
import serial
import time
import threading
from datetime import datetime
from enum import Enum
from collections import deque
import RPi.GPIO as GPIO

class DepthControlState(Enum):
    IDLE = "idle"
    DESCENDING = "descending"
    AT_TRIGGER_DEPTH = "at_trigger_depth"
    WAITING_45_SEC = "waiting_45_sec"
    OSCILLATING_DOWN = "oscillating_down"
    OSCILLATING_UP = "oscillating_up"

class PressureSensorSystem:
    def __init__(self, buffer_size=100):
        # GPIO setup for control pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        
        # Start with both pins HIGH
        GPIO.output(5, GPIO.HIGH)
        GPIO.output(6, GPIO.HIGH)
        
        # Hardware UART setup - Pi GPIO 14 (TX) and 15 (RX)
        uart_devices = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']
        self.uart = None
        
        for device in uart_devices:
            try:
                self.uart = serial.Serial(device, 9600, timeout=0.5)
                self.uart.reset_input_buffer()
                self.uart.reset_output_buffer()
                print(f"Connected to UART on {device}")
                break
            except Exception as e:
                print(f"Failed to connect to {device}: {e}")
        
        if self.uart is None:
            print("ERROR: Could not connect to any UART device")
            print("Make sure UART is enabled: sudo raspi-config -> Interface Options -> Serial")
            exit(1)
        
        # Data buffer
        self.buffer_size = buffer_size
        self.data_buffer = {}
        self.sequence_counter = 0
        self.acknowledged_sequences = set()
        self.buffer_lock = threading.Lock()
        
        # Depth control parameters
        self.params = {
            'depth_trigger': 1050.0,
            'max_depth': 1100.0,
            'stability_threshold': 2.0,
            'stability_duration': 10.0
        }
        
        # State machine variables
        self.state = DepthControlState.IDLE
        self.pressure_history = deque(maxlen=20)
        self.state_start_time = time.time()
        self.last_pressure = 0
        
        print("Pi system initialized with hardware UART")
        print("Wiring: Pi GPIO 14 (Pin 8) -> A2 Pin 1 (RX)")
        print("        Pi GPIO 15 (Pin 10) -> A2 Pin 0 (TX)")
        print("        Pi Ground -> A2 Ground")
        print(f"GPIO pins 5,6 set HIGH (idle state)")
    
    def read_bar30_pressure(self):
        """Simulate BAR30 pressure sensor reading"""
        import random
        base_pressure = 1013.25
        if hasattr(self, 'simulated_depth'):
            self.simulated_depth += random.uniform(-1, 2)
        else:
            self.simulated_depth = random.uniform(0, 10)
        
        return base_pressure + self.simulated_depth
    
    def update_parameters(self, param_name, value):
        """Update system parameters"""
        if param_name in self.params:
            old_value = self.params[param_name]
            self.params[param_name] = value
            print(f"Updated {param_name}: {old_value} -> {value}")
        else:
            print(f"Unknown parameter: {param_name}")
    
    def set_gpio_state(self, pin5_state, pin6_state, description=""):
        """Set GPIO pins and log the change"""
        GPIO.output(5, pin5_state)
        GPIO.output(6, pin6_state)
        pin5_str = "HIGH" if pin5_state else "LOW"
        pin6_str = "HIGH" if pin6_state else "LOW"
        print(f"GPIO: Pin 5={pin5_str}, Pin 6={pin6_str} {description}")
    
    def is_pressure_stable(self):
        """Check if pressure has been stable"""
        if len(self.pressure_history) < 3:
            return False
        
        recent_pressures = list(self.pressure_history)[-int(self.params['stability_duration']):]
        if len(recent_pressures) < self.params['stability_duration']:
            return False
        
        max_pressure = max(recent_pressures)
        min_pressure = min(recent_pressures)
        
        return (max_pressure - min_pressure) <= self.params['stability_threshold']
    
    def update_depth_control_state(self, current_pressure):
        """State machine for depth control"""
        current_time = time.time()
        time_in_state = current_time - self.state_start_time
        
        self.pressure_history.append(current_pressure)
        
        if self.state == DepthControlState.IDLE:
            pass
        elif self.state == DepthControlState.DESCENDING:
            if current_pressure >= self.params['depth_trigger']:
                self.state = DepthControlState.AT_TRIGGER_DEPTH
                self.state_start_time = current_time
                self.set_gpio_state(GPIO.HIGH, GPIO.HIGH, "(reached trigger depth)")
        elif self.state == DepthControlState.AT_TRIGGER_DEPTH:
            if current_pressure >= self.params['max_depth']:
                self.state = DepthControlState.WAITING_45_SEC
                self.state_start_time = current_time
                print("Reached max depth - starting 45 second timer")
        elif self.state == DepthControlState.WAITING_45_SEC:
            if time_in_state >= 45:
                self.state = DepthControlState.OSCILLATING_DOWN
                self.state_start_time = current_time
                self.set_gpio_state(GPIO.HIGH, GPIO.LOW, "(starting oscillation)")
        elif self.state == DepthControlState.OSCILLATING_DOWN:
            if self.is_pressure_stable():
                self.state = DepthControlState.OSCILLATING_UP
                self.state_start_time = current_time
                self.set_gpio_state(GPIO.LOW, GPIO.HIGH, "(reversing)")
        elif self.state == DepthControlState.OSCILLATING_UP:
            if self.is_pressure_stable():
                self.state = DepthControlState.WAITING_45_SEC
                self.state_start_time = current_time
                self.set_gpio_state(GPIO.HIGH, GPIO.HIGH, "(stable)")
        
        self.last_pressure = current_pressure
    
    def start_depth_control(self):
        """Begin the depth control sequence"""
        print("Starting depth control sequence!")
        self.state = DepthControlState.DESCENDING
        self.state_start_time = time.time()
        self.pressure_history.clear()
        self.set_gpio_state(GPIO.HIGH, GPIO.LOW, "(starting descent)")
    
    def safe_uart_write(self, message):
        """Safely write to UART"""
        try:
            if not message.endswith('\n'):
                message += '\n'
            
            self.uart.write(message.encode('ascii'))
            self.uart.flush()
            return True
        except Exception as e:
            print(f"UART write error: {e}")
            return False
    
    def pressure_reader_thread(self):
        """Thread to read pressure sensor every 5 seconds"""
        while True:
            try:
                pressure = self.read_bar30_pressure()
                self.update_depth_control_state(pressure)
                
                reading = {
                    'pressure': pressure,
                    'timestamp': datetime.now().isoformat(),
                    'sequence': self.sequence_counter,
                    'state': self.state.value
                }
                
                with self.buffer_lock:
                    self.data_buffer[self.sequence_counter] = reading
                    
                    if len(self.data_buffer) > self.buffer_size:
                        oldest_sequences = sorted(self.data_buffer.keys())
                        for seq in oldest_sequences:
                            if seq in self.acknowledged_sequences:
                                del self.data_buffer[seq]
                                self.acknowledged_sequences.discard(seq)
                                break
                
                # Send to A2
                message = f"PRESSURE:{pressure:.2f}:{self.sequence_counter}"
                if self.safe_uart_write(message):
                    print(f"Sent: P={pressure:.2f}, State={self.state.value}, Seq={self.sequence_counter}")
                
                self.sequence_counter += 1
                
            except Exception as e:
                print(f"Error in pressure reading: {e}")
            
            time.sleep(5)
    
    def uart_listener_thread(self):
        """Thread to listen for UART commands"""
        buffer = b''
        
        while True:
            try:
                if self.uart.in_waiting > 0:
                    new_data = self.uart.read(self.uart.in_waiting)
                    buffer += new_data
                    
                    while b'\n' in buffer:
                        line_bytes, buffer = buffer.split(b'\n', 1)
                        
                        try:
                            line = line_bytes.decode('ascii').strip()
                            if line:
                                print(f"Received from A2: '{line}'")
                                self.process_command(line)
                        except UnicodeDecodeError:
                            print(f"Invalid bytes: {[hex(b) for b in line_bytes]}")
                        
            except Exception as e:
                print(f"UART listener error: {e}")
                buffer = b''
            
            time.sleep(0.1)
    
    def process_command(self, line):
        """Process received command"""
        try:
            if line.startswith("CMD:") and len(line) >= 5:
                command = line[4]
                self.handle_gpio_command(command)
            elif line == "START":
                self.start_depth_control()
            elif line.startswith("PARAM:"):
                parts = line.split(':')
                if len(parts) == 3:
                    param_name = parts[1]
                    param_value = float(parts[2])
                    self.update_parameters(param_name, param_value)
            elif line.startswith("ACK:"):
                sequence = int(line[4:])
                self.handle_acknowledgment(sequence)
            elif line.startswith("REQ:"):
                sequence = int(line[4:])
                self.handle_retransmission_request(sequence)
            elif line == "A2_READY" or line.startswith("A2_"):
                print(f"A2 status: {line}")
        except Exception as e:
            print(f"Error processing command '{line}': {e}")
    
    def handle_gpio_command(self, command):
        """Handle GPIO commands"""
        if self.state != DepthControlState.IDLE:
            print(f"GPIO command '{command}' ignored - system in {self.state.value} state")
            return
        
        if command == '0':
            self.set_gpio_state(GPIO.LOW, GPIO.LOW, "(manual)")
        elif command == '1':
            self.set_gpio_state(GPIO.HIGH, GPIO.HIGH, "(manual)")
    
    def handle_acknowledgment(self, sequence):
        """Handle acknowledgment"""
        with self.buffer_lock:
            self.acknowledged_sequences.add(sequence)
    
    def handle_retransmission_request(self, sequence):
        """Handle retransmission request"""
        with self.buffer_lock:
            if sequence in self.data_buffer:
                reading = self.data_buffer[sequence]
                message = f"PRESSURE:{reading['pressure']:.2f}:{sequence}"
                self.safe_uart_write(message)
    
    def run(self):
        """Start system"""
        pressure_thread = threading.Thread(target=self.pressure_reader_thread, daemon=True)
        pressure_thread.start()
        
        uart_thread = threading.Thread(target=self.uart_listener_thread, daemon=True)
        uart_thread.start()
        
        print("System running with hardware UART...")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            GPIO.cleanup()
            self.uart.close()

if __name__ == "__main__":
    system = PressureSensorSystem(buffer_size=200)
    system.run()
