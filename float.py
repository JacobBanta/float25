#!/usr/bin/env python3
import RPi.GPIO as GPIO
import serial
import threading
import time
import json
import logging
from enum import Enum
from dataclasses import dataclass
from typing import Dict, Any

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class DepthState(Enum):
    IDLE = "idle"
    DESCENDING = "descending"
    AT_DEPTH = "at_depth"
    OSCILLATING_DOWN = "oscillating_down"
    OSCILLATING_UP = "oscillating_up"

@dataclass
class SystemParameters:
    depth_trigger: float = 1500.0  # mbar
    max_depth: float = 2000.0      # mbar
    stability_threshold: float = 50.0  # mbar
    stability_duration: int = 10   # seconds

class DepthController:
    def __init__(self):
        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(5, GPIO.OUT)
        GPIO.setup(6, GPIO.OUT)
        
        # Initialize GPIO pins HIGH
        GPIO.output(5, GPIO.HIGH)
        GPIO.output(6, GPIO.HIGH)
        
        # UART setup
        self.uart = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        
        # System state
        self.state = DepthState.IDLE
        self.parameters = SystemParameters()
        self.current_depth = 0.0
        self.sequence_number = 0
        self.running = True
        self.depth_control_active = False
        
        # Stability tracking
        self.stable_start_time = None
        self.at_depth_start_time = None
        
        # Thread locks
        self.state_lock = threading.Lock()
        self.param_lock = threading.Lock()
        
    def start(self):
        """Start the system threads"""
        # Start communication thread
        comm_thread = threading.Thread(target=self._communication_handler, daemon=True)
        comm_thread.start()
        
        # Start depth control thread
        depth_thread = threading.Thread(target=self._depth_control_loop, daemon=True)
        depth_thread.start()
        
        logger.info("System started - GPIO 5 and 6 HIGH, waiting for START command")
        
    def _communication_handler(self):
        """Handle UART communication with A2"""
        while self.running:
            try:
                if self.uart.in_waiting > 0:
                    line = self.uart.readline().decode('utf-8').strip()
                    if line:
                        self._process_command(line)
            except Exception as e:
                logger.error(f"Communication error: {e}")
            time.sleep(0.1)
    
    def _process_command(self, command: str):
        """Process incoming commands from A2"""
        try:
            parts = command.split()
            cmd = parts[0].upper()
            
            if cmd == "START":
                with self.state_lock:
                    self.depth_control_active = True
                    self.state = DepthState.DESCENDING
                logger.info("START command received - beginning depth control")
                
            elif cmd == "SET" and len(parts) >= 3:
                param_name = parts[1]
                param_value = float(parts[2])
                self._update_parameter(param_name, param_value)
                
            elif cmd in ["0", "1"] and len(parts) >= 2:
                pin_num = int(parts[1])
                if pin_num in [5, 6]:
                    value = GPIO.HIGH if cmd == "1" else GPIO.LOW
                    GPIO.output(pin_num, value)
                    logger.info(f"Manual GPIO {pin_num} set to {cmd}")
                    
        except Exception as e:
            logger.error(f"Command processing error: {e}")
    
    def _update_parameter(self, param_name: str, value: float):
        """Update system parameters"""
        with self.param_lock:
            if hasattr(self.parameters, param_name):
                setattr(self.parameters, param_name, value)
                logger.info(f"Parameter {param_name} updated to {value}")
            else:
                logger.warning(f"Unknown parameter: {param_name}")
    
    def _simulate_depth_sensor(self) -> float:
        """Simulate depth sensor reading"""
        # Simple simulation - replace with actual sensor code
        import random
        base_depth = 1000 + (time.time() % 100) * 10
        noise = random.uniform(-20, 20)
        return base_depth + noise
    
    def _depth_control_loop(self):
        """Main depth control state machine"""
        while self.running:
            if not self.depth_control_active:
                time.sleep(1)
                continue
                
            # Read depth sensor every 5 seconds
            self.current_depth = self._simulate_depth_sensor()
            self._send_pressure_reading()
            
            with self.state_lock:
                self._update_state_machine()
            
            time.sleep(5)
    
    def _update_state_machine(self):
        """Update the depth control state machine"""
        if self.state == DepthState.DESCENDING:
            GPIO.output(5, GPIO.HIGH)
            GPIO.output(6, GPIO.LOW)
            
            if self.current_depth >= self.parameters.depth_trigger:
                self.state = DepthState.AT_DEPTH
                self.at_depth_start_time = time.time()
                logger.info("Reached depth trigger - both pins HIGH")
                
        elif self.state == DepthState.AT_DEPTH:
            GPIO.output(5, GPIO.HIGH)
            GPIO.output(6, GPIO.HIGH)
            
            if self.current_depth >= self.parameters.max_depth:
                if self.at_depth_start_time is None:
                    self.at_depth_start_time = time.time()
                elif time.time() - self.at_depth_start_time >= 45:
                    self.state = DepthState.OSCILLATING_DOWN
                    self.at_depth_start_time = None
                    logger.info("Max depth timeout - starting oscillation")
                    
        elif self.state == DepthState.OSCILLATING_DOWN:
            GPIO.output(5, GPIO.HIGH)
            GPIO.output(6, GPIO.LOW)
            
            if self._is_depth_stable():
                self.state = DepthState.OSCILLATING_UP
                logger.info("Depth stable - switching to oscillating up")
                
        elif self.state == DepthState.OSCILLATING_UP:
            GPIO.output(5, GPIO.LOW)
            GPIO.output(6, GPIO.HIGH)
            
            if not self._is_depth_stable():
                self.state = DepthState.OSCILLATING_DOWN
                self.stable_start_time = None
                logger.info("Depth unstable - switching to oscillating down")
    
    def _is_depth_stable(self) -> bool:
        """Check if depth is stable within threshold"""
        # Simplified stability check - in real implementation, 
        # you'd want to track depth history
        if abs(self.current_depth - self.parameters.max_depth) <= self.parameters.stability_threshold:
            if self.stable_start_time is None:
                self.stable_start_time = time.time()
            return (time.time() - self.stable_start_time) >= self.parameters.stability_duration
        else:
            self.stable_start_time = None
            return False
    
    def _send_pressure_reading(self):
        """Send pressure reading to A2"""
        try:
            self.sequence_number += 1
            message = f"PRESSURE:{self.sequence_number}:{self.current_depth:.2f}\n"
            self.uart.write(message.encode('utf-8'))
            logger.debug(f"Sent pressure reading: {self.current_depth:.2f} mbar")
        except Exception as e:
            logger.error(f"Failed to send pressure reading: {e}")
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        GPIO.cleanup()
        self.uart.close()

if __name__ == "__main__":
    controller = DepthController()
    try:
        controller.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        controller.cleanup()
