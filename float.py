#!/usr/bin/env python3
"""
Bar30 Depth Sensor Reader with Arduino Communication and GPIO Control
Reads from Bar30 sensor, communicates with Arduino, and controls GPIO pins
"""

import time
import ms5837
import serial
import sys
import RPi.GPIO as GPIO
from threading import Thread
import queue
import smbus

# Configuration
ARDUINO_PORT = "/dev/ttyUSB0"
ARDUINO_BAUD = 9600
DEPTH_SEND_INTERVAL = 5.0  # Send depth every 5 seconds
SENSOR_READ_INTERVAL = 0.1  # Read sensor every 100ms

# GPIO Pin Configuration
GPIO_PIN_5 = 5
GPIO_PIN_6 = 6

class DepthSensorController:
    def __init__(self):
        self.sensor = None
        self.arduino = None
        self.current_depth = 0.0
        self.last_depth_send_time = 0
        self.gpio_state = {"pin5": True, "pin6": True}
        self.state = "DESCEND"
        self.time = time.time()
        
        # Setup GPIO
        self.setup_gpio()
        
        # Setup sensor
        self.setup_sensor()
        
        # Setup Arduino communication
        self.setup_arduino()
    
    def setup_gpio(self):
        """Initialize GPIO pins"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup output pins
        GPIO.setup(GPIO_PIN_5, GPIO.OUT)
        GPIO.setup(GPIO_PIN_6, GPIO.OUT)
        
        # Initialize pins to HIGH
        GPIO.output(GPIO_PIN_5, GPIO.HIGH)
        GPIO.output(GPIO_PIN_6, GPIO.HIGH)
        
        print("GPIO pins initialized")
    
    def setup_sensor(self):
        """Initialize the Bar30 depth sensor"""
        self.sensor = ms5837.MS5837_30BA()
        
        if not self.sensor.init():
            print("Failed to initialize MS5837 sensor!")
            print("Check I2C wiring:")
            print("  VCC -> 3.3V, GND -> Ground")
            print("  SDA -> GPIO 2, SCL -> GPIO 3")
            sys.exit(1)
        
        # Set fluid density (1025 for saltwater, 997 for freshwater)
        self.sensor.setFluidDensity(1025)  # Saltwater
        print("Bar30 sensor initialized successfully!")
    
    def setup_arduino(self):
        """Initialize Arduino serial communication"""
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(2)  # Allow Arduino to reset
            print(f"Arduino connected on {ARDUINO_PORT}")
            
            # Wait for START signal from Arduino
            print("Waiting for START signal from Arduino...")
            while True:
                if self.arduino.in_waiting > 0:
                    message = self.arduino.readline().decode().strip()
                    print(f"Received from Arduino: {message}")
                    if message == "START":
                        print("START signal received! Beginning operation...")
                        break
                time.sleep(0.1)
                
        except serial.SerialException as e:
            print(f"Failed to connect to Arduino: {e}")
            print("Make sure Arduino is connected to /dev/ttyUSB0")
            sys.exit(1)
    
    def read_sensor(self):
        """Read sensor data and update current depth"""
        if self.sensor.read():
            self.current_depth = self.sensor.depth()
            pressure = self.sensor.pressure()
            temperature = self.sensor.temperature()
            
            # Display current readings
            print(f"Depth: {self.current_depth:.2f}m | "
                  f"Temp: {temperature:.2f}°C | "
                  f"Pressure: {pressure:.2f}mbar")
            
            return True
        else:
            print("Failed to read sensor data!")
            return False
    
    def send_depth_to_arduino(self):
        """Send depth reading to Arduino"""
        current_time = time.time()
        
        if current_time - self.last_depth_send_time >= DEPTH_SEND_INTERVAL:
            message = f"DEPTH:{self.current_depth:.2f}\n"
            try:
                self.arduino.write(message.encode())
                print(f"Sent to Arduino: {message.strip()}")
                self.last_depth_send_time = current_time
            except Exception as e:
                print(f"Error sending to Arduino: {e}")
    
    def control_gpio_pins(self):
        # Initialize state variables if they don't exist
        if not hasattr(self, 'mission_state'):
            self.mission_state = 'DESCENDING_TO_PAUSE'
            self.prev_depth = self.current_depth
            self.pause_timer = 0
            self.max_depth = 0
            self.depth_stable_count = 0
            self.velocity_history = []
        
        # Calculate velocity (positive = descending, negative = ascending)
        velocity = (self.current_depth - self.prev_depth) / 0.1  # m/s
        self.velocity_history.append(velocity)
        if len(self.velocity_history) > 10:  # Keep last 1 second of data
            self.velocity_history.pop(0)
        
        avg_velocity = sum(self.velocity_history) / len(self.velocity_history)
        self.prev_depth = self.current_depth
        
        # Update max depth for bottom detection
        if self.current_depth > self.max_depth:
            self.max_depth = self.current_depth
            self.depth_stable_count = 0
        else:
            self.depth_stable_count += 1
        
        # State machine logic
        if self.mission_state == 'DESCENDING_TO_PAUSE':
            target_depth = 2.5
            depth_error = target_depth - self.current_depth
            
            if abs(depth_error) <= 0.5:  # Within pause zone
                self.mission_state = 'PAUSING'
                self.pause_timer = 0
            elif depth_error > 0:  # Need to descend more
                if abs(avg_velocity) > 0.3 or depth_error < 0.5:  # Brake if too fast or close
                    GPIO.output(5, GPIO.HIGH)
                    GPIO.output(6, GPIO.HIGH)  # Neutral
                else:
                    GPIO.output(5, GPIO.HIGH)
                    GPIO.output(6, GPIO.LOW)   # Increase density
            else:  # Overshot, need to ascend slightly
                GPIO.output(5, GPIO.LOW)
                GPIO.output(6, GPIO.HIGH)      # Decrease density
        
        elif self.mission_state == 'PAUSING':
            self.pause_timer += 100  # ms
            depth_error = 2.5 - self.current_depth
            
            if self.pause_timer >= 60000:  # 60 seconds
                self.mission_state = 'DESCENDING_TO_BOTTOM'
            elif abs(depth_error) > 0.3:  # Tighter control band - start correcting at 0.3m
                if depth_error > 0:  # Drifting too shallow
                    GPIO.output(5, GPIO.HIGH)
                    GPIO.output(6, GPIO.LOW)   # Increase density
                else:  # Drifting too deep
                    GPIO.output(5, GPIO.LOW)
                    GPIO.output(6, GPIO.HIGH)  # Decrease density
            elif abs(depth_error) > 0.1 and abs(avg_velocity) > 0.05:  # Gentle correction for drift
                if depth_error > 0 and avg_velocity < 0:  # Rising too fast
                    GPIO.output(5, GPIO.HIGH)
                    GPIO.output(6, GPIO.LOW)   # Gentle increase density
                elif depth_error < 0 and avg_velocity > 0:  # Sinking too fast
                    GPIO.output(5, GPIO.LOW)
                    GPIO.output(6, GPIO.HIGH)  # Gentle decrease density
                else:
                    GPIO.output(5, GPIO.HIGH)
                    GPIO.output(6, GPIO.HIGH)  # Neutral
            else:  # Stay neutral in pause zone
                GPIO.output(5, GPIO.HIGH)
                GPIO.output(6, GPIO.HIGH)      # Neutral
        
        elif self.mission_state == 'DESCENDING_TO_BOTTOM':
            # Bottom detection: depth hasn't increased for 3 seconds
            if self.depth_stable_count >= 30:  # 3 seconds at 100ms intervals
                self.mission_state = 'ASCENDING'
            elif abs(avg_velocity) > 0.4:  # Control descent speed
                GPIO.output(5, GPIO.HIGH)
                GPIO.output(6, GPIO.HIGH)      # Neutral/brake
            else:
                GPIO.output(5, GPIO.HIGH)
                GPIO.output(6, GPIO.LOW)       # Increase density
        
        elif self.mission_state == 'ASCENDING':
            # Surface detection: depth very small or stopped ascending
            if self.current_depth <= 0.2:  # Near surface
                self.mission_state = 'AT_SURFACE'
                GPIO.output(5, GPIO.HIGH)
                GPIO.output(6, GPIO.HIGH)      # Neutral
            elif abs(avg_velocity) > 0.4:  # Control ascent speed
                GPIO.output(5, GPIO.HIGH)
                GPIO.output(6, GPIO.HIGH)      # Neutral/brake
            else:
                GPIO.output(5, GPIO.LOW)
                GPIO.output(6, GPIO.HIGH)      # Decrease density
        
        elif self.mission_state == 'AT_SURFACE':
            # Mission complete - maintain neutral buoyancy
            GPIO.output(5, GPIO.HIGH)
            GPIO.output(6, GPIO.HIGH)          # Neutral


        
    
    def listen_to_arduino(self):
        """Listen for messages from Arduino (runs in separate thread)"""
        while True:
            try:
                if self.arduino.in_waiting > 0:
                    message = self.arduino.readline().decode().strip()
                    if message:
                        print(f"Arduino says: {message}")
            except Exception as e:
                print(f"Error reading from Arduino: {e}")
            time.sleep(0.1)
    
    def run(self):
        """Main program loop"""
        print("Starting depth sensor controller...")
        print(f"Sending depth to Arduino every {DEPTH_SEND_INTERVAL} seconds")
        print("Press Ctrl+C to exit")
        print("-" * 60)
        
        # Start Arduino listener in separate thread
        arduino_thread = Thread(target=self.listen_to_arduino, daemon=True)
        arduino_thread.start()

        GPIO.output(GPIO_PIN_5, GPIO.LOW)
        self.gpio_state["pin5"] = False
        
        try:
            while True:
                # Read sensor data
                if self.read_sensor():
                    # Control GPIO pins based on depth
                    self.control_gpio_pins()
                    
                    # Send depth to Arduino (if interval elapsed)
                    self.send_depth_to_arduino()
                
                time.sleep(SENSOR_READ_INTERVAL)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        # Turn off all GPIO pins
        GPIO.output(GPIO_PIN_5, GPIO.LOW)
        GPIO.output(GPIO_PIN_6, GPIO.LOW)
        GPIO.cleanup()
        
        # Close Arduino connection
        if self.arduino:
            self.arduino.close()
        
        print("Cleanup completed. Goodbye!")

def reset_bar30():
    bus = smbus.SMBus(1)
    for attempt in range(5):  # Try up to 5 times
        try:
            bus.write_byte(0x76, 0x1E)  # Reset command
            time.sleep(0.1)
            
            # Verify reset worked by reading PROM
            data = bus.read_word_data(0x76, 0xA0)
            if data != 0x0000 and data != 0xFFFF:
                return True
        except:
            time.sleep(0.2)  # Wait longer between attempts
    return False

def main():
    reset_bar30()
    time.sleep(5)

    controller = DepthSensorController()
    controller.run()

if __name__ == "__main__":
    main()
