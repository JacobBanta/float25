import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import queue
from datetime import datetime, timedelta
import re


class ArduinoDataLogger:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Data Logger")
        self.root.geometry("800x600")
        
        # Serial connection
        self.serial_conn = None
        self.is_connected = False
        
        # Data storage
        self.data_queue = queue.Queue()
        self.timestamps = []
        self.values = []
        self.start_time = None
        
        # Threading
        self.serial_thread = None
        self.stop_thread = False
        
        self.setup_gui()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Connection frame
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="5")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Port selection
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, padx=(5, 0))
        self.refresh_ports()
        
        # Baud rate
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, sticky=tk.W, padx=(20, 0))
        self.baud_var = tk.StringVar(value="9600")
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=10,
                                  values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=0, column=3, padx=(5, 0))
        
        # Connection buttons
        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=4, padx=(20, 0))
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=(5, 0))
        
        # Control frame
        control_frame = ttk.LabelFrame(main_frame, text="Control", padding="5")
        control_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # START button
        self.start_btn = ttk.Button(control_frame, text="Send START", 
                                   command=self.send_start, state="disabled")
        self.start_btn.grid(row=0, column=0)
        
        # Clear data button
        self.clear_btn = ttk.Button(control_frame, text="Clear Data", command=self.clear_data)
        self.clear_btn.grid(row=0, column=1, padx=(10, 0))
        
        # Status label
        self.status_var = tk.StringVar(value="Disconnected")
        status_label = ttk.Label(control_frame, textvariable=self.status_var)
        status_label.grid(row=0, column=2, padx=(20, 0))
        
        # Info display
        info_frame = ttk.LabelFrame(main_frame, text="Arduino Info", padding="5")
        info_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.info_text = tk.Text(info_frame, height=6, width=70)
        info_scrollbar = ttk.Scrollbar(info_frame, orient="vertical", command=self.info_text.yview)
        self.info_text.configure(yscrollcommand=info_scrollbar.set)
        self.info_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        info_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # Graph frame
        graph_frame = ttk.LabelFrame(main_frame, text="Data Graph", padding="5")
        graph_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Matplotlib setup
        self.fig, self.ax = plt.subplots(figsize=(10, 4))
        self.ax.set_title("Arduino Data Over Time")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(3, weight=1)
        graph_frame.columnconfigure(0, weight=1)
        graph_frame.rowconfigure(0, weight=1)
        info_frame.columnconfigure(0, weight=1)
        
        # Start graph update timer
        self.update_graph()
        
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            if "/dev/ttyUSB0" in port_list:
                self.port_combo.set("/dev/ttyUSB0")
            elif "COM3" in port_list and len(port_list) >= 2:
                self.port_combo.set(port_list[1])
            else:
                self.port_combo.set(port_list[0])
    
    def toggle_connection(self):
        """Connect or disconnect from Arduino"""
        if not self.is_connected:
            self.connect_arduino()
        else:
            self.disconnect_arduino()
    
    def connect_arduino(self):
        """Connect to Arduino"""
        try:
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            if not port:
                messagebox.showerror("Error", "Please select a port")
                return
            
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            
            self.is_connected = True
            self.status_var.set(f"Connected to {port}")
            self.connect_btn.config(text="Disconnect")
            self.start_btn.config(state="normal")
            
            # Start serial listening thread
            self.stop_thread = False
            self.serial_thread = threading.Thread(target=self.listen_serial)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            self.add_info_message(f"Connected to {port} at {baud} baud")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
    
    def disconnect_arduino(self):
        """Disconnect from Arduino"""
        self.is_connected = False
        self.stop_thread = True
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect")
        self.start_btn.config(state="disabled")
        
        if self.serial_thread:
            self.serial_thread.join(timeout=1)
        
        self.add_info_message("Disconnected from Arduino")
    
    def send_start(self):
        """Send START command to Arduino"""
        if self.is_connected and self.serial_conn:
            try:
                self.serial_conn.write(b"START\n")
                self.add_info_message("Sent: START")
            except Exception as e:
                messagebox.showerror("Send Error", f"Failed to send command: {str(e)}")
    
    def listen_serial(self):
        """Listen for serial data from Arduino"""
        while not self.stop_thread and self.is_connected:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.process_arduino_message(line)
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            except Exception as e:
                if self.is_connected:  # Only show error if we're supposed to be connected
                    self.root.after(0, lambda: messagebox.showerror("Serial Error", f"Serial communication error: {str(e)}"))
                break
    
    def process_arduino_message(self, message):
        """Process incoming message from Arduino"""
        if message.startswith("DATA:"):
            # Extract data value
            try:
                data_str = message[5:].strip()  # Remove "DATA:" prefix
                value = float(data_str)
                
                # Add to data queue
                current_time = time.time()
                if self.start_time is None:
                    self.start_time = current_time
                
                relative_time = current_time - self.start_time
                self.data_queue.put((relative_time, value))
                
            except ValueError:
                self.add_info_message(f"Invalid data format: {message}")
        
        elif message.startswith("INFO:"):
            # Print info message
            info_content = message[5:].strip()  # Remove "INFO:" prefix
            self.add_info_message(f"Arduino: {info_content}")
        
        else:
            # Unknown message format
            self.add_info_message(f"Unknown: {message}")
    
    def add_info_message(self, message):
        """Add message to info display"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        # Use after() to ensure GUI updates happen in main thread
        self.root.after(0, lambda: self._update_info_display(formatted_message))
    
    def _update_info_display(self, message):
        """Update info display in main thread"""
        self.info_text.insert(tk.END, message)
        self.info_text.see(tk.END)
    
    def update_graph(self):
        """Update the graph with new data"""
        # Process all queued data
        new_data = False
        while not self.data_queue.empty():
            try:
                timestamp, value = self.data_queue.get_nowait()
                self.timestamps.append(timestamp)
                self.values.append(value)
                new_data = True
            except queue.Empty:
                break
        
        # Update graph if we have new data
        if new_data and self.timestamps:
            self.ax.clear()
            self.ax.plot(self.timestamps, self.values, 'b-', linewidth=2)
            self.ax.set_title("Arduino Data Over Time")
            self.ax.set_xlabel("Time (seconds)")
            self.ax.set_ylabel("Value")
            self.ax.grid(True)
            
            # Auto-scale the view
            if len(self.timestamps) > 1:
                self.ax.set_xlim(0, max(self.timestamps) * 1.1)
            
            self.canvas.draw()
        
        # Schedule next update in 5 seconds
        self.root.after(5000, self.update_graph)
    
    def clear_data(self):
        """Clear all collected data"""
        self.timestamps.clear()
        self.values.clear()
        self.start_time = None
        
        # Clear the queue
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break
        
        # Clear the graph
        self.ax.clear()
        self.ax.set_title("Arduino Data Over Time")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Value")
        self.ax.grid(True)
        self.canvas.draw()
        
        self.add_info_message("Data cleared")
    
    def on_closing(self):
        """Handle application closing"""
        if self.is_connected:
            self.disconnect_arduino()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = ArduinoDataLogger(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
