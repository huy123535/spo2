#!/usr/bin/env python3
"""
Real-time Heart Rate Signal Plotting Tool
Requirements:
    - pyserial
    - matplotlib
    - numpy
"""
import serial
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import time
import serial.tools.list_ports

def list_com_ports():
    """List all available COM ports"""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No COM ports found!")
        return None
    
    print("\nAvailable COM ports:")
    for i, port in enumerate(ports):
        print(f"{i + 1}: {port.device} - {port.description}")
    
    return ports

def select_com_port():
    """Let user select COM port"""
    ports = list_com_ports()
    if not ports:
        return None
    
    if len(ports) == 1:
        print(f"\nAutomatically selecting the only available port: {ports[0].device}")
        return ports[0].device
    
    while True:
        try:
            choice = input("\nSelect COM port number (or press Enter for COM5): ")
            if not choice:  # If user just pressed Enter
                return 'COM5'
            
            choice = int(choice)
            if 1 <= choice <= len(ports):
                return ports[choice - 1].device
        except ValueError:
            print("Please enter a valid number")

# Configure plot
WINDOW_SIZE = 200  # Number of samples to display
data_buffer = deque(maxlen=WINDOW_SIZE)

def main():
    # Set up the plot
    plt.ion()  # Enable interactive mode
    fig, ax = plt.subplots(figsize=(10, 6))
    line, = ax.plot(np.zeros(WINDOW_SIZE))
    
    # Set up plot parameters
    ax.set_title('Real-time Heart Rate Signal')
    ax.set_xlabel('Samples')
    ax.set_ylabel('Signal Value')
    ax.grid(True)
    
    # Select COM port
    port = select_com_port()
    if not port:
        print("No COM port selected. Exiting...")
        return

    try:
        # Open serial port
        ser = serial.Serial(port, 115200)
        print(f"\nConnected to {port}")
        
        while True:
            try:
                # Read data from serial
                if ser.in_waiting > 0:
                    try:
                        line_data = ser.readline().decode('utf-8').strip()
                        if not line_data:  # Skip empty lines
                            continue
                            
                        value = float(line_data)
                        data_buffer.append(value)
                        
                        # Update plot
                        line.set_ydata(list(data_buffer))
                        if len(data_buffer) < WINDOW_SIZE:
                            line.set_xdata(range(len(data_buffer)))
                        
                        # Adjust y axis limits
                        ax.relim()
                        ax.autoscale_view()
                        
                        # Draw plot
                        fig.canvas.draw()
                        fig.canvas.flush_events()
                        
                    except ValueError as e:
                        print(f"Invalid data received: '{line_data}'")
                        print(f"Error details: {str(e)}")
                        time.sleep(0.1)  # Add small delay to prevent console flooding
                    except UnicodeDecodeError as e:
                        print("Error decoding serial data")
                        time.sleep(0.1)
                        
            except KeyboardInterrupt:
                print("\nStopping...")
                break

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")

    finally:
        # Clean up
        if 'ser' in locals():
            ser.close()
        plt.ioff()
        plt.close()

if __name__ == '__main__':
    main()
