#!/usr/bin/env python3
"""
Simple ESP32 Serial Monitor
Reads live ax,ay,az,gx,gy,gz readings from /dev/ttyACM0
"""
import serial
import time
import sys
import glob

def find_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    return ports[0] if ports else '/dev/ttyACM0'

port = sys.argv[1] if len(sys.argv) > 1 else find_port()
baud = 115200

print(f"Connecting to ESP32 on {port} @ {baud} baud...")
try:
    s = serial.Serial(port, baud, timeout=1)
    # Release ESP32 auto-reset
    s.dtr = False
    s.rts = False
    time.sleep(0.1)
    s.dtr = True
    time.sleep(0.4)
    s.reset_input_buffer()
    print("Connected! Live data stream (Press Ctrl+C to stop):\n")
    print(f"{'Ax (m/s²)':>10} {'Ay (m/s²)':>10} {'Az (m/s²)':>10} {'Gx (rad/s)':>12} {'Gy (rad/s)':>12} {'Gz (rad/s)':>12}")
    print("-" * 72)

    while True:
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) >= 6:
            try:
                vals = [float(x) for x in parts[:6]]
                print(f"{vals[0]:10.4f} {vals[1]:10.4f} {vals[2]:10.4f} {vals[3]:12.5f} {vals[4]:12.5f} {vals[5]:12.5f}")
            except ValueError:
                print(f"[raw] {line}")
        else:
            print(f"[raw] {line}")

except KeyboardInterrupt:
    print("\nDisconnected.")
except Exception as e:
    print(f"\nError: {e}")
finally:
    if 's' in locals() and s.is_open:
        s.close()
