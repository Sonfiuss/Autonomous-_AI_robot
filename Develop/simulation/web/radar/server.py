"""
Radar Visualization Server
Flask + Socket.IO bridge for Arduino serial data

Usage:
    python server.py [--port COM5] [--test]
    
Then open http://localhost:5000 in browser
"""

import json
import threading
import time
import argparse
import math
from flask import Flask, send_from_directory, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import serial
import serial.tools.list_ports

app = Flask(__name__, static_folder='.', static_url_path='')
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global state
serial_thread = None
serial_running = False
current_serial = None


def list_serial_ports():
    """List available serial ports"""
    ports = []
    for port in serial.tools.list_ports.comports():
        ports.append({
            'device': port.device,
            'description': port.description,
            'hwid': port.hwid
        })
    return ports


def serial_reader_thread(port, baud):
    """Background thread to read serial data and emit to WebSocket"""
    global serial_running, current_serial
    
    print(f"[Serial] Connecting to {port} at {baud} baud...")
    
    try:
        current_serial = serial.Serial(port, baud, timeout=0.5)
        time.sleep(2)  # Wait for Arduino to reset
        print(f"[Serial] Connected to {port}")
        socketio.emit('status', {'connected': True, 'port': port})
        
        while serial_running:
            try:
                if current_serial.in_waiting > 0:
                    line = current_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[Serial] Received: {line}")
                        socketio.emit('serial_data', {'data': line})
                else:
                    time.sleep(0.01)  # Small delay to prevent CPU spinning
            except Exception as e:
                print(f"[Serial] Read error: {e}")
                break
                
    except serial.SerialException as e:
        print(f"[Serial] Connection error: {e}")
        socketio.emit('status', {'connected': False, 'error': str(e)})
    finally:
        if current_serial and current_serial.is_open:
            current_serial.close()
        current_serial = None
        serial_running = False
        print(f"[Serial] Disconnected from {port}")
        socketio.emit('status', {'connected': False, 'port': port})


def test_data_thread():
    """Generate test data simulating radar sweep"""
    global serial_running
    
    print("[Test] Starting test data generator...")
    socketio.emit('status', {'connected': True, 'port': 'TEST_MODE'})
    
    angle = -30
    direction = 1
    
    while serial_running:
        # Simulate distance reading (random with some pattern)
        import random
        if 10 <= angle <= 20:
            distance = random.uniform(15, 25)  # Object detected
        else:
            distance = random.uniform(50, 200)  # Background
            
        line = f"RADAR:{angle},{distance:.2f}"
        print(f"[Test] {line}")
        socketio.emit('serial_data', {'data': line})
        
        # Update angle
        angle += direction
        if angle >= 30:
            direction = -1
        elif angle <= -30:
            direction = 1
            
        time.sleep(0.035)  # ~28.6 deg/s
        
    socketio.emit('status', {'connected': False, 'port': 'TEST_MODE'})


@app.route('/')
def index():
    """Serve the main HTML page"""
    return send_from_directory('.', 'index.html')


@app.route('/<path:path>')
def static_files(path):
    """Serve static files"""
    return send_from_directory('.', path)


@app.route('/ports')
def get_ports():
    """Return list of available serial ports"""
    return json.dumps(list_serial_ports())


@socketio.on('connect')
def handle_connect():
    print('[WebSocket] Client connected')
    emit('status', {'connected': False, 'message': 'Ready to connect to serial port'})


@socketio.on('disconnect')
def handle_disconnect():
    print('[WebSocket] Client disconnected')


@socketio.on('connect_serial')
def handle_connect_serial(data):
    global serial_thread, serial_running
    
    port = data.get('port', 'COM5')
    baud = data.get('baud', 115200)
    test_mode = data.get('test', False)
    
    # Stop existing connection
    if serial_running:
        serial_running = False
        if serial_thread:
            serial_thread.join(timeout=2)
    
    # Start new connection
    serial_running = True
    if test_mode:
        serial_thread = threading.Thread(target=test_data_thread, daemon=True)
    else:
        serial_thread = threading.Thread(target=serial_reader_thread, args=(port, baud), daemon=True)
    serial_thread.start()


@socketio.on('disconnect_serial')
def handle_disconnect_serial():
    global serial_running, current_serial
    
    serial_running = False
    if current_serial and current_serial.is_open:
        current_serial.close()
    emit('status', {'connected': False})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Radar Visualization Server')
    parser.add_argument('--test', action='store_true', help='Run with test data (no Arduino needed)')
    args = parser.parse_args()
    
    print("=" * 50)
    print("Radar Visualization Server")
    print("=" * 50)
    print("\nAvailable serial ports:")
    for port in list_serial_ports():
        print(f"  - {port['device']}: {port['description']}")
    print("\nStarting server on http://localhost:5000")
    if args.test:
        print("TEST MODE: Will generate simulated data")
    print("Press Ctrl+C to stop\n")
    
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)



if __name__ == '__main__':
    print("=" * 50)
    print("Radar Visualization Server")
    print("=" * 50)
    print("\nAvailable serial ports:")
    for port in list_serial_ports():
        print(f"  - {port['device']}: {port['description']}")
    print("\nStarting server on http://localhost:5000")
    print("Press Ctrl+C to stop\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False)
