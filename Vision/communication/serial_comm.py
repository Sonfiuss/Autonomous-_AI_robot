"""
Serial Communication Module
Handles low-level serial communication with Arduino
"""

import serial
import time
import threading
import queue
from typing import List, Optional
import logging

logger = logging.getLogger(__name__)


class ArduinoSerialComm:
    """
    Low-level serial communication with Arduino
    Thread-safe, non-blocking command queue
    """
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize serial connection
        
        Args:
            port: Serial port (e.g., 'COM3', '/dev/ttyUSB0')
            baudrate: Communication speed
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connected = False
        self._serial = None
        
        # Command queue for thread-safe operation
        self._command_queue = queue.Queue(maxsize=10)
        self._running = False
        self._sender_thread = None
        
        # Statistics
        self._commands_sent = 0
        self._commands_failed = 0
        
        self._connect()
    
    def _connect(self):
        """Establish serial connection"""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino reset
            self.connected = True
            logger.info(f"✓ Connected to {self.port} at {self.baudrate} baud")
            
            # Read welcome message
            time.sleep(0.1)
            while self._serial.in_waiting:
                msg = self._serial.readline().decode().strip()
                logger.debug(f"Arduino: {msg}")
            
            # Start sender thread
            self._running = True
            self._sender_thread = threading.Thread(
                target=self._sender_loop,
                daemon=True,
                name="SerialSenderThread"
            )
            self._sender_thread.start()
            
        except serial.SerialException as e:
            logger.error(f"✗ Failed to connect to {self.port}: {e}")
            self.connected = False
    
    def _sender_loop(self):
        """Background thread for sending commands"""
        while self._running:
            try:
                # Get command from queue (blocking with timeout)
                command = self._command_queue.get(timeout=0.05)
                self._send_direct(command)
                self._command_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in sender loop: {e}")
    
    def _send_direct(self, command: str) -> bool:
        """
        Send command directly to serial port
        
        Args:
            command: Command string to send
            
        Returns:
            True if successful, False otherwise
        """
        if not self.connected or not self._serial:
            return False
        
        try:
            self._serial.write(command.encode())
            self._commands_sent += 1
            logger.debug(f"Sent: {command.strip()}")
            return True
        except Exception as e:
            self._commands_failed += 1
            logger.error(f"Error sending command: {e}")
            return False
    
    def send_command(self, command: str, blocking: bool = False) -> bool:
        """
        Send command via queue (non-blocking by default)
        
        Args:
            command: Command string to send
            blocking: Wait for command to be sent
            
        Returns:
            True if queued successfully
        """
        if not self.connected:
            return False
        
        try:
            self._command_queue.put_nowait(command)
            if blocking:
                self._command_queue.join()
            return True
        except queue.Full:
            logger.warning("Command queue full, dropping command")
            return False
    
    def read_line(self) -> Optional[str]:
        """
        Read a line from serial port
        
        Returns:
            Decoded string or None if no data
        """
        if not self.connected or not self._serial:
            return None
        
        try:
            if self._serial.in_waiting:
                return self._serial.readline().decode().strip()
        except Exception as e:
            logger.error(f"Error reading from serial: {e}")
        
        return None
    
    def get_statistics(self) -> dict:
        """Get communication statistics"""
        return {
            'commands_sent': self._commands_sent,
            'commands_failed': self._commands_failed,
            'queue_size': self._command_queue.qsize(),
            'connected': self.connected
        }
    
    def close(self):
        """Close serial connection"""
        self._running = False
        
        if self._sender_thread:
            self._sender_thread.join(timeout=1.0)
        
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("✓ Serial connection closed")
        
        self.connected = False
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
