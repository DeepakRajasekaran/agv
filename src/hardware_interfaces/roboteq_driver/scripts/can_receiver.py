import socket
import struct
import time
import threading
import argparse
import logging
import queue
import sys

# ANSI Terminal Colors
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_RED = "\033[31m"

CAN_FRAME_FORMAT = "=IB3x8s"

class RoboteqCANopenDriver:
    """
    Object-oriented CANopen driver for Roboteq Motor Controller.
    Handles thread-safe asynchronous telemetry polling and command transmission.
    """
    def __init__(self, interface='can0', node_id=1, log_raw=False):
        self.interface = interface
        self.node_id = node_id
        self.log_raw = log_raw
        self.running = False
        self.sock = None
        self.last_update_time = 0
        
        # Telemetry storage (Thread-safe read access)
        self.feedback_rpm = [0, 0]
        self.encoder_counts = [0, 0]
        self.battery_voltage = 0.0
        self.motor_currents = [0.0, 0.0]
        self.fault_flags = 0
        self.controller_temp = 0
        self.target_speed = [0, 0] # Store target to prevent watchdog timeout
        
        # Async Logging
        self.log_queue = queue.Queue()
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        
    def _log_worker(self):
        while self.running:
            try:
                msg = self.log_queue.get(timeout=0.1)
                if msg:
                    logging.info(msg)
            except queue.Empty:
                pass
                
    def log_info(self, msg):
        """Thread-safe logging"""
        if self.running:
            self.log_queue.put(msg)
        else:
            logging.info(msg)
        
    def connect(self):
        """Establishes connection to the CAN interface."""
        try:
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((self.interface,))
            self.log_info(f"Connected successfully to CANopen Node {self.node_id} on {self.interface}")
        except Exception as e:
            logging.error(f"Failed to bind to CAN interface '{self.interface}': {e}")
            raise

    def start(self):
        """Starts background threads for receiving and polling telemetry."""
        if self.running:
            return
            
        self.running = True
        self.log_thread.start()
        
        self.rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.rx_thread.start()
        
        self.query_thread = threading.Thread(target=self._query_loop, daemon=True)
        self.query_thread.start()

    def stop(self):
        """Stops all threads and closes the CAN socket safely."""
        self.running = False
        if self.sock:
            self.sock.close()

    def _recv_loop(self):
        while self.running:
            try:
                cf, _ = self.sock.recvfrom(16)
                if len(cf) < 16:
                    continue
                can_id, dlc, data_bytes = struct.unpack(CAN_FRAME_FORMAT, cf)
                can_id &= socket.CAN_EFF_MASK
                self._parse_frame(can_id, data_bytes[:dlc])
            except Exception as e:
                if not self.running:
                    break
                time.sleep(0.01)

    def _parse_frame(self, can_id, data):
        raw_hex = data.hex()
        raw_suffix = f" | Raw: {raw_hex}" if self.log_raw else ""
        
        # SDO Response
        sdo_rx_id = 0x580 + self.node_id
        
        if can_id == sdo_rx_id and len(data) >= 8:
            cmd, idx, subidx = struct.unpack("<BHB", data[:4])
            
            # Identify abort
            if cmd == 0x80:
                err_code = struct.unpack("<I", data[4:8])[0]
                self.log_info(f"SDO Abort on Index 0x{idx:04X} Sub {subidx}: Error 0x{err_code:08X}")
                return
                
            # Safely unpack based on data length indicator in the command specifier
            val_s32 = 0
            val_u32 = 0
            
            if cmd == 0x4F: # 1-byte response
                val_s32 = struct.unpack("<b", data[4:5])[0]
                val_u32 = struct.unpack("<B", data[4:5])[0]
            elif cmd == 0x4B: # 2-byte response
                val_s32 = struct.unpack("<h", data[4:6])[0]
                val_u32 = struct.unpack("<H", data[4:6])[0]
            else: # 4-byte response (0x43) or unspecified
                val_s32 = struct.unpack("<i", data[4:8])[0]
                val_u32 = struct.unpack("<I", data[4:8])[0]
            
            if idx == 0x210D:
                self.battery_voltage = val_u32 / 10.0
                self.log_info(f"Battery: {self.battery_voltage}V{raw_suffix}")
                
            elif idx == 0x2100:
                if subidx == 1:
                    self.motor_currents[0] = val_s32 / 10.0
                    self.log_info(f"Motor 1 Amps: {self.motor_currents[0]}A{raw_suffix}")
                elif subidx == 2:
                    self.motor_currents[1] = val_s32 / 10.0
                    self.log_info(f"Motor 2 Amps: {self.motor_currents[1]}A{raw_suffix}")
                    
            elif idx == 0x2104:
                if subidx == 1:
                    self.encoder_counts[0] = val_s32
                elif subidx == 2:
                    self.encoder_counts[1] = val_s32
                    self.log_info(f"Encoders: {self.encoder_counts}{raw_suffix}")
                    
            elif idx == 0x2109:
                if subidx == 1:
                    self.feedback_rpm[0] = val_s32
                elif subidx == 2:
                    self.feedback_rpm[1] = val_s32
                    self.log_info(f"RPM: {self.feedback_rpm}{raw_suffix}")
            
            elif idx == 0x2111:
                self.log_info(f"Controller Status (FS): 0x{val_u32:04X}{raw_suffix}")
            
            elif idx == 0x2113:
                self.fault_flags = val_u32
                if self.fault_flags != 0:
                    self.log_info(f"FAULT FLAG TRIGGERED (FF): 0x{self.fault_flags:04X}{raw_suffix}")
                else:
                    self.log_info(f"Fault Flags (FF): 0x0000{raw_suffix}")
            
            elif idx == 0x2114:
                self.log_info(f"Closed Loop Error Sub {subidx}: {val_s32}{raw_suffix}")
                
            elif idx == 0x2122:
                suffix = "\n" if subidx == 2 else ""
                self.log_info(f"Motor Status (FM) Sub {subidx}: 0x{val_u32:04X}{raw_suffix}{suffix}")
                
            self.last_update_time = time.time()
            
        elif can_id == (0x700 + self.node_id):
            if len(data) >= 1:
                state = data[0]
                if self.log_raw:
                    self.log_info(f"Heartbeat Node {self.node_id}: State 0x{state:02X}")

    def _send_sdo_read(self, index, subindex):
        sdo_tx_id = 0x600 + self.node_id
        payload = struct.pack("<BHBxxxx", 0x40, index, subindex)
        self._send_frame(sdo_tx_id, payload)

    def _send_sdo_write(self, index, subindex, value, size=4):
        sdo_tx_id = 0x600 + self.node_id
        if size == 4:
            payload = struct.pack("<BHBi", 0x23, index, subindex, int(value))
        elif size == 2:
            payload = struct.pack("<BHBh", 0x2B, index, subindex, int(value))
        elif size == 1:
            payload = struct.pack("<BHBb", 0x2F, index, subindex, int(value))
        else:
            payload = struct.pack("<BHBi", 0x22, index, subindex, int(value)) # Unspecified
        self._send_frame(sdo_tx_id, payload)

    def send_speed_command(self, speed1, speed2):
        """Updates target speeds. The background loop continuously sends this to prevent watchdog timeout."""
        self.target_speed = [speed1, speed2]
        
    def reset_encoders(self):
        """Resets both encoder counters to zero (!C command)"""
        # 0x2003 is the Roboteq Object Dictionary index for Set Encoder Counter
        self._send_sdo_write(0x2003, 1, 0, size=4)
        self._send_sdo_write(0x2003, 2, 0, size=4)
        self.log_info("Sent command to RESET Encoders to 0")

    def _send_frame(self, can_id, payload):
        try:
            payload = payload.ljust(8, b'\x00')
            frame = struct.pack(CAN_FRAME_FORMAT, can_id, len(payload), payload)
            self.sock.send(frame)
        except Exception as e:
            self.log_info(f"TX Error: {e}")

    def _query_loop(self):
        """Background thread polling telemetry via SDO at 20Hz."""
        poll_list = [
            (0x210D, 2), # Volts 2 (Battery 48V)
            (0x2100, 1), # Amps 1
            (0x2100, 2), # Amps 2
            (0x2104, 1), # Enc 1
            (0x2104, 2), # Enc 2
            (0x2109, 1), # RPM 1
            (0x2109, 2), # RPM 2
            (0x2111, 0), # FS (Status Flag)
            (0x2113, 0), # FF (Fault Flag)
            (0x2114, 1), # Closed Loop Error 1
            (0x2114, 2), # Closed Loop Error 2
            (0x2122, 1), # FM 1 (Motor 1 Status)
            (0x2122, 2), # FM 2 (Motor 2 Status)
        ]
        
        while self.running:
            # Continually send the target speed to prevent the Roboteq Watchdog from cutting power
            self._send_sdo_write(0x2000, 1, self.target_speed[0], size=4)
            time.sleep(0.005)
            self._send_sdo_write(0x2000, 2, self.target_speed[1], size=4)
            time.sleep(0.005)
            
            for idx, subidx in poll_list:
                if not self.running:
                    break
                self._send_sdo_read(idx, subidx)
                time.sleep(0.005) # 5ms interval
            time.sleep(0.05) # ~20Hz cycle

def main():
    parser = argparse.ArgumentParser(description="AGV CAN Driver App")
    parser.add_argument("--interface", type=str, default="can0", help="CAN interface (default: can0)")
    parser.add_argument("--node-id", type=int, default=1, help="Roboteq Node ID (default: 1)")
    parser.add_argument("--raw", action="store_true", help="Log raw CAN frames")
    args = parser.parse_args()

    # Configure logging to file only (so it doesn't spam CLI)
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%H:%M:%S',
        handlers=[
            logging.FileHandler("agv_driver.log")
        ]
    )
    
    # Initialize the CANopen driver
    driver = RoboteqCANopenDriver(interface=args.interface, node_id=args.node_id, log_raw=args.raw)
    driver.connect()
    driver.start()
    
    print("AGV Driver active. Polling telemetry and writing to agv_driver.log...")
    print("COMMANDS:")
    print("  g <left> <right>  : Send speed command (e.g., 'g 100 -100')")
    print("  s                 : Stop motors (speed 0 0)")
    print("  c                 : Reset encoder counters to 0")
    print("  q                 : Quit")
    
    try:
        while True:
            cmd_input = input("CMD> ").strip().lower()
            if not cmd_input:
                continue
                
            parts = cmd_input.split()
            cmd = parts[0]
            
            if cmd == 'q':
                break
            elif cmd == 's':
                driver.send_speed_command(0, 0)
                print("Motors stopped.")
            elif cmd == 'c':
                driver.reset_encoders()
                print("Encoder reset command sent.")
            elif cmd == 'g':
                if len(parts) == 3:
                    try:
                        spd1 = int(parts[1])
                        spd2 = int(parts[2])
                        driver.send_speed_command(spd1, spd2)
                        print(f"Speed command sent: {spd1}, {spd2}")
                    except ValueError:
                        print("Invalid speed values. Use integers.")
                else:
                    print("Usage: g <left> <right>")
            else:
                print("Unknown command.")
                
    except KeyboardInterrupt:
        print("\nShutting down driver safely...")
    finally:
        driver.send_speed_command(0, 0) # Safety stop
        driver.stop()
        sys.exit(0)

if __name__ == "__main__":
    main()
