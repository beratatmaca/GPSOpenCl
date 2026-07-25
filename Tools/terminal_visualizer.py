#!/usr/bin/env python3
"""
GPSOpenCl Terminal Visualizer
Subscribes to ZMQ telemetry sockets or reads binary log files to render live C/N0, PVT fix, and NMEA sentences in the terminal.
"""

import sys
import time
import struct

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description="GPSOpenCl Terminal Visualizer")
    parser.add_argument("--endpoint", type=str, default="ipc:///tmp/gpsopencl/telemetry.sock", help="ZMQ PUB socket endpoint")
    return parser.parse_args()

def main():
    args = parse_args()
    print("=========================================================")
    print("        GPSOpenCl Live Terminal Telemetry Visualizer     ")
    print(f" Listening on: {args.endpoint}")
    print("=========================================================")

    try:
        import zmq
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(args.endpoint)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")

        while True:
            topic = socket.recv_string()
            data = socket.recv()
            print(f"[{time.strftime('%H:%M:%S')}] Received Topic: {topic} ({len(data)} bytes)")
            if topic == "PvtSolverOutput" and len(data) >= 100:
                fields = struct.unpack("<I12dI", data[:108])
                print(f"  --> PVT Fix: Lat={fields[4]:.4f}°, Lon={fields[5]:.4f}°, Alt={fields[6]:.1f}m | HDOP={fields[11]:.2f}")
            elif topic == "NmeaGeneratorOutput" and len(data) >= 260:
                version = struct.unpack("<I", data[:4])[0]
                sentence = data[4:260].decode('ascii', errors='ignore').rstrip('\x00\r\n')
                print(f"  --> NMEA: {sentence}")
            elif topic == "AcquisitionOutput" and len(data) >= 56:
                fields = struct.unpack("<IiidddddI", data[:60])
                prn = fields[1]
                cn0 = fields[6]
                acquired = fields[8]
                if acquired:
                    print(f"  --> SV PRN {prn:02d} ACQUIRED | C/N0: {cn0:.1f} dB-Hz")

    except ImportError:
        print("pyzmq library not installed. Terminal subscriber is ready when pyzmq is installed.")

if __name__ == "__main__":
    main()
