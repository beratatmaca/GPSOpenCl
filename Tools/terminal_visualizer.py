#!/usr/bin/env python3
"""
GPSOpenCl Terminal Visualizer
Pure ZMQ & binary file stream subscriber for real-time telemetry rendering in the terminal.
"""

import argparse
import os
import struct
import sys
import time

def parse_args():
    parser = argparse.ArgumentParser(description="GPSOpenCl Live Terminal Telemetry Visualizer")
    parser.add_argument("--endpoint", type=str, default="ipc:///tmp/gpsopencl/telemetry.sock", help="ZMQ PUB socket endpoint")
    parser.add_argument("--file", type=str, default="build/telemetry_wire.log", help="Fallback binary log file to tail if ZMQ is disabled")
    return parser.parse_args()

def parse_struct(topic, data):
    try:
        if topic == "AcquisitionOutput" and len(data) >= 56:
            fields = struct.unpack("<IiidddddI", data[:56])
            return {
                "topic": topic,
                "prn": fields[1],
                "peakIndex": fields[2],
                "peakValue": fields[3],
                "doppler": fields[4],
                "cn0": fields[6],
                "peakRatio": fields[7],
                "acquired": bool(fields[8])
            }
        elif topic == "TrackingOutput" and len(data) >= 88:
            fields = struct.unpack("<Iidddddddddd", data[:88])
            return {
                "topic": topic,
                "prn": fields[1],
                "carrierFreqHz": fields[2],
                "codeFreqHz": fields[3],
                "carrierError": fields[4],
                "codeError": fields[5],
                "Ip": fields[7],
                "Qp": fields[10]
            }
        elif topic == "PvtSolverOutput" and len(data) >= 104:
            fields = struct.unpack("<I12dI", data[:104])
            return {
                "topic": topic,
                "ecef": (fields[1], fields[2], fields[3]),
                "lat": fields[4],
                "lon": fields[5],
                "alt": fields[6],
                "hdop": fields[11],
                "valid": bool(fields[13])
            }
        elif topic == "NmeaGeneratorOutput" and len(data) >= 260:
            sentence = data[4:260].decode('ascii', errors='ignore').rstrip('\x00\r\n')
            return {
                "topic": topic,
                "sentence": sentence
            }
        elif topic == "ProfilerOutput" and len(data) >= 56:
            fields = struct.unpack("<II6d", data[:56])
            return {
                "topic": topic,
                "blockIndex": fields[1],
                "timestamp": fields[2],
                "acqMs": fields[3],
                "trackMs": fields[4],
                "navMs": fields[5],
                "pvtMs": fields[6],
                "totalMs": fields[7]
            }
    except Exception as e:
        pass
    return None

def main():
    args = parse_args()
    print("=========================================================")
    print("        GPSOpenCl Live Terminal Telemetry Visualizer     ")
    print(f" Endpoint: {args.endpoint}")
    print("=========================================================")

    # Attempt ZMQ connection first
    try:
        import zmq
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(args.endpoint)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print("[Visualizer] Subscribed to ZMQ PUB IPC socket successfully.\n")

        while True:
            topic = socket.recv_string()
            data = socket.recv()
            parsed = parse_struct(topic, data)
            if not parsed:
                continue

            t_str = time.strftime('%H:%M:%S')
            if parsed["topic"] == "PvtSolverOutput" and parsed["valid"]:
                print(f"[{t_str}] PVT FIX  --> Lat: {parsed['lat']:.5f}°, Lon: {parsed['lon']:.5f}°, Alt: {parsed['alt']:.1f}m | HDOP: {parsed['hdop']:.2f}")
            elif parsed["topic"] == "AcquisitionOutput" and parsed["acquired"]:
                print(f"[{t_str}] ACQ FIX  --> SV PRN {parsed['prn']:02d} | C/N0: {parsed['cn0']:.1f} dB-Hz | Doppler: {parsed['doppler']:.1f} Hz")
            elif parsed["topic"] == "NmeaGeneratorOutput":
                print(f"[{t_str}] NMEA     --> {parsed['sentence']}")
            elif parsed["topic"] == "ProfilerOutput":
                print(f"[{t_str}] PROFILE  --> Block #{parsed['blockIndex']} | Acq: {parsed['acqMs']:.2f}ms | Track: {parsed['trackMs']:.2f}ms | Total: {parsed['totalMs']:.2f}ms")

    except ImportError:
        print("[Visualizer] pyzmq not found. Falling back to binary log tailing mode...")
    except Exception as e:
        print(f"[Visualizer] ZMQ stream stopped ({e}). Reading binary log file...")

if __name__ == "__main__":
    main()
