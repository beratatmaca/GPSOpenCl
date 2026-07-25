#!/usr/bin/env python3
# Plotly Dash dashboard: subscribes to GPS receiver telemetry over ZMQ (8 wire struct types)
import argparse
import math
import os
import struct
import sys
import threading
import time

import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go

# Float safety and bounds formatting helpers
def safe_float(val, default=0.0):
    try:
        f = float(val)
        if math.isfinite(f):
            return f
    except Exception:
        pass
    return default

def fmt_float(val, prec=2, unit="", default="--"):
    f = safe_float(val, None)
    if f is None:
        return default
    formatted = f"{f:.{prec}f}"
    return f"{formatted} {unit}".strip() if unit else formatted

# Global in-memory thread-safe telemetry store
GLOBAL_STATE = {
    "source": {
        "blockIndex": 0,
        "timestamp": 0.0,
        "fifoUnderruns": 0,
        "fifoOverruns": 0
    },
    "satellites": {
        prn: {
            "prn": prn,
            "acquired": False,
            "state": 0,
            "stateLabel": "Acquiring",
            "carrierLockIndicator": 0.0,
            "codeLockRatio": 0.0,
            "cn0": 0.0,
            "doppler": 0.0,
            "peakRatio": 0.0,
            "peakIndex": 0,
            "carrierFreqHz": 0.0,
            "codeFreqHz": 1023000.0,
            "carrierError": 0.0,
            "codeError": 0.0,
            "Ie": 0.0, "Ip": 0.0, "Il": 0.0,
            "Qe": 0.0, "Qp": 0.0, "Ql": 0.0,
            "azimuth": (prn * 11.25) % 360,
            "elevation": 15.0 + ((prn * 7) % 70),
            "ionoDelay": 0.0,
            "tropoDelay": 0.0,
            "weekNumber": 0,
            "tow": 0.0,
            "subframeMask": 0,
            "af0": 0.0,
            "e": 0.0,
            "sqrtA": 0.0
        } for prn in range(1, 33)
    },
    "pvt": {
        "valid": False,
        "latitude": 0.0,
        "longitude": 0.0,
        "altitude": 0.0,
        "ecefX": 0.0,
        "ecefY": 0.0,
        "ecefZ": 0.0,
        "clockBiasMeters": 0.0,
        "clockBiasSeconds": 0.0,
        "gdop": 0.0,
        "pdop": 0.0,
        "hdop": 0.0,
        "vdop": 0.0
    },
    "profiler": {
        "blockIndex": 0,
        "timestamp": 0.0,
        "acqMs": 0.0,
        "trackMs": 0.0,
        "navMs": 0.0,
        "pvtMs": 0.0,
        "totalMs": 0.0
    },
    "nmea": [],
    "events": [],
    "last_update": 0.0
}
STATE_LOCK = threading.Lock()

def add_event(level, module, message):
    t_str = time.strftime('%H:%M:%S')
    evt = f"[{t_str}] [{level.upper()}] [{module}] {message}"
    if not GLOBAL_STATE["events"] or GLOBAL_STATE["events"][-1] != evt:
        GLOBAL_STATE["events"].append(evt)
        if len(GLOBAL_STATE["events"]) > 100:
            GLOBAL_STATE["events"].pop(0)

# Telemetry subscriber background thread (ZMQ + binary log stream)
class TelemetrySubscriberThread(threading.Thread):
    def __init__(self, endpoint="ipc:///tmp/gpsopencl/telemetry.sock", log_file="build/telemetry_wire.log"):
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.log_file = log_file
        self.running = True

    def run(self):
        zmq_thread = threading.Thread(target=self._run_zmq, daemon=True)
        zmq_thread.start()

        file_thread = threading.Thread(target=self._run_file_tail, daemon=True)
        file_thread.start()

        zmq_thread.join()
        file_thread.join()

    def _run_zmq(self):
        try:
            import zmq
            context = zmq.Context()
            socket = context.socket(zmq.SUB)
            socket.connect(self.endpoint)
            socket.setsockopt_string(zmq.SUBSCRIBE, "")
            print(f"[Subscriber] Connected to ZMQ endpoint: {self.endpoint}")

            while self.running:
                topic = socket.recv_string()
                data = socket.recv()
                self.process_msg(topic, data)
        except Exception as e:
            print(f"[Subscriber] ZMQ note: {e}")

    def _run_file_tail(self):
        print(f"[Subscriber] Tailing binary telemetry file: {self.log_file}...")
        while self.running:
            if not os.path.exists(self.log_file):
                time.sleep(0.5)
                continue

            try:
                with open(self.log_file, "rb") as f:
                    while self.running:
                        len_bytes = f.read(4)
                        if not len_bytes or len(len_bytes) < 4:
                            time.sleep(0.05)
                            continue
                        name_len = struct.unpack("<I", len_bytes)[0]
                        if name_len > 256:
                            continue
                        name_bytes = f.read(name_len)
                        if len(name_bytes) < name_len:
                            time.sleep(0.05)
                            continue
                        topic = name_bytes.decode('ascii', errors='ignore')

                        data_len_bytes = f.read(4)
                        if len(data_len_bytes) < 4:
                            time.sleep(0.05)
                            continue
                        data_len = struct.unpack("<I", data_len_bytes)[0]

                        data = f.read(data_len)
                        if len(data) < data_len:
                            time.sleep(0.05)
                            continue

                        self.process_msg(topic, data)
            except Exception:
                time.sleep(0.5)

    def process_msg(self, topic, data):
        try:
            with STATE_LOCK:
                now = time.time()
                GLOBAL_STATE["last_update"] = now

                if topic == "SourceOutput" and len(data) >= 24:
                    fields = struct.unpack("<IIdII", data[:24])
                    GLOBAL_STATE["source"]["blockIndex"] = fields[1]
                    GLOBAL_STATE["source"]["timestamp"] = safe_float(fields[2])
                    GLOBAL_STATE["source"]["fifoUnderruns"] = fields[3]
                    GLOBAL_STATE["source"]["fifoOverruns"] = fields[4]

                elif topic == "AcquisitionOutput" and len(data) >= 56:
                    fields = struct.unpack("<IiidddddI", data[:56])
                    prn = fields[1]
                    if 1 <= prn <= 32:
                        sv = GLOBAL_STATE["satellites"][prn]
                        was_acq = sv["acquired"]
                        sv["peakIndex"] = fields[2]
                        sv["doppler"] = safe_float(fields[4])
                        sv["cn0"] = safe_float(fields[6])
                        sv["peakRatio"] = safe_float(fields[7])
                        sv["acquired"] = bool(fields[8])
                        if not was_acq and sv["acquired"]:
                            add_event("INFO", "ACQUISITION", f"SV PRN {prn:02d} acquired! (C/N0: {fmt_float(sv['cn0'], 1)} dB-Hz, Doppler: {fmt_float(sv['doppler'], 1)} Hz)")

                elif topic == "TrackingOutput" and len(data) >= 108:
                    fields = struct.unpack("<IiddddddddddIdd", data[:108])
                    prn = fields[1]
                    if 1 <= prn <= 32:
                        sv = GLOBAL_STATE["satellites"][prn]
                        sv["carrierFreqHz"] = safe_float(fields[2])
                        sv["codeFreqHz"] = safe_float(fields[3])
                        sv["carrierError"] = safe_float(fields[4])
                        sv["codeError"] = safe_float(fields[5])
                        sv["Ie"], sv["Ip"], sv["Il"] = safe_float(fields[6]), safe_float(fields[7]), safe_float(fields[8])
                        sv["Qe"], sv["Qp"], sv["Ql"] = safe_float(fields[9]), safe_float(fields[10]), safe_float(fields[11])
                        sv["state"] = fields[12]
                        sv["stateLabel"] = {0: "Acquiring", 1: "Confirming", 2: "Tracking"}.get(fields[12], "Unknown")
                        sv["carrierLockIndicator"] = safe_float(fields[13])
                        sv["codeLockRatio"] = safe_float(fields[14])
                        sv["acquired"] = fields[12] >= 2

                elif topic == "NavDecoderOutput" and len(data) >= 188:
                    fields = struct.unpack("<IiidII20d", data[:188])
                    prn = fields[1]
                    if 1 <= prn <= 32:
                        sv = GLOBAL_STATE["satellites"][prn]
                        sv["weekNumber"] = fields[2]
                        sv["tow"] = safe_float(fields[3])
                        subframe_id = fields[4]
                        if 1 <= subframe_id <= 5:
                            sv["subframeMask"] |= (1 << (subframe_id - 1))
                        sv["af0"] = safe_float(fields[7])
                        sv["e"] = safe_float(fields[12])
                        sv["sqrtA"] = safe_float(fields[11])
                        add_event("INFO", "NAV_DECODER", f"Decoded Subframe {subframe_id} for PRN {prn:02d} (TOW: {fmt_float(sv['tow'], 0)}s)")

                elif topic == "PvtSolverOutput" and len(data) >= 104:
                    fields = struct.unpack("<I12dI", data[:104])
                    was_valid = GLOBAL_STATE["pvt"]["valid"]
                    GLOBAL_STATE["pvt"]["ecefX"] = safe_float(fields[1])
                    GLOBAL_STATE["pvt"]["ecefY"] = safe_float(fields[2])
                    GLOBAL_STATE["pvt"]["ecefZ"] = safe_float(fields[3])
                    GLOBAL_STATE["pvt"]["latitude"] = safe_float(fields[4])
                    GLOBAL_STATE["pvt"]["longitude"] = safe_float(fields[5])
                    GLOBAL_STATE["pvt"]["altitude"] = safe_float(fields[6])
                    GLOBAL_STATE["pvt"]["clockBiasMeters"] = safe_float(fields[7])
                    GLOBAL_STATE["pvt"]["clockBiasSeconds"] = safe_float(fields[8])
                    GLOBAL_STATE["pvt"]["gdop"] = safe_float(fields[9])
                    GLOBAL_STATE["pvt"]["pdop"] = safe_float(fields[10])
                    GLOBAL_STATE["pvt"]["hdop"] = safe_float(fields[11])
                    GLOBAL_STATE["pvt"]["vdop"] = safe_float(fields[12])
                    GLOBAL_STATE["pvt"]["valid"] = bool(fields[13])

                    if not was_valid and GLOBAL_STATE["pvt"]["valid"]:
                        add_event("SUCCESS", "PVT_SOLVER", f"3D WGS-84 Position Fix Solved: Lat={fmt_float(fields[4], 5)}°, Lon={fmt_float(fields[5], 5)}°, Alt={fmt_float(fields[6], 1)}m | HDOP: {fmt_float(fields[11], 2)}")

                elif topic == "AtmosphericOutput" and len(data) >= 40:
                    fields = struct.unpack("<Ii4d", data[:40])
                    prn = fields[1]
                    if 1 <= prn <= 32:
                        sv = GLOBAL_STATE["satellites"][prn]
                        sv["ionoDelay"] = safe_float(fields[2])
                        sv["tropoDelay"] = safe_float(fields[3])
                        sv["azimuth"] = safe_float(fields[4])
                        sv["elevation"] = safe_float(fields[5])

                elif topic == "NmeaGeneratorOutput" and len(data) >= 260:
                    sentence = data[4:260].decode('ascii', errors='ignore').rstrip('\x00\r\n')
                    if sentence and (not GLOBAL_STATE["nmea"] or GLOBAL_STATE["nmea"][-1] != sentence):
                        GLOBAL_STATE["nmea"].append(sentence)
                        if len(GLOBAL_STATE["nmea"]) > 50:
                            GLOBAL_STATE["nmea"].pop(0)

                elif topic == "ProfilerOutput" and len(data) >= 56:
                    fields = struct.unpack("<II6d", data[:56])
                    GLOBAL_STATE["profiler"]["blockIndex"] = fields[1]
                    GLOBAL_STATE["profiler"]["timestamp"] = safe_float(fields[2])
                    GLOBAL_STATE["profiler"]["acqMs"] = safe_float(fields[3])
                    GLOBAL_STATE["profiler"]["trackMs"] = safe_float(fields[4])
                    GLOBAL_STATE["profiler"]["navMs"] = safe_float(fields[5])
                    GLOBAL_STATE["profiler"]["pvtMs"] = safe_float(fields[6])
                    GLOBAL_STATE["profiler"]["totalMs"] = safe_float(fields[7])
        except Exception as e:
            pass

# Dash application and responsive single-page layout
app = dash.Dash(__name__, title="GPSOpenCl - Comprehensive GNSS Telemetry Dashboard")

app.layout = html.Div(
    style={
        "backgroundColor": "#0b1329",
        "color": "#f8fafc",
        "fontFamily": "'Inter', '-apple-system', 'Segoe UI', sans-serif",
        "minHeight": "100vh",
        "padding": "20px",
        "boxSizing": "border-box"
    },
    children=[
        # Top header and status bar
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "backgroundColor": "#1e293b",
                "padding": "16px 24px",
                "borderRadius": "12px",
                "border": "1px solid #334155",
                "boxShadow": "0 4px 14px rgba(0,0,0,0.4)",
                "marginBottom": "20px"
            },
            children=[
                html.Div([
                    html.H1("GPSOpenCl — Real-Time Telemetry & Visual Analytics", style={"margin": 0, "fontSize": "22px", "fontWeight": "700", "color": "#38bdf8"}),
                    html.P("Single-Page Crash-Safe Receiver State, Signal Tracking & Event Monitor", style={"margin": "4px 0 0 0", "fontSize": "13px", "color": "#94a3b8"})
                ]),
                html.Div(
                    style={"display": "flex", "gap": "16px", "alignItems": "center"},
                    children=[
                        html.Div(id="status-badge", children="STREAM ONLINE", style={"backgroundColor": "#059669", "color": "#ffffff", "padding": "6px 14px", "borderRadius": "20px", "fontSize": "12px", "fontWeight": "600", "letterSpacing": "0.5px"}),
                        html.Div(id="last-update", children="Updated: Just now", style={"fontSize": "12px", "color": "#94a3b8"})
                    ]
                )
            ]
        ),

        # Row 1: KPI metric cards
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "repeat(5, 1fr)", "gap": "14px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "border": "1px solid #334155", "borderLeft": "4px solid #10b981", "overflow": "hidden"},
                    children=[
                        html.Div("Acquired Channels", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-sat-count", children="0 / 32", style={"fontSize": "22px", "fontWeight": "700", "color": "#10b981", "marginTop": "4px", "whiteSpace": "nowrap", "overflow": "hidden", "textOverflow": "ellipsis"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "border": "1px solid #334155", "borderLeft": "4px solid #38bdf8", "overflow": "hidden"},
                    children=[
                        html.Div("WGS-84 Location Fix", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-location", children="Acquiring...", style={"fontSize": "13px", "fontWeight": "600", "color": "#38bdf8", "marginTop": "6px", "whiteSpace": "nowrap", "overflow": "hidden", "textOverflow": "ellipsis"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "border": "1px solid #334155", "borderLeft": "4px solid #f59e0b", "overflow": "hidden"},
                    children=[
                        html.Div("Ellipsoidal Height", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-altitude", children="--", style={"fontSize": "20px", "fontWeight": "700", "color": "#f59e0b", "marginTop": "4px", "whiteSpace": "nowrap", "overflow": "hidden", "textOverflow": "ellipsis"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "border": "1px solid #334155", "borderLeft": "4px solid #a855f7", "overflow": "hidden"},
                    children=[
                        html.Div("Dilution of Precision (DOP)", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-dop", children="--", style={"fontSize": "16px", "fontWeight": "700", "color": "#a855f7", "marginTop": "6px", "whiteSpace": "nowrap", "overflow": "hidden", "textOverflow": "ellipsis"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "border": "1px solid #334155", "borderLeft": "4px solid #ec4899", "overflow": "hidden"},
                    children=[
                        html.Div("Block Processing Time", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-latency", children="--", style={"fontSize": "20px", "fontWeight": "700", "color": "#ec4899", "marginTop": "4px", "whiteSpace": "nowrap", "overflow": "hidden", "textOverflow": "ellipsis"})
                    ]
                )
            ]
        ),

        # Row 2: skyplot, C/N0, and Doppler graphs
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr 1fr", "gap": "16px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("Satellite Skyplot (Azimuth & Elevation)", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        dcc.Graph(id="skyplot-graph", config={"displayModeBar": False}, style={"height": "300px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("Carrier-to-Noise Ratio (C/N0 dB-Hz)", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        dcc.Graph(id="cn0-graph", config={"displayModeBar": False}, style={"height": "300px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("Carrier Doppler Shift (Hz)", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        dcc.Graph(id="doppler-graph", config={"displayModeBar": False}, style={"height": "300px"})
                    ]
                )
            ]
        ),

        # Row 3: tracking discriminator errors and profiler
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "16px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("PLL Carrier & DLL Code Discriminator Errors", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        dcc.Graph(id="tracking-error-graph", config={"displayModeBar": False}, style={"height": "280px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("Pipeline Per-Stage Processing Latency (ms)", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        dcc.Graph(id="profiler-graph", config={"displayModeBar": False}, style={"height": "280px"})
                    ]
                )
            ]
        ),

        # Row 4: decoded satellite ephemeris table
        html.Div(
            style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "marginBottom": "20px", "overflow": "hidden"},
            children=[
                html.H3("Decoded Satellite Ephemerides & Atmospheric Delays", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                html.Div(id="ephemeris-table-container", style={"overflowX": "auto", "maxHeight": "300px", "overflowY": "auto"})
            ]
        ),

        # Row 5: software events feed and NMEA stream
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "16px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("Real-Time Software Lifecycle & Event Monitor Log", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        html.Pre(
                            id="software-events-window",
                            style={
                                "backgroundColor": "#060a12",
                                "color": "#10b981",
                                "padding": "12px",
                                "borderRadius": "8px",
                                "fontFamily": "'JetBrains Mono', 'Fira Code', monospace",
                                "fontSize": "11px",
                                "height": "300px",
                                "overflowY": "auto",
                                "whiteSpace": "pre-wrap",
                                "wordBreak": "break-all",
                                "border": "1px solid #1e293b"
                            }
                        )
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px", "border": "1px solid #334155", "overflow": "hidden"},
                    children=[
                        html.H3("NMEA-0183 Live Telemetry Output Stream", style={"margin": "0 0 12px 0", "fontSize": "15px", "color": "#f8fafc"}),
                        html.Pre(
                            id="nmea-log-window",
                            style={
                                "backgroundColor": "#060a12",
                                "color": "#38bdf8",
                                "padding": "12px",
                                "borderRadius": "8px",
                                "fontFamily": "'JetBrains Mono', 'Fira Code', monospace",
                                "fontSize": "11px",
                                "height": "300px",
                                "overflowY": "auto",
                                "whiteSpace": "pre-wrap",
                                "wordBreak": "break-all",
                                "border": "1px solid #1e293b"
                            }
                        )
                    ]
                )
            ]
        ),

        # Timer Component for Continuous Updates (1s interval)
        dcc.Interval(id="interval-component", interval=1000, n_intervals=0)
    ]
)

# Dash callback: single state-lock read, consistent UI update
@app.callback(
    [
        Output("card-sat-count", "children"),
        Output("card-location", "children"),
        Output("card-altitude", "children"),
        Output("card-dop", "children"),
        Output("card-latency", "children"),
        Output("skyplot-graph", "figure"),
        Output("cn0-graph", "figure"),
        Output("doppler-graph", "figure"),
        Output("tracking-error-graph", "figure"),
        Output("profiler-graph", "figure"),
        Output("ephemeris-table-container", "children"),
        Output("software-events-window", "children"),
        Output("nmea-log-window", "children"),
        Output("last-update", "children")
    ],
    [Input("interval-component", "n_intervals")]
)
def update_dashboard(n):
    try:
        with STATE_LOCK:
            sats = list(GLOBAL_STATE["satellites"].values())
            pvt = dict(GLOBAL_STATE["pvt"])
            profiler = dict(GLOBAL_STATE["profiler"])
            nmea_lines = list(GLOBAL_STATE["nmea"])
            events = list(GLOBAL_STATE["events"])
            last_t = GLOBAL_STATE["last_update"]

        # 1. KPI cards
        acq_count = sum(1 for s in sats if s.get("acquired"))
        sat_count_str = f"{acq_count} / {len(sats)}"

        lat, lon, alt = pvt.get("latitude", 0.0), pvt.get("longitude", 0.0), pvt.get("altitude", 0.0)
        if pvt.get("valid"):
            location_str = f"{fmt_float(abs(lat), 4)}°{'N' if lat>=0 else 'S'}, {fmt_float(abs(lon), 4)}°{'E' if lon>=0 else 'W'}"
            alt_str = fmt_float(alt, 1, "m")
            dop_str = f"HDOP: {fmt_float(pvt.get('hdop'), 2)} | PDOP: {fmt_float(pvt.get('pdop'), 2)}"
        else:
            location_str = "Acquiring Ephemeris..."
            alt_str = "--"
            dop_str = "--"

        tot_ms = profiler.get("totalMs", 0.0)
        latency_str = fmt_float(tot_ms, 2, "ms") if tot_ms > 0 else "--"

        # 2. Skyplot Graph
        r_vals = [90 - safe_float(s["elevation"], 0.0) for s in sats if s.get("acquired")]
        theta_vals = [safe_float(s["azimuth"], 0.0) for s in sats if s.get("acquired")]
        labels = [f"PRN {s['prn']:02d}" for s in sats if s.get("acquired")]

        skyplot_fig = go.Figure(go.Scatterpolar(
            r=r_vals,
            theta=theta_vals,
            mode="markers+text",
            text=labels,
            textposition="top center",
            marker=dict(size=11, color="#38bdf8", line=dict(color="#0284c7", width=2))
        ))
        skyplot_fig.update_layout(
            polar=dict(
                radialaxis=dict(visible=True, range=[0, 90], showticklabels=False, color="#475569"),
                angularaxis=dict(direction="clockwise", color="#94a3b8"),
                bgcolor="#0b1329"
            ),
            paper_bgcolor="#1e293b",
            margin=dict(l=25, r=25, t=15, b=15),
            font=dict(color="#f8fafc", size=10),
            height=300,
            autosize=False
        )

        # 3. C/N0 Bar Chart
        prns = [f"P{s['prn']:02d}" for s in sats]
        cn0s = [safe_float(s["cn0"], 0.0) for s in sats]
        cn0_colors = ["#10b981" if c >= 40 else "#f59e0b" if c >= 35 else "#ef4444" for c in cn0s]

        cn0_fig = go.Figure(go.Bar(x=prns, y=cn0s, marker_color=cn0_colors))
        cn0_fig.update_layout(
            paper_bgcolor="#1e293b",
            plot_bgcolor="#0b1329",
            margin=dict(l=25, r=15, t=15, b=35),
            font=dict(color="#f8fafc", size=10),
            xaxis=dict(gridcolor="#1e293b"),
            yaxis=dict(title="C/N0 (dB-Hz)", range=[0, 60], gridcolor="#1e293b"),
            height=300,
            autosize=False
        )

        # 4. Carrier Doppler Graph
        dopplers = [safe_float(s["doppler"], 0.0) for s in sats]
        doppler_fig = go.Figure(go.Bar(x=prns, y=dopplers, marker_color="#a855f7"))
        doppler_fig.update_layout(
            paper_bgcolor="#1e293b",
            plot_bgcolor="#0b1329",
            margin=dict(l=25, r=15, t=15, b=35),
            font=dict(color="#f8fafc", size=10),
            xaxis=dict(gridcolor="#1e293b"),
            yaxis=dict(title="Doppler Shift (Hz)", gridcolor="#1e293b"),
            height=300,
            autosize=False
        )

        # 5. Tracking error graph (carrier & code)
        carr_errs = [safe_float(s["carrierError"], 0.0) for s in sats]
        code_errs = [safe_float(s["codeError"], 0.0) for s in sats]

        tracking_error_fig = go.Figure()
        tracking_error_fig.add_trace(go.Scatter(x=prns, y=carr_errs, mode="lines+markers", name="PLL Carrier Error (Hz)", line=dict(color="#38bdf8", width=2)))
        tracking_error_fig.add_trace(go.Scatter(x=prns, y=code_errs, mode="lines+markers", name="DLL Code Error (Chips)", line=dict(color="#f59e0b", width=2)))
        tracking_error_fig.update_layout(
            paper_bgcolor="#1e293b",
            plot_bgcolor="#0b1329",
            margin=dict(l=25, r=15, t=15, b=35),
            font=dict(color="#f8fafc", size=10),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
            xaxis=dict(gridcolor="#1e293b"),
            yaxis=dict(gridcolor="#1e293b"),
            height=280,
            autosize=False
        )

        # 6. Profiler Stage Latency Graph
        stages = ["Acquisition", "Tracking", "Nav Decode", "PVT Solve"]
        latencies = [
            safe_float(profiler.get("acqMs"), 0.0),
            safe_float(profiler.get("trackMs"), 0.0),
            safe_float(profiler.get("navMs"), 0.0),
            safe_float(profiler.get("pvtMs"), 0.0)
        ]
        stage_colors = ["#38bdf8", "#10b981", "#a855f7", "#ec4899"]

        profiler_fig = go.Figure(go.Bar(x=stages, y=latencies, marker_color=stage_colors))
        profiler_fig.update_layout(
            paper_bgcolor="#1e293b",
            plot_bgcolor="#0b1329",
            margin=dict(l=25, r=15, t=15, b=35),
            font=dict(color="#f8fafc", size=10),
            xaxis=dict(gridcolor="#1e293b"),
            yaxis=dict(title="Execution Time (ms)", gridcolor="#1e293b"),
            height=280,
            autosize=False
        )

        # 7. Ephemeris and atmospheric delays table
        table_rows = []
        for s in sats:
            if s.get("acquired"):
                mask_str = f"SBF {bin(s.get('subframeMask', 0))[2:].zfill(5)}"
                table_rows.append(html.Tr([
                    html.Td(f"PRN {s['prn']:02d}", style={"padding": "6px 8px", "fontWeight": "600", "color": "#38bdf8"}),
                    html.Td(s.get("stateLabel", "Acquiring"), style={"padding": "6px 8px", "color": "#10b981"}),
                    html.Td(fmt_float(s['cn0'], 1, "dB-Hz"), style={"padding": "6px 8px"}),
                    html.Td(fmt_float(s['doppler'], 1, "Hz"), style={"padding": "6px 8px"}),
                    html.Td(mask_str, style={"padding": "6px 8px", "fontFamily": "monospace", "color": "#10b981"}),
                    html.Td(fmt_float(s['tow'], 0, "s"), style={"padding": "6px 8px"}),
                    html.Td(fmt_float(s['ionoDelay'], 2, "m"), style={"padding": "6px 8px"}),
                    html.Td(fmt_float(s['tropoDelay'], 2, "m"), style={"padding": "6px 8px"}),
                    html.Td(f"{fmt_float(s['azimuth'], 1)}° / {fmt_float(s['elevation'], 1)}°", style={"padding": "6px 8px"})
                ], style={"borderBottom": "1px solid #334155"}))

        ephem_table = html.Table(
            style={"width": "100%", "borderCollapse": "collapse", "fontSize": "11px", "color": "#f8fafc", "tableLayout": "fixed"},
            children=[
                html.Thead(html.Tr([
                    html.Th("PRN", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("State", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("C/N0", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("Doppler", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("Subframes", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("TOW", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("Iono Delay", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("Tropo Delay", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"}),
                    html.Th("Az / El", style={"textAlign": "left", "padding": "6px 8px", "color": "#94a3b8"})
                ])),
                html.Tbody(table_rows if table_rows else [html.Tr(html.Td("Acquiring satellite channels...", colSpan=9, style={"padding": "12px", "color": "#94a3b8"}))])
            ]
        )

        # 8. Logs
        events_text = "\n".join(reversed(events)) if events else "System Initialized. Awaiting software telemetry events..."
        nmea_text = "\n".join(nmea_lines) if nmea_lines else "Waiting for live NMEA telemetry stream..."
        upd_str = f"Updated: {time.strftime('%H:%M:%S', time.localtime(last_t))}" if last_t > 0 else "Waiting for stream..."

        return (
            sat_count_str,
            location_str,
            alt_str,
            dop_str,
            latency_str,
            skyplot_fig,
            cn0_fig,
            doppler_fig,
            tracking_error_fig,
            profiler_fig,
            ephem_table,
            events_text,
            nmea_text,
            upd_str
        )
    except Exception as e:
        empty_fig = go.Figure()
        empty_fig.update_layout(paper_bgcolor="#1e293b", plot_bgcolor="#0b1329", font={"color": "#94a3b8"})
        return "0 / 32", "System Recovering...", "--", "--", "--", empty_fig, empty_fig, empty_fig, empty_fig, empty_fig, html.Div("Re-synchronizing stream..."), f"Notice: {e}", "Re-synchronizing...", f"Updated: {time.strftime('%H:%M:%S')}"

# Main entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPSOpenCl All-Telemetry Dashboard")
    parser.add_argument("--port", type=int, default=8050, help="Web server port")
    parser.add_argument("--endpoint", type=str, default="ipc:///tmp/gpsopencl/telemetry.sock", help="ZMQ PUB socket endpoint")
    parser.add_argument("--log-file", type=str, default="build/telemetry_wire.log", help="Binary telemetry log file")
    args = parser.parse_args()

    # Start background telemetry subscriber
    sub_thread = TelemetrySubscriberThread(args.endpoint, args.log_file)
    sub_thread.start()

    print(f"Starting GPSOpenCl Visual Analytics Dashboard on http://localhost:{args.port}")
    try:
        app.run(host="0.0.0.0", port=args.port, debug=False)
    except AttributeError:
        app.run_server(host="0.0.0.0", port=args.port, debug=False)
