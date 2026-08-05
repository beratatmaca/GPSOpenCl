#!/usr/bin/env python3
# Plotly Dash dashboard: subscribes to GPSOpenCl receiver telemetry over ZMQ or the binary wire log
import argparse
import collections
import datetime
import math
import os
import struct
import threading
import time

import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Dark chart chrome tokens from the validated reference palette
PAGE_BG = "#0d0d0d"
SURFACE = "#1a1a19"
WELL_BG = "#141413"
INK = "#ffffff"
INK_2 = "#c3c2b7"
MUTED = "#898781"
GRID = "#2c2c2a"
BASELINE = "#383835"
BORDER = "1px solid rgba(255,255,255,0.10)"
FONT_STACK = "system-ui, -apple-system, 'Segoe UI', sans-serif"

# Validated dark categorical slots, assigned to a PRN once and never re-ranked
SERIES = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"]

# Status palette: channel/stream state only, never series identity
ST_GOOD = "#0ca30c"
ST_WARN = "#fab219"
ST_SERIOUS = "#ec835a"
ST_CRIT = "#d03b3b"
ST_IDLE = "#4a4a47"

STATE_LABELS = {0: "Idle", 1: "Confirming", 2: "Tracking"}
STATE_COLORS = {0: ST_IDLE, 1: ST_WARN, 2: ST_GOOD}
ACQ_CN0_THRESHOLD = 43.0
BLOCK_DEADLINE_MS = 1.0

# Wall-clock spacing between stored history samples per stream
HISTORY_DECIMATE_SEC = 0.25


def series_color(prn):
    return SERIES[(prn - 1) % len(SERIES)]


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


def stat_new():
    return {"last": 0.0, "count": 0, "mean": 0.0, "max": 0.0}


def stat_update(st, value):
    st["last"] = value
    st["count"] += 1
    st["mean"] += (value - st["mean"]) / st["count"]
    st["max"] = max(st["max"], value)


def channel_new(prn):
    return {
        "prn": prn,
        "acquired": False,
        "state": 0,
        "cn0": 0.0,
        "acqDoppler": 0.0,
        "peakRatio": 0.0,
        "carrierFreqHz": 0.0,
        "codeFreqHz": 0.0,
        "carrierError": 0.0,
        "codeError": 0.0,
        "Ip": 0.0,
        "Qp": 0.0,
        "carrierLock": 0.0,
        "codeLock": 0.0,
        "hasAtmo": False,
        "azimuth": 0.0,
        "elevation": 0.0,
        "ionoDelay": 0.0,
        "tropoDelay": 0.0,
        "hasNav": False,
        "weekNumber": 0,
        "tow": 0.0,
        "subframeMask": 0,
        "toe": 0.0,
        "toc": 0.0,
        "af0": 0.0,
        "e": 0.0,
        "sqrtA": 0.0,
        "acqStat": stat_new(),
        "trkStat": stat_new(),
        "lastTrackSample": 0.0,
    }


GLOBAL_STATE = {
    "source": {"blockIndex": 0, "timestamp": 0.0, "fifoUnderruns": 0, "fifoOverruns": 0},
    "channels": {prn: channel_new(prn) for prn in range(1, 33)},
    "trackHistory": {prn: collections.deque(maxlen=1200) for prn in range(1, 33)},
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
        "vdop": 0.0,
        "satellitesUsed": 0,
        "maxResidualMeters": 0.0,
        "fixCount": 0,
        "firstFixTime": 0.0,
        "lastFixTime": 0.0,
    },
    "pvtHistory": collections.deque(maxlen=2400),
    "pvtLastSample": 0.0,
    "profiler": {
        "blockIndex": 0,
        "timestamp": 0.0,
        "acqMs": 0.0,
        "trackMs": 0.0,
        "navMs": 0.0,
        "pvtMs": 0.0,
        "totalMs": 0.0,
        "elpGenMs": 0.0,
        "ncoMs": 0.0,
        "accumMs": 0.0,
        "maxWorkerMs": 0.0,
    },
    "stageStats": {name: stat_new() for name in ("acq", "track", "nav", "pvt", "total")},
    "profilerHistory": collections.deque(maxlen=1200),
    "profilerLastSample": 0.0,
    "events": [],
    "lastUpdate": 0.0,
}
STATE_LOCK = threading.Lock()


def add_event(level, module, message):
    stamp = time.strftime("%H:%M:%S")
    evt = f"[{stamp}] [{level.upper()}] [{module}] {message}"
    if not GLOBAL_STATE["events"] or GLOBAL_STATE["events"][-1] != evt:
        GLOBAL_STATE["events"].append(evt)
        if len(GLOBAL_STATE["events"]) > 200:
            GLOBAL_STATE["events"].pop(0)


def parse_source(data):
    fields = struct.unpack("<IIdII", data[:24])
    src = GLOBAL_STATE["source"]
    src["blockIndex"] = fields[1]
    src["timestamp"] = safe_float(fields[2])
    src["fifoUnderruns"] = fields[3]
    src["fifoOverruns"] = fields[4]


def parse_acquisition(data):
    if len(data) >= 64:
        fields = struct.unpack("<IiidddddId", data[:64])
        correlate_ms = safe_float(fields[9])
    else:
        fields = struct.unpack("<IiidddddI", data[:56])
        correlate_ms = None
    prn = fields[1]
    if not 1 <= prn <= 32:
        return
    ch = GLOBAL_STATE["channels"][prn]
    was_acquired = ch["acquired"]
    ch["acqDoppler"] = safe_float(fields[4])
    ch["cn0"] = safe_float(fields[6])
    ch["peakRatio"] = safe_float(fields[7])
    ch["acquired"] = bool(fields[8])
    if correlate_ms is not None and correlate_ms > 0.0:
        stat_update(ch["acqStat"], correlate_ms)
    if not was_acquired and ch["acquired"]:
        add_event(
            "INFO",
            "ACQ",
            f"PRN {prn:02d} acquired (C/N0 {fmt_float(ch['cn0'], 1)} dB-Hz, Doppler {fmt_float(ch['acqDoppler'], 1)} Hz)",
        )


def parse_tracking(data, now):
    if len(data) >= 116:
        fields = struct.unpack("<IiddddddddddIddd", data[:116])
        correlator_ms = safe_float(fields[15])
    else:
        fields = struct.unpack("<IiddddddddddIdd", data[:108])
        correlator_ms = None
    prn = fields[1]
    if not 1 <= prn <= 32:
        return
    ch = GLOBAL_STATE["channels"][prn]
    old_state = ch["state"]
    ch["carrierFreqHz"] = safe_float(fields[2])
    ch["codeFreqHz"] = safe_float(fields[3])
    ch["carrierError"] = safe_float(fields[4])
    ch["codeError"] = safe_float(fields[5])
    ch["Ip"] = safe_float(fields[7])
    ch["Qp"] = safe_float(fields[10])
    ch["state"] = fields[12]
    ch["carrierLock"] = safe_float(fields[13])
    ch["codeLock"] = safe_float(fields[14])
    if correlator_ms is not None and correlator_ms > 0.0:
        stat_update(ch["trkStat"], correlator_ms)
    if ch["state"] != old_state:
        add_event("INFO", "TRACK", f"PRN {prn:02d} -> {STATE_LABELS.get(ch['state'], '?')}")
    if now - ch["lastTrackSample"] >= HISTORY_DECIMATE_SEC:
        ch["lastTrackSample"] = now
        GLOBAL_STATE["trackHistory"][prn].append(
            (
                now,
                ch["carrierFreqHz"],
                ch["codeError"],
                ch["carrierLock"],
                ch["codeLock"],
            )
        )


def parse_navdecoder(data):
    fields = struct.unpack("<IiidiI21d", data[:196])
    prn = fields[1]
    if not 1 <= prn <= 32:
        return
    ch = GLOBAL_STATE["channels"][prn]
    ch["hasNav"] = True
    ch["weekNumber"] = fields[2]
    ch["tow"] = safe_float(fields[3])
    subframe_id = fields[4]
    ch["toc"] = safe_float(fields[6])
    ch["af0"] = safe_float(fields[7])
    ch["toe"] = safe_float(fields[11])
    ch["sqrtA"] = safe_float(fields[12])
    ch["e"] = safe_float(fields[13])
    if 1 <= subframe_id <= 5:
        bit = 1 << (subframe_id - 1)
        if not ch["subframeMask"] & bit:
            ch["subframeMask"] |= bit
            add_event("INFO", "NAV", f"PRN {prn:02d} decoded subframe {subframe_id} (TOW {fmt_float(ch['tow'], 0)} s)")


def parse_pvt(data, now):
    if len(data) >= 116:
        fields = struct.unpack("<I12dIId", data[:116])
        sats_used = fields[14]
        max_residual = safe_float(fields[15])
    else:
        fields = struct.unpack("<I12dI", data[:104])
        sats_used = 0
        max_residual = 0.0
    pvt = GLOBAL_STATE["pvt"]
    was_valid = pvt["valid"]
    pvt["ecefX"] = safe_float(fields[1])
    pvt["ecefY"] = safe_float(fields[2])
    pvt["ecefZ"] = safe_float(fields[3])
    pvt["latitude"] = safe_float(fields[4])
    pvt["longitude"] = safe_float(fields[5])
    pvt["altitude"] = safe_float(fields[6])
    pvt["clockBiasMeters"] = safe_float(fields[7])
    pvt["clockBiasSeconds"] = safe_float(fields[8])
    pvt["gdop"] = safe_float(fields[9])
    pvt["pdop"] = safe_float(fields[10])
    pvt["hdop"] = safe_float(fields[11])
    pvt["vdop"] = safe_float(fields[12])
    pvt["valid"] = bool(fields[13])
    pvt["satellitesUsed"] = sats_used
    pvt["maxResidualMeters"] = max_residual
    if pvt["valid"]:
        pvt["fixCount"] += 1
        pvt["lastFixTime"] = now
        if pvt["firstFixTime"] == 0.0:
            pvt["firstFixTime"] = now
            add_event(
                "SUCCESS",
                "PVT",
                f"First fix: {fmt_float(pvt['latitude'], 5)}, {fmt_float(pvt['longitude'], 5)}, "
                f"alt {fmt_float(pvt['altitude'], 1)} m, HDOP {fmt_float(pvt['hdop'], 2)}",
            )
    if not was_valid and pvt["valid"] and pvt["fixCount"] > 1:
        add_event("INFO", "PVT", "Fix re-established")
    if now - GLOBAL_STATE["pvtLastSample"] >= HISTORY_DECIMATE_SEC:
        GLOBAL_STATE["pvtLastSample"] = now
        GLOBAL_STATE["pvtHistory"].append(
            (
                now,
                pvt["valid"],
                pvt["latitude"],
                pvt["longitude"],
                pvt["altitude"],
                pvt["clockBiasMeters"],
                pvt["hdop"],
                pvt["satellitesUsed"],
                pvt["maxResidualMeters"],
            )
        )


def parse_atmospheric(data):
    fields = struct.unpack("<Ii4d", data[:40])
    prn = fields[1]
    if not 1 <= prn <= 32:
        return
    ch = GLOBAL_STATE["channels"][prn]
    ch["hasAtmo"] = True
    ch["ionoDelay"] = safe_float(fields[2])
    ch["tropoDelay"] = safe_float(fields[3])
    ch["azimuth"] = safe_float(fields[4])
    ch["elevation"] = safe_float(fields[5])


def parse_profiler(data, now):
    fields = struct.unpack("<II10d", data[: struct.calcsize("<II10d")])
    prof = GLOBAL_STATE["profiler"]
    prof["blockIndex"] = fields[1]
    prof["timestamp"] = safe_float(fields[2])
    prof["acqMs"] = safe_float(fields[3])
    prof["trackMs"] = safe_float(fields[4])
    prof["navMs"] = safe_float(fields[5])
    prof["pvtMs"] = safe_float(fields[6])
    prof["totalMs"] = safe_float(fields[7])
    prof["elpGenMs"] = safe_float(fields[8])
    prof["ncoMs"] = safe_float(fields[9])
    prof["accumMs"] = safe_float(fields[10])
    prof["maxWorkerMs"] = safe_float(fields[11])
    stats = GLOBAL_STATE["stageStats"]
    stat_update(stats["acq"], prof["acqMs"])
    stat_update(stats["track"], prof["trackMs"])
    stat_update(stats["nav"], prof["navMs"])
    stat_update(stats["pvt"], prof["pvtMs"])
    stat_update(stats["total"], prof["totalMs"])
    if now - GLOBAL_STATE["profilerLastSample"] >= HISTORY_DECIMATE_SEC:
        GLOBAL_STATE["profilerLastSample"] = now
        GLOBAL_STATE["profilerHistory"].append(
            (now, prof["acqMs"], prof["trackMs"], prof["navMs"], prof["pvtMs"], prof["totalMs"], prof["maxWorkerMs"])
        )


# Minimum payload length accepted per topic (version-1 prefix)
MIN_LENGTHS = {
    "SourceOutput": 24,
    "AcquisitionOutput": 56,
    "TrackingOutput": 108,
    "NavDecoderOutput": 196,
    "PvtSolverOutput": 104,
    "AtmosphericOutput": 40,
    "ProfilerOutput": 88,
}


class TelemetrySubscriberThread(threading.Thread):
    def __init__(self, endpoint, log_file):
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.log_file = log_file
        self.running = True
        self.zmq_active = False

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
            print(f"[Subscriber] Listening on ZMQ endpoint: {self.endpoint}")
            while self.running:
                topic = socket.recv_string()
                data = socket.recv()
                self.zmq_active = True
                self.process_msg(topic, data)
        except Exception as exc:
            print(f"[Subscriber] ZMQ unavailable ({exc}); falling back to wire log tail")

    def _run_file_tail(self):
        print(f"[Subscriber] Tailing binary telemetry log: {self.log_file}")
        # Offset of the next unread record, kept across reopens so records are never double-counted
        tail_offset = 0
        corrupt = False
        while self.running:
            if self.zmq_active or not os.path.exists(self.log_file):
                time.sleep(0.5)
                continue
            try:
                size = os.path.getsize(self.log_file)
                # A shrunken file means a new receiver run truncated the log: restart from the top
                if size < tail_offset:
                    tail_offset = 0
                    corrupt = False
                if corrupt or size <= tail_offset:
                    time.sleep(0.5)
                    continue
                with open(self.log_file, "rb") as f:
                    f.seek(tail_offset)
                    while self.running and not self.zmq_active:
                        record_start = f.tell()
                        header = f.read(4)
                        if len(header) < 4:
                            tail_offset = record_start
                            break
                        name_len = struct.unpack("<I", header)[0]
                        if name_len == 0 or name_len > 256:
                            print("[Subscriber] Corrupt record framing in wire log; tail stopped until truncation")
                            corrupt = True
                            break
                        name_bytes = f.read(name_len)
                        len_bytes = f.read(4)
                        if len(name_bytes) < name_len or len(len_bytes) < 4:
                            tail_offset = record_start
                            break
                        data_len = struct.unpack("<I", len_bytes)[0]
                        if data_len > 4096:
                            print("[Subscriber] Corrupt record framing in wire log; tail stopped until truncation")
                            corrupt = True
                            break
                        data = f.read(data_len)
                        if len(data) < data_len:
                            tail_offset = record_start
                            break
                        tail_offset = f.tell()
                        self.process_msg(name_bytes.decode("ascii", errors="ignore"), data)
                time.sleep(0.1)
            except Exception:
                time.sleep(0.5)

    def process_msg(self, topic, data):
        min_len = MIN_LENGTHS.get(topic)
        if min_len is None or len(data) < min_len:
            return
        try:
            with STATE_LOCK:
                now = time.time()
                GLOBAL_STATE["lastUpdate"] = now
                if topic == "SourceOutput":
                    parse_source(data)
                elif topic == "AcquisitionOutput":
                    parse_acquisition(data)
                elif topic == "TrackingOutput":
                    parse_tracking(data, now)
                elif topic == "NavDecoderOutput":
                    parse_navdecoder(data)
                elif topic == "PvtSolverOutput":
                    parse_pvt(data, now)
                elif topic == "AtmosphericOutput":
                    parse_atmospheric(data)
                elif topic == "ProfilerOutput":
                    parse_profiler(data, now)
        except Exception:
            pass


def apply_chrome(fig, height, uirev, showlegend=False):
    fig.update_layout(
        paper_bgcolor=SURFACE,
        plot_bgcolor=SURFACE,
        font=dict(color=INK_2, size=11, family=FONT_STACK),
        margin=dict(l=48, r=12, t=8, b=32),
        height=height,
        uirevision=uirev,
        showlegend=showlegend,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, x=0, bgcolor="rgba(0,0,0,0)"),
    )
    fig.update_xaxes(
        gridcolor=GRID, linecolor=BASELINE, zerolinecolor=BASELINE, tickcolor=BASELINE, tickfont=dict(color=MUTED)
    )
    fig.update_yaxes(
        gridcolor=GRID, linecolor=BASELINE, zerolinecolor=BASELINE, tickcolor=BASELINE, tickfont=dict(color=MUTED)
    )
    return fig


def to_dt(timestamps):
    return [datetime.datetime.fromtimestamp(t) for t in timestamps]


CARD_STYLE = {
    "backgroundColor": SURFACE,
    "padding": "14px",
    "borderRadius": "10px",
    "border": BORDER,
    "overflow": "hidden",
}

TILE_LABEL_STYLE = {"fontSize": "12px", "color": MUTED}
TILE_VALUE_STYLE = {"fontSize": "20px", "fontWeight": "600", "color": INK, "marginTop": "4px", "whiteSpace": "nowrap"}
TILE_VALUE_COMPACT_STYLE = {"fontSize": "14px", "fontWeight": "600", "color": INK, "marginTop": "6px", "lineHeight": "1.35"}
H3_STYLE = {"margin": "0 0 10px 0", "fontSize": "14px", "fontWeight": "600", "color": INK}
TH_STYLE = {"textAlign": "left", "padding": "5px 8px", "color": MUTED, "fontWeight": "500", "whiteSpace": "nowrap"}
TD_STYLE = {"padding": "5px 8px", "color": INK_2, "whiteSpace": "nowrap"}
TD_NUM_STYLE = {**TD_STYLE, "fontVariantNumeric": "tabular-nums"}


def card(title, children, extra_style=None):
    style = dict(CARD_STYLE)
    if extra_style:
        style.update(extra_style)
    return html.Div(style=style, children=[html.H3(title, style=H3_STYLE)] + children)


def stat_tile(label, value_id, compact=False):
    return html.Div(
        style=CARD_STYLE,
        children=[
            html.Div(label, style=TILE_LABEL_STYLE),
            html.Div(id=value_id, children="--", style=TILE_VALUE_COMPACT_STYLE if compact else TILE_VALUE_STYLE),
        ],
    )


def data_table(headers, rows, empty_text):
    if not rows:
        body = [html.Tr(html.Td(empty_text, colSpan=len(headers), style={"padding": "10px", "color": MUTED}))]
    else:
        body = rows
    return html.Table(
        style={"width": "100%", "borderCollapse": "collapse", "fontSize": "11px"},
        children=[
            html.Thead(html.Tr([html.Th(h, style=TH_STYLE) for h in headers])),
            html.Tbody(body),
        ],
    )


def table_row(cells):
    return html.Tr([html.Td(c, style=TD_NUM_STYLE) for c in cells], style={"borderBottom": f"1px solid {GRID}"})


app = dash.Dash(__name__, title="GPSOpenCl Telemetry")

app.layout = html.Div(
    style={
        "backgroundColor": PAGE_BG,
        "color": INK_2,
        "fontFamily": FONT_STACK,
        "minHeight": "100vh",
        "padding": "18px",
        "boxSizing": "border-box",
    },
    children=[
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "backgroundColor": SURFACE,
                "padding": "14px 20px",
                "borderRadius": "10px",
                "border": BORDER,
                "marginBottom": "16px",
            },
            children=[
                html.Div(
                    [
                        html.H1("GPSOpenCl receiver telemetry", style={"margin": 0, "fontSize": "18px", "fontWeight": "600", "color": INK}),
                        html.P("Acquisition, tracking, navigation, PVT and pipeline timing", style={"margin": "3px 0 0 0", "fontSize": "12px", "color": MUTED}),
                    ]
                ),
                html.Div(
                    style={"display": "flex", "gap": "14px", "alignItems": "center"},
                    children=[
                        html.Div(
                            id="status-badge",
                            children="WAITING",
                            style={"backgroundColor": ST_IDLE, "color": INK, "padding": "5px 12px", "borderRadius": "16px", "fontSize": "11px", "fontWeight": "600"},
                        ),
                        html.Div(id="last-update", children="No data yet", style={"fontSize": "12px", "color": MUTED}),
                    ],
                ),
            ],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "repeat(6, 1fr)", "gap": "12px", "marginBottom": "16px"},
            children=[
                stat_tile("Position fix", "tile-fix", compact=True),
                stat_tile("Altitude", "tile-alt"),
                stat_tile("Satellites used / tracked", "tile-sats"),
                stat_tile("HDOP / PDOP", "tile-dop"),
                stat_tile("Block time (mean / WCET)", "tile-block", compact=True),
                stat_tile("Blocks processed", "tile-blocks"),
            ],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "12px", "marginBottom": "16px"},
            children=[
                card("Channel status", [html.Div(id="channel-board")]),
                card("Acquisition C/N0 by channel", [dcc.Graph(id="cn0-graph", config={"displayModeBar": False})]),
            ],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "12px", "marginBottom": "16px"},
            children=[
                card("Skyplot (solved azimuth / elevation)", [dcc.Graph(id="skyplot-graph", config={"displayModeBar": False})]),
                card("Doppler by channel", [dcc.Graph(id="doppler-graph", config={"displayModeBar": False})]),
            ],
        ),
        card(
            "Tracking history",
            [
                html.Div(
                    style={"display": "flex", "gap": "10px", "alignItems": "center", "marginBottom": "8px"},
                    children=[
                        html.Div("Channels", style={"fontSize": "12px", "color": MUTED}),
                        dcc.Dropdown(
                            id="track-prn-select",
                            options=[],
                            value=[],
                            multi=True,
                            persistence=True,
                            placeholder="Tracked channels (default: first four)",
                            style={"minWidth": "360px", "flex": "1", "backgroundColor": WELL_BG, "color": "#111111", "fontSize": "12px"},
                        ),
                    ],
                ),
                dcc.Graph(id="tracking-graph", config={"displayModeBar": False}),
            ],
            {"marginBottom": "16px"},
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "12px", "marginBottom": "16px"},
            children=[
                card("Per-channel compute time (ms)", [html.Div(id="timing-table", style={"overflowX": "auto", "maxHeight": "320px", "overflowY": "auto"})]),
                card(
                    "Application status",
                    [
                        html.Div(id="app-status-table", style={"marginBottom": "10px"}),
                        html.Div(id="stage-table"),
                    ],
                ),
            ],
        ),
        card(
            "Pipeline stage timing",
            [dcc.Graph(id="profiler-graph", config={"displayModeBar": False})],
            {"marginBottom": "16px"},
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 2fr", "gap": "12px", "marginBottom": "16px"},
            children=[
                card("PVT solution", [html.Div(id="pvt-table")]),
                card("PVT history", [dcc.Graph(id="pvt-graph", config={"displayModeBar": False})]),
            ],
        ),
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "2fr 1fr", "gap": "12px", "marginBottom": "16px"},
            children=[
                card("Position fix map", [dcc.Graph(id="map-graph", config={"displayModeBar": False})]),
                card(
                    "Channel details",
                    [html.Div(id="channel-table", style={"overflowX": "auto", "maxHeight": "430px", "overflowY": "auto"})],
                ),
            ],
        ),
        card(
            "Event log",
            [
                html.Pre(
                    id="events-window",
                    style={
                        "backgroundColor": WELL_BG,
                        "color": INK_2,
                        "padding": "10px",
                        "borderRadius": "8px",
                        "fontFamily": "ui-monospace, monospace",
                        "fontSize": "11px",
                        "height": "220px",
                        "overflowY": "auto",
                        "whiteSpace": "pre-wrap",
                        "margin": 0,
                    },
                )
            ],
            {"marginBottom": "16px"},
        ),
        dcc.Interval(id="interval-component", interval=1000, n_intervals=0),
    ],
)


def build_cn0_figure(channels):
    prns = [f"{ch['prn']:02d}" for ch in channels]
    groups = {2: ("Tracking", ST_GOOD), 1: ("Confirming", ST_WARN), 0: ("Idle", ST_IDLE)}
    fig = go.Figure()
    for state, (label, color) in groups.items():
        xs = [f"{ch['prn']:02d}" for ch in channels if ch["state"] == state]
        ys = [max(ch["cn0"], 0.0) for ch in channels if ch["state"] == state]
        if xs:
            fig.add_trace(
                go.Bar(
                    x=xs,
                    y=ys,
                    name=label,
                    marker=dict(color=color, line=dict(width=0)),
                    hovertemplate="PRN %{x}: %{y:.1f} dB-Hz<extra>" + label + "</extra>",
                )
            )
    fig.add_hline(y=ACQ_CN0_THRESHOLD, line=dict(color=MUTED, width=1, dash="dot"))
    fig.add_annotation(
        x=1.0,
        xref="paper",
        y=ACQ_CN0_THRESHOLD,
        text=f"acq threshold {ACQ_CN0_THRESHOLD:.0f}",
        showarrow=False,
        yshift=8,
        xanchor="right",
        font=dict(color=MUTED, size=10),
    )
    apply_chrome(fig, 300, "cn0", showlegend=True)
    fig.update_layout(bargap=0.35, barmode="overlay")
    fig.update_xaxes(categoryorder="array", categoryarray=prns, tickfont=dict(size=9))
    fig.update_yaxes(title_text="C/N0 (dB-Hz)", range=[0, 60], title_font=dict(color=MUTED, size=11))
    return fig


def build_skyplot_figure(channels):
    fig = go.Figure()
    visible = [ch for ch in channels if ch["hasAtmo"] and ch["state"] >= 1]
    for state in (1, 2):
        members = [ch for ch in visible if ch["state"] == state]
        if not members:
            continue
        fig.add_trace(
            go.Scatterpolar(
                r=[90.0 - ch["elevation"] for ch in members],
                theta=[ch["azimuth"] for ch in members],
                mode="markers+text",
                name=STATE_LABELS[state],
                text=[f"{ch['prn']:02d}" for ch in members],
                textposition="top center",
                textfont=dict(color=INK_2, size=10),
                marker=dict(size=10, color=STATE_COLORS[state], line=dict(color=SURFACE, width=2)),
                hovertemplate="PRN %{text}: az %{theta:.1f} deg, el %{customdata:.1f} deg<extra></extra>",
                customdata=[ch["elevation"] for ch in members],
            )
        )
    fig.update_layout(
        polar=dict(
            bgcolor=SURFACE,
            radialaxis=dict(range=[0, 90], showticklabels=False, gridcolor=GRID, linecolor=BASELINE),
            angularaxis=dict(direction="clockwise", rotation=90, gridcolor=GRID, linecolor=BASELINE, tickfont=dict(color=MUTED)),
        ),
    )
    apply_chrome(fig, 320, "skyplot", showlegend=True)
    fig.update_layout(margin=dict(l=30, r=30, t=25, b=25))
    return fig


def build_doppler_figure(channels):
    active = [ch for ch in channels if ch["state"] >= 1 or ch["acquired"]]
    xs = [f"{ch['prn']:02d}" for ch in active]
    ys = [ch["carrierFreqHz"] if ch["state"] >= 1 else ch["acqDoppler"] for ch in active]
    labels = ["tracked" if ch["state"] >= 1 else "acquisition" for ch in active]
    fig = go.Figure(
        go.Bar(
            x=xs,
            y=ys,
            marker=dict(color=SERIES[0], line=dict(width=0)),
            customdata=labels,
            hovertemplate="PRN %{x}: %{y:.1f} Hz (%{customdata})<extra></extra>",
        )
    )
    apply_chrome(fig, 320, "doppler")
    fig.update_layout(bargap=0.35)
    fig.update_yaxes(title_text="Doppler (Hz)", title_font=dict(color=MUTED, size=11))
    return fig


TRACK_PANELS = (
    ("Carrier Doppler (Hz)", 1),
    ("DLL code error (chips)", 2),
    ("Carrier lock indicator", 3),
    ("Code lock ratio", 4),
)


def build_tracking_figure(track_history, selected_prns):
    fig = make_subplots(rows=2, cols=2, subplot_titles=[p[0] for p in TRACK_PANELS], vertical_spacing=0.16, horizontal_spacing=0.08)
    for prn in selected_prns:
        history = list(track_history.get(prn, ()))
        if not history:
            continue
        times = to_dt([s[0] for s in history])
        color = series_color(prn)
        for panel_index, (_, field) in enumerate(TRACK_PANELS):
            fig.add_trace(
                go.Scatter(
                    x=times,
                    y=[s[field] for s in history],
                    mode="lines",
                    name=f"PRN {prn:02d}",
                    legendgroup=f"prn{prn}",
                    showlegend=panel_index == 0,
                    line=dict(color=color, width=2),
                    hovertemplate="PRN " + f"{prn:02d}" + ": %{y:.3f}<extra></extra>",
                ),
                row=panel_index // 2 + 1,
                col=panel_index % 2 + 1,
            )
    apply_chrome(fig, 480, "tracking", showlegend=True)
    fig.update_layout(hovermode="x unified", margin=dict(l=48, r=12, t=40, b=32))
    fig.update_annotations(font=dict(color=INK_2, size=12))
    return fig


def build_profiler_figure(profiler_history):
    history = list(profiler_history)
    fig = go.Figure()
    if history:
        times = to_dt([s[0] for s in history])
        stage_series = (
            ("Total", 5, SERIES[0]),
            ("Tracking", 2, SERIES[1]),
            ("Acquisition", 1, SERIES[2]),
            ("Nav decode", 3, SERIES[3]),
            ("PVT solve", 4, SERIES[4]),
        )
        for name, field, color in stage_series:
            fig.add_trace(
                go.Scatter(
                    x=times,
                    y=[s[field] for s in history],
                    mode="lines",
                    name=name,
                    line=dict(color=color, width=2),
                )
            )
        fig.add_hline(y=BLOCK_DEADLINE_MS, line=dict(color=ST_CRIT, width=1, dash="dot"))
        fig.add_annotation(
            x=1.0,
            xref="paper",
            y=BLOCK_DEADLINE_MS,
            text="1 ms real-time deadline",
            showarrow=False,
            yshift=8,
            xanchor="right",
            font=dict(color=ST_CRIT, size=10),
        )
    apply_chrome(fig, 300, "profiler", showlegend=True)
    fig.update_layout(hovermode="x unified")
    fig.update_yaxes(title_text="Stage time (ms)", title_font=dict(color=MUTED, size=11))
    return fig


PVT_PANELS = (
    ("Altitude (m)", 4),
    ("Clock bias (m)", 5),
    ("Satellites used", 7),
    ("Max residual (m)", 8),
)


def build_pvt_figure(pvt_history):
    fig = make_subplots(rows=2, cols=2, subplot_titles=[p[0] for p in PVT_PANELS], vertical_spacing=0.16, horizontal_spacing=0.08)
    history = [s for s in pvt_history if s[1]]
    if history:
        times = to_dt([s[0] for s in history])
        for panel_index, (_, field) in enumerate(PVT_PANELS):
            fig.add_trace(
                go.Scatter(
                    x=times,
                    y=[s[field] for s in history],
                    mode="lines",
                    line=dict(color=SERIES[0], width=2),
                    hovertemplate="%{y:.2f}<extra></extra>",
                ),
                row=panel_index // 2 + 1,
                col=panel_index % 2 + 1,
            )
    apply_chrome(fig, 380, "pvt")
    fig.update_layout(hovermode="x unified", margin=dict(l=48, r=12, t=40, b=32))
    fig.update_annotations(font=dict(color=INK_2, size=12))
    return fig


def build_map_figure(pvt, pvt_history):
    fixes = [s for s in pvt_history if s[1]]
    lats = [s[2] for s in fixes]
    lons = [s[3] for s in fixes]
    has_fix = bool(fixes)
    use_maplibre = hasattr(go, "Scattermap")
    trace_cls = go.Scattermap if use_maplibre else go.Scattermapbox
    fig = go.Figure()
    if has_fix:
        fig.add_trace(
            trace_cls(
                lat=lats,
                lon=lons,
                mode="lines",
                name="Fix trail",
                line=dict(color=SERIES[0], width=2),
                hoverinfo="skip",
            )
        )
        fig.add_trace(
            trace_cls(
                lat=[lats[-1]],
                lon=[lons[-1]],
                mode="markers",
                name="Current fix",
                marker=dict(color=ST_GOOD, size=14),
                hovertemplate="%{lat:.6f}, %{lon:.6f}<extra>Current fix</extra>",
            )
        )
    center = dict(lat=lats[-1] if has_fix else 0.0, lon=lons[-1] if has_fix else 0.0)
    map_config = dict(style="open-street-map", center=center, zoom=15 if has_fix else 1)
    layout = dict(
        paper_bgcolor=SURFACE,
        font=dict(color=INK_2, size=11, family=FONT_STACK),
        margin=dict(l=0, r=0, t=0, b=0),
        height=430,
        uirevision=f"map-{has_fix}",
        showlegend=has_fix,
        legend=dict(orientation="h", yanchor="bottom", y=1.02, x=0, bgcolor="rgba(0,0,0,0)"),
    )
    if use_maplibre:
        layout["map"] = map_config
    else:
        layout["mapbox"] = map_config
    fig.update_layout(**layout)
    return fig


def build_channel_board(channels):
    chips = []
    for ch in channels:
        color = STATE_COLORS.get(ch["state"], ST_IDLE)
        label = STATE_LABELS.get(ch["state"], "?")
        cn0_text = fmt_float(ch["cn0"], 1) if ch["cn0"] > 0 else "--"
        chips.append(
            html.Div(
                style={
                    "backgroundColor": WELL_BG,
                    "border": f"1px solid {GRID}",
                    "borderRadius": "8px",
                    "padding": "6px 4px",
                    "textAlign": "center",
                },
                children=[
                    html.Div(
                        style={"display": "flex", "alignItems": "center", "justifyContent": "center", "gap": "4px"},
                        children=[
                            html.Span(style={"width": "8px", "height": "8px", "borderRadius": "50%", "backgroundColor": color, "display": "inline-block"}),
                            html.Span(f"{ch['prn']:02d}", style={"fontSize": "12px", "fontWeight": "600", "color": INK}),
                        ],
                    ),
                    html.Div(label, style={"fontSize": "10px", "color": MUTED, "marginTop": "2px"}),
                    html.Div(cn0_text, style={"fontSize": "10px", "color": INK_2, "fontVariantNumeric": "tabular-nums"}),
                ],
            )
        )
    return html.Div(style={"display": "grid", "gridTemplateColumns": "repeat(8, 1fr)", "gap": "6px"}, children=chips)


def build_timing_table(channels):
    rows = []
    for ch in channels:
        acq = ch["acqStat"]
        trk = ch["trkStat"]
        if acq["count"] == 0 and trk["count"] == 0:
            continue
        rows.append(
            table_row(
                [
                    f"{ch['prn']:02d}",
                    STATE_LABELS.get(ch["state"], "?"),
                    str(acq["count"]),
                    fmt_float(acq["last"], 2) if acq["count"] else "--",
                    fmt_float(acq["mean"], 2) if acq["count"] else "--",
                    fmt_float(acq["max"], 2) if acq["count"] else "--",
                    str(trk["count"]),
                    fmt_float(trk["last"], 4) if trk["count"] else "--",
                    fmt_float(trk["mean"], 4) if trk["count"] else "--",
                    fmt_float(trk["max"], 4) if trk["count"] else "--",
                ]
            )
        )
    headers = ["PRN", "State", "Acq runs", "Acq last", "Acq mean", "Acq WCET", "Trk blocks", "Trk last", "Trk mean", "Trk WCET"]
    return data_table(headers, rows, "No per-channel timing received yet (requires struct v2 telemetry)")


def build_stage_table(stage_stats):
    labels = (("total", "Total block"), ("track", "Tracking"), ("acq", "Acquisition"), ("nav", "Nav decode"), ("pvt", "PVT solve"))
    rows = []
    for key, label in labels:
        st = stage_stats[key]
        if st["count"] == 0:
            continue
        rows.append(table_row([label, fmt_float(st["last"], 3), fmt_float(st["mean"], 3), fmt_float(st["max"], 3)]))
    return data_table(["Stage", "Last (ms)", "Nominal (ms)", "WCET (ms)"], rows, "No profiler telemetry yet")


def build_app_status_table(source, profiler, stage_stats, channels, data_age):
    tracked = sum(1 for ch in channels if ch["state"] == 2)
    confirming = sum(1 for ch in channels if ch["state"] == 1)
    total_stat = stage_stats["total"]
    margin_text = "--"
    if total_stat["count"]:
        margin = (1.0 - total_stat["mean"] / BLOCK_DEADLINE_MS) * 100.0
        margin_text = f"{margin:.1f} % of the 1 ms block budget"
    imbalance_text = "--"
    if profiler["trackMs"] > 0.0 and profiler["maxWorkerMs"] > 0.0:
        imbalance_text = f"{profiler['trackMs'] - profiler['maxWorkerMs']:.3f} ms barrier overhead"
    rows = [
        ("Stream", "live" if data_age < 3.0 else f"stale ({data_age:.0f} s since last message)"),
        ("Block index", f"{source['blockIndex']}"),
        ("Stream time", fmt_float(source["timestamp"], 2, "s")),
        ("FIFO underruns / overruns", f"{source['fifoUnderruns']} / {source['fifoOverruns']}"),
        ("Channels tracking / confirming", f"{tracked} / {confirming}"),
        ("Real-time margin", margin_text),
        ("Tracking worker imbalance", imbalance_text),
        ("Correlator / NCO / accumulate", f"{fmt_float(profiler['elpGenMs'], 3)} / {fmt_float(profiler['ncoMs'], 3)} / {fmt_float(profiler['accumMs'], 3)} ms"),
    ]
    body = [
        html.Tr(
            [html.Td(k, style={**TD_STYLE, "color": MUTED}), html.Td(v, style=TD_NUM_STYLE)],
            style={"borderBottom": f"1px solid {GRID}"},
        )
        for k, v in rows
    ]
    return html.Table(style={"width": "100%", "borderCollapse": "collapse", "fontSize": "11px"}, children=[html.Tbody(body)])


def build_pvt_table(pvt, fix_availability):
    rows = [
        ("Fix valid", "yes" if pvt["valid"] else "no"),
        ("Latitude", fmt_float(pvt["latitude"], 6, "deg")),
        ("Longitude", fmt_float(pvt["longitude"], 6, "deg")),
        ("Altitude", fmt_float(pvt["altitude"], 2, "m")),
        ("ECEF X / Y / Z", f"{fmt_float(pvt['ecefX'], 1)} / {fmt_float(pvt['ecefY'], 1)} / {fmt_float(pvt['ecefZ'], 1)} m"),
        ("Clock bias", f"{fmt_float(pvt['clockBiasMeters'], 2)} m ({fmt_float(pvt['clockBiasSeconds'] * 1e6, 3)} us)"),
        ("GDOP / PDOP", f"{fmt_float(pvt['gdop'], 2)} / {fmt_float(pvt['pdop'], 2)}"),
        ("HDOP / VDOP", f"{fmt_float(pvt['hdop'], 2)} / {fmt_float(pvt['vdop'], 2)}"),
        ("Satellites used", str(pvt["satellitesUsed"]) if pvt["satellitesUsed"] else "--"),
        ("Max residual", fmt_float(pvt["maxResidualMeters"], 2, "m") if pvt["satellitesUsed"] else "--"),
        ("Fix count", str(pvt["fixCount"])),
        ("Fix availability (last minute)", f"{fix_availability:.0f} %" if fix_availability >= 0 else "--"),
        ("First fix", time.strftime("%H:%M:%S", time.localtime(pvt["firstFixTime"])) if pvt["firstFixTime"] else "--"),
    ]
    body = [
        html.Tr(
            [html.Td(k, style={**TD_STYLE, "color": MUTED}), html.Td(v, style=TD_NUM_STYLE)],
            style={"borderBottom": f"1px solid {GRID}"},
        )
        for k, v in rows
    ]
    return html.Table(style={"width": "100%", "borderCollapse": "collapse", "fontSize": "11px"}, children=[html.Tbody(body)])


def build_channel_table(channels):
    rows = []
    for ch in channels:
        if ch["state"] == 0 and not ch["hasNav"]:
            continue
        subframes = f"{ch['subframeMask']:05b}" if ch["hasNav"] else "--"
        azel = f"{fmt_float(ch['azimuth'], 0)} / {fmt_float(ch['elevation'], 0)}" if ch["hasAtmo"] else "--"
        atmo = f"{fmt_float(ch['ionoDelay'], 1)} / {fmt_float(ch['tropoDelay'], 1)}" if ch["hasAtmo"] else "--"
        rows.append(
            table_row(
                [
                    f"{ch['prn']:02d}",
                    STATE_LABELS.get(ch["state"], "?"),
                    fmt_float(ch["cn0"], 1),
                    fmt_float(ch["carrierFreqHz"], 1),
                    fmt_float(ch["carrierLock"], 2),
                    fmt_float(ch["codeLock"], 2),
                    subframes,
                    fmt_float(ch["tow"], 0) if ch["hasNav"] else "--",
                    azel,
                    atmo,
                ]
            )
        )
    headers = ["PRN", "State", "C/N0", "Doppler", "Carrier lock", "Code lock", "Subframes", "TOW", "Az / El", "Iono / Tropo (m)"]
    return data_table(headers, rows, "No active channels yet")


@app.callback(
    [
        Output("status-badge", "children"),
        Output("status-badge", "style"),
        Output("last-update", "children"),
        Output("tile-fix", "children"),
        Output("tile-alt", "children"),
        Output("tile-sats", "children"),
        Output("tile-dop", "children"),
        Output("tile-block", "children"),
        Output("tile-blocks", "children"),
        Output("channel-board", "children"),
        Output("cn0-graph", "figure"),
        Output("skyplot-graph", "figure"),
        Output("doppler-graph", "figure"),
        Output("track-prn-select", "options"),
        Output("tracking-graph", "figure"),
        Output("timing-table", "children"),
        Output("app-status-table", "children"),
        Output("stage-table", "children"),
        Output("profiler-graph", "figure"),
        Output("pvt-table", "children"),
        Output("pvt-graph", "figure"),
        Output("map-graph", "figure"),
        Output("channel-table", "children"),
        Output("events-window", "children"),
    ],
    [Input("interval-component", "n_intervals"), Input("track-prn-select", "value")],
)
def update_dashboard(_, selected_prns):
    with STATE_LOCK:
        channels = [dict(ch, acqStat=dict(ch["acqStat"]), trkStat=dict(ch["trkStat"])) for ch in GLOBAL_STATE["channels"].values()]
        track_history = {prn: list(dq) for prn, dq in GLOBAL_STATE["trackHistory"].items() if dq}
        pvt = dict(GLOBAL_STATE["pvt"])
        pvt_history = list(GLOBAL_STATE["pvtHistory"])
        profiler = dict(GLOBAL_STATE["profiler"])
        stage_stats = {k: dict(v) for k, v in GLOBAL_STATE["stageStats"].items()}
        profiler_history = list(GLOBAL_STATE["profilerHistory"])
        source = dict(GLOBAL_STATE["source"])
        events = list(GLOBAL_STATE["events"])
        last_update = GLOBAL_STATE["lastUpdate"]

    now = time.time()
    data_age = now - last_update if last_update > 0 else float("inf")
    if last_update == 0:
        badge_text, badge_color = "WAITING", ST_IDLE
    elif data_age < 3.0:
        badge_text, badge_color = "LIVE", ST_GOOD
    elif data_age < 15.0:
        badge_text, badge_color = "STALE", ST_WARN
    else:
        badge_text, badge_color = "OFFLINE", ST_CRIT
    badge_style = {"backgroundColor": badge_color, "color": INK, "padding": "5px 12px", "borderRadius": "16px", "fontSize": "11px", "fontWeight": "600"}
    update_text = f"Last message {time.strftime('%H:%M:%S', time.localtime(last_update))}" if last_update else "No data yet"

    if pvt["valid"]:
        lat, lon = pvt["latitude"], pvt["longitude"]
        fix_text = f"{abs(lat):.5f} {'N' if lat >= 0 else 'S'}, {abs(lon):.5f} {'E' if lon >= 0 else 'W'}"
        alt_text = fmt_float(pvt["altitude"], 1, "m")
        dop_text = f"{fmt_float(pvt['hdop'], 2)} / {fmt_float(pvt['pdop'], 2)}"
    else:
        fix_text = "No fix"
        alt_text = "--"
        dop_text = "--"
    tracked = sum(1 for ch in channels if ch["state"] == 2)
    sats_text = f"{pvt['satellitesUsed'] if pvt['valid'] else 0} / {tracked}"
    total_stat = stage_stats["total"]
    block_text = f"{fmt_float(total_stat['mean'], 3)} / {fmt_float(total_stat['max'], 3)} ms" if total_stat["count"] else "--"
    blocks_text = f"{source['blockIndex']:,}" if source["blockIndex"] else "--"

    tracked_prns = sorted(track_history.keys())
    options = [{"label": f"PRN {prn:02d}", "value": prn} for prn in tracked_prns]
    shown_prns = [p for p in (selected_prns or []) if p in track_history] or tracked_prns[:4]

    recent = [s for s in pvt_history if now - s[0] <= 60.0]
    fix_availability = 100.0 * sum(1 for s in recent if s[1]) / len(recent) if recent else -1.0

    return (
        badge_text,
        badge_style,
        update_text,
        fix_text,
        alt_text,
        sats_text,
        dop_text,
        block_text,
        blocks_text,
        build_channel_board(channels),
        build_cn0_figure(channels),
        build_skyplot_figure(channels),
        build_doppler_figure(channels),
        options,
        build_tracking_figure(track_history, shown_prns),
        build_timing_table(channels),
        build_app_status_table(source, profiler, stage_stats, channels, data_age),
        build_stage_table(stage_stats),
        build_profiler_figure(profiler_history),
        build_pvt_table(pvt, fix_availability),
        build_pvt_figure(pvt_history),
        build_map_figure(pvt, pvt_history),
        build_channel_table(channels),
        "\n".join(reversed(events)) if events else "Waiting for telemetry events...",
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GPSOpenCl telemetry dashboard")
    parser.add_argument("--port", type=int, default=8050, help="Web server port")
    parser.add_argument("--endpoint", type=str, default="ipc:///tmp/gpsopencl/telemetry.sock", help="ZMQ PUB socket endpoint")
    parser.add_argument("--log-file", type=str, default="build/telemetry_wire.log", help="Binary telemetry log file")
    args = parser.parse_args()

    subscriber = TelemetrySubscriberThread(args.endpoint, args.log_file)
    subscriber.start()

    print(f"GPSOpenCl dashboard on http://localhost:{args.port}")
    try:
        app.run(host="0.0.0.0", port=args.port, debug=False)
    except AttributeError:
        app.run_server(host="0.0.0.0", port=args.port, debug=False)
