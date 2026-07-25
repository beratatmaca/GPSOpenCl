#!/usr/bin/env python3
"""
GPSOpenCl Plotly/Dash Interactive Web Dashboard
Provides real-time visual analytics, skyplots, signal C/N0 charts, PVT position tracking, and NMEA logs.
"""

import json
import os
import sys
import time

import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go

app = dash.Dash(__name__, title="GPSOpenCl - Real-Time GNSS Dashboard")

TELEMETRY_PATH = "telemetry_stream.json"

app.layout = html.Div(
    style={
        "backgroundColor": "#0f172a",
        "color": "#f8fafc",
        "fontFamily": "'Inter', 'Segoe UI', sans-serif",
        "minHeight": "100vh",
        "padding": "20px"
    },
    children=[
        # Header Bar
        html.Div(
            style={
                "display": "flex",
                "justifyContent": "space-between",
                "alignItems": "center",
                "backgroundColor": "#1e293b",
                "padding": "16px 24px",
                "borderRadius": "12px",
                "boxShadow": "0 4px 12px rgba(0,0,0,0.3)",
                "marginBottom": "20px"
            },
            children=[
                html.Div([
                    html.H1("GPSOpenCl - GNSS Receiver Visual Analytics", style={"margin": 0, "fontSize": "24px", "fontWeight": "700", "color": "#38bdf8"}),
                    html.P("OpenCL Accelerated Multi-Channel Satellite Acquisition, Tracking & PVT Pipeline", style={"margin": "4px 0 0 0", "fontSize": "13px", "color": "#94a3b8"})
                ]),
                html.Div(
                    style={"display": "flex", "gap": "12px", "alignItems": "center"},
                    children=[
                        html.Div(id="status-badge", children="ONLINE", style={"backgroundColor": "#059669", "color": "#ffffff", "padding": "6px 14px", "borderRadius": "20px", "fontSize": "12px", "fontWeight": "600"}),
                        html.Div(id="last-update", children="Updated: Just now", style={"fontSize": "12px", "color": "#94a3b8"})
                    ]
                )
            ]
        ),

        # Top KPI Metric Cards
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "repeat(4, 1fr)", "gap": "16px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "borderLeft": "4px solid #10b981"},
                    children=[
                        html.Div("Acquired Satellites", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-sat-count", children="0 / 32", style={"fontSize": "26px", "fontWeight": "700", "color": "#10b981", "marginTop": "4px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "borderLeft": "4px solid #38bdf8"},
                    children=[
                        html.Div("Receiver WGS-84 Location", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-location", children="--", style={"fontSize": "15px", "fontWeight": "600", "color": "#38bdf8", "marginTop": "6px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "borderLeft": "4px solid #f59e0b"},
                    children=[
                        html.Div("Receiver Altitude", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-altitude", children="--", style={"fontSize": "24px", "fontWeight": "700", "color": "#f59e0b", "marginTop": "4px"})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "10px", "borderLeft": "4px solid #a855f7"},
                    children=[
                        html.Div("Dilution of Precision (HDOP/PDOP)", style={"fontSize": "12px", "color": "#94a3b8"}),
                        html.Div(id="card-dop", children="--", style={"fontSize": "22px", "fontWeight": "700", "color": "#a855f7", "marginTop": "4px"})
                    ]
                )
            ]
        ),

        # Charts Section Row 1: Skyplot & C/N0 Bar Chart
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "20px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px"},
                    children=[
                        html.H3("Satellite Skyplot (Azimuth & Elevation)", style={"margin": "0 0 12px 0", "fontSize": "16px", "color": "#f8fafc"}),
                        dcc.Graph(id="skyplot-graph", config={"displayModeBar": False})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px"},
                    children=[
                        html.H3("Signal Carrier-to-Noise Ratio (C/N0 dB-Hz)", style={"margin": "0 0 12px 0", "fontSize": "16px", "color": "#f8fafc"}),
                        dcc.Graph(id="cn0-graph", config={"displayModeBar": False})
                    ]
                )
            ]
        ),

        # Charts Section Row 2: Doppler & ECEF Coordinates
        html.Div(
            style={"display": "grid", "gridTemplateColumns": "1fr 1fr", "gap": "20px", "marginBottom": "20px"},
            children=[
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px"},
                    children=[
                        html.H3("Doppler Shift per Satellite Channel (Hz)", style={"margin": "0 0 12px 0", "fontSize": "16px", "color": "#f8fafc"}),
                        dcc.Graph(id="doppler-graph", config={"displayModeBar": False})
                    ]
                ),
                html.Div(
                    style={"backgroundColor": "#1e293b", "padding": "16px", "borderRadius": "12px"},
                    children=[
                        html.H3("Real-Time NMEA-0183 Terminal Log Stream", style={"margin": "0 0 12px 0", "fontSize": "16px", "color": "#f8fafc"}),
                        html.Pre(
                            id="nmea-log-window",
                            style={
                                "backgroundColor": "#090d16",
                                "color": "#38bdf8",
                                "padding": "14px",
                                "borderRadius": "8px",
                                "fontFamily": "'JetBrains Mono', 'Fira Code', monospace",
                                "fontSize": "12px",
                                "height": "370px",
                                "overflowY": "auto",
                                "border": "1px solid #334155"
                            }
                        )
                    ]
                )
            ]
        ),

        # Live Update Interval Timer
        dcc.Interval(id="interval-component", interval=1000, n_intervals=0)
    ]
)

@app.callback(
    [
        Output("card-sat-count", "children"),
        Output("card-location", "children"),
        Output("card-altitude", "children"),
        Output("card-dop", "children"),
        Output("skyplot-graph", "figure"),
        Output("cn0-graph", "figure"),
        Output("doppler-graph", "figure"),
        Output("nmea-log-window", "children"),
        Output("last-update", "children")
    ],
    [Input("interval-component", "n_intervals")]
)
def update_dashboard(n):
    data = None
    if os.path.exists(TELEMETRY_PATH):
        try:
            with open(TELEMETRY_PATH, "r") as f:
                data = json.load(f)
        except Exception:
            pass

    if not data:
        empty_fig = go.Figure()
        empty_fig.update_layout(paper_bgcolor="#1e293b", plot_bgcolor="#1e293b", font={"color": "#94a3b8"})
        return "0 / 32", "Waiting for data...", "--", "--", empty_fig, empty_fig, empty_fig, "Waiting for stream...", f"Updated: {time.strftime('%H:%M:%S')}"

    # Extract Metrics
    satellites = data.get("satellites", [])
    pvt = data.get("pvt", {})
    nmea = data.get("nmea", [])

    acquired_count = sum(1 for s in satellites if s.get("acquired"))
    sat_count_str = f"{acquired_count} / {len(satellites)}"

    lat = pvt.get("latitude", 0.0)
    lon = pvt.get("longitude", 0.0)
    alt = pvt.get("altitude", 0.0)
    location_str = f"{abs(lat):.4f}°{'N' if lat>=0 else 'S'}, {abs(lon):.4f}°{'E' if lon>=0 else 'W'}"
    alt_str = f"{alt:.1f} m"

    hdop = pvt.get("hdop", 0.0)
    pdop = pvt.get("pdop", 0.0)
    dop_str = f"HDOP: {hdop:.2f} | PDOP: {pdop:.2f}"

    # Skyplot Figure
    r_vals = [90 - s.get("elevation", 0) for s in satellites if s.get("acquired")]
    theta_vals = [s.get("azimuth", 0) for s in satellites if s.get("acquired")]
    labels = [f"PRN {s.get('prn'):02d}" for s in satellites if s.get("acquired")]

    skyplot_fig = go.Figure(go.Scatterpolar(
        r=r_vals,
        theta=theta_vals,
        mode="markers+text",
        text=labels,
        textposition="top center",
        marker=dict(size=12, color="#38bdf8", line=dict(color="#0284c7", width=2))
    ))
    skyplot_fig.update_layout(
        polar=dict(
            radialaxis=dict(visible=True, range=[0, 90], showticklabels=False, color="#475569"),
            angularaxis=dict(direction="clockwise", color="#94a3b8"),
            bgcolor="#0f172a"
        ),
        paper_bgcolor="#1e293b",
        margin=dict(l=30, r=30, t=20, b=20),
        font=dict(color="#f8fafc", size=11),
        height=380
    )

    # C/N0 Bar Chart
    prns = [f"P{s.get('prn'):02d}" for s in satellites]
    cn0s = [s.get("cn0", 0.0) for s in satellites]
    colors = ["#10b981" if c >= 40 else "#f59e0b" if c >= 35 else "#ef4444" for c in cn0s]

    cn0_fig = go.Figure(go.Bar(x=prns, y=cn0s, marker_color=colors))
    cn0_fig.update_layout(
        paper_bgcolor="#1e293b",
        plot_bgcolor="#0f172a",
        margin=dict(l=30, r=20, t=20, b=40),
        font=dict(color="#f8fafc", size=10),
        xaxis=dict(title="Satellite PRN Channel", gridcolor="#334155"),
        yaxis=dict(title="C/N0 (dB-Hz)", range=[0, 60], gridcolor="#334155"),
        height=380
    )

    # Doppler Shift Figure
    dopplers = [s.get("doppler", 0.0) for s in satellites]
    doppler_fig = go.Figure(go.Bar(x=prns, y=dopplers, marker_color="#a855f7"))
    doppler_fig.update_layout(
        paper_bgcolor="#1e293b",
        plot_bgcolor="#0f172a",
        margin=dict(l=30, r=20, t=20, b=40),
        font=dict(color="#f8fafc", size=10),
        xaxis=dict(title="Satellite PRN Channel", gridcolor="#334155"),
        yaxis=dict(title="Doppler Shift (Hz)", gridcolor="#334155"),
        height=370
    )

    # NMEA Terminal Text
    nmea_text = "\n".join(nmea)

    return (
        sat_count_str,
        location_str,
        alt_str,
        dop_str,
        skyplot_fig,
        cn0_fig,
        doppler_fig,
        nmea_text,
        f"Updated: {time.strftime('%H:%M:%S')}"
    )

if __name__ == "__main__":
    port = 8050
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    print(f"Starting GPSOpenCl Plotly/Dash Dashboard on http://localhost:{port}")
    
    try:
        app.run(host="0.0.0.0", port=port, debug=False)
    except AttributeError:
        app.run_server(host="0.0.0.0", port=port, debug=False)
