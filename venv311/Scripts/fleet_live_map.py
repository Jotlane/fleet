#!/usr/bin/env python3
"""
Fleet Live Map (Folium) for ArduPilot Rovers (boats)
- Connects to multiple MAVLink endpoints (same JSON format as the coordinator).
- Renders an auto-refreshing HTML map with markers, trails, and optional separation rings.
- Can also draw leader→follower lines if you provide simple "groups".

Install:
  pip install dronekit pymavlink folium

Config JSON example (fleet_map.json):
{
  "map_center": [1.3000, 103.8000],
  "zoom": 14,
  "refresh_s": 2.0,
  "separation_m": 10.0,
  "vehicles": [
    {"name":"BoatA","endpoint":"udp:192.168.10.41:14550","color":"red","group":"G1","role":"leader"},
    {"name":"BoatB","endpoint":"udp:192.168.10.42:14550","color":"blue","group":"G1","role":"follower"},
    {"name":"BoatC","endpoint":"udp:192.168.10.43:14550","color":"green","group":"G2","role":"leader"},
    {"name":"BoatD","endpoint":"udp:192.168.10.44:14550","color":"orange","group":"G2","role":"follower"},
    {"name":"BoatE","endpoint":"udp:192.168.10.45:14550","color":"purple","group":"G3","role":"solo"}
  ]
}

Run:
  python fleet_live_map.py --config fleet_map.json --out fleet_map.html
Open the HTML in a browser; it auto-refreshes.
"""
import argparse
import json
import time
import math
import webbrowser
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import folium
from folium.plugins import MeasureControl

from dronekit import connect

@dataclass
class VInfo:
    name: str
    endpoint: str
    color: str = "blue"
    group: str = ""
    role: str = "solo"

def to_tuple(v):
    return (float(v[0]), float(v[1]))

def create_map_html(center_lat, center_lon, zoom, positions, colors, sep_m, groups, out_html, refresh_s):
    m = folium.Map(location=[center_lat, center_lon], zoom_start=zoom, control_scale=True, prefer_canvas=True)
    m.add_child(MeasureControl())

    # meta refresh
    m.get_root().header.add_child(folium.Element(f'<meta http-equiv="refresh" content="{max(1.0, refresh_s):.0f}">'))

    # draw markers
    for name, (lat, lon) in positions.items():
        color = colors.get(name, "blue")
        folium.CircleMarker(
            location=[lat, lon],
            radius=6,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=0.9,
            popup=f"{name}"
        ).add_to(m)

        # separation ring
        if sep_m > 0:
            folium.Circle(
                location=[lat, lon],
                radius=sep_m,
                color=color,
                weight=1,
                fill=False
            ).add_to(m)

    # draw leader->follower lines per group
    # Expect groups as {group_name: {"leader": "BoatA", "followers": ["BoatB","BoatD"]}}
    for gname, ginfo in groups.items():
        leader = ginfo.get("leader")
        if not leader or leader not in positions:
            continue
        for foll in ginfo.get("followers", []):
            if foll in positions:
                lat1, lon1 = positions[leader]
                lat2, lon2 = positions[foll]
                folium.PolyLine(locations=[[lat1, lon1], [lat2, lon2]], color="black", weight=2, opacity=0.6).add_to(m)

    m.save(out_html)

def main():
    ap = argparse.ArgumentParser(description="Fleet Live Map (Folium)")
    ap.add_argument("--config", required=True, help="Path to JSON config (see docstring example)")
    ap.add_argument("--out", default="fleet_map.html", help="Output HTML file")
    ap.add_argument("--open", action="store_true", help="Open the map in default browser")
    args = ap.parse_args()

    with open(args.config, "r") as f:
        cfg = json.load(f)

    center = cfg.get("map_center", [1.3000, 103.8000])
    zoom = int(cfg.get("zoom", 14))
    refresh_s = float(cfg.get("refresh_s", 2.0))
    sep_m = float(cfg.get("separation_m", 0.0))

    vinfos = {}
    links = {}
    groups = {}

    # Build vehicle list
    for v in cfg.get("vehicles", []):
        info = VInfo(
            name=v["name"],
            endpoint=v["endpoint"],
            color=v.get("color", "blue"),
            group=v.get("group", ""),
            role=v.get("role", "solo")
        )
        vinfos[info.name] = info

    # Build groups (leader->followers) for polylines
    for g in set([vi.group for vi in vinfos.values() if vi.group]):
        leader = None
        followers = []
        for vi in vinfos.values():
            if vi.group != g: continue
            if vi.role == "leader" and leader is None:
                leader = vi.name
            elif vi.role == "follower":
                followers.append(vi.name)
        if leader:
            groups[g] = {"leader": leader, "followers": followers}

    # Connect
    src = 230
    for name, vi in vinfos.items():
        print(f"Connecting {name} @ {vi.endpoint}")
        link = connect(vi.endpoint, wait_ready=True, baud=57600, source_system=src)
        src += 1
        links[name] = link

    # Initial write & open
    positions = {name: (1.3000, 103.8000) for name in vinfos.keys()}  # default
    colors = {name: vi.color for name, vi in vinfos.items()}
    create_map_html(center[0], center[1], zoom, positions, colors, sep_m, groups, args.out, refresh_s)
    if args.open:
        webbrowser.open(f"file://{args.out}")

    # Loop
    print(f"Running. Updating map every {refresh_s}s…")
    while True:
        # Read the latest positions
        any_fix = False
        for name, link in links.items():
            loc = link.location.global_frame
            if loc.lat is not None and loc.lon is not None:
                positions[name] = (float(loc.lat), float(loc.lon))
                any_fix = True

        # Recenter to average when we have fixes
        if any_fix:
            lat_avg = sum(lat for lat, _ in positions.values()) / len(positions)
            lon_avg = sum(lon for _, lon in positions.values()) / len(positions)
            ctr_lat, ctr_lon = lat_avg, lon_avg
        else:
            ctr_lat, ctr_lon = center[0], center[1]

        create_map_html(ctr_lat, ctr_lon, zoom, positions, colors, sep_m, groups, args.out, refresh_s)
        time.sleep(max(1.0, refresh_s))

if __name__ == "__main__":
    main()
