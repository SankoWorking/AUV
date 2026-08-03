#!/usr/bin/env python3
"""Receive AUV NAV logs from a serial port and plot the trajectory."""

from __future__ import annotations

import argparse
import csv
import queue
import re
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:  # pragma: no cover
    plt = None
    FuncAnimation = None


NAV_RE = re.compile(r"\[NAV\](?P<body>.*)")
FIELD_RE = re.compile(
    r"(?P<key>[A-Za-z_]+):\s*"
    r"(?P<value>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
)


@dataclass(frozen=True)
class NavSample:
    received_s: float
    x_m: float
    y_m: float
    vn_mps: float = 0.0
    ve_mps: float = 0.0
    body_vx_mps: float = 0.0
    body_vy_mps: float = 0.0
    yaw_deg: float = 0.0
    frame: int = 0
    filter_timestamp: int = 0
    integrated: int = 0
    invalid: int = 0
    filter_fail: int = 0
    consecutive_invalid: int = 0


def parse_nav_line(line: str) -> Optional[NavSample]:
    match = NAV_RE.search(line)
    if match is None:
        return None

    fields = {
        field_match.group("key"): float(field_match.group("value"))
        for field_match in FIELD_RE.finditer(match.group("body"))
    }

    if "x_m" not in fields or "y_m" not in fields:
        return None

    return NavSample(
        received_s=time.time(),
        x_m=fields["x_m"],
        y_m=fields["y_m"],
        vn_mps=fields.get("vn_mps", 0.0),
        ve_mps=fields.get("ve_mps", 0.0),
        body_vx_mps=fields.get("body_vx_mps", 0.0),
        body_vy_mps=fields.get("body_vy_mps", 0.0),
        yaw_deg=fields.get("yaw_deg", 0.0),
        frame=int(fields.get("frame", 0.0)),
        filter_timestamp=int(fields.get("filter_timestamp", 0.0)),
        integrated=int(fields.get("integrated", 0.0)),
        invalid=int(fields.get("invalid", 0.0)),
        filter_fail=int(fields.get("filter_fail", 0.0)),
        consecutive_invalid=int(fields.get("consecutive_invalid", 0.0)),
    )


def serial_reader(
    port: str,
    baudrate: int,
    samples: "queue.Queue[NavSample]",
    stop_event: threading.Event,
    raw_echo: bool,
) -> None:
    assert serial is not None

    try:
        with serial.Serial(port=port, baudrate=baudrate, timeout=0.2) as ser:
            print(f"Listening on {ser.port} at {ser.baudrate} baud")
            while not stop_event.is_set():
                raw_line = ser.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").strip()
                if raw_echo:
                    print(line)

                sample = parse_nav_line(line)
                if sample is not None:
                    samples.put(sample)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        stop_event.set()


def write_csv_header(csv_writer: csv.writer) -> None:
    csv_writer.writerow(
        [
            "received_s",
            "x_m",
            "y_m",
            "vn_mps",
            "ve_mps",
            "body_vx_mps",
            "body_vy_mps",
            "yaw_deg",
            "frame",
            "filter_timestamp",
            "integrated",
            "invalid",
            "filter_fail",
            "consecutive_invalid",
        ]
    )


def write_csv_sample(csv_writer: csv.writer, sample: NavSample) -> None:
    csv_writer.writerow(
        [
            f"{sample.received_s:.3f}",
            f"{sample.x_m:.6f}",
            f"{sample.y_m:.6f}",
            f"{sample.vn_mps:.6f}",
            f"{sample.ve_mps:.6f}",
            f"{sample.body_vx_mps:.6f}",
            f"{sample.body_vy_mps:.6f}",
            f"{sample.yaw_deg:.3f}",
            sample.frame,
            sample.filter_timestamp,
            sample.integrated,
            sample.invalid,
            sample.filter_fail,
            sample.consecutive_invalid,
        ]
    )


def set_equal_axes(ax, xs: list[float], ys: list[float]) -> None:
    if not xs or not ys:
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        return

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    center_x = (min_x + max_x) * 0.5
    center_y = (min_y + max_y) * 0.5
    span = max(max_x - min_x, max_y - min_y, 1.0)
    margin = span * 0.15
    half = span * 0.5 + margin
    ax.set_xlim(center_x - half, center_x + half)
    ax.set_ylim(center_y - half, center_y + half)


def run_plot(args: argparse.Namespace) -> int:
    if serial is None:
        print("Missing dependency: pyserial. Install with: python -m pip install pyserial", file=sys.stderr)
        return 2
    if plt is None or FuncAnimation is None:
        print("Missing dependency: matplotlib. Install with: python -m pip install matplotlib", file=sys.stderr)
        return 2

    sample_queue: "queue.Queue[NavSample]" = queue.Queue()
    stop_event = threading.Event()
    path: deque[NavSample] = deque(maxlen=args.max_points if args.max_points > 0 else None)
    csv_file = None
    csv_writer = None

    if args.csv:
        csv_path = Path(args.csv)
        csv_file = csv_path.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        write_csv_header(csv_writer)
        csv_file.flush()

    reader = threading.Thread(
        target=serial_reader,
        args=(args.port, args.baudrate, sample_queue, stop_event, args.echo),
        daemon=True,
    )
    reader.start()

    fig, ax = plt.subplots()
    (line_plot,) = ax.plot([], [], "-", linewidth=1.6, label="trajectory")
    (current_plot,) = ax.plot([], [], "o", markersize=6, label="current")
    status_text = ax.text(0.01, 0.99, "Waiting for [NAV] data", transform=ax.transAxes, va="top")

    ax.set_title("AUV NAV Trajectory")
    ax.set_xlabel("x_m")
    ax.set_ylabel("y_m")
    ax.grid(True)
    ax.legend(loc="lower right")
    ax.set_aspect("equal", adjustable="box")
    set_equal_axes(ax, [], [])

    def update(_frame):
        updated = False
        latest = path[-1] if path else None

        while True:
            try:
                sample = sample_queue.get_nowait()
            except queue.Empty:
                break

            path.append(sample)
            latest = sample
            updated = True

            if csv_writer is not None and csv_file is not None:
                write_csv_sample(csv_writer, sample)

        if csv_file is not None and updated:
            csv_file.flush()

        if latest is None:
            return line_plot, current_plot, status_text

        xs = [sample.x_m for sample in path]
        ys = [sample.y_m for sample in path]
        line_plot.set_data(xs, ys)
        current_plot.set_data([latest.x_m], [latest.y_m])
        status_text.set_text(
            "x={:.3f} m  y={:.3f} m  yaw={:.2f} deg  frame={}  invalid={}  filter_fail={}  consecutive_invalid={}".format(
                latest.x_m,
                latest.y_m,
                latest.yaw_deg,
                latest.frame,
                latest.invalid,
                latest.filter_fail,
                latest.consecutive_invalid,
            )
        )
        set_equal_axes(ax, xs, ys)
        return line_plot, current_plot, status_text

    def on_close(_event):
        stop_event.set()

    fig.canvas.mpl_connect("close_event", on_close)
    animation = FuncAnimation(fig, update, interval=args.interval_ms, blit=False)

    try:
        plt.show()
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        if csv_file is not None:
            csv_file.close()
        # Keep a reference until shutdown; some matplotlib backends require it.
        _ = animation

    return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive [NAV] serial logs and plot x_m/y_m trajectory."
    )
    parser.add_argument("--port", default="COM10", help="Serial port, default: COM10")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate, default: 115200")
    parser.add_argument("--interval-ms", type=int, default=100, help="Plot refresh interval in ms")
    parser.add_argument("--max-points", type=int, default=0, help="Maximum points to keep; 0 keeps all")
    parser.add_argument("--csv", help="Optional CSV output path")
    parser.add_argument("--echo", action="store_true", help="Print every received serial line")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    return run_plot(args)


if __name__ == "__main__":
    raise SystemExit(main())
