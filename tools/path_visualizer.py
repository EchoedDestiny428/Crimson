"""Visualize Crimson telemetry and overlay raw LemLib auton commands.

Run:
    python tools/path_visualizer.py

The CSV is expected to contain pose_x_in, pose_y_in, and optionally
pose_theta_deg columns, as produced by Crimson's telemetry logger.
"""

from __future__ import annotations

import csv
import math
import re
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Iterable


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
MOVE_RE = re.compile(
    rf"\b(?:chassis\.)?moveToPoint\s*\(\s*({NUMBER})\s*,\s*({NUMBER})",
    re.IGNORECASE,
)
TURN_POINT_RE = re.compile(
    rf"\b(?:chassis\.)?turnToPoint\s*\(\s*({NUMBER})\s*,\s*({NUMBER})",
    re.IGNORECASE,
)
TURN_HEADING_RE = re.compile(
    rf"\b(?:chassis\.)?turnToHeading\s*\(\s*({NUMBER})",
    re.IGNORECASE,
)
SET_POSE_RE = re.compile(
    rf"\b(?:chassis\.)?setPose\s*\(\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})",
    re.IGNORECASE,
)
CURVE_RE = re.compile(
    rf"\b(?:curve)\s*\(\s*[^,]+,\s*[^,]+,\s*({NUMBER})\s*,\s*({NUMBER})\s*,"
    rf"\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float | None = None


@dataclass(frozen=True)
class PathPoint:
    pose: Pose
    label: str
    kind: str


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def parse_path_commands(text: str) -> list[PathPoint]:
    """Extract numeric path targets in source order from pasted C++."""
    source = strip_comments(text)
    matches: list[tuple[int, PathPoint]] = []

    for match in MOVE_RE.finditer(source):
        x, y = map(float, match.groups())
        matches.append((match.start(), PathPoint(Pose(x, y), "moveToPoint", "move")))
    for match in TURN_POINT_RE.finditer(source):
        x, y = map(float, match.groups())
        matches.append((match.start(), PathPoint(Pose(x, y), "turnToPoint", "turn")))
    for match in TURN_HEADING_RE.finditer(source):
        theta = float(match.group(1))
        matches.append((match.start(), PathPoint(Pose(0, 0, theta), "turnToHeading", "heading")))
    for match in SET_POSE_RE.finditer(source):
        x, y, theta = map(float, match.groups())
        matches.append((match.start(), PathPoint(Pose(x, y, theta), "setPose", "pose")))
    for match in CURVE_RE.finditer(source):
        x, y, exit_angle, entry_angle, exit_length, entry_length = map(float, match.groups())
        # A curve's initial point may be dynamic, so only its numeric final point
        # and entry control point are drawn. This still shows the commanded approach.
        entry_radians = math.radians(entry_angle)
        entry_x = x + math.sin(entry_radians) * entry_length
        entry_y = y + math.cos(entry_radians) * entry_length
        matches.append((match.start(), PathPoint(Pose(entry_x, entry_y), "curve entry", "curve")))
        matches.append((match.start() + 1, PathPoint(Pose(x, y), "curve target", "curve")))

    matches.sort(key=lambda item: item[0])
    return [point for _, point in matches]


def read_pose_csv(path: Path) -> list[Pose]:
    poses: list[Pose] = []
    with path.open("r", newline="", encoding="utf-8-sig") as file:
        lines = (line for line in file if not line.lstrip().startswith("#"))
        reader = csv.DictReader(lines)
        if reader.fieldnames is None:
            raise ValueError("The CSV has no header.")
        required = {"pose_x_in", "pose_y_in"}
        missing = required - set(reader.fieldnames)
        if missing:
            raise ValueError("CSV is missing: " + ", ".join(sorted(missing)))
        for row in reader:
            try:
                x = float(row["pose_x_in"])
                y = float(row["pose_y_in"])
                theta_text = row.get("pose_theta_deg", "")
                theta = float(theta_text) if theta_text not in ("", "nan", "NaN") else None
            except (TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                poses.append(Pose(x, y, theta))
    if not poses:
        raise ValueError("The CSV contains no finite pose samples.")
    return poses


class Visualizer:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Crimson Path Visualizer")
        self.root.geometry("1100x760")
        self.csv_poses: list[Pose] = []
        self.path_points: list[PathPoint] = []
        self.csv_name = "No telemetry CSV loaded"

        controls = ttk.Frame(root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Button(controls, text="Load telemetry CSV", command=self.load_csv).pack(side=tk.LEFT)
        ttk.Button(controls, text="Load auton TXT", command=self.load_txt).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Plot", command=self.plot).pack(side=tk.LEFT, padx=(8, 0))
        self.status = ttk.Label(controls, text=self.csv_name)
        self.status.pack(side=tk.LEFT, padx=12)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        text_frame = ttk.Frame(body, padding=(0, 0, 8, 0))
        ttk.Label(text_frame, text="Paste raw auton commands or load a .txt file:").pack(anchor=tk.W)
        self.source = tk.Text(text_frame, width=44, wrap=tk.NONE, undo=True)
        self.source.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        body.add(text_frame, weight=1)

        canvas_frame = ttk.Frame(body)
        self.canvas = tk.Canvas(canvas_frame, background="#101419", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        body.add(canvas_frame, weight=3)
        self.canvas.bind("<Configure>", lambda _: self.plot())

    def load_csv(self) -> None:
        filename = filedialog.askopenfilename(
            title="Load telemetry CSV", filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not filename:
            return
        try:
            self.csv_poses = read_pose_csv(Path(filename))
        except (OSError, ValueError) as error:
            messagebox.showerror("CSV error", str(error))
            return
        self.csv_name = f"{Path(filename).name}: {len(self.csv_poses)} pose samples"
        self.status.configure(text=self.csv_name)
        self.plot()

    def load_txt(self) -> None:
        filename = filedialog.askopenfilename(
            title="Load raw auton commands", filetypes=[("Text files", "*.txt"), ("C++ files", "*.cpp"), ("All files", "*.*")]
        )
        if not filename:
            return
        try:
            text = Path(filename).read_text(encoding="utf-8")
        except OSError as error:
            messagebox.showerror("Text file error", str(error))
            return
        self.source.delete("1.0", tk.END)
        self.source.insert("1.0", text)
        self.plot()

    def plot(self) -> None:
        self.path_points = parse_path_commands(self.source.get("1.0", tk.END))
        self.canvas.delete("all")
        samples = self.csv_poses + [point.pose for point in self.path_points if point.kind != "heading"]
        if not samples:
            self.canvas.create_text(
                20, 20, anchor=tk.NW, fill="white",
                text="Load a telemetry CSV or paste auton commands to begin.",
            )
            return

        width = max(self.canvas.winfo_width(), 400)
        height = max(self.canvas.winfo_height(), 300)
        min_x = min(p.x for p in samples)
        max_x = max(p.x for p in samples)
        min_y = min(p.y for p in samples)
        max_y = max(p.y for p in samples)
        span = max(max_x - min_x, max_y - min_y, 20.0)
        padding = max(8.0, span * 0.12)
        min_x -= padding
        max_x += padding
        min_y -= padding
        max_y += padding
        scale = min((width - 80) / (max_x - min_x), (height - 80) / (max_y - min_y))

        def screen(point: Pose) -> tuple[float, float]:
            # Canvas Y is downward; field Y is upward.
            return 40 + (point.x - min_x) * scale, height - 40 - (point.y - min_y) * scale

        self.draw_grid(width, height, min_x, max_x, min_y, max_y, scale, screen)
        if len(self.csv_poses) > 1:
            coordinates = [value for pose in self.csv_poses for value in screen(pose)]
            self.canvas.create_line(*coordinates, fill="#41d17d", width=2, smooth=True)
            self.draw_robot(self.csv_poses[-1], screen, "#41d17d")
        self.draw_path(screen)
        self.canvas.create_text(12, 12, anchor=tk.NW, fill="#41d17d", text="Telemetry")
        if self.path_points:
            self.canvas.create_text(92, 12, anchor=tk.NW, fill="#ffbf69", text="Raw path")

    def draw_grid(self, width: int, height: int, min_x: float, max_x: float,
                  min_y: float, max_y: float, scale: float, screen) -> None:
        def tick_start(value: float) -> int:
            return math.floor(value / 12) * 12

        for x in range(tick_start(min_x), math.ceil(max_x / 12) * 12 + 1, 12):
            sx, _ = screen(Pose(float(x), 0))
            color = "#68717c" if x == 0 else "#29323b"
            self.canvas.create_line(sx, 0, sx, height, fill=color)
            self.canvas.create_text(sx + 2, height - 25, anchor=tk.SW, fill="#aab4bf", text=f"{x:g}")
        for y in range(tick_start(min_y), math.ceil(max_y / 12) * 12 + 1, 12):
            _, sy = screen(Pose(0, float(y)))
            color = "#68717c" if y == 0 else "#29323b"
            self.canvas.create_line(0, sy, width, sy, fill=color)
            self.canvas.create_text(4, sy - 2, anchor=tk.SW, fill="#aab4bf", text=f"{y:g}")
        origin = screen(Pose(0, 0))
        self.canvas.create_oval(origin[0] - 3, origin[1] - 3, origin[0] + 3, origin[1] + 3, fill="#ffffff", outline="")
        self.canvas.create_text(origin[0] + 6, origin[1] - 6, anchor=tk.SW, fill="white", text="0,0")

    def draw_path(self, screen) -> None:
        previous: tuple[float, float] | None = None
        for index, point in enumerate(self.path_points, start=1):
            if point.kind == "heading":
                continue
            current = screen(point.pose)
            if previous is not None:
                self.canvas.create_line(*previous, *current, fill="#ffbf69", width=2, dash=(6, 4))
            previous = current
            radius = 5 if point.kind != "pose" else 7
            self.canvas.create_oval(
                current[0] - radius, current[1] - radius,
                current[0] + radius, current[1] + radius,
                fill="#ffbf69", outline="#fff1d6",
            )
            self.canvas.create_text(current[0] + 8, current[1] - 8, anchor=tk.SW,
                                    fill="#ffdfaa", text=f"{index}: {point.label}")

    def draw_robot(self, pose: Pose, screen, color: str) -> None:
        center = screen(pose)
        radius = 7
        self.canvas.create_oval(center[0] - radius, center[1] - radius,
                                center[0] + radius, center[1] + radius,
                                outline=color, width=2)
        if pose.theta is not None:
            radians = math.radians(pose.theta)
            # 0 degrees is north/positive Y, matching the project's convention.
            tip = (center[0] + math.sin(radians) * 18,
                   center[1] - math.cos(radians) * 18)
            self.canvas.create_line(*center, *tip, fill=color, width=2, arrow=tk.LAST)


def main() -> None:
    root = tk.Tk()
    Visualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
