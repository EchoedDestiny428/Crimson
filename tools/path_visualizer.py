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


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
TRACK_WIDTH_IN = 12.5
SIM_MAX_SPEED_IN_S = 48.0
SIM_STEP_S = 0.02
SIM_POSITION_TOLERANCE_IN = 0.75
SIM_HEADING_TOLERANCE_DEG = 2.0
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
SET_GOAL_RE = re.compile(
    rf"\b(?:chassis\.)?setPoseFromGoal\s*\(\s*({NUMBER})\s*,\s*({NUMBER})",
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


@dataclass(frozen=True)
class SimulationCommand:
    kind: str
    args: tuple[float, ...]
    forwards: bool = True
    early_exit: float | None = None


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
    for match in SET_GOAL_RE.finditer(source):
        x, y = map(float, match.groups())
        matches.append((match.start(), PathPoint(Pose(x, y), "setPoseFromGoal", "pose")))
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


def option_forwards(arguments: str, default: bool = True) -> bool:
    match = re.search(r"\.forwards\s*=\s*(true|false)", arguments, re.IGNORECASE)
    return default if match is None else match.group(1).lower() == "true"


def option_early_exit(arguments: str) -> float | None:
    match = re.search(r"\.earlyExitRange\s*=\s*(" + NUMBER + ")", arguments, re.IGNORECASE)
    return None if match is None else float(match.group(1))


def parse_simulation_commands(text: str) -> list[SimulationCommand]:
    """Extract executable movement commands for the approximate simulator."""
    source = strip_comments(text)
    commands: list[tuple[int, SimulationCommand]] = []

    patterns = (
        ("setPose", re.compile(rf"\b(?:chassis\.)?setPose\s*\(\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*\)", re.IGNORECASE)),
        ("set_goal", re.compile(rf"\b(?:chassis\.)?setPoseFromGoal\s*\(\s*({NUMBER})\s*,\s*({NUMBER})(?P<rest>[^)]*)\)", re.IGNORECASE)),
        ("move", re.compile(rf"\b(?:chassis\.)?moveToPoint\s*\(\s*({NUMBER})\s*,\s*({NUMBER})(?P<rest>[^)]*)\)", re.IGNORECASE)),
        ("turn_point", re.compile(rf"\b(?:chassis\.)?turnToPoint\s*\(\s*({NUMBER})\s*,\s*({NUMBER})(?P<rest>[^)]*)\)", re.IGNORECASE)),
        ("turn_heading", re.compile(rf"\b(?:chassis\.)?turnToHeading\s*\(\s*({NUMBER})(?P<rest>[^)]*)\)", re.IGNORECASE)),
        ("curve", re.compile(
            rf"\bcurve\s*\(\s*[^,]+,\s*[^,]+,\s*({NUMBER})\s*,\s*({NUMBER})\s*,"
            rf"\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})\s*,\s*({NUMBER})"
            rf"(?P<rest>[^)]*)\)", re.IGNORECASE)),
    )
    for kind, pattern in patterns:
        for match in pattern.finditer(source):
            values = tuple(float(value) for value in match.groups() if value is not None and value != match.groupdict().get("rest"))
            rest = match.groupdict().get("rest", "")
            commands.append((match.start(), SimulationCommand(
                kind, values, option_forwards(rest, kind != "turn_point"), option_early_exit(rest)
            )))
    commands.sort(key=lambda item: item[0])
    return [command for _, command in commands]


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def bearing_degrees(start: Pose, target: Pose) -> float:
    return math.degrees(math.atan2(target.x - start.x, target.y - start.y))


def simulate_segment(
    pose: Pose,
    target: Pose,
    forwards: bool,
    rotate_only: bool = False,
    early_exit: float | None = None,
) -> list[Pose]:
    """Simulate a bounded differential drive moving toward one target."""
    if pose.theta is None:
        pose = Pose(pose.x, pose.y, 0.0)
    result = [pose]
    current = pose
    for _ in range(5000):
        distance = math.hypot(target.x - current.x, target.y - current.y)
        desired = target.theta if rotate_only and target.theta is not None else bearing_degrees(current, target)
        if not rotate_only and not forwards:
            desired = wrap_degrees(desired + 180.0)
        heading_error = wrap_degrees(desired - (current.theta or 0.0))
        if rotate_only:
            linear = 0.0
        else:
            linear = min(SIM_MAX_SPEED_IN_S, distance * 3.0)
            if not forwards:
                linear = -linear
            linear *= max(0.15, math.cos(math.radians(heading_error)))
        angular = max(-180.0, min(180.0, heading_error * 5.0))
        if rotate_only:
            angular = max(-180.0, min(180.0, heading_error * 6.0))
        left_speed = linear - math.radians(angular) * TRACK_WIDTH_IN / 2.0
        right_speed = linear + math.radians(angular) * TRACK_WIDTH_IN / 2.0
        wheel_limit = max(abs(left_speed), abs(right_speed), 1.0)
        if wheel_limit > SIM_MAX_SPEED_IN_S:
            left_speed *= SIM_MAX_SPEED_IN_S / wheel_limit
            right_speed *= SIM_MAX_SPEED_IN_S / wheel_limit
        center_speed = (left_speed + right_speed) / 2.0
        angular_speed = (right_speed - left_speed) / TRACK_WIDTH_IN
        theta = math.radians(current.theta or 0.0)
        next_theta = wrap_degrees((current.theta or 0.0) + math.degrees(angular_speed * SIM_STEP_S))
        next_pose = Pose(
            current.x + center_speed * math.sin(theta) * SIM_STEP_S,
            current.y + center_speed * math.cos(theta) * SIM_STEP_S,
            next_theta,
        )
        result.append(next_pose)
        current = next_pose
        position_exit = SIM_POSITION_TOLERANCE_IN if early_exit is None else max(0.0, early_exit)
        heading_exit = SIM_HEADING_TOLERANCE_DEG if early_exit is None else max(0.0, early_exit)
        if (distance <= position_exit and not rotate_only) or (
            rotate_only and abs(heading_error) <= heading_exit
        ):
            break
    return result


def simulate_commands(commands: list[SimulationCommand]) -> list[Pose]:
    """Run a lightweight drivetrain simulation from the parsed commands."""
    current = Pose(0.0, 0.0, 0.0)
    result: list[Pose] = [current]
    for command in commands:
        if command.kind == "setPose":
            current = Pose(*command.args)
            result.append(current)
        elif command.kind == "move":
            target = Pose(command.args[0], command.args[1])
            segment = simulate_segment(current, target, command.forwards, early_exit=command.early_exit)
            result.extend(segment[1:])
            current = segment[-1]
        elif command.kind == "set_goal":
            heading = math.radians(current.theta or 0.0)
            candidate = Pose(
                command.args[0] + 7.0 * math.sin(heading),
                command.args[1] + 7.0 * math.cos(heading),
                current.theta,
            )
            if math.hypot(candidate.x - current.x, candidate.y - current.y) <= 10.0:
                current = candidate
                result.append(current)
        elif command.kind == "turn_point":
            target = Pose(command.args[0], command.args[1])
            target_heading = bearing_degrees(current, target)
            if not command.forwards:
                target_heading = wrap_degrees(target_heading + 180.0)
            segment = simulate_segment(
                current, Pose(current.x, current.y, target_heading), True, True, command.early_exit
            )
            result.extend(segment[1:])
            current = segment[-1]
        elif command.kind == "turn_heading":
            segment = simulate_segment(
                current, Pose(current.x, current.y, command.args[0]), True, True, command.early_exit
            )
            result.extend(segment[1:])
            current = segment[-1]
        elif command.kind == "curve":
            final_x, final_y, exit_angle, entry_angle, exit_length, entry_length = command.args
            direction = 1.0 if command.forwards else -1.0
            exit_radians = math.radians(exit_angle)
            entry_radians = math.radians(entry_angle)
            first = Pose(
                current.x + direction * math.sin(exit_radians) * (exit_length + 2.0),
                current.y + direction * math.cos(exit_radians) * (exit_length + 2.0),
            )
            second = Pose(
                final_x - direction * math.sin(entry_radians) * entry_length,
                final_y - direction * math.cos(entry_radians) * entry_length,
            )
            for target in (first, second, Pose(final_x, final_y)):
                curve_exit = 2.0 if target is first else 6.0 if target is second else None
                segment = simulate_segment(current, target, command.forwards, early_exit=curve_exit)
                result.extend(segment[1:])
                current = segment[-1]
    return result


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
        self.sim_poses: list[Pose] = []
        self.current_frame = 0
        self.playing = False
        self.play_job: str | None = None
        self.csv_name = "No telemetry CSV loaded"

        controls = ttk.Frame(root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Button(controls, text="Load telemetry CSV", command=self.load_csv).pack(side=tk.LEFT)
        ttk.Button(controls, text="Load auton TXT", command=self.load_txt).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Plot", command=self.plot).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Simulate drivetrain", command=self.simulate).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Play", command=self.play).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(controls, text="Pause", command=self.pause).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(controls, text="Step", command=self.step).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(controls, text="Reset", command=self.reset_playback).pack(side=tk.LEFT, padx=(4, 0))
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
        timeline = ttk.Frame(root, padding=(8, 0, 8, 8))
        timeline.pack(fill=tk.X)
        self.timeline = ttk.Scale(timeline, from_=0, to=1, orient=tk.HORIZONTAL, command=self.seek)
        self.timeline.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.timeline_label = ttk.Label(timeline, text="No simulation")
        self.timeline_label.pack(side=tk.LEFT, padx=(8, 0))
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
        self.pause()
        self.path_points = parse_path_commands(self.source.get("1.0", tk.END))
        self.sim_poses = []
        self.current_frame = 0
        self.timeline.configure(to=1, value=0)
        self.timeline_label.configure(text="No simulation")
        self.render()

    def simulate(self) -> None:
        commands = parse_simulation_commands(self.source.get("1.0", tk.END))
        if not commands:
            messagebox.showerror("Simulation", "No supported movement commands were found.")
            return
        self.path_points = parse_path_commands(self.source.get("1.0", tk.END))
        self.sim_poses = simulate_commands(commands)
        self.current_frame = 0
        self.timeline.configure(to=max(1, len(self.sim_poses) - 1), value=0)
        self.timeline_label.configure(text=f"0 / {len(self.sim_poses) - 1}")
        self.render()

    def seek(self, value: str) -> None:
        if not self.sim_poses:
            return
        self.current_frame = max(0, min(len(self.sim_poses) - 1, int(float(value))))
        self.timeline_label.configure(text=f"{self.current_frame} / {len(self.sim_poses) - 1}")
        self.render()

    def play(self) -> None:
        if not self.sim_poses:
            return
        self.playing = True
        self.advance_playback()

    def pause(self) -> None:
        self.playing = False
        if self.play_job is not None:
            self.root.after_cancel(self.play_job)
            self.play_job = None

    def step(self) -> None:
        self.pause()
        if self.sim_poses:
            self.seek(str(min(self.current_frame + 1, len(self.sim_poses) - 1)))

    def reset_playback(self) -> None:
        self.pause()
        if self.sim_poses:
            self.seek("0")

    def advance_playback(self) -> None:
        if not self.playing or not self.sim_poses:
            return
        if self.current_frame >= len(self.sim_poses) - 1:
            self.pause()
            return
        self.seek(str(self.current_frame + 1))
        self.play_job = self.root.after(20, self.advance_playback)

    def render(self) -> None:
        self.canvas.delete("all")
        samples = (
            self.csv_poses
            + self.sim_poses
            + [point.pose for point in self.path_points if point.kind != "heading"]
        )
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
            return 40 + (point.x - min_x) * scale, height - 40 - (point.y - min_y) * scale

        self.draw_grid(width, height, min_x, max_x, min_y, max_y, scale, screen)
        if len(self.csv_poses) > 1:
            self.draw_trace(self.csv_poses, screen, "#41d17d", "Telemetry")
        if len(self.sim_poses) > 1:
            visible = self.sim_poses[: self.current_frame + 1]
            self.draw_trace(visible, screen, "#72b7ff", "Simulation")
            self.draw_robot_box(visible[-1], screen, "#72b7ff")
        self.draw_path(screen)
        if self.path_points:
            self.canvas.create_text(12, 12, anchor=tk.NW, fill="#ffbf69", text="Raw path")

    def draw_trace(self, poses: list[Pose], screen, color: str, label: str) -> None:
        coordinates = [value for pose in poses for value in screen(pose)]
        self.canvas.create_line(*coordinates, fill=color, width=2, smooth=True)
        self.canvas.create_text(
            12 if label == "Telemetry" else 78, 12, anchor=tk.NW, fill=color, text=label
        )

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

    def draw_robot_box(self, pose: Pose, screen, color: str) -> None:
        """Draw a 18 x 14 inch robot box with its front aligned to theta."""
        center_x, center_y = screen(pose)
        radians = math.radians(pose.theta or 0.0)
        forward = (math.sin(radians), -math.cos(radians))
        right = (math.cos(radians), math.sin(radians))
        half_length = 9.0
        half_width = 7.0
        corners = []
        for forward_sign, right_sign in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            field_x = forward_sign * half_length * forward[0] + right_sign * half_width * right[0]
            field_y = forward_sign * half_length * forward[1] + right_sign * half_width * right[1]
            corners.extend((center_x + field_x, center_y + field_y))
        self.canvas.create_polygon(*corners, outline=color, fill="", width=2)
        front_x = center_x + forward[0] * 12
        front_y = center_y + forward[1] * 12
        self.canvas.create_line(center_x, center_y, front_x, front_y, fill=color, width=3, arrow=tk.LAST)


def main() -> None:
    root = tk.Tk()
    Visualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
