"""Visualize Crimson telemetry and simulate LemLib auton routes on a tank drive.

Run:
    python tools/path_visualizer.py

The CSV is expected to contain pose_x_in, pose_y_in, and optionally
pose_theta_deg columns, as produced by Crimson's telemetry logger.

The simulator models each side of the drivetrain as two V5 motors driving the
wheels (linear torque-speed curve, coast when commanded 0), and the robot as a
rigid body with traction limits and field walls. The motion commands are a port
of LemLib 0.5's moveToPoint/turnToPoint/turnToHeading loops plus Crimson's
donut(), run every 10 ms with the PID and exit settings from robotconfig.cpp.
Auton timing follows the C++: LemLib motions are async, so pros::delay() after a
motion overlaps it, and parallel() lambdas start at the same time.
"""

from __future__ import annotations

import ast
import csv
import math
import re
import time
import tkinter as tk
from dataclasses import dataclass, field
from pathlib import Path
from tkinter import filedialog, messagebox, ttk


NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
FIELD_SIZE_IN = 144.0

# Drivetrain, from src/robotconfig.cpp.
TRACK_WIDTH_IN = 12.5
WHEEL_DIAMETER_IN = 3.25  # lemlib::Omniwheel::NEW_325
DRIVE_RPM = 360.0
MOTORS_PER_SIDE = 2
MOTOR_CARTRIDGE_RPM = 600.0  # blue cartridge

# Physical estimates. Measure the robot and update these.
ROBOT_MASS_LB = 15.0
ROBOT_LENGTH_IN = 18.0
ROBOT_WIDTH_IN = 14.0
MOTOR_STALL_TORQUE_NM = 2.1 * 100.0 / MOTOR_CARTRIDGE_RPM  # V5: 2.1 N*m on the 100 rpm cartridge
DRIVE_EFFICIENCY = 0.8
ROLLING_RESISTANCE = 0.03  # fraction of the robot's weight
LONGITUDINAL_MU = 1.0
LATERAL_MU = 0.7

CONTROL_PERIOD_MS = 10
PHYSICS_SUBSTEPS = 5
ASYNC_START_MS = 10  # LemLib async motions delay the caller 10 ms after starting
MAX_SIM_MS = 300_000

GRAVITY = 9.81
IN_TO_M = 0.0254


@dataclass(frozen=True)
class ControllerSettings:
    kP: float
    kI: float
    kD: float
    windup_range: float
    small_error: float
    small_error_timeout: float
    large_error: float
    large_error_timeout: float
    slew: float


# lemlib::ControllerSettings from src/robotconfig.cpp.
LINEAR_SETTINGS = ControllerSettings(9.5, 0, 12, 1, 0.2, 100, 1, 500, 40)
ANGULAR_SETTINGS = ControllerSettings(5.5, 0, 40, 3, 1, 50, 3, 300, 0)


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
class Frame:
    time_ms: int
    pose: Pose
    left: float
    right: float
    label: str
    reset: bool = False


# ---------------------------------------------------------------- parsing

@dataclass
class Stmt:
    kind: str
    args: list[float | None] = field(default_factory=list)
    options: dict[str, object] = field(default_factory=dict)
    blocks: list[list["Stmt"]] = field(default_factory=list)
    text: str = ""
    raw: list[str] = field(default_factory=list)


CALL_RE = re.compile(
    r"(?:\bchassis\s*\.\s*)?\b(?P<name>moveToPoint|turnToPoint|turnToHeading|setPoseFromGoal|setPose"
    r"|donut|waitUntilDone|parallel|curve)\s*\(|\bpros::delay\s*\("
)
LAMBDA_RE = re.compile(r"\[[^\]]*\]\s*(?:\([^)]*\)\s*)?(?:mutable\s*)?\{")
TYPE_BEFORE_RE = re.compile(r"\b(?:void|float|double|int|auto|bool)\s*$")


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    return "\n".join(line.split("//", 1)[0] for line in text.splitlines())


def find_close(text: str, open_index: int) -> int:
    """Index of the bracket closing text[open_index], or len(text) if unbalanced."""
    pairs = {"(": ")", "{": "}", "[": "]"}
    stack = [pairs[text[open_index]]]
    index = open_index + 1
    while index < len(text) and stack:
        char = text[index]
        if char in pairs:
            stack.append(pairs[char])
        elif char == stack[-1]:
            stack.pop()
        elif char == '"':
            index = text.find('"', index + 1)
            if index < 0:
                return len(text)
        index += 1
    return index - 1 if not stack else len(text)


def split_top_level(text: str) -> list[str]:
    parts: list[str] = []
    depth = 0
    start = 0
    for index, char in enumerate(text):
        if char in "({[":
            depth += 1
        elif char in ")}]":
            depth -= 1
        elif char == "," and depth == 0:
            parts.append(text[start:index].strip())
            start = index + 1
    tail = text[start:].strip()
    if tail:
        parts.append(tail)
    return parts


def safe_eval(expression: str) -> float | None:
    """Evaluate a numeric literal expression like -60 + 2.5; anything else is None."""
    expression = re.sub(r"(?<=\d)[fF]\b", "", expression.strip())
    try:
        tree = ast.parse(expression, mode="eval")
    except SyntaxError:
        return None

    def evaluate(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return evaluate(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)) and not isinstance(node.value, bool):
            return float(node.value)
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            value = evaluate(node.operand)
            return value if isinstance(node.op, ast.UAdd) else -value
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            left, right = evaluate(node.left), evaluate(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            return left / right
        raise ValueError(expression)

    try:
        return evaluate(tree)
    except (ValueError, ZeroDivisionError):
        return None


def parse_options(text: str) -> dict[str, object]:
    options: dict[str, object] = {}
    for key, value in re.findall(r"\.(\w+)\s*=\s*([^,}]+)", text):
        value = value.strip()
        if value in ("true", "false"):
            options[key] = value == "true"
        elif "CCW" in value:
            options[key] = "ccw"
        elif "CW" in value:
            options[key] = "cw"
        else:
            number = safe_eval(value)
            if number is not None:
                options[key] = number
    return options


def lambda_bodies(text: str) -> list[str]:
    bodies: list[str] = []
    position = 0
    while (match := LAMBDA_RE.search(text, position)) is not None:
        open_index = match.end() - 1
        close_index = find_close(text, open_index)
        bodies.append(text[open_index + 1:close_index])
        position = close_index + 1
    return bodies


def parse_block(text: str) -> list[Stmt]:
    statements: list[Stmt] = []
    position = 0
    while (match := CALL_RE.search(text, position)) is not None:
        name = match.group("name") or "delay"
        open_index = match.end() - 1
        close_index = find_close(text, open_index)
        inner = text[open_index + 1:close_index]
        position = close_index + 1
        call_text = " ".join(text[match.start():close_index + 1].split())
        if TYPE_BEFORE_RE.search(text[:match.start()]):
            # A function definition such as `void curve(...) { ... }`: skip its body.
            brace = text.find("{", close_index)
            if brace >= 0:
                position = find_close(text, brace) + 1
            continue
        if name == "parallel":
            blocks = [parse_block(body) for body in lambda_bodies(inner)]
            statements.append(Stmt("parallel", blocks=blocks, text="parallel"))
            continue
        parts = split_top_level(inner)
        options: dict[str, object] = {}
        if parts and parts[-1].startswith("{"):
            options = parse_options(parts.pop())
        values = [safe_eval(part) for part in parts]
        statements.append(Stmt(name, values, options, text=call_text, raw=parts))
    return statements


def extract_run_body(source: str) -> str:
    match = re.search(r"\bvoid\s+run\s*\(\s*\)\s*\{", source)
    if match is None:
        return source
    return source[match.end():find_close(source, match.end() - 1)]


def curve_uses_turn(source: str) -> bool:
    """Whether the pasted curve() definition still ends with a turnToPoint()."""
    match = re.search(r"\bvoid\s+curve\s*\([^)]*\)\s*\{", source)
    if match is None:
        return False
    return "turnToPoint" in source[match.end():find_close(source, match.end() - 1)]


def parse_program(text: str) -> tuple[list[Stmt], bool]:
    source = strip_comments(text)
    return parse_block(extract_run_body(source)), curve_uses_turn(source)


def required(stmt: Stmt, count: int) -> bool:
    return len(stmt.args) >= count and all(value is not None for value in stmt.args[:count])


def arg(stmt: Stmt, index: int, default: float) -> float:
    if index < len(stmt.args) and stmt.args[index] is not None:
        return float(stmt.args[index])
    return default


GET_POSE_RE = re.compile(r"getPose\s*\(\s*\)\s*\.\s*(x|y|theta)\s*$")


def resolve(stmt: Stmt, index: int, pose: Pose) -> float | None:
    """A numeric argument, or chassis.getPose().x/y/theta read from the robot now."""
    if index < len(stmt.args) and stmt.args[index] is not None:
        return float(stmt.args[index])
    if index < len(stmt.raw) and (match := GET_POSE_RE.search(stmt.raw[index])) is not None:
        return getattr(pose, match.group(1))
    return None


def flatten(statements: list[Stmt]) -> list[Stmt]:
    result: list[Stmt] = []
    for stmt in statements:
        if stmt.kind == "parallel":
            for block in stmt.blocks:
                result.extend(flatten(block))
        else:
            result.append(stmt)
    return result


def parse_path_commands(text: str) -> list[PathPoint]:
    """Numeric path targets in source order, for the dashed raw path."""
    statements, _ = parse_program(text)
    points: list[PathPoint] = []
    previous = Pose(0.0, 0.0)
    for stmt in flatten(statements):
        if stmt.kind == "moveToPoint" and required(stmt, 2):
            previous = Pose(stmt.args[0], stmt.args[1])
            points.append(PathPoint(previous, "moveToPoint", "move"))
        elif stmt.kind == "turnToPoint" and required(stmt, 2):
            points.append(PathPoint(Pose(stmt.args[0], stmt.args[1]), "turnToPoint", "turn"))
        elif stmt.kind == "setPose" and required(stmt, 2):
            previous = Pose(stmt.args[0], stmt.args[1], resolve(stmt, 2, previous))
            points.append(PathPoint(previous, "setPose", "pose"))
        elif stmt.kind == "setPoseFromGoal" and required(stmt, 2):
            points.append(PathPoint(Pose(stmt.args[0], stmt.args[1]), "goal", "goal"))
        elif stmt.kind == "curve" and len(stmt.args) >= 8 and all(v is not None for v in stmt.args[2:8]):
            final_x, final_y, exit_angle, entry_angle, exit_length, entry_length = stmt.args[2:8]
            direction = 1.0 if arg(stmt, 11, 1.0) != 0 else -1.0
            start = Pose(arg(stmt, 0, previous.x), arg(stmt, 1, previous.y))
            exit_radians, entry_radians = math.radians(exit_angle), math.radians(entry_angle)
            points.append(PathPoint(Pose(
                start.x + direction * math.sin(exit_radians) * (exit_length + 2.0),
                start.y + direction * math.cos(exit_radians) * (exit_length + 2.0),
            ), "curve exit", "curve"))
            points.append(PathPoint(Pose(
                final_x - direction * math.sin(entry_radians) * entry_length,
                final_y - direction * math.cos(entry_radians) * entry_length,
            ), "curve entry", "curve"))
            previous = Pose(final_x, final_y)
            points.append(PathPoint(previous, "curve target", "move"))
    return points


# ---------------------------------------------------------------- LemLib port

def sgn(value: float) -> int:
    return (value > 0) - (value < 0)


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def angle_error_deg(target: float, current: float, direction: str = "auto") -> float:
    """lemlib::angleError(target, current, false, direction)."""
    if direction == "cw":
        return (target - current) % 360.0
    if direction == "ccw":
        error = (target - current) % 360.0
        return error - 360.0 if error > 0 else error
    return wrap_degrees(target - current)


def angle_error_rad(first: float, second: float) -> float:
    return math.remainder(first - second, math.tau)


def bearing_degrees(start: Pose, target: Pose) -> float:
    return math.degrees(math.atan2(target.x - start.x, target.y - start.y))


def slew(target: float, current: float, max_change: float) -> float:
    if max_change == 0:
        return target
    return current + max(-max_change, min(max_change, target - current))


class PID:
    def __init__(self, settings: ControllerSettings) -> None:
        self.settings = settings
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, error: float) -> float:
        self.integral += error
        if sgn(error) != sgn(self.previous_error):
            self.integral = 0.0
        if self.settings.windup_range != 0 and abs(error) > self.settings.windup_range:
            self.integral = 0.0
        derivative = error - self.previous_error
        self.previous_error = error
        return error * self.settings.kP + self.integral * self.settings.kI + derivative * self.settings.kD


class ExitCondition:
    def __init__(self, error_range: float, time_ms: float) -> None:
        self.error_range = error_range
        self.time_ms = time_ms
        self.reset()

    def reset(self) -> None:
        self.start_ms: int | None = None
        self.done = False

    def update(self, error: float, now_ms: int) -> bool:
        if abs(error) > self.error_range:
            self.start_ms = None
        elif self.start_ms is None:
            self.start_ms = now_ms
        elif now_ms - self.start_ms > self.time_ms:
            self.done = True
        return self.done


class Controllers:
    """The PIDs and exit conditions a lemlib::Chassis shares across motions."""

    def __init__(self) -> None:
        self.lateral = PID(LINEAR_SETTINGS)
        self.angular = PID(ANGULAR_SETTINGS)
        self.lateral_small = ExitCondition(LINEAR_SETTINGS.small_error, LINEAR_SETTINGS.small_error_timeout)
        self.lateral_large = ExitCondition(LINEAR_SETTINGS.large_error, LINEAR_SETTINGS.large_error_timeout)
        self.angular_small = ExitCondition(ANGULAR_SETTINGS.small_error, ANGULAR_SETTINGS.small_error_timeout)
        self.angular_large = ExitCondition(ANGULAR_SETTINGS.large_error, ANGULAR_SETTINGS.large_error_timeout)


class Motion:
    label = ""

    def __init__(self, timeout_ms: float, now_ms: int) -> None:
        self.timeout_ms = timeout_ms
        self.start_ms = now_ms
        self.exit_reason = ""

    def timed_out(self, now_ms: int) -> bool:
        if now_ms - self.start_ms >= self.timeout_ms:
            self.exit_reason = "timeout"
            return True
        return False

    def update(self, pose: Pose, now_ms: int) -> tuple[float, float] | None:
        raise NotImplementedError


class MoveToPoint(Motion):
    """lemlib::Chassis::moveToPoint (0.5)."""

    def __init__(self, ctl: Controllers, pose: Pose, now_ms: int, x: float, y: float, timeout_ms: float,
                 forwards: bool = True, max_speed: float = 127, min_speed: float = 0,
                 early_exit_range: float = 0) -> None:
        super().__init__(timeout_ms, now_ms)
        self.ctl = ctl
        self.target = (x, y)
        self.forwards = forwards
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.early_exit_range = abs(early_exit_range)
        ctl.lateral.reset()
        ctl.lateral_small.reset()
        ctl.lateral_large.reset()
        ctl.angular.reset()
        self.target_theta = math.atan2(y - pose.y, x - pose.x)  # standard-position radians
        self.close = False
        self.previous_lateral = 0.0
        self.previous_side: bool | None = None
        self.label = f"moveToPoint({x:g}, {y:g}){'' if forwards else ' reverse'}"

    def update(self, pose: Pose, now_ms: int) -> tuple[float, float] | None:
        if self.timed_out(now_ms):
            return None
        if self.close and (self.ctl.lateral_small.done or self.ctl.lateral_large.done):
            self.exit_reason = "settled"
            return None
        target_x, target_y = self.target
        theta = math.radians(90.0 - (pose.theta or 0.0))
        distance = math.hypot(target_x - pose.x, target_y - pose.y)
        if distance < 7.5 and not self.close:
            self.close = True
            self.max_speed = max(abs(self.previous_lateral), 60.0)
        side = ((pose.y - target_y) * -math.sin(self.target_theta)
                <= (pose.x - target_x) * math.cos(self.target_theta) + self.early_exit_range)
        if self.previous_side is None:
            self.previous_side = side
        if side != self.previous_side and self.min_speed != 0:
            self.exit_reason = "chained"
            return None
        self.previous_side = side

        angle_to_target = math.atan2(target_y - pose.y, target_x - pose.x)
        adjusted_theta = theta if self.forwards else theta + math.pi
        angular_error = angle_error_rad(adjusted_theta, angle_to_target)
        lateral_error = distance * math.cos(angle_error_rad(theta, angle_to_target))
        self.ctl.lateral_small.update(lateral_error, now_ms)
        self.ctl.lateral_large.update(lateral_error, now_ms)

        lateral = self.ctl.lateral.update(lateral_error)
        angular = self.ctl.angular.update(math.degrees(angular_error))
        lateral = max(-self.max_speed, min(self.max_speed, lateral))
        angular = max(-self.max_speed, min(self.max_speed, angular))
        if self.close:
            angular = 0.0
        if not self.close:
            lateral = slew(lateral, self.previous_lateral, LINEAR_SETTINGS.slew)
            lateral = max(lateral, 0.0) if self.forwards else min(lateral, 0.0)
        min_speed = abs(self.min_speed)
        if self.forwards and 0 < lateral < min_speed:
            lateral = min_speed
        if not self.forwards and lateral < 0 and -lateral < min_speed:
            lateral = -min_speed
        self.previous_lateral = lateral

        left, right = lateral + angular, lateral - angular
        ratio = max(abs(left), abs(right)) / self.max_speed
        if ratio > 1:
            left, right = left / ratio, right / ratio
        return left, right


class Turn(Motion):
    """lemlib::Chassis::turnToHeading / turnToPoint (0.5)."""

    def __init__(self, ctl: Controllers, pose: Pose, now_ms: int, timeout_ms: float,
                 heading: float | None = None, point: tuple[float, float] | None = None,
                 forwards: bool = True, direction: str = "auto", max_speed: float = 127,
                 min_speed: float = 0, early_exit_range: float = 0) -> None:
        super().__init__(timeout_ms, now_ms)
        self.ctl = ctl
        self.heading = heading
        self.point = point
        self.forwards = forwards
        self.direction = direction
        self.max_speed = max_speed
        self.min_speed = abs(min_speed)
        self.early_exit_range = early_exit_range
        ctl.angular.reset()
        ctl.angular_small.reset()
        ctl.angular_large.reset()
        self.settling = False
        self.previous_raw: float | None = None
        self.previous_delta: float | None = None
        self.previous_power = 0.0
        if point is not None:
            self.label = f"turnToPoint({point[0]:g}, {point[1]:g}){'' if forwards else ' reverse'}"
        else:
            self.label = f"turnToHeading({heading:g})"

    def update(self, pose: Pose, now_ms: int) -> tuple[float, float] | None:
        if self.timed_out(now_ms):
            return None
        if self.ctl.angular_small.done or self.ctl.angular_large.done:
            self.exit_reason = "settled"
            return None
        current = pose.theta or 0.0
        if self.point is not None:
            target = bearing_degrees(pose, Pose(*self.point))
            if not self.forwards:
                target += 180.0
        else:
            target = self.heading or 0.0
        raw = angle_error_deg(target, current)
        if self.previous_raw is None:
            self.previous_raw = raw
        if sgn(raw) != sgn(self.previous_raw):
            self.settling = True
        self.previous_raw = raw
        delta = raw if self.settling else angle_error_deg(target, current, self.direction)
        if self.previous_delta is None:
            self.previous_delta = delta
        if self.min_speed != 0 and abs(delta) < self.early_exit_range:
            self.exit_reason = "chained"
            return None
        if self.min_speed != 0 and sgn(delta) != sgn(self.previous_delta):
            self.exit_reason = "chained"
            return None
        self.previous_delta = delta

        power = self.ctl.angular.update(delta)
        self.ctl.angular_small.update(delta, now_ms)
        self.ctl.angular_large.update(delta, now_ms)
        power = max(-self.max_speed, min(self.max_speed, power))
        if abs(delta) > 20:
            power = slew(power, self.previous_power, ANGULAR_SETTINGS.slew)
        if -self.min_speed < power < 0:
            power = -self.min_speed
        elif 0 < power < self.min_speed:
            power = self.min_speed
        self.previous_power = power
        return power, -power


class Donut(Motion):
    """crimson::Chassis::donut from src/motion/chassis.cpp."""

    SETTLE_WINDOW_DEG = 90.0

    def __init__(self, ctl: Controllers, pose: Pose, now_ms: int, theta: float, timeout_ms: float,
                 direction: str = "auto", forwards: bool = True, fast_speed: float = 100,
                 slow_speed: float = 75, early_exit_range: float = 0) -> None:
        super().__init__(timeout_ms, now_ms)
        self.ctl = ctl
        self.theta = theta
        start = pose.theta or 0.0
        self.direction = "cw" if angle_error_deg(theta, start, direction) >= 0 else "ccw"
        travel = 1.0 if forwards else -1.0
        self.push = travel * (fast_speed - slow_speed) / 2.0
        self.max_spin = (fast_speed + slow_speed) / 2.0
        self.early_exit_range = early_exit_range
        ctl.angular.reset()
        ctl.angular_large.reset()
        ctl.angular_small.reset()
        self.previous_raw: float | None = None
        self.settling = False
        self.label = f"donut({theta:g})"

    def update(self, pose: Pose, now_ms: int) -> tuple[float, float] | None:
        if self.timed_out(now_ms):
            return None
        if self.ctl.angular_large.done or self.ctl.angular_small.done:
            self.exit_reason = "settled"
            return None
        heading = pose.theta or 0.0
        raw = angle_error_deg(self.theta, heading)
        if (self.previous_raw is not None and abs(raw) < self.SETTLE_WINDOW_DEG
                and (raw > 0) != (self.previous_raw > 0)):
            self.settling = True
        self.previous_raw = raw
        error = raw if self.settling else angle_error_deg(self.theta, heading, self.direction)
        self.ctl.angular_large.update(error, now_ms)
        self.ctl.angular_small.update(error, now_ms)
        if self.early_exit_range > 0 and abs(error) < self.early_exit_range:
            self.exit_reason = "early exit"
            return None
        spin = max(-self.max_spin, min(self.max_spin, self.ctl.angular.update(error)))
        return float(round(self.push + spin)), float(round(self.push - spin))


# ---------------------------------------------------------------- physics

class TankDrive:
    """Rigid-body tank drive: two motor-driven sides, traction limits, field walls."""

    def __init__(self) -> None:
        self.mass = ROBOT_MASS_LB * 0.45359237
        length, width = ROBOT_LENGTH_IN * IN_TO_M, ROBOT_WIDTH_IN * IN_TO_M
        self.inertia = self.mass * (length ** 2 + width ** 2) / 12.0
        self.half_track = TRACK_WIDTH_IN * IN_TO_M / 2.0
        wheel_radius = WHEEL_DIAMETER_IN * IN_TO_M / 2.0
        gear_ratio = MOTOR_CARTRIDGE_RPM / DRIVE_RPM
        self.stall_force = MOTORS_PER_SIDE * MOTOR_STALL_TORQUE_NM * gear_ratio * DRIVE_EFFICIENCY / wheel_radius
        self.free_speed = DRIVE_RPM / 60.0 * math.pi * WHEEL_DIAMETER_IN * IN_TO_M  # m/s
        self.traction_force = LONGITUDINAL_MU * self.mass * GRAVITY / 2.0
        self.reset(Pose(0.0, 0.0, 0.0))

    def reset(self, pose: Pose) -> None:
        self.x = pose.x * IN_TO_M
        self.y = pose.y * IN_TO_M
        self.theta = math.radians(pose.theta or 0.0)  # compass, clockwise positive
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0  # rad/s, clockwise positive

    def pose(self) -> Pose:
        return Pose(self.x / IN_TO_M, self.y / IN_TO_M, wrap_degrees(math.degrees(self.theta)))

    def set_pose(self, pose: Pose) -> None:
        """Odometry reset: keep the robot's momentum, move it to the stated pose."""
        old_theta = self.theta
        self.x, self.y = pose.x * IN_TO_M, pose.y * IN_TO_M
        self.theta = math.radians(pose.theta if pose.theta is not None else math.degrees(old_theta))
        turn = self.theta - old_theta
        cos_turn, sin_turn = math.cos(turn), math.sin(turn)
        self.vx, self.vy = (self.vx * cos_turn + self.vy * sin_turn, -self.vx * sin_turn + self.vy * cos_turn)

    def side_force(self, power: float, speed: float) -> float:
        if power == 0:
            return 0.0  # drive motors are in coast
        voltage = max(-1.0, min(1.0, power / 127.0))
        force = self.stall_force * (voltage - speed / self.free_speed)
        force = max(-self.stall_force, min(self.stall_force, force))
        return max(-self.traction_force, min(self.traction_force, force))

    def step(self, left_power: float, right_power: float, dt: float) -> None:
        forward = (math.sin(self.theta), math.cos(self.theta))
        right = (math.cos(self.theta), -math.sin(self.theta))
        v_forward = self.vx * forward[0] + self.vy * forward[1]
        v_lateral = self.vx * right[0] + self.vy * right[1]
        left_speed = v_forward + self.omega * self.half_track
        right_speed = v_forward - self.omega * self.half_track
        left_force = self.side_force(left_power, left_speed)
        right_force = self.side_force(right_power, right_speed)

        v_forward += (left_force + right_force) / self.mass * dt
        self.omega += (left_force - right_force) * self.half_track / self.inertia * dt

        roll = ROLLING_RESISTANCE * GRAVITY * dt
        v_forward -= math.copysign(min(abs(v_forward), roll), v_forward)
        spin_roll = ROLLING_RESISTANCE * self.mass * GRAVITY * self.half_track / self.inertia * dt
        self.omega -= math.copysign(min(abs(self.omega), spin_roll), self.omega)
        v_lateral -= math.copysign(min(abs(v_lateral), LATERAL_MU * GRAVITY * dt), v_lateral)

        self.vx = v_forward * forward[0] + v_lateral * right[0]
        self.vy = v_forward * forward[1] + v_lateral * right[1]
        self.theta += self.omega * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.collide_with_walls()

    def collide_with_walls(self) -> None:
        limit = FIELD_SIZE_IN / 2.0 * IN_TO_M
        half_length = ROBOT_LENGTH_IN / 2.0 * IN_TO_M
        half_width = ROBOT_WIDTH_IN / 2.0 * IN_TO_M
        forward = (math.sin(self.theta), math.cos(self.theta))
        right = (math.cos(self.theta), -math.sin(self.theta))
        corners = [
            (self.x + a * half_length * forward[0] + b * half_width * right[0],
             self.y + a * half_length * forward[1] + b * half_width * right[1])
            for a in (-1, 1) for b in (-1, 1)
        ]
        over_x = max(c[0] for c in corners) - limit
        under_x = -limit - min(c[0] for c in corners)
        over_y = max(c[1] for c in corners) - limit
        under_y = -limit - min(c[1] for c in corners)
        if over_x > 0:
            self.x -= over_x
            self.vx = min(self.vx, 0.0)
        elif under_x > 0:
            self.x += under_x
            self.vx = max(self.vx, 0.0)
        if over_y > 0:
            self.y -= over_y
            self.vy = min(self.vy, 0.0)
        elif under_y > 0:
            self.y += under_y
            self.vy = max(self.vy, 0.0)


# ---------------------------------------------------------------- simulator

class Simulator:
    def __init__(self, curve_turns: bool) -> None:
        self.drive = TankDrive()
        self.ctl = Controllers()
        self.curve_turns = curve_turns
        self.time_ms = 0
        self.motion: Motion | None = None
        self.powers = (0.0, 0.0)
        self.label = "idle"
        self.frames: list[Frame] = [Frame(0, self.drive.pose(), 0.0, 0.0, "start")]
        self.log: list[str] = []
        self.timeouts = 0
        self.warnings: list[str] = []

    # -- time
    def tick(self) -> None:
        if self.motion is not None:
            output = self.motion.update(self.drive.pose(), self.time_ms)
            if output is None:
                self.finish_log(self.motion)
                # Chained exits leave the motors at their last command, like LemLib.
                if self.motion.exit_reason not in ("chained", "early exit"):
                    self.powers = (0.0, 0.0)
                self.motion = None
                self.label = "idle"
            else:
                self.powers = output
        dt = CONTROL_PERIOD_MS / 1000.0 / PHYSICS_SUBSTEPS
        for _ in range(PHYSICS_SUBSTEPS):
            self.drive.step(self.powers[0], self.powers[1], dt)
        self.time_ms += CONTROL_PERIOD_MS
        self.frames.append(Frame(self.time_ms, self.drive.pose(), self.powers[0], self.powers[1], self.label))

    def advance_to(self, time_ms: float) -> None:
        while self.time_ms < time_ms and self.time_ms < MAX_SIM_MS:
            self.tick()

    def finish_motion(self) -> None:
        while self.motion is not None and self.time_ms < MAX_SIM_MS:
            self.tick()

    def finish_log(self, motion: Motion) -> None:
        pose = self.drive.pose()
        if motion.exit_reason == "timeout":
            self.timeouts += 1
        self.log.append(
            f"{motion.start_ms / 1000:6.2f}-{self.time_ms / 1000:6.2f}s  {motion.label}: {motion.exit_reason}"
            f"  at ({pose.x:.1f}, {pose.y:.1f}, {pose.theta:.0f}°)"
        )

    # -- program
    def start_motion(self, motion: Motion) -> None:
        self.motion = motion
        self.label = motion.label

    def run_motion(self, factory, thread_ms: float) -> float:
        """requestMotionStart(): wait for the running motion, then start async."""
        self.advance_to(thread_ms)
        if self.motion is not None:
            self.finish_motion()
            thread_ms = max(thread_ms, self.time_ms)
        self.start_motion(factory(self.drive.pose(), self.time_ms))
        return thread_ms + ASYNC_START_MS

    def run_block(self, statements: list[Stmt], thread_ms: float) -> float:
        for stmt in statements:
            if self.time_ms >= MAX_SIM_MS:
                self.warnings.append("Stopped at the 300 s limit.")
                break
            thread_ms = self.run_statement(stmt, thread_ms)
        return thread_ms

    def run_statement(self, stmt: Stmt, thread_ms: float) -> float:
        o = stmt.options
        ctl = self.ctl
        if stmt.kind == "delay":
            return thread_ms + arg(stmt, 0, 0.0)
        if stmt.kind == "waitUntilDone":
            self.advance_to(thread_ms)
            self.finish_motion()
            return max(thread_ms, self.time_ms)
        if stmt.kind == "parallel":
            ends = [thread_ms]
            chassis_blocks = [block for block in stmt.blocks if uses_chassis(block)]
            if len(chassis_blocks) > 1:
                self.warnings.append("A parallel() has chassis calls in several lambdas; they are run in order.")
            for block in stmt.blocks:
                if any(block is chassis_block for chassis_block in chassis_blocks):
                    ends.append(self.run_block(block, thread_ms))
                else:
                    ends.append(thread_ms + sum(arg(s, 0, 0.0) for s in flatten(block) if s.kind == "delay"))
            return max(ends)
        if stmt.kind == "setPose":
            self.advance_to(thread_ms)
            now = self.drive.pose()
            values = [resolve(stmt, index, now) for index in range(3)]
            if any(value is None for value in values):
                return self.skip(stmt, thread_ms)
            self.reset_pose(Pose(*values), "setPose")
            return thread_ms
        if stmt.kind == "setPoseFromGoal":
            if not required(stmt, 2):
                return self.skip(stmt, thread_ms)
            self.advance_to(thread_ms)
            now = self.drive.pose()
            heading = math.radians(now.theta or 0.0)
            side = -1.0 if o.get("forwards", False) else 1.0
            distance = float(o.get("distance", 7.0))
            target = Pose(stmt.args[0] + side * distance * math.sin(heading),
                          stmt.args[1] + side * distance * math.cos(heading), now.theta)
            if math.hypot(target.x - now.x, target.y - now.y) <= float(o.get("maxCorrection", 10.0)):
                self.reset_pose(target, "setPoseFromGoal")
            else:
                self.log.append(f"{self.time_ms / 1000:6.2f}s  setPoseFromGoal: rejected (more than max correction)")
            return thread_ms
        if stmt.kind == "moveToPoint":
            if not required(stmt, 3):
                return self.skip(stmt, thread_ms)
            x, y, timeout = stmt.args[:3]
            return self.run_motion(lambda pose, now: MoveToPoint(
                ctl, pose, now, x, y, timeout, bool(o.get("forwards", True)), float(o.get("maxSpeed", 127)),
                float(o.get("minSpeed", 0)), float(o.get("earlyExitRange", 0))), thread_ms)
        if stmt.kind == "turnToHeading":
            if not required(stmt, 2):
                return self.skip(stmt, thread_ms)
            heading, timeout = stmt.args[:2]
            return self.run_motion(lambda pose, now: Turn(
                ctl, pose, now, timeout, heading=heading, direction=str(o.get("direction", "auto")),
                max_speed=float(o.get("maxSpeed", 127)), min_speed=float(o.get("minSpeed", 0)),
                early_exit_range=float(o.get("earlyExitRange", 0))), thread_ms)
        if stmt.kind == "turnToPoint":
            if not required(stmt, 3):
                return self.skip(stmt, thread_ms)
            x, y, timeout = stmt.args[:3]
            return self.run_motion(lambda pose, now: Turn(
                ctl, pose, now, timeout, point=(x, y), forwards=bool(o.get("forwards", True)),
                direction=str(o.get("direction", "auto")), max_speed=float(o.get("maxSpeed", 127)),
                min_speed=float(o.get("minSpeed", 0)), early_exit_range=float(o.get("earlyExitRange", 0))),
                thread_ms)
        if stmt.kind == "donut":
            if not required(stmt, 2):
                return self.skip(stmt, thread_ms)
            theta, timeout = stmt.args[:2]
            return self.run_motion(lambda pose, now: Donut(
                ctl, pose, now, theta, timeout, str(o.get("direction", "auto")), bool(o.get("forwards", True)),
                float(o.get("fastSpeed", 100)), float(o.get("slowSpeed", 75)),
                float(o.get("earlyExitRange", 0))), thread_ms)
        if stmt.kind == "curve":
            return self.run_curve(stmt, thread_ms)
        return thread_ms

    def run_curve(self, stmt: Stmt, thread_ms: float) -> float:
        """Expand the skills curve() helper the same way the C++ does."""
        if len(stmt.args) < 9 or any(value is None for value in stmt.args[2:9]):
            return self.skip(stmt, thread_ms)
        self.advance_to(thread_ms)
        now = self.drive.pose()
        initial_x = resolve(stmt, 0, now)
        initial_y = resolve(stmt, 1, now)
        initial_x = now.x if initial_x is None else initial_x
        initial_y = now.y if initial_y is None else initial_y
        final_x, final_y, exit_angle, entry_angle, exit_length, entry_length, timeout = stmt.args[2:9]
        speed, max_speed = arg(stmt, 9, 10.0), arg(stmt, 10, 127.0)
        forwards = arg(stmt, 11, 1.0) != 0
        direction = 1.0 if forwards else -1.0
        exit_radians, entry_radians = math.radians(exit_angle), math.radians(entry_angle)
        common = {"forwards": forwards, "maxSpeed": max_speed}
        moves = [
            Stmt("moveToPoint", [initial_x + direction * math.sin(exit_radians) * (exit_length + 2.0),
                                 initial_y + direction * math.cos(exit_radians) * (exit_length + 2.0), 800.0],
                 {**common, "minSpeed": speed, "earlyExitRange": 2.0}),
            Stmt("moveToPoint", [final_x - direction * math.sin(entry_radians) * entry_length,
                                 final_y - direction * math.cos(entry_radians) * entry_length, timeout],
                 {**common, "minSpeed": speed, "earlyExitRange": 6.0}),
        ]
        if self.curve_turns:
            moves += [
                Stmt("turnToPoint", [final_x, final_y, 600.0], {**common, "minSpeed": speed, "earlyExitRange": 2.0}),
                Stmt("moveToPoint", [final_x, final_y, 800.0], common),
            ]
        else:
            moves.append(Stmt("moveToPoint", [final_x, final_y, 800.0], {**common, "minSpeed": speed}))
        return self.run_block(moves, thread_ms)

    def reset_pose(self, pose: Pose, label: str) -> None:
        if len(self.frames) == 1:
            # The first setPose before anything moves is the starting position.
            self.drive.reset(pose)
            self.frames[0] = Frame(0, self.drive.pose(), 0.0, 0.0, "start")
            self.log.append(f"  0.00s  start at ({pose.x:g}, {pose.y:g}, {pose.theta:g}°)")
            return
        before = self.drive.pose()
        self.drive.set_pose(pose)
        after = self.drive.pose()
        moved = math.hypot(after.x - before.x, after.y - before.y)
        turned = abs(wrap_degrees((after.theta or 0.0) - (before.theta or 0.0)))
        self.log.append(f"{self.time_ms / 1000:6.2f}s  {label}: corrected {moved:.1f} in, {turned:.0f}°")
        self.frames.append(Frame(self.time_ms, after, self.powers[0], self.powers[1], label, reset=True))

    def skip(self, stmt: Stmt, thread_ms: float) -> float:
        self.warnings.append(f"Skipped (non-numeric arguments): {stmt.text}")
        return thread_ms

    def run(self, statements: list[Stmt]) -> None:
        thread_ms = self.run_block(statements, 0.0)
        self.advance_to(thread_ms)
        self.finish_motion()


def uses_chassis(block: list[Stmt]) -> bool:
    return any(stmt.kind not in ("delay",) for stmt in flatten(block))


def simulate_program(text: str) -> Simulator:
    statements, curve_turns = parse_program(text)
    simulator = Simulator(curve_turns)
    simulator.run(statements)
    return simulator


def read_pose_csv(path: Path) -> list[Pose]:
    poses: list[Pose] = []
    with path.open("r", newline="", encoding="utf-8-sig") as file:
        lines = (line for line in file if not line.lstrip().startswith("#"))
        reader = csv.DictReader(lines)
        if reader.fieldnames is None:
            raise ValueError("The CSV has no header.")
        required_columns = {"pose_x_in", "pose_y_in"}
        missing = required_columns - set(reader.fieldnames)
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


# ---------------------------------------------------------------- UI

class Visualizer:
    PLAYBACK_TICK_MS = 20

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Crimson Path Visualizer")
        self.root.geometry("1200x800")
        self.csv_poses: list[Pose] = []
        self.path_points: list[PathPoint] = []
        self.frames: list[Frame] = []
        self.current_frame = 0
        self.playing = False
        self.play_job: str | None = None
        self.play_started = (0.0, 0)
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
        ttk.Label(text_frame, text="Paste auton code or load a .txt/.cpp file:").pack(anchor=tk.W)
        self.source = tk.Text(text_frame, width=48, wrap=tk.NONE, undo=True)
        self.source.pack(fill=tk.BOTH, expand=True, pady=(4, 0))
        ttk.Label(text_frame, text="Simulation log:").pack(anchor=tk.W, pady=(6, 0))
        self.log = tk.Text(text_frame, width=48, height=12, wrap=tk.NONE, state=tk.DISABLED)
        self.log.pack(fill=tk.BOTH, pady=(4, 0))
        body.add(text_frame, weight=1)

        canvas_frame = ttk.Frame(body)
        self.canvas = tk.Canvas(canvas_frame, background="#101419", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        body.add(canvas_frame, weight=3)
        timeline = ttk.Frame(root, padding=(8, 0, 8, 8))
        timeline.pack(fill=tk.X)
        self.timeline = ttk.Scale(timeline, from_=0, to=1, orient=tk.HORIZONTAL, command=self.seek)
        self.timeline.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.timeline_label = ttk.Label(timeline, text="No simulation", width=64)
        self.timeline_label.pack(side=tk.LEFT, padx=(8, 0))
        self.canvas.bind("<Configure>", lambda _: self.render())

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
        self.render()

    def load_txt(self) -> None:
        filename = filedialog.askopenfilename(
            title="Load auton commands",
            filetypes=[("Text files", "*.txt"), ("C++ files", "*.cpp"), ("All files", "*.*")],
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

    def set_log(self, lines: list[str]) -> None:
        self.log.configure(state=tk.NORMAL)
        self.log.delete("1.0", tk.END)
        self.log.insert("1.0", "\n".join(lines))
        self.log.configure(state=tk.DISABLED)

    def plot(self) -> None:
        self.pause()
        self.path_points = parse_path_commands(self.source.get("1.0", tk.END))
        self.frames = []
        self.current_frame = 0
        self.timeline.configure(to=1, value=0)
        self.timeline_label.configure(text="No simulation")
        self.render()

    def simulate(self) -> None:
        self.pause()
        text = self.source.get("1.0", tk.END)
        simulator = simulate_program(text)
        if len(simulator.frames) <= 1:
            messagebox.showerror("Simulation", "No supported movement commands were found.")
            return
        self.path_points = parse_path_commands(text)
        self.frames = simulator.frames
        self.current_frame = 0
        self.timeline.configure(to=max(1, len(self.frames) - 1), value=0)
        total = self.frames[-1].time_ms / 1000.0
        self.status.configure(text=f"Simulated {total:.2f} s, {simulator.timeouts} motion timeouts")
        self.set_log(simulator.warnings + simulator.log)
        self.seek("0")

    def seek(self, value: str) -> None:
        if not self.frames:
            return
        self.current_frame = max(0, min(len(self.frames) - 1, int(float(value))))
        frame = self.frames[self.current_frame]
        total = self.frames[-1].time_ms / 1000.0
        self.timeline_label.configure(
            text=f"{frame.time_ms / 1000:6.2f} / {total:.2f} s   L {frame.left:4.0f}  R {frame.right:4.0f}   {frame.label}"
        )
        self.render()

    def play(self) -> None:
        if not self.frames:
            return
        if self.current_frame >= len(self.frames) - 1:
            self.current_frame = 0
        self.playing = True
        self.play_started = (time.perf_counter(), self.frames[self.current_frame].time_ms)
        self.advance_playback()

    def pause(self) -> None:
        self.playing = False
        if self.play_job is not None:
            self.root.after_cancel(self.play_job)
            self.play_job = None

    def step(self) -> None:
        self.pause()
        if self.frames:
            self.seek(str(min(self.current_frame + 1, len(self.frames) - 1)))

    def reset_playback(self) -> None:
        self.pause()
        if self.frames:
            self.seek("0")

    def advance_playback(self) -> None:
        if not self.playing or not self.frames:
            return
        if self.current_frame >= len(self.frames) - 1:
            self.pause()
            return
        # Play in real time, skipping frames if drawing falls behind the wall clock.
        wall_start, sim_start = self.play_started
        target_ms = sim_start + (time.perf_counter() - wall_start) * 1000.0
        index = self.current_frame + 1
        while index < len(self.frames) - 1 and self.frames[index].time_ms < target_ms:
            index += 1
        self.timeline.set(index)
        self.seek(str(index))
        self.play_job = self.root.after(self.PLAYBACK_TICK_MS, self.advance_playback)

    def render(self) -> None:
        self.canvas.delete("all")
        half = FIELD_SIZE_IN / 2.0
        samples = (
            self.csv_poses
            + [frame.pose for frame in self.frames]
            + [point.pose for point in self.path_points]
            + [Pose(-half, -half), Pose(half, half)]
        )
        width = max(self.canvas.winfo_width(), 400)
        height = max(self.canvas.winfo_height(), 300)
        min_x = min(p.x for p in samples) - 6
        max_x = max(p.x for p in samples) + 6
        min_y = min(p.y for p in samples) - 6
        max_y = max(p.y for p in samples) + 6
        scale = min((width - 80) / (max_x - min_x), (height - 80) / (max_y - min_y))

        def screen(point: Pose) -> tuple[float, float]:
            return 40 + (point.x - min_x) * scale, height - 40 - (point.y - min_y) * scale

        self.draw_grid(width, height, min_x, max_x, min_y, max_y, screen)
        corner_a, corner_b = screen(Pose(-half, -half)), screen(Pose(half, half))
        self.canvas.create_rectangle(*corner_a, *corner_b, outline="#8a96a3", width=2)
        if len(self.csv_poses) > 1:
            self.draw_trace(self.csv_poses, screen, "#41d17d", "Telemetry", 12)
        if self.frames:
            visible = self.frames[: self.current_frame + 1]
            segments: list[list[Pose]] = [[]]
            for frame in visible:
                if frame.reset and segments[-1]:
                    segments.append([])
                segments[-1].append(frame.pose)
            for segment in segments:
                if len(segment) > 1:
                    self.draw_trace(segment, screen, "#72b7ff", "Simulation", 90)
            for frame in visible:
                if frame.reset:
                    sx, sy = screen(frame.pose)
                    self.canvas.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, outline="#ff7a90", width=2)
            self.draw_robot_box(visible[-1].pose, screen, scale, "#72b7ff")
        self.draw_path(screen)
        if self.path_points:
            self.canvas.create_text(180, 12, anchor=tk.NW, fill="#ffbf69", text="Raw path")
        if not (self.csv_poses or self.frames or self.path_points):
            self.canvas.create_text(20, 20, anchor=tk.NW, fill="white",
                                    text="Load a telemetry CSV or paste auton code to begin.")

    def draw_trace(self, poses: list[Pose], screen, color: str, label: str, label_x: int) -> None:
        coordinates = [value for pose in poses for value in screen(pose)]
        self.canvas.create_line(*coordinates, fill=color, width=2)
        self.canvas.create_text(label_x, 12, anchor=tk.NW, fill=color, text=label)

    def draw_grid(self, width: int, height: int, min_x: float, max_x: float,
                  min_y: float, max_y: float, screen) -> None:
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
            current = screen(point.pose)
            if point.kind in ("move", "pose", "curve"):
                if previous is not None:
                    self.canvas.create_line(*previous, *current, fill="#ffbf69", width=2, dash=(6, 4))
                previous = current
            radius = 7 if point.kind in ("pose", "goal") else 3 if point.kind == "curve" else 5
            outline = "#ff7a90" if point.kind == "goal" else "#fff1d6"
            fill = "" if point.kind in ("turn", "goal") else "#ffbf69"
            self.canvas.create_oval(current[0] - radius, current[1] - radius,
                                    current[0] + radius, current[1] + radius, fill=fill, outline=outline)
            self.canvas.create_text(current[0] + 8, current[1] - 8, anchor=tk.SW,
                                    fill="#ffdfaa", text=f"{index}: {point.label}")

    def draw_robot_box(self, pose: Pose, screen, scale: float, color: str) -> None:
        """Draw the robot footprint with its front aligned to theta."""
        center_x, center_y = screen(pose)
        radians = math.radians(pose.theta or 0.0)
        forward = (math.sin(radians), -math.cos(radians))
        right = (math.cos(radians), math.sin(radians))
        half_length = ROBOT_LENGTH_IN / 2.0 * scale
        half_width = ROBOT_WIDTH_IN / 2.0 * scale
        corners = []
        for forward_sign, right_sign in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            corners.extend((
                center_x + forward_sign * half_length * forward[0] + right_sign * half_width * right[0],
                center_y + forward_sign * half_length * forward[1] + right_sign * half_width * right[1],
            ))
        self.canvas.create_polygon(*corners, outline=color, fill="", width=2)
        front_x = center_x + forward[0] * half_length * 1.3
        front_y = center_y + forward[1] * half_length * 1.3
        self.canvas.create_line(center_x, center_y, front_x, front_y, fill=color, width=3, arrow=tk.LAST)


def main() -> None:
    root = tk.Tk()
    Visualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()
