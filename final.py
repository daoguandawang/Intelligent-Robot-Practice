import os
import time
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
#纯闭环版本，打滑块用pos开环
import cv2
import numpy as np

try:
    import serial
except ImportError:
    serial = None

from open_test import (  # reuse proven implementations
    AxisPID,
    DirectPosPIDController,
    MultiProfileDetector,
    OpenCommandUI,
    ROISelectorUI,
    RedLaserDetector,
    SerialPIDSender,
    TargetTunerUI,
    UpperPIDController,
    _load_dense_pos_table,
    _lookup_pos_from_dense_table,
)


Point = Tuple[int, int]
PosCmd = Tuple[int, int]
CAL_TABLE_DENSE_CSV = "calibration_table_dense_step1.csv"
POS_MAX_VALUE = 3200
POS_DEFAULT_YAW = 200
POS_DEFAULT_PITCH = 470


def nothing(_=None):
    pass


@dataclass
class RedProfile:
    name: str
    target: Dict[str, int]
    block: Dict[str, int]


@dataclass
class DetectionResult:
    target_center: Optional[Point]
    block_center: Optional[Point]
    target_mask: np.ndarray
    block_mask: np.ndarray


@dataclass
class TaskStep:
    label: str
    profile_idx: int
    kind: str  # "target" or "block"
    rough_positions: List[PosCmd] = field(default_factory=list)


@dataclass
class TaskExecutor:
    open_loop_wait_s: float = 0.18
    align_tol_x: int = 15
    align_tol_y: int = 15
    stable_frames_required: int = 6
    align_timeout_s: float = 4.0
    hold_after_hit_s: float = 1.5
    hold_tol_x: int = 10
    hold_tol_y: int = 10
    near_hit_tol_x: int = 12
    near_hit_tol_y: int = 12
    laser_loss_grace_s: float = 0.35
    laser_loss_success_s: float = 3.0
    lost_visual_frames_allowed: int = 4
    roi_guard_margin: int = 18
    block_yaw_tol_x: int = 8
    block_fixed_yaw: int = 400
    block_fixed_pitch: int = 660    
    block_pitch_levels: Tuple[int, int, int] = (440, 520, 575)
    block_pitch_feedback_tol: int = 8
    tangent_lead_pixels: float = 10.0
    tangent_clockwise: bool = False
    rotation_center: Optional[Point] = None

    state: str = "IDLE"
    steps: List[TaskStep] = field(default_factory=list)
    step_index: int = 0
    rough_index: int = 0
    stable_frames: int = 0
    state_since: float = 0.0

    last_message: str = "IDLE"
    last_error: Optional[Tuple[int, int]] = None
    last_near_hit_ts: float = -1.0
    lost_visual_frames: int = 0 
    target_stable_since: Optional[float] = None
    target_laser_missing_since: Optional[float] = None
    block_last_yaw: int = POS_DEFAULT_YAW
    block_pitch_target: Optional[int] = None
    block_pitch_stable_since: Optional[float] = None
    block_last_centers: Dict[int, Point] = field(default_factory=dict)

    def reset(self) -> None:
        self.state = "IDLE"
        self.steps = []
        self.step_index = 0
        self.rough_index = 0
        self.stable_frames = 0
        self.state_since = 0.0
        self.last_message = "IDLE"
        self.last_error = None
        self.last_near_hit_ts = -1.0
        self.lost_visual_frames = 0
        self.target_stable_since = None
        self.target_laser_missing_since = None
        self.block_last_yaw = POS_DEFAULT_YAW
        self.block_pitch_target = None
        self.block_pitch_stable_since = None
        self.block_last_centers = {}

    def is_busy(self) -> bool:
        return self.state not in ("IDLE", "DONE")

    def current_step(self) -> Optional[TaskStep]:
        if 0 <= self.step_index < len(self.steps):
            return self.steps[self.step_index]
        return None

    def _get_target(self, step: TaskStep, results: List[DetectionResult]) -> Optional[Point]:
        if not (0 <= step.profile_idx < len(results)):
            return None
        r = results[step.profile_idx]
        return r.target_center if step.kind == "target" else r.block_center

    def _get_block_pitch_target(self, step: TaskStep, results: List[DetectionResult]) -> int:
        visible_blocks: List[Tuple[int, int]] = []
        for idx, result in enumerate(results):
            if result.block_center is not None:
                visible_blocks.append((idx, result.block_center[1]))
            elif idx in self.block_last_centers:
                visible_blocks.append((idx, self.block_last_centers[idx][1]))

        visible_blocks.sort(key=lambda item: item[1])
        for rank, (idx, _) in enumerate(visible_blocks):
            if idx == step.profile_idx and rank < len(self.block_pitch_levels):
                return self.block_pitch_levels[rank]

        fallback_idx = min(max(step.profile_idx, 0), len(self.block_pitch_levels) - 1)
        return self.block_pitch_levels[fallback_idx]

    def build_steps(self, parsed_colors: List[str], rough_positions: Dict[Tuple[int, str], List[PosCmd]]) -> List[TaskStep]:
        color_to_idx = {"red": 0, "green": 1, "blue": 2}
        steps: List[TaskStep] = []
        for c in parsed_colors:
            idx = color_to_idx.get(c)
            if idx is None:
                continue
            steps.append(
                TaskStep(
                    label=f"{c.upper()} P{idx + 1} BLOCK",
                    profile_idx=idx,
                    kind="block",
                    rough_positions=list(rough_positions.get((idx, "block"), [])),
                )
            )
            steps.append(
                TaskStep(
                    label=f"{c.upper()} P{idx + 1} TARGET",
                    profile_idx=idx,
                    kind="target",
                    rough_positions=list(rough_positions.get((idx, "target"), [])),
                )
            )
        return steps

    def start(self, steps: List[TaskStep], now: float) -> bool:
        if not steps:
            return False
        self.steps = steps
        self.state = "OPEN_LOOP"
        self.step_index = 0
        self.rough_index = 0
        self.stable_frames = 0
        self.state_since = now
        self.last_message = f"Start {steps[0].label}"
        self.last_error = None
        self.last_near_hit_ts = -1.0
        self.lost_visual_frames = 0
        self.target_stable_since = None
        self.target_laser_missing_since = None
        self.block_last_yaw = POS_DEFAULT_YAW
        self.block_pitch_target = None
        self.block_pitch_stable_since = None
        self.block_last_centers = {}
        return True

    def stop(self, sender: SerialPIDSender, speed_pid: UpperPIDController, dpos_pid: DirectPosPIDController) -> None:
        sender.send_stop()
        speed_pid.reset()
        dpos_pid.reset()
        self.reset()

    def _finish_task(
        self,
        sender: SerialPIDSender,
        speed_pid: UpperPIDController,
        dpos_pid: DirectPosPIDController,
    ) -> None:
        sender.send_stop()
        speed_pid.reset()
        dpos_pid.reset()
        sender.send_pos(0, 0)
        self.state = "DONE"
        self.last_message = "Task done -> POS 0,0"
        self.stable_frames = 0
        self.rough_index = 0
        self.target_stable_since = None
        self.target_laser_missing_since = None
        self.block_pitch_stable_since = None
        self.block_pitch_target = None

    def update(
        self,
        now: float,
        sender: SerialPIDSender,
        results: List[DetectionResult],
        laser_center: Optional[Point],
        speed_pid: UpperPIDController,
        dpos_pid: DirectPosPIDController,
        pid_mode: str,
        roi_bounds: Optional[Tuple[int, int, int, int]] = None,
    ) -> Optional[Tuple[int, ...]]:
        if self.state in ("IDLE", "DONE"):
            return None

        step = self.current_step()
        if step is None:
            self._finish_task(sender, speed_pid, dpos_pid)
            return None

        for idx, result in enumerate(results):
            if result.block_center is not None:
                self.block_last_centers[idx] = result.block_center

        target = self._get_target(step, results)
        if step.kind == "target" and target is not None:
            target = lead_rotating_target(target, self.rotation_center, self.tangent_lead_pixels, self.tangent_clockwise)
        pid_cmd: Optional[Tuple[int, ...]] = None

        if self.state == "OPEN_LOOP":
            if self.rough_index < len(step.rough_positions):
                yaw, pitch = step.rough_positions[self.rough_index]
                sender.send_pos(yaw, pitch)
                self.block_last_yaw = yaw
                self.last_message = f"{step.label}: POS {self.rough_index + 1}/{len(step.rough_positions)} -> {yaw},{pitch}"
                self.rough_index += 1
                self.state = "OPEN_LOOP_WAIT"
                self.state_since = now
                return None

            if step.kind == "block":
                sender.send_pos(self.block_fixed_yaw, self.block_fixed_pitch)
                self.block_last_yaw = self.block_fixed_yaw
                self.rough_index = 0
                self.stable_frames = 0
                self.lost_visual_frames = 0
                self.state = "BLOCK_PITCH_PRESET_WAIT"
                self.state_since = now
                self.last_message = f"{step.label}: preset yaw/pitch -> {self.block_fixed_yaw},{self.block_fixed_pitch}"
                return None

            self.state = "WAIT_REACQUIRE"
            self.state_since = now
            self.last_message = f"{step.label}: wait target and laser"
            return None

        if self.state == "OPEN_LOOP_WAIT":
            if (now - self.state_since) >= self.open_loop_wait_s:
                self.state = "OPEN_LOOP"
                self.state_since = now
            return None

        if self.state == "BLOCK_PITCH_PRESET_WAIT":
            if (now - self.state_since) >= self.open_loop_wait_s:
                self.state = "BLOCK_WAIT_REACQUIRE_YAW"
                self.state_since = now
                self.lost_visual_frames = 0
                self.last_message = f"{step.label}: wait target and laser for yaw"
            return None

        if self.state == "BLOCK_WAIT_REACQUIRE_YAW":
            if target is not None and laser_center is not None:
                self.state = "BLOCK_ALIGN_YAW"
                self.state_since = now
                self.stable_frames = 0
                self.lost_visual_frames = 0
                self.last_message = f"{step.label}: yaw align"
            return None

        if self.state == "BLOCK_WAIT_REACQUIRE_PITCH":
            if target is not None or step.profile_idx in self.block_last_centers:
                target_pitch = self._get_block_pitch_target(step, results)
                current_yaw = self.block_last_yaw
                if getattr(sender, "current_pos", None) is not None:
                    current_yaw = int(sender.current_pos[0])
                sender.send_pos(current_yaw, target_pitch)
                self.block_last_yaw = current_yaw
                self.block_pitch_target = target_pitch
                self.block_pitch_stable_since = None
                self.state = "BLOCK_PITCH_SETTLE"
                self.state_since = now
                self.stable_frames = 0
                self.lost_visual_frames = 0
                self.last_message = f"{step.label}: pitch open-loop -> {target_pitch}"
            return None

        if self.state == "BLOCK_PITCH_SETTLE":
            current_pos = getattr(sender, "current_pos", None)
            if current_pos is None or self.block_pitch_target is None:
                self.block_pitch_stable_since = None
                self.last_message = f"{step.label}: wait POS feedback"
                return None

            pitch_err = self.block_pitch_target - int(current_pos[1])
            self.last_error = (0, pitch_err)
            if abs(pitch_err) <= self.block_pitch_feedback_tol:
                if self.block_pitch_stable_since is None:
                    self.block_pitch_stable_since = now
                dwell_elapsed = now - self.block_pitch_stable_since
                self.last_message = f"{step.label}: pitch fb dwell={dwell_elapsed:.1f}/{self.hold_after_hit_s:.1f}s err=({pitch_err:+d})"
                if dwell_elapsed >= self.hold_after_hit_s:
                    self.rough_index = 0
                    self.stable_frames = 0
                    self.block_pitch_stable_since = None
                    self.block_pitch_target = None
                    self.state = "HOLD_BLOCK"
                    self.state_since = now
                    self.last_message = f"{step.label}: hold 0.0/{self.hold_after_hit_s:.1f}s"
                return None

            self.block_pitch_stable_since = None
            self.last_message = f"{step.label}: pitch fb err=({pitch_err:+d})"
            return None

        if self.state == "WAIT_REACQUIRE":
            if target is not None and laser_center is not None:
                self.state = "CLOSED_LOOP"
                self.state_since = now
                self.stable_frames = 0
                self.lost_visual_frames = 0
                self.target_stable_since = None
                self.target_laser_missing_since = None
                self.last_message = f"{step.label}: closed loop"
            return None

        if self.state == "HOLD_BLOCK":
            elapsed = now - self.state_since
            if elapsed >= self.hold_after_hit_s:
                self.step_index += 1
                self.rough_index = 0
                self.stable_frames = 0
                self.state_since = now
                if self.step_index >= len(self.steps):
                    self._finish_task(sender, speed_pid, dpos_pid)
                else:
                    self.state = "OPEN_LOOP"
                    self.last_message = f"Next {self.steps[self.step_index].label}"
                return None

            self.last_message = f"{step.label}: hold {elapsed:.1f}/{self.hold_after_hit_s:.1f}s"
            return None

        if step.kind == "target" and target is not None and laser_center is None:
            if self.target_laser_missing_since is None:
                self.target_laser_missing_since = now
            if (
                self.target_stable_since is not None
                and self.last_near_hit_ts > 0
                and (now - self.last_near_hit_ts) <= self.laser_loss_grace_s
            ):
                dwell_elapsed = now - self.target_stable_since
                if dwell_elapsed >= self.hold_after_hit_s:
                    self.step_index += 1
                    self.rough_index = 0
                    self.stable_frames = 0
                    self.state_since = now
                    self.target_stable_since = None
                    self.lost_visual_frames = 0
                    if self.step_index >= len(self.steps):
                        self._finish_task(sender, speed_pid, dpos_pid)
                    else:
                        self.state = "OPEN_LOOP"
                        self.last_message = f"Next {self.steps[self.step_index].label}"
                else:
                    self.last_message = f"{step.label}: dwell {dwell_elapsed:.1f}/{self.hold_after_hit_s:.1f}s (laser grace)"
                return None

            if self.last_near_hit_ts > 0 and (now - self.last_near_hit_ts) <= self.laser_loss_success_s + self.laser_loss_grace_s:
                missing_elapsed = now - self.target_laser_missing_since
                if missing_elapsed >= self.laser_loss_success_s:
                    sender.send_stop()
                    speed_pid.reset()
                    dpos_pid.reset()
                    self.step_index += 1
                    self.rough_index = 0
                    self.stable_frames = 0
                    self.state_since = now
                    self.target_stable_since = None
                    self.target_laser_missing_since = None
                    self.lost_visual_frames = 0
                    if self.step_index >= len(self.steps):
                        self._finish_task(sender, speed_pid, dpos_pid)
                    else:
                        self.state = "OPEN_LOOP"
                        self.last_message = f"Next {self.steps[self.step_index].label}"
                else:
                    self.last_message = (
                        f"{step.label}: laser lost dwell "
                        f"{missing_elapsed:.1f}/{self.laser_loss_success_s:.1f}s"
                    )
                return None
        else:
            self.target_laser_missing_since = None

        if target is None or laser_center is None:
            self.lost_visual_frames += 1
            self.target_stable_since = None
            if self.lost_visual_frames <= self.lost_visual_frames_allowed:
                self.last_message = (
                    f"{step.label}: target/laser lost "
                    f"{self.lost_visual_frames}/{self.lost_visual_frames_allowed}"
                )
                return None
            sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()
            if self.state == "BLOCK_ALIGN_YAW":
                self.state = "BLOCK_WAIT_REACQUIRE_YAW"
            elif self.state in ("BLOCK_ALIGN_PITCH", "BLOCK_PITCH_SETTLE"):
                self.state = "BLOCK_WAIT_REACQUIRE_PITCH"
            else:
                self.state = "WAIT_REACQUIRE"
            self.state_since = now
            self.stable_frames = 0
            self.lost_visual_frames = 0
            self.last_message = f"{step.label}: target/laser lost -> reacquire"
            return None

        self.lost_visual_frames = 0

        err_x = target[0] - laser_center[0]
        err_y = target[1] - laser_center[1]
        self.last_error = (err_x, err_y)
        if abs(err_x) <= self.near_hit_tol_x and abs(err_y) <= self.near_hit_tol_y:
            self.last_near_hit_ts = now

        block_left = block_right = block_up = block_down = False
        if roi_bounds is not None:
            roi_x, roi_y, roi_w, roi_h = roi_bounds
            roi_left = roi_x + self.roi_guard_margin
            roi_right = roi_x + roi_w - 1 - self.roi_guard_margin
            roi_top = roi_y + self.roi_guard_margin
            roi_bottom = roi_y + roi_h - 1 - self.roi_guard_margin

            block_left = laser_center[0] <= roi_left and err_x < 0
            block_right = laser_center[0] >= roi_right and err_x > 0
            block_up = laser_center[1] <= roi_top and err_y < 0
            block_down = laser_center[1] >= roi_bottom and err_y > 0

        if pid_mode == "speed":
            pid_cmd = speed_pid.update(target, laser_center, now)
            if pid_cmd is not None:
                send_yaw = -pid_cmd[0]
                send_pitch = -pid_cmd[1]
                if self.state == "BLOCK_ALIGN_YAW":
                    send_pitch = 0
                elif self.state == "BLOCK_ALIGN_PITCH":
                    send_yaw = 0
                if block_left or block_right:
                    send_yaw = 0
                if block_up or block_down:
                    send_pitch = 0
                sender.send_speed(send_yaw, send_pitch)
        else:
            pid_cmd = dpos_pid.update(target, laser_center, now)
            if pid_cmd is not None:
                send_yaw = -pid_cmd[0]
                send_pitch = -pid_cmd[1]
                if self.state == "BLOCK_ALIGN_YAW":
                    send_pitch = 0
                elif self.state == "BLOCK_ALIGN_PITCH":
                    send_yaw = 0
                if block_left or block_right:
                    send_yaw = 0
                if block_up or block_down:
                    send_pitch = 0
                sender.send_dpos(send_yaw, send_pitch, pid_cmd[2])

        within_tol = abs(err_x) <= self.align_tol_x and abs(err_y) <= self.align_tol_y
        if self.state == "BLOCK_ALIGN_YAW":
            axis_within_tol = abs(err_x) <= self.block_yaw_tol_x
        elif self.state == "BLOCK_ALIGN_PITCH":
            axis_within_tol = abs(err_y) <= self.align_tol_y
        else:
            axis_within_tol = within_tol
        self.stable_frames = self.stable_frames + 1 if axis_within_tol else 0

        if step.kind == "target":
            if within_tol:
                if self.target_stable_since is None:
                    self.target_stable_since = now
                dwell_elapsed = now - self.target_stable_since
                self.last_message = f"{step.label}: err=({err_x:+d},{err_y:+d}) dwell={dwell_elapsed:.1f}/{self.hold_after_hit_s:.1f}s"
            else:
                self.target_stable_since = None
                self.last_message = f"{step.label}: err=({err_x:+d},{err_y:+d}) track"
        elif self.state == "BLOCK_ALIGN_YAW":
            self.last_message = f"{step.label}: yaw err=({err_x:+d}) stable={self.stable_frames}/{self.stable_frames_required}"
        else:
            self.last_message = f"{step.label}: err=({err_x:+d},{err_y:+d}) stable={self.stable_frames}/{self.stable_frames_required}"

        if block_left or block_right or block_up or block_down:
            self.last_message += " roi-guard"

        if step.kind == "target":
            if self.target_stable_since is not None and (now - self.target_stable_since) >= self.hold_after_hit_s:
                sender.send_stop()
                speed_pid.reset()
                dpos_pid.reset()
                self.step_index += 1
                self.rough_index = 0
                self.stable_frames = 0
                self.state_since = now
                self.target_stable_since = None
                if self.step_index >= len(self.steps):
                    self._finish_task(sender, speed_pid, dpos_pid)
                else:
                    self.state = "OPEN_LOOP"
                    self.last_message = f"Next {self.steps[self.step_index].label}"
                return pid_cmd
        elif self.state == "BLOCK_ALIGN_YAW" and self.stable_frames >= self.stable_frames_required:
            sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()
            self.stable_frames = 0
            self.state = "BLOCK_WAIT_REACQUIRE_PITCH"
            self.state_since = now
            self.lost_visual_frames = 0
            self.last_message = f"{step.label}: yaw done, wait for pitch preset"
            return pid_cmd
        elif self.stable_frames >= self.stable_frames_required:
            sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()
            self.rough_index = 0
            self.stable_frames = 0
            self.state_since = now
            self.step_index += 1
            if self.step_index >= len(self.steps):
                self._finish_task(sender, speed_pid, dpos_pid)
            else:
                self.state = "OPEN_LOOP"
                self.last_message = f"Next {self.steps[self.step_index].label}"
            return pid_cmd

        if (now - self.state_since) > self.align_timeout_s:
            sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()
            if self.state == "BLOCK_ALIGN_YAW":
                self.state = "BLOCK_WAIT_REACQUIRE_YAW"
            elif self.state in ("BLOCK_ALIGN_PITCH", "BLOCK_PITCH_SETTLE"):
                self.state = "BLOCK_WAIT_REACQUIRE_PITCH"
            else:
                self.state = "WAIT_REACQUIRE"
            self.state_since = now
            self.stable_frames = 0
            self.lost_visual_frames = 0
            self.block_pitch_stable_since = None
            self.last_message = f"{step.label}: align timeout, retry"

        return pid_cmd


def normalize_qr_text(text: str) -> List[str]:
    compact = text.strip().replace(" ", "").replace(",", "").replace("，", "")
    if not compact:
        return []

    char_map = {"红": "red", "绿": "green", "蓝": "blue", "R": "red", "G": "green", "B": "blue", "r": "red", "g": "green", "b": "blue"}
    colors: List[str] = []
    i = 0
    while i < len(compact):
        matched = False
        for token, color in (("red", "red"), ("green", "green"), ("blue", "blue")):
            if compact[i : i + len(token)].lower() == token:
                colors.append(color)
                i += len(token)
                matched = True
                break
        if matched:
            continue

        color = char_map.get(compact[i])
        if color is not None:
            colors.append(color)
        i += 1
    return colors


def detect_qr_text(frame: np.ndarray, detector: cv2.QRCodeDetector) -> Tuple[str, Optional[np.ndarray]]:
    text, points, _ = detector.detectAndDecode(frame)
    return (text, points) if text else ("", None)


def estimate_rotation_center(results: List[DetectionResult], previous_center: Optional[Point] = None) -> Optional[Point]:
    pts = [r.target_center for r in results if r.target_center is not None]
    if len(pts) >= 3:
        return (
            int(round(sum(p[0] for p in pts[:3]) / 3.0)),
            int(round(sum(p[1] for p in pts[:3]) / 3.0)),
        )
    return previous_center


def lead_rotating_target(target: Point, center: Optional[Point], lead_pixels: float, clockwise: bool) -> Point:
    if center is None or lead_pixels <= 0:
        return target

    dx = float(target[0] - center[0])
    dy = float(target[1] - center[1])
    radius = math.hypot(dx, dy)
    if radius < 1e-6:
        return target

    tx, ty = (dy / radius, -dx / radius) if clockwise else (-dy / radius, dx / radius)
    return int(round(target[0] + tx * lead_pixels)), int(round(target[1] + ty * lead_pixels))


def build_profiles() -> List[RedProfile]:
    return [
        RedProfile(
            name="P1",
            target={"H1_min": 0, "H1_max": 10, "H2_min": 156, "H2_max": 180, "S_min": 122, "S_max": 255, "V_min": 32, "V_max": 255, "Kernel": 2, "Dilate": 3, "MinArea": 8000, "MaxArea": 14099},
            block={"H1_min": 0, "H1_max": 10, "H2_min": 156, "H2_max": 180, "S_min": 122, "S_max": 255, "V_min": 68, "V_max": 255, "Kernel": 5, "OpenIter": 2, "CloseIter": 2, "MinArea": 700, "MaxArea": 12000},
        ),
        RedProfile(
            name="P2",
            target={"H1_min": 180, "H1_max": 180, "H2_min": 45, "H2_max": 70, "S_min": 54, "S_max": 144, "V_min": 26, "V_max": 84, "Kernel": 2, "Dilate": 3, "MinArea": 8000, "MaxArea": 13000},
            block={"H1_min": 34, "H1_max": 59, "H2_min": 0, "H2_max": 0, "S_min": 126, "S_max": 242, "V_min": 18, "V_max": 66, "Kernel": 4, "OpenIter": 1, "CloseIter": 10, "MinArea": 700, "MaxArea": 12000},
        ),
        RedProfile(
            name="P3",
            target={"H1_min": 180, "H1_max": 180, "H2_min": 110, "H2_max": 141, "S_min": 57, "S_max": 134, "V_min": 26, "V_max": 86, "Kernel": 6, "Dilate": 1, "MinArea": 8000, "MaxArea": 12000},
            block={"H1_min": 111, "H1_max": 145, "H2_min": 0, "H2_max": 0, "S_min": 170, "S_max": 235, "V_min": 0, "V_max": 82, "Kernel": 2, "OpenIter": 1, "CloseIter": 4, "MinArea": 700, "MaxArea": 12000},
        ),
    ]


def build_default_rough_positions() -> Dict[Tuple[int, str], List[PosCmd]]:
    # Replace these zeros with the rough POS values you measured by hand.
    return {
        (0, "target"): [ (150, 400)],
        (0, "block"): [(375, 500)],
        (1, "target"): [ (150, 400)],
        (1, "block"): [(375, 600)],
        (2, "target"): [ (150, 400)],
        (2, "block"): [ (375, 400)],
    }


def draw_profile_result(frame: np.ndarray, result: DetectionResult, profile_idx: int, color_target, color_block) -> None:
    if result.target_center is not None:
        cv2.circle(frame, result.target_center, 7, color_target, -1)
        cv2.putText(frame, f"Target{profile_idx}: {result.target_center}", (result.target_center[0] - 55, result.target_center[1] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_target, 2)
    if result.block_center is not None:
        cv2.circle(frame, result.block_center, 7, color_block, -1)
        cv2.putText(frame, f"Block{profile_idx}: {result.block_center}", (result.block_center[0] - 55, result.block_center[1] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_block, 2)


def draw_qr(frame: np.ndarray, text: str, points: Optional[np.ndarray]) -> None:
    if points is not None and len(points) > 0:
        pts = np.int32(points).reshape(-1, 2)
        cv2.polylines(frame, [pts], True, (255, 255, 255), 2)
    if text:
        cv2.putText(frame, f"QR: {text}", (15, 138), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)


def show_p3_target_mask(results: List[DetectionResult], show_mask: bool) -> None:
    win = "P2 Target Mask"
    if not show_mask:
        try:
            cv2.destroyWindow(win)
        except cv2.error:
            pass
        return

    if len(results) < 2:
        return

    mask = results[1].target_mask
    h, w = mask.shape[:2]
    small = cv2.resize(mask, (max(1, w // 2), max(1, h // 2)), interpolation=cv2.INTER_NEAREST)
    cv2.imshow(win, small)


def main() -> None:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open camera.")
        return

    profiles = build_profiles()
    block_positions = build_default_rough_positions()
    dense_table, dense_bounds = _load_dense_pos_table(CAL_TABLE_DENSE_CSV)
    multi = MultiProfileDetector(profiles)
    laser = RedLaserDetector(history_len=2)
    laser.hsv_mask = lambda hsv, lower, upper, _orig=laser.hsv_mask: cv2.dilate(_orig(hsv, lower, upper), np.ones((3, 3), np.uint8), iterations=3)
    laser.apply_auto_exposure(cap)
    sender = SerialPIDSender(port="COM8", baudrate=115200, enabled=True)
    qr_detector = cv2.QRCodeDetector()
    executor = TaskExecutor()
    target_ui = TargetTunerUI(profiles)

    speed_pid = UpperPIDController(
        yaw=AxisPID(kp=2.8 ,ki=0.03, kd=0.85, integral_limit=90.0, output_limit=1200.0, deadband=4.0, min_abs_output=30.0),
        pitch=AxisPID(kp=2.8, ki=0.03, kd=0.85, integral_limit=90.0, output_limit=1200.0, deadband=4.0, min_abs_output=30.0),
        update_interval_s=0.04,
    )
    dpos_pid = DirectPosPIDController(
        yaw=AxisPID(kp=0.32, ki=0.012, kd=0.10, integral_limit=120.0, output_limit=80.0, deadband=4.0),
        pitch=AxisPID(kp=0.28, ki=0.010, kd=0.08, integral_limit=120.0, output_limit=80.0, deadband=4.0),
        update_interval_s=0.04,
        dpos_min_freq_hz=300,
        dpos_max_freq_hz=1800,
        dpos_max_step_per_cycle=80,
    )

    pid_output_mode = "speed"
    roi_ui: Optional[ROISelectorUI] = None
    open_ui: Optional[OpenCommandUI] = None
    latest_laser_center: Optional[Point] = None
    rotation_center: Optional[Point] = None
    cached_qr_text = ""
    cached_qr_points: Optional[np.ndarray] = None
    cached_parsed_colors: List[str] = []
    manual_profile_idx = 0
    manual_kind = "block"
    target_tuner_visible = False
    show_p2_target_mask = False

    colors_target = [(0, 255, 0), (255, 255, 0), (255, 0, 255)]
    colors_block = [(0, 255, 255), (255, 128, 0), (128, 255, 0)]

    if dense_table:
        print(f"Dense table loaded: {CAL_TABLE_DENSE_CSV} ({len(dense_table)} points)")
    else:
        print(f"Warning: dense table not found or empty: {CAL_TABLE_DENSE_CSV}")
    print("Keys: q quit, g start QR task, x stop task, y switch SPEED/DPOS")
    print("      1/2/3 choose profile, b/t choose block or target")
    print("      o send fixed block POS, u set pixel slider to selected target, i/j/k/l nudge pixel")
    print("      v toggle P2 target tuner+mask, s print target threshold params")

    while True:
        sender.process_incoming(latest_laser_center)
        if target_tuner_visible:
            target_ui.update_profiles_from_ui()
            for win in ("P1 Target Tuner", "P3 Target Tuner"):
                try:
                    cv2.destroyWindow(win)
                except cv2.error:
                    pass

        ok, frame = cap.read()
        if not ok:
            print("Error: cannot read frame.")
            break

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (960, int((960 / w) * h)))

        if roi_ui is None:
            roi_ui = ROISelectorUI(frame_w=frame.shape[1], frame_h=frame.shape[0])
            roi_ui.create()
        if open_ui is None:
            open_ui = OpenCommandUI(frame_w=frame.shape[1], frame_h=frame.shape[0])
            open_ui.create()

        roi_x, roi_y, roi_w, roi_h = roi_ui.get_roi(frame.shape[1], frame.shape[0])
        open_x, open_y = open_ui.get_xy(frame.shape[1], frame.shape[0])
        pos_cmd_preview, pos_query_xy = _lookup_pos_from_dense_table(dense_table, dense_bounds, open_x, open_y)
        roi_frame = frame[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
        vis = frame.copy()

        roi_results = multi.detect_all(roi_frame)
        laser_center_roi, _ = laser.detect(roi_frame)
        qr_text, qr_points = detect_qr_text(frame, qr_detector)
        if qr_text:
            if qr_text != cached_qr_text:
                print(f"QR updated: {qr_text}")
            cached_qr_text = qr_text
            cached_qr_points = qr_points
            cached_parsed_colors = normalize_qr_text(qr_text)

        qr_text = cached_qr_text
        qr_points = cached_qr_points
        parsed_colors = cached_parsed_colors

        results: List[DetectionResult] = []
        for r in roi_results:
            target_center = None if r.target_center is None else (r.target_center[0] + roi_x, r.target_center[1] + roi_y)
            block_center = None if r.block_center is None else (r.block_center[0] + roi_x, r.block_center[1] + roi_y)
            results.append(DetectionResult(target_center, block_center, r.target_mask, r.block_mask))

        rotation_center = estimate_rotation_center(results, rotation_center)
        executor.rotation_center = rotation_center

        laser_center = None if laser_center_roi is None else (laser_center_roi[0] + roi_x, laser_center_roi[1] + roi_y)
        latest_laser_center = laser_center

        for i, r in enumerate(results, start=1):
            draw_profile_result(vis, r, i, colors_target[i - 1], colors_block[i - 1])

        selected_target = None
        if 0 <= manual_profile_idx < len(results):
            selected_result = results[manual_profile_idx]
            selected_target = selected_result.target_center if manual_kind == "target" else selected_result.block_center
            if selected_target is not None:
                cv2.circle(vis, selected_target, 10, (255, 255, 255), 2)

        if laser_center is not None:
            cv2.circle(vis, laser_center, 8, (0, 0, 255), 2)
            cv2.putText(vis, f"Laser: {laser_center}", (laser_center[0] + 10, laser_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        cv2.rectangle(vis, (roi_x, roi_y), (roi_x + roi_w - 1, roi_y + roi_h - 1), (255, 255, 255), 2)
        cv2.putText(vis, f"ROI x={roi_x} y={roi_y} w={roi_w} h={roi_h}", (15, max(60, roi_y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        draw_qr(vis, qr_text, qr_points)
        cv2.circle(vis, (open_x, open_y), 8, (255, 255, 255), 2)
        if rotation_center is not None:
            cv2.circle(vis, rotation_center, 6, (200, 200, 200), 2)
            cv2.putText(vis, f"Center: {rotation_center}", (rotation_center[0] + 8, rotation_center[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 2)

        pid_cmd = executor.update(
            now=time.monotonic(),
            sender=sender,
            results=results,
            laser_center=laser_center,
            speed_pid=speed_pid,
            dpos_pid=dpos_pid,
            pid_mode=pid_output_mode,
            roi_bounds=(roi_x, roi_y, roi_w, roi_h),
        )

        tx_state = "ON" if sender.is_ready() else "OFF"
        current_step = executor.current_step()
        current_name = current_step.label if current_step is not None else "-"
        cv2.putText(vis, f"Serial {tx_state} | PID {pid_output_mode.upper()} | Task {executor.state} | Step {current_name}", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (240, 240, 240), 2)
        cv2.putText(vis, executor.last_message, (15, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (220, 220, 220), 2)
        cv2.putText(vis, f"QR parsed: {parsed_colors}", (15, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (220, 220, 220), 2)
        cv2.putText(vis, f"Pixel({open_x},{open_y})", (15, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (220, 220, 220), 2)

        if executor.last_error is not None:
            err_x, err_y = executor.last_error
            cv2.putText(vis, f"PID err=({err_x:+d},{err_y:+d})", (15, 168), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (220, 220, 220), 2)

        manual_positions = block_positions.get((manual_profile_idx, "block"), [])
        cv2.putText(vis, f"Manual P{manual_profile_idx + 1} {manual_kind} | Block POS: {manual_positions}", (15, 196), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 2)
        if pos_cmd_preview is not None and pos_query_xy is not None:
            cv2.putText(vis, f"Pixel -> Query({pos_query_xy[0]},{pos_query_xy[1]}) -> POS {pos_cmd_preview[0]} {pos_cmd_preview[1]}", (15, 222), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 2)
        else:
            cv2.putText(vis, f"Pixel -> POS N/A ({CAL_TABLE_DENSE_CSV})", (15, 222), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (100, 220, 255), 2)
        if pid_cmd is not None:
            cv2.putText(vis, f"PID cmd: {pid_cmd}", (15, 248), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 2)

        cv2.imshow("6+1 Detector", vis)
        show_p3_target_mask(results, show_p2_target_mask)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("v"):
            if target_tuner_visible:
                for win in ("P1 Target Tuner", "P2 Target Tuner", "P3 Target Tuner"):
                    try:
                        cv2.destroyWindow(win)
                    except cv2.error:
                        pass
                target_tuner_visible = False
                show_p2_target_mask = False
            else:
                target_ui.create()
                for win in ("P1 Target Tuner", "P3 Target Tuner"):
                    try:
                        cv2.destroyWindow(win)
                    except cv2.error:
                        pass
                target_tuner_visible = True
                show_p2_target_mask = True
        if key == ord("s"):
            target_ui.print_current_params()
        if key == ord("y"):
            pid_output_mode = "speed" if pid_output_mode == "dpos" else "dpos"
            sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()
            print(f"Upper PID output mode switched to {pid_output_mode.upper()}")
        if key == ord("x"):
            executor.stop(sender, speed_pid, dpos_pid)
            print("Task stopped.")
        if key == ord("g"):
            if executor.is_busy():
                print("Task is running, stop it first with x if you want to restart.")
            else:
                steps = executor.build_steps(parsed_colors, block_positions)
                if executor.start(steps, time.monotonic()):
                    print("Task started:", [step.label for step in steps])
                else:
                    print(f"Warning: invalid QR text: {qr_text!r}")
        if key in (ord("1"), ord("2"), ord("3")):
            manual_profile_idx = min(max(key - ord("1"), 0), 2)
            print(f"Manual profile switched to P{manual_profile_idx + 1}")
        if key == ord("b"):
            manual_kind = "block"
            print("Manual source switched to block")
        if key == ord("t"):
            manual_kind = "target"
            print("Manual source switched to target")
        if key == ord("u") and open_ui is not None and selected_target is not None:
            open_ui.set_xy(selected_target[0], selected_target[1], frame.shape[1], frame.shape[0])
            print(f"Pixel slider set to {selected_target}")
        if key == ord("o"):
            positions = block_positions.get((manual_profile_idx, "block"), [])
            if not positions:
                print(f"Warning: no block POS for P{manual_profile_idx + 1}")
            else:
                for yaw, pitch in positions:
                    sender.send_pos(yaw, pitch)
                    time.sleep(0.08)
                print(f"Block POS sent for P{manual_profile_idx + 1}: {positions}")
        if key == ord("j") and open_ui is not None:
            open_ui.nudge(-1, 0, frame.shape[1], frame.shape[0])
        if key == ord("l") and open_ui is not None:
            open_ui.nudge(1, 0, frame.shape[1], frame.shape[0])
        if key == ord("i") and open_ui is not None:
            open_ui.nudge(0, -1, frame.shape[1], frame.shape[0])
        if key == ord("k") and open_ui is not None:
            open_ui.nudge(0, 1, frame.shape[1], frame.shape[0])

    executor.stop(sender, speed_pid, dpos_pid)
    sender.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
