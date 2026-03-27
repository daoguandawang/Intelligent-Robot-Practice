import cv2
import numpy as np
from dataclasses import dataclass
from collections import deque
from typing import Dict, Optional, Tuple, List
import time
import os
#开环，可以成功跑追踪版本，不打表，直接发pos
try:
    import serial
except ImportError:
    serial = None


Point = Tuple[int, int]
PosCmd = Tuple[int, int]

CAL_TABLE_RAW_CSV = "calibration_table.csv"
CAL_TABLE_FILTERED_CSV = "calibration_table_filtered.csv"
CAL_TABLE_DENSE_CSV = "calibration_table_dense_step1.csv"
CAL_TABLE_REPORT_TXT = "calibration_table_fit_report.txt"

#
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
class AxisPID:
    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    deadband: float = 0.0
    min_abs_output: float = 0.0

    integral: float = 0.0
    prev_error: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = None

    def update(self, error: float, dt: float) -> float:
        if abs(error) <= self.deadband:
            self.prev_error = error
            self.integral *= 0.7
            return 0.0

        dt = max(dt, 1e-3)
        self.integral += error * dt
        self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))

        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = float(np.clip(output, -self.output_limit, self.output_limit))

        if 0.0 < abs(output) < self.min_abs_output:
            output = self.min_abs_output if output > 0 else -self.min_abs_output
        return output


@dataclass
class UpperPIDController:
    yaw: AxisPID
    pitch: AxisPID
    update_interval_s: float = 0.04
    dpos_min_freq_hz: int = 300
    dpos_max_freq_hz: int = 1800
    dpos_max_step_per_cycle: int = 120

    last_ts: Optional[float] = None
    last_cmd: Tuple[int, int] = (0, 0)
    last_send_ts: float = 0.0
    yaw_step_residual: float = 0.0
    pitch_step_residual: float = 0.0

    def reset(self) -> None:
        self.yaw.reset()
        self.pitch.reset()
        self.last_ts = None
        self.last_cmd = (0, 0)
        self.last_send_ts = 0.0
        self.yaw_step_residual = 0.0
        self.pitch_step_residual = 0.0

    def _compute_speed(self, target: Point, current: Point, now: float) -> Tuple[Tuple[int, int], float]:
        dt = self.update_interval_s if self.last_ts is None else max(now - self.last_ts, 1e-3)
        self.last_ts = now

        err_x = float(target[0] - current[0])
        err_y = float(target[1] - current[1])

        # From the calibration table:
        # yaw position grows -> laser spot moves left, so x error needs inverted yaw command.
        yaw_speed = self.yaw.update(-err_x, dt)
        # pitch position grows -> laser spot moves down, so y error keeps the same sign.
        pitch_speed = self.pitch.update(err_y, dt)
        return (int(round(yaw_speed)), int(round(pitch_speed))), dt

    def update(self, target: Point, current: Point, now: float) -> Optional[Tuple[int, int]]:
        cmd, _ = self._compute_speed(target, current, now)
        if (now - self.last_send_ts) < self.update_interval_s and cmd == self.last_cmd:
            return None

        self.last_cmd = cmd
        self.last_send_ts = now
        return cmd

    def update_dpos(self, target: Point, current: Point, now: float) -> Optional[Tuple[int, int, int]]:
        speed_cmd, dt = self._compute_speed(target, current, now)
        yaw_speed, pitch_speed = speed_cmd

        self.yaw_step_residual += yaw_speed * dt
        self.pitch_step_residual += pitch_speed * dt

        yaw_delta = int(np.trunc(self.yaw_step_residual))
        pitch_delta = int(np.trunc(self.pitch_step_residual))

        yaw_delta = int(np.clip(yaw_delta, -self.dpos_max_step_per_cycle, self.dpos_max_step_per_cycle))
        pitch_delta = int(np.clip(pitch_delta, -self.dpos_max_step_per_cycle, self.dpos_max_step_per_cycle))

        self.yaw_step_residual -= yaw_delta
        self.pitch_step_residual -= pitch_delta

        if yaw_delta == 0 and pitch_delta == 0:
            if (now - self.last_send_ts) < self.update_interval_s:
                return None
            self.last_cmd = (0, 0)
            self.last_send_ts = now
            return 0, 0, self.dpos_min_freq_hz

        max_speed = max(abs(yaw_speed), abs(pitch_speed), 1)
        freq_hz = int(np.clip(max_speed, self.dpos_min_freq_hz, self.dpos_max_freq_hz))
        cmd = (yaw_delta, pitch_delta)
        if (now - self.last_send_ts) < self.update_interval_s and cmd == self.last_cmd:
            return None

        self.last_cmd = cmd
        self.last_send_ts = now
        return yaw_delta, pitch_delta, freq_hz


@dataclass
class DirectPosPIDController:
    yaw: AxisPID
    pitch: AxisPID
    update_interval_s: float = 0.04
    dpos_min_freq_hz: int = 300
    dpos_max_freq_hz: int = 1800
    dpos_max_step_per_cycle: int = 80

    last_ts: Optional[float] = None
    last_cmd: Tuple[int, int] = (0, 0)
    last_send_ts: float = 0.0

    def reset(self) -> None:
        self.yaw.reset()
        self.pitch.reset()
        self.last_ts = None
        self.last_cmd = (0, 0)
        self.last_send_ts = 0.0

    def update(self, target: Point, current: Point, now: float) -> Optional[Tuple[int, int, int]]:
        dt = self.update_interval_s if self.last_ts is None else max(now - self.last_ts, 1e-3)
        self.last_ts = now

        err_x = float(target[0] - current[0])
        err_y = float(target[1] - current[1])

        yaw_delta = int(round(self.yaw.update(-err_x, dt)))
        pitch_delta = int(round(self.pitch.update(err_y, dt)))

        yaw_delta = int(np.clip(yaw_delta, -self.dpos_max_step_per_cycle, self.dpos_max_step_per_cycle))
        pitch_delta = int(np.clip(pitch_delta, -self.dpos_max_step_per_cycle, self.dpos_max_step_per_cycle))

        cmd = (yaw_delta, pitch_delta)
        if yaw_delta == 0 and pitch_delta == 0:
            if (now - self.last_send_ts) < self.update_interval_s:
                return None
            self.last_cmd = cmd
            self.last_send_ts = now
            return 0, 0, self.dpos_min_freq_hz

        # Choose a frequency that can execute the requested delta within about one control period.
        freq_hz = int(np.ceil(max(abs(yaw_delta), abs(pitch_delta), 1) / max(self.update_interval_s, 1e-3)))
        freq_hz = int(np.clip(freq_hz, self.dpos_min_freq_hz, self.dpos_max_freq_hz))

        if (now - self.last_send_ts) < self.update_interval_s and cmd == self.last_cmd:
            return None

        self.last_cmd = cmd
        self.last_send_ts = now
        return yaw_delta, pitch_delta, freq_hz


def _poly_features(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [
            np.ones_like(x, dtype=np.float64),
            x.astype(np.float64),
            y.astype(np.float64),
            x.astype(np.float64) * x.astype(np.float64),
            x.astype(np.float64) * y.astype(np.float64),
            y.astype(np.float64) * y.astype(np.float64),
        ]
    )


def _fit_and_export_dense_pos_map(
    rows: List[Tuple[int, float, float, int, int]],
    source_csv: str,
    grid_step: int = 1,
    outlier_z_thresh: float = 3.5,
) -> Tuple[Optional[str], Optional[str], Optional[str]]:
    if not rows:
        return None, None, None

    arr = np.array(rows, dtype=np.float64)
    idx_arr = arr[:, 0].astype(np.int64)
    yaw_arr = arr[:, 1]
    pitch_arr = arr[:, 2]
    laser_x_arr = arr[:, 3]
    laser_y_arr = arr[:, 4]

    valid_mask = np.isfinite(yaw_arr) & np.isfinite(pitch_arr) & np.isfinite(laser_x_arr) & np.isfinite(laser_y_arr)
    valid_mask &= (laser_x_arr >= 0) & (laser_y_arr >= 0)
    if int(valid_mask.sum()) < 8:
        return None, None, None

    inlier_mask = valid_mask.copy()
    coef_yaw = None
    coef_pitch = None
    residual = np.zeros_like(yaw_arr, dtype=np.float64)

    for _ in range(6):
        if int(inlier_mask.sum()) < 8:
            break
        A = _poly_features(laser_x_arr[inlier_mask], laser_y_arr[inlier_mask])
        coef_yaw = np.linalg.lstsq(A, yaw_arr[inlier_mask], rcond=None)[0]
        coef_pitch = np.linalg.lstsq(A, pitch_arr[inlier_mask], rcond=None)[0]

        A_all = _poly_features(laser_x_arr, laser_y_arr)
        pred_yaw_all = A_all @ coef_yaw
        pred_pitch_all = A_all @ coef_pitch
        residual = np.hypot(pred_yaw_all - yaw_arr, pred_pitch_all - pitch_arr)

        cur = residual[inlier_mask]
        med = float(np.median(cur))
        mad = float(np.median(np.abs(cur - med))) + 1e-9
        robust_z = 0.6745 * (residual - med) / mad

        new_inlier = valid_mask & (np.abs(robust_z) <= outlier_z_thresh)
        if int(new_inlier.sum()) < 8 or np.array_equal(new_inlier, inlier_mask):
            inlier_mask = new_inlier if int(new_inlier.sum()) >= 8 else inlier_mask
            break
        inlier_mask = new_inlier

    if coef_yaw is None or coef_pitch is None or int(inlier_mask.sum()) < 8:
        return None, None, None
 
    A_all = _poly_features(laser_x_arr, laser_y_arr)
    pred_yaw_all = A_all @ coef_yaw
    pred_pitch_all = A_all @ coef_pitch
    residual = np.hypot(pred_yaw_all - yaw_arr, pred_pitch_all - pitch_arr)

    # Local residual interpolation (IDW) on top of polynomial fit.
    in_x = laser_x_arr[inlier_mask]
    in_y = laser_y_arr[inlier_mask]
    in_yaw_res = yaw_arr[inlier_mask] - pred_yaw_all[inlier_mask]
    in_pitch_res = pitch_arr[inlier_mask] - pred_pitch_all[inlier_mask]
    k_neighbors = min(10, int(inlier_mask.sum()))

    def predict_pos(qx: np.ndarray, qy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        qx = qx.astype(np.float64)
        qy = qy.astype(np.float64)
        A = _poly_features(qx, qy)
        yaw_base = A @ coef_yaw
        pitch_base = A @ coef_pitch
        if k_neighbors <= 0:
            return yaw_base, pitch_base

        dx = qx[:, None] - in_x[None, :]
        dy = qy[:, None] - in_y[None, :]
        d2 = dx * dx + dy * dy
        knn_idx = np.argpartition(d2, kth=max(0, k_neighbors - 1), axis=1)[:, :k_neighbors]
        knn_d2 = np.take_along_axis(d2, knn_idx, axis=1)
        w = 1.0 / np.maximum(knn_d2, 1e-6)
        w_sum = np.sum(w, axis=1, keepdims=True)

        yaw_res_local = np.sum(w * in_yaw_res[knn_idx], axis=1) / np.maximum(w_sum[:, 0], 1e-9)
        pitch_res_local = np.sum(w * in_pitch_res[knn_idx], axis=1) / np.maximum(w_sum[:, 0], 1e-9)
        return yaw_base + yaw_res_local, pitch_base + pitch_res_local

    base_noext = os.path.splitext(source_csv)[0]
    filtered_csv = f"{base_noext}_filtered.csv"
    dense_csv = f"{base_noext}_dense_step{max(1, int(grid_step))}.csv"
    report_txt = f"{base_noext}_fit_report.txt"

    with open(filtered_csv, "w", encoding="utf-8", newline="") as f:
        f.write("idx,yawPos,pitchPos,laserX,laserY,isInlier,residual\n")
        for i in range(len(rows)):
            f.write(
                f"{int(idx_arr[i])},{yaw_arr[i]},{pitch_arr[i]},{int(laser_x_arr[i])},{int(laser_y_arr[i])},"
                f"{1 if inlier_mask[i] else 0},{residual[i]:.6f}\n"
            )

    x_min = int(np.floor(np.min(in_x)))
    x_max = int(np.ceil(np.max(in_x)))
    y_min = int(np.floor(np.min(in_y)))
    y_max = int(np.ceil(np.max(in_y)))
    step = max(1, int(grid_step))

    yaw_lo = float(np.min(yaw_arr[inlier_mask]))
    yaw_hi = float(np.max(yaw_arr[inlier_mask]))
    pitch_lo = float(np.min(pitch_arr[inlier_mask]))
    pitch_hi = float(np.max(pitch_arr[inlier_mask]))

    with open(dense_csv, "w", encoding="utf-8", newline="") as f:
        f.write("laserX,laserY,yawPos,pitchPos,yawFloat,pitchFloat\n")
        xs = np.arange(x_min, x_max + 1, step, dtype=np.float64)
        for y in range(y_min, y_max + 1, step):
            ys = np.full_like(xs, float(y))
            yaw_pred, pitch_pred = predict_pos(xs, ys)
            yaw_pred = np.clip(yaw_pred, yaw_lo, yaw_hi)
            pitch_pred = np.clip(pitch_pred, pitch_lo, pitch_hi)
            yaw_cmd = np.rint(yaw_pred).astype(np.int64)
            pitch_cmd = np.rint(pitch_pred).astype(np.int64)
            for i in range(xs.shape[0]):
                f.write(
                    f"{int(xs[i])},{int(y)},{int(yaw_cmd[i])},{int(pitch_cmd[i])},"
                    f"{yaw_pred[i]:.6f},{pitch_pred[i]:.6f}\n"
                )

    inlier_rmse = float(np.sqrt(np.mean((pred_yaw_all[inlier_mask] - yaw_arr[inlier_mask]) ** 2 + (pred_pitch_all[inlier_mask] - pitch_arr[inlier_mask]) ** 2)))
    with open(report_txt, "w", encoding="utf-8") as f:
        f.write("Calibration Fit Report\n")
        f.write(f"source_csv={source_csv}\n")
        f.write(f"total_points={len(rows)}\n")
        f.write(f"valid_points={int(valid_mask.sum())}\n")
        f.write(f"inlier_points={int(inlier_mask.sum())}\n")
        f.write(f"outlier_points={int(valid_mask.sum()) - int(inlier_mask.sum())}\n")
        f.write(f"inlier_rmse={inlier_rmse:.6f}\n")
        f.write(f"dense_x_range=[{x_min}, {x_max}] step={step}\n")
        f.write(f"dense_y_range=[{y_min}, {y_max}] step={step}\n")
        f.write(f"filtered_csv={filtered_csv}\n")
        f.write(f"dense_csv={dense_csv}\n")

    return filtered_csv, dense_csv, report_txt


def _load_dense_pos_table(
    dense_csv: str,
) -> Tuple[Dict[Point, PosCmd], Optional[Tuple[int, int, int, int]]]:
    if not os.path.exists(dense_csv):
        return {}, None

    table: Dict[Point, PosCmd] = {}
    min_x = 10**9
    max_x = -10**9
    min_y = 10**9
    max_y = -10**9

    try:
        with open(dense_csv, "r", encoding="utf-8") as f:
            _ = f.readline()  # header
            for line in f:
                parts = line.strip().split(",")
                if len(parts) < 4:
                    continue
                try:
                    lx = int(float(parts[0]))
                    ly = int(float(parts[1]))
                    yaw = int(float(parts[2]))
                    pitch = int(float(parts[3]))
                except ValueError:
                    continue
                table[(lx, ly)] = (yaw, pitch)
                if lx < min_x:
                    min_x = lx
                if lx > max_x:
                    max_x = lx
                if ly < min_y:
                    min_y = ly
                if ly > max_y:
                    max_y = ly
    except Exception as e:
        print(f"Warning: failed to load dense table {dense_csv}: {e}")
        return {}, None

    if not table:
        return {}, None
    return table, (min_x, max_x, min_y, max_y)


def _find_latest_legacy_dense_csv() -> Optional[str]:
    try:
        files = []
        for name in os.listdir("."):
            if name.startswith("calibration_table_") and name.endswith("_dense_step1.csv"):
                files.append(name)
        if not files:
            return None
        files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
        return files[0]
    except Exception:
        return None


def _lookup_pos_from_dense_table(
    dense_table: Dict[Point, PosCmd],
    dense_bounds: Optional[Tuple[int, int, int, int]],
    x: int,
    y: int,
) -> Tuple[Optional[PosCmd], Optional[Point]]:
    if not dense_table or dense_bounds is None:
        return None, None

    min_x, max_x, min_y, max_y = dense_bounds
    qx = min(max(int(x), min_x), max_x)
    qy = min(max(int(y), min_y), max_y)

    pos = dense_table.get((qx, qy))
    if pos is not None:
        return pos, (qx, qy)

    for r in range(1, 5):
        for ny in range(qy - r, qy + r + 1):
            for nx in range(qx - r, qx + r + 1):
                pos = dense_table.get((nx, ny))
                if pos is not None:
                    return pos, (nx, ny)
    return None, (qx, qy)


class SerialPIDSender:
    def __init__(self, port: str = "COM3", baudrate: int = 115200, enabled: bool = True):
        self.port = port
        self.baudrate = baudrate
        self.enabled = enabled
        self.ser = None
        self._rx_buffer = bytearray()
        self.calibration_active = False
        self.stream_pid_enabled = False
        self.last_req: Optional[Tuple[str, str]] = None
        self.auto_dump_on_done = True
        self.dump_collecting = False
        self.dump_rows: List[Tuple[int, float, float, int, int]] = []
        self.last_dump_row_ts = 0.0
        self.dump_idle_timeout_s = 0.8
        self.last_dump_file: Optional[str] = None
        self.last_dense_file: Optional[str] = None
        self.last_filtered_file: Optional[str] = None
        self.last_fit_report_file: Optional[str] = None
        self.dense_version = 0
        self.current_pos: Optional[Tuple[int, int]] = None

        if not self.enabled:
            return

        if serial is None:
            print("Warning: pyserial not installed, serial send disabled.")
            self.enabled = False
            return

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0, write_timeout=0)
            print(f"Serial opened: {self.port} @ {self.baudrate}")
        except Exception as e:
            print(f"Warning: cannot open serial {self.port} @ {self.baudrate}: {e}")
            self.enabled = False
            self.ser = None

    def is_ready(self) -> bool:
        return self.enabled and self.ser is not None and self.ser.is_open

    def _write_line(self, line: str, tag: str = "TX") -> None:
        if not self.is_ready():
            return
        if not line.endswith("\n"):
            line = line + "\n"
        try:
            self.ser.write(line.encode("ascii", errors="replace"))
            print(f"{tag}: {line.strip()}")
        except Exception as e:
            print(f"Warning: serial write failed: {e}")
            self.enabled = False

    def send_pid(self, target: Point, current: Point) -> None:
        self._write_line(f"PID {target[0]} {target[1]} {current[0]} {current[1]}")

    def send_speed(self, yaw_speed: int, pitch_speed: int) -> None:
        self._write_line(f"SPEED {int(yaw_speed)} {int(pitch_speed)}")

    def send_dpos(self, yaw_delta: int, pitch_delta: int, freq_hz: int) -> None:
        self._write_line(f"DPOS {int(yaw_delta)} {int(pitch_delta)} {int(freq_hz)}")

    def send_dangle(self, pitch_delta_deg: float, yaw_delta_deg: float, freq_hz: int) -> None:
        self._write_line(f"DANGLE {pitch_delta_deg:g} {yaw_delta_deg:g} {int(freq_hz)}")

    def send_stop(self) -> None:
        self._write_line("STOP")

    def send_point(self, laser_center: Optional[Point]) -> None:
        if laser_center is None:
            self._write_line("POINT -1 -1")
        else:
            self._write_line(f"POINT {laser_center[0]} {laser_center[1]}")

    def send_open(self, x: int, y: int) -> None:
        self._write_line(f"OPEN {int(x)} {int(y)}")

    def send_pos(self, yaw: int, pitch: int) -> None:
        self._write_line(f"POS {int(yaw)} {int(pitch)}")

    def send_cal_start(
        self,
        pitch_start: float,
        pitch_end: float,
        pitch_step: float,
        yaw_start: float,
        yaw_end: float,
        yaw_step: float,
    ) -> None:
        line = (
            f"CAL_START {pitch_start:g} {pitch_end:g} {pitch_step:g} "
            f"{yaw_start:g} {yaw_end:g} {yaw_step:g}"
        )
        self._write_line(line)
        self.calibration_active = True

    def send_cal_dump(self, begin_collect: bool = True) -> None:
        if begin_collect:
            self.dump_collecting = True
            self.dump_rows = []
            self.last_dump_row_ts = time.monotonic()
            print("CAL_DUMP collection started")
        self._write_line("CAL_DUMP")

    def read_lines(self) -> List[str]:
        lines: List[str] = []
        if not self.is_ready():
            return lines
        try:
            waiting = int(self.ser.in_waiting)
            if waiting <= 0:
                return lines

            data = self.ser.read(waiting)
            if not data:
                return lines

            self._rx_buffer.extend(data)
            while b"\n" in self._rx_buffer:
                idx = self._rx_buffer.index(0x0A)
                raw = bytes(self._rx_buffer[:idx]).rstrip(b"\r")
                del self._rx_buffer[:idx + 1]
                if not raw:
                    continue
                text = raw.decode("utf-8", errors="replace")
                print(f"RX: {text}")
                lines.append(text.strip())
        except Exception as e:
            print(f"Warning: serial read failed: {e}")
            self.enabled = False
        return lines

    @staticmethod
    def _try_parse_cal_start(parts: List[str]) -> Optional[Tuple[float, float, float, float, float, float]]:
        if len(parts) != 7:
            return None
        try:
            vals = tuple(float(x) for x in parts[1:7])
            return vals
        except ValueError:
            return None

    @staticmethod
    def _try_parse_dump_row(parts: List[str]) -> Optional[Tuple[int, float, float, int, int]]:
        if len(parts) < 5:
            return None
        try:
            idx = int(parts[0])
            yaw_pos = float(parts[1])
            pitch_pos = float(parts[2])
            laser_x = int(float(parts[3]))
            laser_y = int(float(parts[4]))
            return idx, yaw_pos, pitch_pos, laser_x, laser_y
        except ValueError:
            return None

    def _save_dump_rows_to_file(self) -> None:
        if not self.dump_rows:
            return
        filename = CAL_TABLE_RAW_CSV
        try:
            with open(filename, "w", encoding="utf-8", newline="") as f:
                f.write("idx,yawPos,pitchPos,laserX,laserY\n")
                for idx, yaw_pos, pitch_pos, laser_x, laser_y in self.dump_rows:
                    f.write(f"{idx},{yaw_pos},{pitch_pos},{laser_x},{laser_y}\n")
            self.last_dump_file = filename
            print(f"CAL_DUMP saved: {filename} (rows={len(self.dump_rows)})")
        except Exception as e:
            print(f"Warning: failed to save CAL_DUMP file: {e}")
            return

        try:
            filtered_csv, dense_csv, report_txt = _fit_and_export_dense_pos_map(self.dump_rows, filename, grid_step=1)
            if filtered_csv is not None and dense_csv is not None and report_txt is not None:
                try:
                    if filtered_csv != CAL_TABLE_FILTERED_CSV:
                        os.replace(filtered_csv, CAL_TABLE_FILTERED_CSV)
                    filtered_csv = CAL_TABLE_FILTERED_CSV
                    if dense_csv != CAL_TABLE_DENSE_CSV:
                        os.replace(dense_csv, CAL_TABLE_DENSE_CSV)
                    dense_csv = CAL_TABLE_DENSE_CSV
                    if report_txt != CAL_TABLE_REPORT_TXT:
                        os.replace(report_txt, CAL_TABLE_REPORT_TXT)
                    report_txt = CAL_TABLE_REPORT_TXT
                except Exception as e:
                    print(f"Warning: failed to normalize calibration filenames: {e}")

                self.last_filtered_file = filtered_csv
                self.last_dense_file = dense_csv
                self.last_fit_report_file = report_txt
                self.dense_version += 1
                print(f"FIT saved: {filtered_csv}")
                print(f"DENSE saved: {dense_csv}")
                print(f"REPORT saved: {report_txt}")
            else:
                print("Warning: fit/dense export skipped (insufficient valid points).")
        except Exception as e:
            print(f"Warning: fit/dense export failed: {e}")

    def maybe_finalize_dump(self) -> None:
        if not self.dump_collecting:
            return
        now = time.monotonic()
        if (now - self.last_dump_row_ts) < self.dump_idle_timeout_s:
            return

        self.dump_collecting = False
        if self.dump_rows:
            self._save_dump_rows_to_file()
        else:
            print("CAL_DUMP finished with no rows")

    def handle_line(self, line: str, laser_center: Optional[Point]) -> None:
        if not line:
            return
        parts = line.strip().split()
        if not parts:
            return

        cmd = parts[0].upper()

        if cmd == "CAL_START":
            vals = self._try_parse_cal_start(parts)
            if vals is None:
                print(f"Warning: malformed CAL_START: {line}")
                return
            self.calibration_active = True
            pitch_start, pitch_end, pitch_step, yaw_start, yaw_end, yaw_step = vals
            print(
                "CAL_START parsed: "
                f"pitch=({pitch_start},{pitch_end},{pitch_step}) "
                f"yaw=({yaw_start},{yaw_end},{yaw_step})"
            )
            return

        if cmd == "REQ_POINT":
            if len(parts) >= 3:
                self.last_req = (parts[1], parts[2])
            else:
                self.last_req = None
            self.send_point(laser_center)
            return

        if cmd == "POS" and len(parts) >= 3:
            try:
                yaw = int(float(parts[1]))
                pitch = int(float(parts[2]))
            except ValueError:
                print(f"Warning: bad POS frame: {line}")
                return
            self.current_pos = (yaw, pitch)
            print(f"POS feedback: yaw={yaw} pitch={pitch}")
            return

        if cmd == "CAL_DONE":
            self.calibration_active = False
            print(f"CAL_DONE received: {line}")
            if self.auto_dump_on_done:
                self.send_cal_dump(begin_collect=True)
            return

        dump_row = self._try_parse_dump_row(parts)
        if dump_row is not None:
            if self.dump_collecting:
                self.dump_rows.append(dump_row)
                self.last_dump_row_ts = time.monotonic()
            # Row format: <idx> <yawPos> <pitchPos> <laserX> <laserY>
            print(f"CAL_ROW: {line}")

    def process_incoming(self, laser_center: Optional[Point]) -> None:
        for line in self.read_lines():
            self.handle_line(line, laser_center)
        self.maybe_finalize_dump()

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


class RedLaserDetector:
    def __init__(self, history_len: int = 3):
        self.h_red = deque(maxlen=history_len)
        self.params = {
            "h1_low": 0,
            "h1_high": 10,
            "h2_low": 160,
            "h2_high": 179,
            "s_low": 60,
            "v_low": 200,
            "close_iter": 10,
            "open_iter": 0,
            "min_area": 5,
            "circularity_pct": 10,
            "max_jump": 5,
        }

    def apply_auto_exposure(self, cap) -> None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -7)

    def stable_center(self, new_pt: Optional[Point]) -> Optional[Point]:
        if new_pt is None:
            return None
        if not self.h_red:
            self.h_red.append(new_pt)
            return None

        last = self.h_red[-1]
        if np.linalg.norm(np.array(new_pt) - np.array(last)) > self.params["max_jump"]:
            self.h_red.clear()
            self.h_red.append(new_pt)
            return None

        self.h_red.append(new_pt)
        if len(self.h_red) == self.h_red.maxlen:
            mean_pt = np.mean(self.h_red, axis=0)
            return int(mean_pt[0]), int(mean_pt[1])
        return None

    def hsv_mask(self, hsv: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        if self.params["close_iter"] > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.params["close_iter"])
        if self.params["open_iter"] > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.params["open_iter"])
        return mask

    def find_center(self, mask: np.ndarray) -> Optional[Point]:
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        best = None
        max_area = 0.0
        min_area = self.params["min_area"]
        min_circularity = self.params["circularity_pct"] / 100.0

        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            circularity = area / (np.pi * r * r + 1e-6)
            if circularity < min_circularity:
                continue
            if area > max_area:
                max_area = area
                best = (int(x), int(y))
        return best

    def detect(self, frame: np.ndarray) -> Tuple[Optional[Point], np.ndarray]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        p = self.params
        lower1 = np.array([p["h1_low"], p["s_low"], p["v_low"]], dtype=np.uint8)
        upper1 = np.array([p["h1_high"], 255, 255], dtype=np.uint8)
        lower2 = np.array([p["h2_low"], p["s_low"], p["v_low"]], dtype=np.uint8)
        upper2 = np.array([p["h2_high"], 255, 255], dtype=np.uint8)

        mask_r = cv2.bitwise_or(self.hsv_mask(hsv, lower1, upper1), self.hsv_mask(hsv, lower2, upper2))
        r_raw = self.find_center(mask_r)
        r_stable = self.stable_center(r_raw)
        return r_stable, mask_r


class TargetTunerUI:
    def __init__(self, profiles: List[RedProfile]):
        self.profiles = profiles
        self.windows = [f"{p.name} Target Tuner" for p in profiles]

    def create(self) -> None:
        for win, profile in zip(self.windows, self.profiles):
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(win, 420, 520)
            p = profile.target
            cv2.createTrackbar("H1_min", win, p["H1_min"], 180, nothing)
            cv2.createTrackbar("H1_max", win, p["H1_max"], 180, nothing)
            cv2.createTrackbar("H2_min", win, p["H2_min"], 180, nothing)
            cv2.createTrackbar("H2_max", win, p["H2_max"], 180, nothing)
            cv2.createTrackbar("S_min", win, p["S_min"], 255, nothing)
            cv2.createTrackbar("S_max", win, p["S_max"], 255, nothing)
            cv2.createTrackbar("V_min", win, p["V_min"], 255, nothing)
            cv2.createTrackbar("V_max", win, p["V_max"], 255, nothing)
            cv2.createTrackbar("Kernel", win, p["Kernel"], 31, nothing)
            cv2.createTrackbar("Dilate", win, p["Dilate"], 10, nothing)
            cv2.createTrackbar("MinArea", win, p["MinArea"], 30000, nothing)
            cv2.createTrackbar("MaxArea", win, p["MaxArea"], 30000, nothing)

    def update_profiles_from_ui(self) -> None:
        for win, profile in zip(self.windows, self.profiles):
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    continue
            except cv2.error:
                continue
            p = profile.target
            p["H1_min"] = cv2.getTrackbarPos("H1_min", win)
            p["H1_max"] = cv2.getTrackbarPos("H1_max", win)
            p["H2_min"] = cv2.getTrackbarPos("H2_min", win)
            p["H2_max"] = cv2.getTrackbarPos("H2_max", win)
            p["S_min"] = cv2.getTrackbarPos("S_min", win)
            p["S_max"] = cv2.getTrackbarPos("S_max", win)
            p["V_min"] = cv2.getTrackbarPos("V_min", win)
            p["V_max"] = cv2.getTrackbarPos("V_max", win)
            p["Kernel"] = cv2.getTrackbarPos("Kernel", win)
            p["Dilate"] = cv2.getTrackbarPos("Dilate", win)
            p["MinArea"] = cv2.getTrackbarPos("MinArea", win)
            p["MaxArea"] = cv2.getTrackbarPos("MaxArea", win)

    def print_current_params(self) -> None:
        print("\n===== Current Target Params =====")
        for profile in self.profiles:
            print(profile.name, profile.target)
        print("===============================\n")


class ROISelectorUI:
    def __init__(self, frame_w: int, frame_h: int):
        self.win = "ROI Selector"
        self.frame_w = max(1, int(frame_w))
        self.frame_h = max(1, int(frame_h))

    def create(self) -> None:
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, 420, 220)
        # cv2.createTrackbar("X", self.win, 0, max(0, self.frame_w - 1), nothing)
        # cv2.createTrackbar("Y", self.win, 0, max(0, self.frame_h - 1), nothing)
        # cv2.createTrackbar("W", self.win, self.frame_w, self.frame_w, nothing)
        # cv2.createTrackbar("H", self.win, self.frame_h, self.frame_h, nothing)
        cv2.createTrackbar("X", self.win, 419, max(0, self.frame_w - 1), nothing)
        cv2.createTrackbar("Y", self.win, 312, max(0, self.frame_h - 1), nothing)
        cv2.createTrackbar("W", self.win, 400, self.frame_w, nothing)
        cv2.createTrackbar("H", self.win, 320, self.frame_h, nothing)

    def get_roi(self, frame_w: int, frame_h: int) -> Tuple[int, int, int, int]:
        frame_w = max(1, int(frame_w))
        frame_h = max(1, int(frame_h))

        x = cv2.getTrackbarPos("X", self.win)
        y = cv2.getTrackbarPos("Y", self.win)
        w = cv2.getTrackbarPos("W", self.win)
        h = cv2.getTrackbarPos("H", self.win)

        x = min(max(0, x), frame_w - 1)
        y = min(max(0, y), frame_h - 1)
        w = min(max(1, w), frame_w - x)
        h = min(max(1, h), frame_h - y)

        cv2.setTrackbarPos("X", self.win, x)
        cv2.setTrackbarPos("Y", self.win, y)
        cv2.setTrackbarPos("W", self.win, w)
        cv2.setTrackbarPos("H", self.win, h)
        return x, y, w, h


class OpenCommandUI:
    def __init__(self, frame_w: int, frame_h: int):
        self.win = "OPEN Sender"
        self.frame_w = max(1, int(frame_w))
        self.frame_h = max(1, int(frame_h))

    def create(self) -> None:
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win, 460, 240)
        cv2.createTrackbar("X", self.win, self.frame_w // 2, max(0, self.frame_w - 1), nothing)
        cv2.createTrackbar("Y", self.win, self.frame_h // 2, max(0, self.frame_h - 1), nothing)
        cv2.createTrackbar("dX", self.win, 50, 100, nothing)  # fine offset: -50..50
        cv2.createTrackbar("dY", self.win, 50, 100, nothing)  # fine offset: -50..50

    def get_xy(self, frame_w: int, frame_h: int) -> Tuple[int, int]:
        frame_w = max(1, int(frame_w))
        frame_h = max(1, int(frame_h))

        x_base = cv2.getTrackbarPos("X", self.win)
        y_base = cv2.getTrackbarPos("Y", self.win)
        x_base_clamped = min(max(0, x_base), frame_w - 1)
        y_base_clamped = min(max(0, y_base), frame_h - 1)
        if x_base_clamped != x_base:
            cv2.setTrackbarPos("X", self.win, x_base_clamped)
        if y_base_clamped != y_base:
            cv2.setTrackbarPos("Y", self.win, y_base_clamped)

        dx = cv2.getTrackbarPos("dX", self.win) - 50
        dy = cv2.getTrackbarPos("dY", self.win) - 50
        x = x_base_clamped + dx
        y = y_base_clamped + dy
        x = min(max(0, x), frame_w - 1)
        y = min(max(0, y), frame_h - 1)
        return x, y

    def nudge(self, dx: int, dy: int, frame_w: int, frame_h: int) -> None:
        frame_w = max(1, int(frame_w))
        frame_h = max(1, int(frame_h))
        x = cv2.getTrackbarPos("X", self.win) + int(dx)
        y = cv2.getTrackbarPos("Y", self.win) + int(dy)
        x = min(max(0, x), frame_w - 1)
        y = min(max(0, y), frame_h - 1)
        cv2.setTrackbarPos("X", self.win, x)
        cv2.setTrackbarPos("Y", self.win, y)


class MultiProfileDetector:
    def __init__(self, profiles: List[RedProfile]):
        self.profiles = profiles

    @staticmethod
    def odd_kernel_size(val: int) -> int:
        val = max(1, int(val))
        if val % 2 == 0:
            val += 1
        return val

    @staticmethod
    def contour_center(cnt) -> Optional[Point]:
        m = cv2.moments(cnt)
        if m["m00"] == 0:
            return None
        return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])

    @staticmethod
    def create_red_mask(hsv: np.ndarray, params: Dict[str, int]) -> np.ndarray:
        lower1 = np.array([params["H1_min"], params["S_min"], params["V_min"]], dtype=np.uint8)
        upper1 = np.array([params["H1_max"], params["S_max"], params["V_max"]], dtype=np.uint8)
        lower2 = np.array([params["H2_min"], params["S_min"], params["V_min"]], dtype=np.uint8)
        upper2 = np.array([params["H2_max"], params["S_max"], params["V_max"]], dtype=np.uint8)
        return cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))

    def detect_one_profile(self, frame: np.ndarray, profile: RedProfile) -> DetectionResult:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        target_mask_raw = self.create_red_mask(hsv, profile.target)
        target_kernel_size = self.odd_kernel_size(profile.target["Kernel"])
        target_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (target_kernel_size, target_kernel_size))
        target_mask = cv2.morphologyEx(target_mask_raw, cv2.MORPH_CLOSE, target_kernel, iterations=2)
        target_mask = cv2.dilate(target_mask, target_kernel, iterations=max(1, profile.target["Dilate"]))
        target_mask = cv2.morphologyEx(target_mask, cv2.MORPH_CLOSE, target_kernel, iterations=1)

        block_mask_raw = self.create_red_mask(hsv, profile.block)
        block_kernel_size = self.odd_kernel_size(profile.block["Kernel"])
        block_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (block_kernel_size, block_kernel_size))
        block_mask = cv2.morphologyEx(
            block_mask_raw,
            cv2.MORPH_OPEN,
            block_kernel,
            iterations=max(0, profile.block["OpenIter"]),
        )
        block_mask = cv2.morphologyEx(
            block_mask,
            cv2.MORPH_CLOSE,
            block_kernel,
            iterations=max(0, profile.block["CloseIter"]),
        )

        target_center = None
        block_center = None
        best_target_contour = None
        best_block_contour = None

        target_contours, _ = cv2.findContours(target_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        block_contours, _ = cv2.findContours(block_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        target_min_area = min(profile.target["MinArea"], profile.target["MaxArea"])
        target_max_area = max(profile.target["MinArea"], profile.target["MaxArea"])
        block_min_area = min(profile.block["MinArea"], profile.block["MaxArea"])
        block_max_area = max(profile.block["MinArea"], profile.block["MaxArea"])

        best_target_score = -1.0
        for cnt in target_contours:
            area = cv2.contourArea(cnt)
            if not (target_min_area <= area <= target_max_area):
                continue

            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area <= 1:
                continue

            rect = cv2.minAreaRect(hull)
            (_, _), (rw, rh), _ = rect
            if rw <= 1 or rh <= 1:
                continue

            rect_ratio = max(rw, rh) / min(rw, rh)
            rect_area = rw * rh
            extent = hull_area / rect_area if rect_area > 1 else 0.0
            solidity = area / hull_area if hull_area > 1 else 0.0

            if rect_ratio > 1.45:
                continue
            if extent < 0.18:
                continue
            if solidity < 0.55:
                continue

            score = area + 1200.0 * extent + 800.0 * solidity - 400.0 * abs(rect_ratio - 1.0)
            if score > best_target_score:
                best_target_score = score
                best_target_contour = hull

        for cnt in block_contours:
            area = cv2.contourArea(cnt)
            if not (block_min_area <= area <= block_max_area):
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if h == 0:
                continue
            aspect_ratio = float(w) / h
            epsilon = 0.04 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 4 and 0.8 <= aspect_ratio <= 1.2 and cv2.isContourConvex(approx):
                continue
            if best_block_contour is None or area > cv2.contourArea(best_block_contour):
                best_block_contour = cnt

        if best_target_contour is not None:
            target_center = self.contour_center(best_target_contour)
        if best_block_contour is not None:
            block_center = self.contour_center(best_block_contour)

        return DetectionResult(target_center, block_center, target_mask, block_mask)

    def detect_all(self, frame: np.ndarray) -> List[DetectionResult]:
        return [self.detect_one_profile(frame, p) for p in self.profiles]


def build_profiles() -> List[RedProfile]:
    return [
        RedProfile(
            name="P1",
            target={
                "H1_min": 0,
                "H1_max": 10,
                "H2_min": 156,
                "H2_max": 180,
                "S_min": 122,
                "S_max": 255,
                "V_min": 32,
                "V_max": 255,
                "Kernel": 2,
                "Dilate": 3,
                "MinArea": 5000,
                "MaxArea": 14099,
            },
            block={
                "H1_min": 0,
                "H1_max": 10,
                "H2_min": 156,
                "H2_max": 180,
                "S_min": 122,
                "S_max": 255,
                "V_min": 68,
                "V_max": 255,
                "Kernel": 5,
                "OpenIter": 2,
                "CloseIter": 2,
                "MinArea": 700,
                "MaxArea": 12000,
            },
        ),
        RedProfile(
            name="P2",
            target={
                "H1_min": 180,
                "H1_max": 180,
                "H2_min": 38,
                "H2_max": 67,
                "S_min": 112,
                "S_max": 182,
                "V_min": 26,
                "V_max": 84,
                "Kernel": 2,
                "Dilate": 4,
                "MinArea": 8000,
                "MaxArea": 13000,
            },
            block={
                "H1_min": 34,
                "H1_max": 59,
                "H2_min": 0,
                "H2_max": 0,
                "S_min": 126,
                "S_max": 242,
                "V_min": 18,
                "V_max": 66,
                "Kernel": 4,
                "OpenIter": 1,
                "CloseIter": 10,
                "MinArea": 700,
                "MaxArea": 12000,
            },
        ),
        RedProfile(
            name="P3",
            target={
                "H1_min": 180,
                "H1_max": 180,
                "H2_min": 110,
                "H2_max": 141,
                "S_min": 57,
                "S_max": 134,
                "V_min": 26,
                "V_max": 86,
                "Kernel": 6,
                "Dilate": 1,
                "MinArea": 5400,
                "MaxArea": 12000,
            },
            block={
                "H1_min": 111,
                "H1_max": 145,
                "H2_min": 0,
                "H2_max": 0,
                "S_min": 170,
                "S_max": 235,
                "V_min": 0,
                "V_max": 82,
                "Kernel": 2,
                "OpenIter": 1,
                "CloseIter": 4,
                "MinArea": 700,
                "MaxArea": 12000,
            },
        ),
    ]


def draw_profile_result(frame: np.ndarray, result: DetectionResult, profile_idx: int, color_target, color_block) -> None:
    if result.target_center is not None:
        cv2.circle(frame, result.target_center, 7, color_target, -1)
        cv2.putText(
            frame,
            f"Target{profile_idx}: {result.target_center}",
            (result.target_center[0] - 55, result.target_center[1] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color_target,
            2,
        )
    if result.block_center is not None:
        cv2.circle(frame, result.block_center, 7, color_block, -1)
        cv2.putText(
            frame,
            f"Block{profile_idx}: {result.block_center}",
            (result.block_center[0] - 55, result.block_center[1] - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color_block,
            2,
        )


def show_mask_windows(results: List[DetectionResult], laser_mask: np.ndarray, show_masks: bool) -> None:
    names_and_masks = []
    for idx, r in enumerate(results, start=1):
        names_and_masks.append((f"P{idx} Target Mask", r.target_mask))
        names_and_masks.append((f"P{idx} Block Mask", r.block_mask))
    names_and_masks.append(("Laser Mask", laser_mask))

    if not show_masks:
        for name, _ in names_and_masks:
            try:
                cv2.destroyWindow(name)
            except cv2.error:
                pass
        return

    for name, mask in names_and_masks:
        h, w = mask.shape[:2]
        small = cv2.resize(mask, (max(1, w // 2), max(1, h // 2)), interpolation=cv2.INTER_NEAREST)
        cv2.imshow(name, small)


def main() -> None:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: cannot open camera.")
        return

    profiles = build_profiles()
    multi = MultiProfileDetector(profiles)
    laser = RedLaserDetector(history_len=1)
    laser.apply_auto_exposure(cap)
    sender = SerialPIDSender(port="COM6", baudrate=115200, enabled=True)
    speed_pid = UpperPIDController(
        yaw=AxisPID(kp=5.0, ki=0.12, kd=0.6, integral_limit=180.0, output_limit=1800.0, deadband=4.0, min_abs_output=120.0),
        pitch=AxisPID(kp=4.2, ki=0.10, kd=0.45, integral_limit=180.0, output_limit=1600.0, deadband=4.0, min_abs_output=100.0),
        update_interval_s=0.04,
        dpos_min_freq_hz=300,
        dpos_max_freq_hz=1800,
        dpos_max_step_per_cycle=120,
    )
    dpos_pid = DirectPosPIDController(
        yaw=AxisPID(kp=0.32, ki=0.012, kd=0.10, integral_limit=120.0, output_limit=80.0, deadband=4.0),
        pitch=AxisPID(kp=0.28, ki=0.010, kd=0.08, integral_limit=120.0, output_limit=80.0, deadband=4.0),
        update_interval_s=0.04,
        dpos_min_freq_hz=300,
        dpos_max_freq_hz=1800,
        dpos_max_step_per_cycle=80,
    )
    pid_output_mode = "speed"  # "speed" or "dpos"
    send_profile_idx = 0
    send_target_kind = "target"  # "target" or "block"
    roi_ui = None
    open_ui = None
    latest_laser_center: Optional[Point] = None
    red_openloop_track_enabled = False
    red_openloop_send_interval_s = 0.10
    last_red_openloop_send_ts = 0.0
    cal_range = (19.0, 33.4, 0.8, 3.0, 16.0, 0.8)  # pitch_start, pitch_end, pitch_step, yaw_start, yaw_end, yaw_step
    open_x, open_y = 0, 0
    dense_table: Dict[Point, PosCmd] = {}
    dense_bounds: Optional[Tuple[int, int, int, int]] = None
    dense_loaded_version = sender.dense_version

    target_ui = TargetTunerUI(profiles)
    # target_ui.create()

    def reload_dense_table(reason: str) -> None:
        nonlocal dense_table, dense_bounds
        dense_path = CAL_TABLE_DENSE_CSV
        if not os.path.exists(dense_path):
            legacy = _find_latest_legacy_dense_csv()
            if legacy is not None:
                dense_path = legacy
        dense_table, dense_bounds = _load_dense_pos_table(dense_path)
        if dense_table:
            print(f"Dense table loaded ({len(dense_table)} points) from {dense_path}, reason={reason}")
        else:
            print(f"Dense table not found/empty: {CAL_TABLE_DENSE_CSV}, reason={reason}")

    reload_dense_table("startup")

    print("Keys: q quit, s print current target params")
    print("      t use target, b use block, 1/2/3 choose profile for serial send")
    print("      c send CAL_START, d send CAL_DUMP, p toggle upper PID stream, y toggle RED target POS stream, o send POS(from table)")
    print("      i/j/k/l nudge target pixel by 1")
    print("      ROI sliders: X, Y, W, H in window 'ROI Selector'")
    print("      only final detector window + OPEN Sender + ROI Selector are enabled")
    # show_masks = True

    colors_target = [(0, 255, 0), (255, 255, 0), (255, 0, 255)]
    colors_block = [(0, 255, 255), (255, 128, 0), (128, 255, 0)]

    while True:
        # Handle possible REQ_POINT as early as possible using last known laser center.
        sender.process_incoming(latest_laser_center)
        if sender.dense_version != dense_loaded_version:
            dense_loaded_version = sender.dense_version
            reload_dense_table("new_calibration")

        ok, frame = cap.read()
        if not ok:
            print("Error: cannot read frame.")
            break

        # target_ui.update_profiles_from_ui()

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        target_width = 960
        target_height = int((target_width / w) * h)
        frame = cv2.resize(frame, (target_width, target_height))

        if open_ui is None:
            open_ui = OpenCommandUI(frame_w=frame.shape[1], frame_h=frame.shape[0])
            open_ui.create()
        open_x, open_y = open_ui.get_xy(frame.shape[1], frame.shape[0])
        pos_cmd_preview, pos_query_xy = _lookup_pos_from_dense_table(dense_table, dense_bounds, open_x, open_y)

        if roi_ui is None:
            roi_ui = ROISelectorUI(frame_w=frame.shape[1], frame_h=frame.shape[0])
            roi_ui.create()

        roi_x, roi_y, roi_w, roi_h = roi_ui.get_roi(frame.shape[1], frame.shape[0])
        roi_frame = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        vis = frame.copy()
        roi_results = multi.detect_all(roi_frame)
        laser_center_roi, laser_mask = laser.detect(roi_frame)

        results = []
        for r in roi_results:
            target_center = None if r.target_center is None else (r.target_center[0] + roi_x, r.target_center[1] + roi_y)
            block_center = None if r.block_center is None else (r.block_center[0] + roi_x, r.block_center[1] + roi_y)
            results.append(DetectionResult(target_center, block_center, r.target_mask, r.block_mask))

        laser_center = None if laser_center_roi is None else (laser_center_roi[0] + roi_x, laser_center_roi[1] + roi_y)
        latest_laser_center = laser_center

        for i, r in enumerate(results, start=1):
            draw_profile_result(vis, r, i, colors_target[i - 1], colors_block[i - 1])

        if laser_center is not None:
            cv2.circle(vis, laser_center, 8, (0, 0, 255), 2)
            cv2.putText(vis, f"Laser: {laser_center}", (laser_center[0] + 10, laser_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        now = time.monotonic()
        if red_openloop_track_enabled and 0 <= send_profile_idx < len(results):
            tracked_target = results[send_profile_idx].target_center if send_target_kind == "target" else results[send_profile_idx].block_center
            if tracked_target is not None and (now - last_red_openloop_send_ts) >= red_openloop_send_interval_s:
                pos_cmd, pos_xy = _lookup_pos_from_dense_table(dense_table, dense_bounds, tracked_target[0], tracked_target[1])
                if pos_cmd is not None and pos_xy is not None:
                    sender.send_pos(pos_cmd[0], pos_cmd[1])
                    last_red_openloop_send_ts = now

        cv2.rectangle(vis, (roi_x, roi_y), (roi_x + roi_w - 1, roi_y + roi_h - 1), (255, 255, 255), 2)
        cv2.putText(
            vis,
            f"ROI x={roi_x} y={roi_y} w={roi_w} h={roi_h}",
            (15, max(60, roi_y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        selected_result = results[send_profile_idx]
        selected_target = selected_result.target_center if send_target_kind == "target" else selected_result.block_center
        if selected_target is not None:
            cv2.circle(vis, selected_target, 10, (255, 255, 255), 2)
            cv2.putText(
                vis,
                f"TX Target: {selected_target}",
                (selected_target[0] + 10, selected_target[1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )

        pid_cmd: Optional[Tuple[int, ...]] = None
        pid_err_text = "PID err=N/A"
        if selected_target is not None and laser_center is not None:
            err_x = selected_target[0] - laser_center[0]
            err_y = selected_target[1] - laser_center[1]
            pid_err_text = f"PID err=({err_x:+d},{err_y:+d}) px"

        if sender.stream_pid_enabled and selected_target is not None and laser_center is not None:
            now = time.monotonic()
            if pid_output_mode == "speed":
                pid_cmd = speed_pid.update(selected_target, laser_center, now)
                if pid_cmd is not None:
                    sender.send_speed(pid_cmd[0], pid_cmd[1])
            else:
                pid_cmd = dpos_pid.update(selected_target, laser_center, now)
                if pid_cmd is not None:
                    sender.send_dpos(pid_cmd[0], pid_cmd[1], pid_cmd[2])
        else:
            active_pid = speed_pid if pid_output_mode == "speed" else dpos_pid
            if sender.stream_pid_enabled and active_pid.last_cmd != (0, 0):
                sender.send_stop()
            speed_pid.reset()
            dpos_pid.reset()

        # Process commands again with the newest frame result.
        sender.process_incoming(latest_laser_center)

        tx_state = "ON" if sender.is_ready() else "OFF"
        cal_state = "CAL" if sender.calibration_active else "IDLE"
        pid_state = "ON" if sender.stream_pid_enabled else "OFF"
        dump_state = "DUMPING" if sender.dump_collecting else "NODUMP"
        tx_text = (
            f"Serial {tx_state} | {cal_state} | {dump_state} | PID {pid_state} {pid_output_mode.upper()} | "
            f"P{send_profile_idx + 1} {send_target_kind} | REDPOS {'ON' if red_openloop_track_enabled else 'OFF'}"
        )
        cv2.putText(vis, tx_text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (240, 240, 240), 2)
        pid_cmd_text = f"PID cmd={pid_output_mode.upper()} idle"
        if sender.stream_pid_enabled:
            if pid_output_mode == "speed":
                shown_cmd = speed_pid.last_cmd if pid_cmd is None else pid_cmd
                pid_cmd_text = f"PID cmd=SPEED {shown_cmd[0]} {shown_cmd[1]}"
            else:
                if pid_cmd is None:
                    shown_cmd = dpos_pid.last_cmd
                    pid_cmd_text = f"PID cmd=DPOS {shown_cmd[0]} {shown_cmd[1]} ..."
                else:
                    pid_cmd_text = f"PID cmd=DPOS {pid_cmd[0]} {pid_cmd[1]} {pid_cmd[2]}"
        cv2.putText(vis, f"{pid_err_text} | {pid_cmd_text}", (15, 112), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (220, 220, 220), 2)
        if pos_cmd_preview is not None and pos_query_xy is not None:
            cv2.putText(
                vis,
                f"Target({open_x},{open_y}) -> Query({pos_query_xy[0]},{pos_query_xy[1]}) -> POS {pos_cmd_preview[0]} {pos_cmd_preview[1]} [press o]",
                (15, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (240, 240, 240),
                2,
            )
        else:
            cv2.putText(vis, f"Target({open_x},{open_y}) -> POS N/A (load {CAL_TABLE_DENSE_CSV})", (15, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 220, 255), 2)
        cv2.putText(vis, "Target fine tune: dX/dY sliders or I/J/K/L", (15, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (220, 220, 220), 2)

        cv2.imshow("6+1 Detector", vis)
        # show_mask_windows(results, laser_mask, show_masks)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        # if key == ord("m"):
        #     show_masks = not show_masks
        if key == ord("s"):
            target_ui.print_current_params()
        if key == ord("t"):
            send_target_kind = "target"
            print(f"Serial source switched to target (P{send_profile_idx + 1})")
        if key == ord("b"):
            send_target_kind = "block"
            print(f"Serial source switched to block (P{send_profile_idx + 1})")
        if key in (ord("1"), ord("2"), ord("3")):
            send_profile_idx = min(max(key - ord("1"), 0), len(results) - 1)
            print(f"Serial profile switched to P{send_profile_idx + 1} ({send_target_kind})")
        if key == ord("p"):
            sender.stream_pid_enabled = not sender.stream_pid_enabled
            if not sender.stream_pid_enabled:
                sender.send_stop()
                speed_pid.reset()
                dpos_pid.reset()
            print(f"Upper PID {pid_output_mode.upper()} stream switched to {'ON' if sender.stream_pid_enabled else 'OFF'}")
        if key == ord("y"):
            red_openloop_track_enabled = not red_openloop_track_enabled
            last_red_openloop_send_ts = 0.0
            print(f"RED target POS stream switched to {'ON' if red_openloop_track_enabled else 'OFF'}")
        if key == ord("d"):
            sender.send_cal_dump(begin_collect=True)
        if key == ord("c"):
            sender.send_cal_start(*cal_range)
        if key == ord("o"):
            pos_cmd, pos_xy = _lookup_pos_from_dense_table(dense_table, dense_bounds, open_x, open_y)
            if pos_cmd is None or pos_xy is None:
                print(f"Warning: no POS mapping for target ({open_x}, {open_y}); run calibration + dense export first.")
            else:
                sender.send_pos(pos_cmd[0], pos_cmd[1])
                print(f"POS lookup: target({open_x}, {open_y}) -> sample({pos_xy[0]}, {pos_xy[1]}) -> POS {pos_cmd[0]} {pos_cmd[1]}")
        if key == ord("j") and open_ui is not None:
            open_ui.nudge(-1, 0, frame.shape[1], frame.shape[0])
        if key == ord("l") and open_ui is not None:
            open_ui.nudge(1, 0, frame.shape[1], frame.shape[0])
        if key == ord("i") and open_ui is not None:
            open_ui.nudge(0, -1, frame.shape[1], frame.shape[0])
        if key == ord("k") and open_ui is not None:
            open_ui.nudge(0, 1, frame.shape[1], frame.shape[0])

    sender.send_stop()
    sender.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
