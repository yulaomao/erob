from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .sdk import AxisMetadata, ErobController


@dataclass(slots=True)
class FollowTestSample:
    elapsed_s: float
    target_angle_deg: float
    actual_angle_deg: float
    actual_velocity_deg_s: float
    position_error_deg: float


@dataclass(slots=True)
class FollowTestResult:
    axis_id: int
    axis_name: str
    duration_s: float
    sample_period_s: float
    amplitude_scale: float
    samples: list[FollowTestSample]
    max_abs_error_deg: float
    mean_abs_error_deg: float
    rms_error_deg: float
    p95_abs_error_deg: float
    peak_velocity_deg_s: float
    target_span_deg: float
    reversal_count: int


def run_follow_performance_test(
    controller: ErobController,
    axis_id: int,
    metadata: AxisMetadata,
    sample_period_s: float = 1.0 / 60.0,
    amplitude_scale: float = 0.6,
) -> FollowTestResult:
    state = controller.get_axis_state(axis_id)
    lower_limit, upper_limit = _safe_follow_window(metadata)
    profile_points, _ = _build_challenging_profile(
        start_angle_deg=state.actual_angle_deg,
        lower_limit_deg=lower_limit,
        upper_limit_deg=upper_limit,
        amplitude_scale=amplitude_scale,
    )
    duration_s = profile_points[-1][0]
    samples: list[FollowTestSample] = []
    next_tick = time.perf_counter()
    start_time = next_tick

    controller.start_follow(axis_id)
    try:
        while True:
            now = time.perf_counter()
            elapsed_s = now - start_time
            if elapsed_s > duration_s:
                break

            target_angle_deg = _interpolate_target(profile_points, elapsed_s)
            controller.update_follow_target(axis_id, target_angle_deg)
            state = controller.get_axis_state(axis_id)
            samples.append(
                FollowTestSample(
                    elapsed_s=elapsed_s,
                    target_angle_deg=target_angle_deg,
                    actual_angle_deg=state.actual_angle_deg,
                    actual_velocity_deg_s=state.actual_velocity_deg_s,
                    position_error_deg=state.position_error_deg,
                )
            )

            next_tick += sample_period_s
            remaining_s = next_tick - time.perf_counter()
            if remaining_s > 0:
                time.sleep(remaining_s)

        final_target_deg = _interpolate_target(profile_points, duration_s)
        controller.update_follow_target(axis_id, final_target_deg)
    finally:
        controller.stop_follow(axis_id)

    return _build_result(
        axis_id=axis_id,
        axis_name=metadata.joint_name,
        sample_period_s=sample_period_s,
        amplitude_scale=_clamp(amplitude_scale, 0.2, 1.0),
        samples=samples,
        profile_points=profile_points,
    )


def _safe_follow_window(metadata: AxisMetadata) -> tuple[float, float]:
    span_deg = metadata.max_angle_deg - metadata.min_angle_deg
    margin_deg = max(6.0, span_deg * 0.08)
    lower_limit_deg = metadata.min_angle_deg + margin_deg
    upper_limit_deg = metadata.max_angle_deg - margin_deg
    if lower_limit_deg >= upper_limit_deg:
        return metadata.min_angle_deg, metadata.max_angle_deg
    return lower_limit_deg, upper_limit_deg


def _build_challenging_profile(
    start_angle_deg: float,
    lower_limit_deg: float,
    upper_limit_deg: float,
    amplitude_scale: float,
) -> tuple[list[tuple[float, float]], float]:
    span_deg = max(upper_limit_deg - lower_limit_deg, 1.0)
    amplitude_scale = _clamp(amplitude_scale, 0.2, 1.0)
    center_deg = _clamp(start_angle_deg, lower_limit_deg + span_deg * 0.18, upper_limit_deg - span_deg * 0.18)
    deep_positive_deg = upper_limit_deg - span_deg * 0.08
    deep_negative_deg = lower_limit_deg + span_deg * 0.08
    mid_positive_deg = upper_limit_deg - span_deg * 0.28
    mid_negative_deg = lower_limit_deg + span_deg * 0.32
    shallow_positive_deg = center_deg + span_deg * 0.18
    shallow_negative_deg = center_deg - span_deg * 0.16

    base_profile_points = [
        (0.0, center_deg),
        (0.45, center_deg),
        (1.65, mid_positive_deg),
        (2.35, deep_negative_deg),
        (3.65, deep_positive_deg),
        (4.35, mid_negative_deg),
        (5.55, shallow_positive_deg),
        (6.15, deep_negative_deg),
        (7.25, mid_positive_deg),
        (7.95, shallow_negative_deg),
        (9.05, deep_positive_deg),
        (9.65, center_deg),
        (10.75, deep_negative_deg),
        (11.35, center_deg),
    ]

    return (
        _scale_profile_about_center(
            base_profile_points,
            center_deg=center_deg,
            amplitude_scale=amplitude_scale,
            lower_limit_deg=lower_limit_deg,
            upper_limit_deg=upper_limit_deg,
        ),
        center_deg,
    )


def _interpolate_target(profile_points: list[tuple[float, float]], elapsed_s: float) -> float:
    if elapsed_s <= profile_points[0][0]:
        return profile_points[0][1]
    for index in range(1, len(profile_points)):
        end_time_s, end_angle_deg = profile_points[index]
        start_time_s, start_angle_deg = profile_points[index - 1]
        if elapsed_s <= end_time_s:
            segment_duration_s = max(end_time_s - start_time_s, 1e-6)
            ratio = (elapsed_s - start_time_s) / segment_duration_s
            smooth_ratio = 0.5 - 0.5 * math.cos(math.pi * ratio)
            return start_angle_deg + (end_angle_deg - start_angle_deg) * smooth_ratio
    return profile_points[-1][1]


def _build_result(
    axis_id: int,
    axis_name: str,
    sample_period_s: float,
    amplitude_scale: float,
    samples: list[FollowTestSample],
    profile_points: list[tuple[float, float]],
) -> FollowTestResult:
    if not samples:
        raise RuntimeError("随动测试未采集到样本")

    abs_errors = [abs(sample.position_error_deg) for sample in samples]
    mean_abs_error_deg = sum(abs_errors) / len(abs_errors)
    rms_error_deg = math.sqrt(sum(error * error for error in abs_errors) / len(abs_errors))
    sorted_errors = sorted(abs_errors)
    p95_index = min(len(sorted_errors) - 1, max(0, math.ceil(len(sorted_errors) * 0.95) - 1))
    target_values = [angle for _, angle in profile_points]

    return FollowTestResult(
        axis_id=axis_id,
        axis_name=axis_name,
        duration_s=samples[-1].elapsed_s,
        sample_period_s=sample_period_s,
        amplitude_scale=amplitude_scale,
        samples=samples,
        max_abs_error_deg=max(abs_errors),
        mean_abs_error_deg=mean_abs_error_deg,
        rms_error_deg=rms_error_deg,
        p95_abs_error_deg=sorted_errors[p95_index],
        peak_velocity_deg_s=max(abs(sample.actual_velocity_deg_s) for sample in samples),
        target_span_deg=max(target_values) - min(target_values),
        reversal_count=_count_reversals(profile_points),
    )


def _scale_profile_about_center(
    profile_points: list[tuple[float, float]],
    center_deg: float,
    amplitude_scale: float,
    lower_limit_deg: float,
    upper_limit_deg: float,
) -> list[tuple[float, float]]:
    scaled_profile_points: list[tuple[float, float]] = []
    for time_s, angle_deg in profile_points:
        scaled_angle_deg = center_deg + (angle_deg - center_deg) * amplitude_scale
        scaled_profile_points.append(
            (time_s, _clamp(scaled_angle_deg, lower_limit_deg, upper_limit_deg))
        )
    return scaled_profile_points


def _count_reversals(profile_points: list[tuple[float, float]]) -> int:
    reversals = 0
    last_sign = 0
    for index in range(1, len(profile_points)):
        delta_deg = profile_points[index][1] - profile_points[index - 1][1]
        if abs(delta_deg) < 1e-6:
            continue
        current_sign = 1 if delta_deg > 0 else -1
        if last_sign and current_sign != last_sign:
            reversals += 1
        last_sign = current_sign
    return reversals


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))