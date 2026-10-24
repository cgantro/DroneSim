"""PX4 ULog를 60Hz ENU/FLU 재생 시나리오와 참조 궤적으로 바꾼다."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
from pathlib import Path

STEP_SECONDS = 1.0 / 60.0
MAXIMUM_SAMPLE_GAP_MICROSECONDS = {
    "vehicle_local_position": 250_000,
    "vehicle_attitude": 20_000,
    "vehicle_angular_velocity": 20_000,
    "actuator_motors": 20_000,
}
MAXIMUM_ESC_SAMPLE_GAP_MICROSECONDS = 20_000
MAXIMUM_LANDED_SAMPLE_GAP_MICROSECONDS = 2_000_000
MOTOR_FUNCTION_TO_INDEX = {101: 0, 102: 1, 103: 2, 104: 3}
QUADROTOR_MODEL = {
    "mass_kg": 2.0643076923076924,
    "principal_inertia_kg_m2": [
        0.023846692647701416,
        0.02394961745804886,
        0.043999953714003955,
    ],
    "thrust_coefficient_newtons_per_speed_squared": 8.54858e-6,
    "moment_to_thrust_ratio": 0.016,
    "motor_time_constant_up_seconds": 0.0125,
    "motor_time_constant_down_seconds": 0.025,
    "maximum_motor_speed_radians_per_second": 1000.0,
}

# PX4의 FRD→NED 자세를 프로젝트의 FLU→ENU 자세로 바꿀 때 쓰는 축 변환이다.
WORLD_NED_FROM_ENU = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
BODY_FRD_FROM_FLU = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]


def _normalize_quaternion(value: list[float]) -> list[float]:
    length = math.sqrt(sum(component * component for component in value))
    if not math.isfinite(length) or length < 1.0e-12:
        raise ValueError("자세 쿼터니언이 0이거나 유한한 값이 아닙니다.")
    return [component / length for component in value]


def _slerp(first: list[float], second: list[float], ratio: float) -> list[float]:
    first = _normalize_quaternion(first)
    second = _normalize_quaternion(second)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = [-component for component in second]
        dot = -dot
    if dot > 0.9995:
        return _normalize_quaternion([
            a + ratio * (b - a) for a, b in zip(first, second)
        ])

    angle = math.acos(max(-1.0, min(1.0, dot)))
    sine = math.sin(angle)
    first_weight = math.sin((1.0 - ratio) * angle) / sine
    second_weight = math.sin(ratio * angle) / sine
    return [
        first_weight * a + second_weight * b
        for a, b in zip(first, second)
    ]


def _quaternion_matrix(value: list[float]) -> list[list[float]]:
    w, x, y, z = _normalize_quaternion(value)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matrix_multiply(first: list[list[float]], second: list[list[float]]) -> list[list[float]]:
    return [
        [sum(first[row][index] * second[index][column] for index in range(3))
         for column in range(3)]
        for row in range(3)
    ]


def _matrix_quaternion(matrix: list[list[float]]) -> list[float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        result = [
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        ]
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        result = [
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        ]
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        result = [
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        ]
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        result = [
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        ]
    return _normalize_quaternion(result)


def _project_attitude(px4_quaternion: list[float]) -> list[float]:
    rotation = _matrix_multiply(
        _matrix_multiply(WORLD_NED_FROM_ENU, _quaternion_matrix(px4_quaternion)),
        BODY_FRD_FROM_FLU,
    )
    return _matrix_quaternion(rotation)


class TopicSamples:
    """한 ULog 주제의 시각 배열과 필요한 값만 보관한다."""

    def __init__(
        self,
        dataset,
        topic_name: str,
        maximum_gap_microseconds: int,
    ) -> None:
        self.data = dataset.data
        self.topic_name = topic_name
        self.maximum_gap_microseconds = maximum_gap_microseconds
        self.maximum_observed_gap_microseconds = 0
        self.times = [int(value) for value in self.data["timestamp"]]
        if len(self.times) < 2 or any(
            current <= previous for previous, current in zip(self.times, self.times[1:])
        ):
            raise ValueError("주제의 시각 자료가 없거나 오름차순이 아닙니다.")

    def _bracket(self, time_us: int) -> tuple[int, int, float]:
        right = bisect.bisect_left(self.times, time_us)
        if right < len(self.times) and self.times[right] == time_us:
            return right, right, 0.0
        if right == 0 or right == len(self.times):
            raise ValueError("선택한 구간에 보간할 표본이 부족합니다.")
        left = right - 1
        gap_microseconds = self.times[right] - self.times[left]
        self.maximum_observed_gap_microseconds = max(
            self.maximum_observed_gap_microseconds,
            gap_microseconds,
        )
        if gap_microseconds > self.maximum_gap_microseconds:
            raise ValueError(
                f"{self.topic_name} 표본 사이 공백 {gap_microseconds / 1000.0:.1f}ms가 "
                f"허용치 {self.maximum_gap_microseconds / 1000.0:.1f}ms를 넘습니다."
            )
        ratio = (time_us - self.times[left]) / gap_microseconds
        return left, right, ratio

    def scalar(self, field: str, time_us: int) -> float:
        left, right, ratio = self._bracket(time_us)
        first = float(self.data[field][left])
        second = float(self.data[field][right])
        value = first + ratio * (second - first)
        if not math.isfinite(value):
            raise ValueError(f"{field}에 유한하지 않은 값이 있습니다.")
        return value

    def vector(self, field: str, time_us: int, size: int = 3) -> list[float]:
        left, right, ratio = self._bracket(time_us)
        if field in self.data:
            first = self.data[field][left]
            second = self.data[field][right]
            value = [
                float(first[index]) + ratio * (float(second[index]) - float(first[index]))
                for index in range(size)
            ]
        else:
            value = []
            for index in range(size):
                first = float(self.data[f"{field}[{index}]"][left])
                second = float(self.data[f"{field}[{index}]"][right])
                value.append(first + ratio * (second - first))
        if not all(math.isfinite(component) for component in value):
            raise ValueError(f"{field}에 유한하지 않은 값이 있습니다.")
        return value

    def quaternion(self, time_us: int) -> list[float]:
        left, right, ratio = self._bracket(time_us)
        first = [float(self.data[f"q[{index}]"][left]) for index in range(4)]
        second = [float(self.data[f"q[{index}]"][right]) for index in range(4)]
        return _slerp(first, second, ratio)


def _state_at(
    position: TopicSamples,
    attitude: TopicSamples,
    angular_velocity: TopicSamples,
    time_us: int,
) -> dict:
    if not all(
        bool(position.data[field][index])
        for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")
        for index in position._bracket(time_us)[:2]
    ):
        raise ValueError("선택한 구간의 위치 또는 속도 표본이 유효하지 않습니다.")

    ned_position = [position.scalar(axis, time_us) for axis in ("x", "y", "z")]
    ned_velocity = [position.scalar(axis, time_us) for axis in ("vx", "vy", "vz")]
    frd_rate = angular_velocity.vector("xyz", time_us)
    return {
        "position_enu": [ned_position[1], ned_position[0], -ned_position[2]],
        "velocity_enu": [ned_velocity[1], ned_velocity[0], -ned_velocity[2]],
        "orientation_wxyz": _project_attitude(attitude.quaternion(time_us)),
        "angular_velocity_flu": [frd_rate[0], -frd_rate[1], -frd_rate[2]],
    }


def _load_topics(
    log: ULog,
    maximum_gap_override_microseconds: int | None = None,
) -> tuple[TopicSamples, TopicSamples, TopicSamples, TopicSamples]:
    datasets = {dataset.name: dataset for dataset in log.data_list if dataset.multi_id == 0}
    required = (
        "vehicle_local_position",
        "vehicle_attitude",
        "vehicle_angular_velocity",
        "actuator_motors",
    )
    missing = [name for name in required if name not in datasets]
    if missing:
        raise ValueError("로그에 필요한 주제가 없습니다: " + ", ".join(missing))
    return tuple(
        TopicSamples(
            datasets[name],
            name,
            maximum_gap_override_microseconds
            if maximum_gap_override_microseconds is not None
            else MAXIMUM_SAMPLE_GAP_MICROSECONDS[name],
        )
        for name in required
    )


def _load_esc_topic(log) -> TopicSamples | None:
    dataset = next((
        item for item in log.data_list
        if item.name == "esc_status" and item.multi_id == 0
    ), None)
    if dataset is None:
        return None
    return TopicSamples(
        dataset,
        "esc_status",
        MAXIMUM_ESC_SAMPLE_GAP_MICROSECONDS,
    )


def _load_landed_topic(log) -> TopicSamples | None:
    dataset = next((
        item for item in log.data_list
        if item.name == "vehicle_land_detected" and item.multi_id == 0
    ), None)
    if dataset is None:
        return None
    if "landed" not in dataset.data:
        raise ValueError("착륙 상태 주제에 landed 값이 없습니다.")
    return TopicSamples(
        dataset,
        "vehicle_land_detected",
        MAXIMUM_LANDED_SAMPLE_GAP_MICROSECONDS,
    )


def _initial_height_above_ground(
    position: TopicSamples,
    landed: TopicSamples | None,
    start_us: int,
    end_us: int,
) -> tuple[float, float | None]:
    """시뮬레이터 지면을 기준으로 시작 높이를 구한다."""
    start_left, start_right, _ = position._bracket(start_us)
    end_left, end_right, _ = position._bracket(end_us)
    reset_counters = position.data.get("z_reset_counter")
    selected_indices = range(start_left, end_right + 1)
    if reset_counters is not None:
        selected_resets = {int(reset_counters[index]) for index in selected_indices}
        if len(selected_resets) > 1:
            raise ValueError("선택 구간 중 PX4 수직 위치 원점이 바뀌어 연속 비교할 수 없습니다.")

    if landed is None:
        return -position.scalar("z", start_us), None

    ground_samples = []
    first_reset = None
    for position_index, time_us in enumerate(position.times):
        if time_us > start_us:
            break
        landed_index = bisect.bisect_right(landed.times, time_us) - 1
        if (landed_index < 0
                or time_us - landed.times[landed_index]
                > MAXIMUM_LANDED_SAMPLE_GAP_MICROSECONDS
                or not bool(landed.data["landed"][landed_index])):
            break

        if reset_counters is not None:
            reset_counter = int(reset_counters[position_index])
            if first_reset is None:
                first_reset = reset_counter
            elif reset_counter != first_reset:
                raise ValueError("착륙 기준 구간에서 PX4 수직 위치 원점이 바뀌었습니다.")
        ground_samples.append(float(position.data["z"][position_index]))

    if len(ground_samples) < 2:
        return -position.scalar("z", start_us), None

    if reset_counters is not None and any(
        int(reset_counters[index]) != first_reset
        for index in selected_indices
    ):
        raise ValueError("착륙 기준과 선택 구간 사이에 PX4 수직 위치 원점이 바뀌었습니다.")

    ground_z = statistics.median(ground_samples)
    height_above_ground = ground_z - position.scalar("z", start_us)
    if not math.isfinite(height_above_ground):
        raise ValueError("시작 높이를 유한한 값으로 계산할 수 없습니다.")
    return height_above_ground, ground_z


def _measured_motor_speeds(
    esc_status: TopicSamples,
    time_us: int,
    maximum_speed_radians_per_second: float,
) -> list[float]:
    left, right, _ = esc_status._bracket(time_us)
    data = esc_status.data
    motor_speeds = [0.0] * len(MOTOR_FUNCTION_TO_INDEX)
    mappings = []

    for sample_index in {left, right}:
        if int(data["esc_count"][sample_index]) != len(motor_speeds):
            raise ValueError("선택한 구간에서 네 ESC의 회전수 자료를 모두 확인할 수 없습니다.")
        online_flags = int(data["esc_online_flags"][sample_index])
        required_flags = (1 << len(motor_speeds)) - 1
        if online_flags & required_flags != required_flags:
            raise ValueError("선택한 구간에 연결이 끊긴 ESC가 있습니다.")

        mapping = []
        mapped_motors = set()
        for esc_index in range(len(motor_speeds)):
            prefix = f"esc[{esc_index}]"
            function = int(data[f"{prefix}.actuator_function"][sample_index])
            motor_index = MOTOR_FUNCTION_TO_INDEX.get(function)
            if motor_index is None or motor_index in mapped_motors:
                raise ValueError("ESC 출력과 PX4 모터 1~4의 연결을 확인할 수 없습니다.")
            mapped_motors.add(motor_index)
            mapping.append(motor_index)

        if mapped_motors != set(range(len(motor_speeds))):
            raise ValueError("ESC 출력과 PX4 모터 1~4의 연결이 완전하지 않습니다.")
        mappings.append(mapping)

    if any(mapping != mappings[0] for mapping in mappings[1:]):
        raise ValueError("선택한 구간에서 ESC와 PX4 모터의 연결 순서가 바뀝니다.")

    for esc_index, motor_index in enumerate(mappings[0]):
        rpm = esc_status.scalar(f"esc[{esc_index}].esc_rpm", time_us)
        if rpm < 0.0:
            raise ValueError("ESC가 음수 회전수를 보고했습니다.")
        speed = rpm * (2.0 * math.pi / 60.0)
        if speed > maximum_speed_radians_per_second:
            raise ValueError("ESC 회전수가 시나리오의 최대 모터 속도를 넘습니다.")
        motor_speeds[motor_index] = speed

    return motor_speeds


def prepare_scenario(
    log_path: Path,
    start_seconds: float,
    duration_seconds: float,
    source_id: str | None,
    maximum_sample_gap_microseconds: int | None = None,
) -> dict:
    from pyulog import ULog

    log = ULog(str(log_path))
    if (maximum_sample_gap_microseconds is not None
            and maximum_sample_gap_microseconds <= 0):
        raise ValueError("표본 간격 허용치는 0보다 커야 합니다.")
    maximum_sample_gaps = {
        name: maximum_sample_gap_microseconds
        if maximum_sample_gap_microseconds is not None
        else default_gap
        for name, default_gap in MAXIMUM_SAMPLE_GAP_MICROSECONDS.items()
    }
    position, attitude, angular_velocity, motors = _load_topics(
        log, maximum_sample_gap_microseconds)
    esc_status = _load_esc_topic(log)
    landed = _load_landed_topic(log)
    tick_count_float = duration_seconds / STEP_SECONDS
    tick_count = round(tick_count_float)
    if tick_count <= 0 or not math.isclose(tick_count_float, tick_count, abs_tol=1.0e-9):
        raise ValueError("재생 구간 길이는 60Hz Tick 수로 정확히 나뉘어야 합니다.")

    start_us = int(log.start_timestamp + start_seconds * 1_000_000)
    end_us = start_us + round(duration_seconds * 1_000_000)
    initial = _state_at(position, attitude, angular_velocity, start_us)
    initial_height, ground_z = _initial_height_above_ground(
        position, landed, start_us, end_us)
    first_command = motors.vector("control", start_us, 4)
    if any(value < 0.0 or value > 1.0 for value in first_command):
        raise ValueError("현재 기체 모델은 0~1 범위의 정규화 추력만 재생합니다.")

    reference_samples = []
    commands = []
    maximum_speed = QUADROTOR_MODEL["maximum_motor_speed_radians_per_second"]
    initial_motor_speeds = (
        _measured_motor_speeds(esc_status, start_us, maximum_speed)
        if esc_status is not None
        else [math.sqrt(value) * maximum_speed for value in first_command]
    )
    for tick in range(tick_count + 1):
        time_us = start_us + round(tick * STEP_SECONDS * 1_000_000)
        state = _state_at(position, attitude, angular_velocity, time_us)
        reference_samples.append({
            "t": tick * STEP_SECONDS,
            "position_delta_enu": [
                state["position_enu"][axis] - initial["position_enu"][axis]
                for axis in range(3)
            ],
            "velocity_enu": state["velocity_enu"],
            "orientation_wxyz": state["orientation_wxyz"],
            "angular_velocity_flu": state["angular_velocity_flu"],
            **({"motor_speeds_radians_per_second": _measured_motor_speeds(
                esc_status, time_us, maximum_speed)} if esc_status is not None else {}),
        })
        if tick == tick_count:
            continue
        command_time_us = start_us + round(tick * STEP_SECONDS * 1_000_000)
        thrust = motors.vector("control", command_time_us, 4)
        if any(value < 0.0 or value > 1.0 for value in thrust):
            raise ValueError("현재 기체 모델은 0~1 범위의 정규화 추력만 재생합니다.")
        commands.append({
            "tick": tick,
            "type": "set_rotor_thrusts",
            "normalized_thrust": thrust,
        })

    initial_position = [0.0, 0.0, initial_height]
    return {
        "version": 1,
        "dynamics": "rigid_body",
        "ticks": tick_count,
        "drones": [{
            "name": "x500",
            "initial_position_meters": initial_position,
            "initial_velocity_meters_per_second": initial["velocity_enu"],
            "initial_orientation_quaternion_wxyz": initial["orientation_wxyz"],
            "initial_angular_velocity_radians_per_second": initial["angular_velocity_flu"],
            "initial_motor_speeds_radians_per_second": initial_motor_speeds,
            "initial_mode": "flying",
        }],
        "commands": commands,
        "quadrotor_model": QUADROTOR_MODEL,
        "record_trajectory": {"sample_every_ticks": 1},
        "expected": {
            "drones": [{
                "name": "x500",
                "mode": "flying",
                "position_meters": initial_position,
            }],
            "tolerance_meters": 1000.0,
        },
        "reference_metadata": {
            "source": f"PX4 Flight Review ULog {source_id}"
                if source_id else "사용자가 제공한 PX4 ULog",
            "source_url": (
                f"https://logs.px4.io/plot_app?log={source_id}" if source_id else None
            ),
            "license": "CC-BY PX4" if source_id else "확인 필요",
            "window_start_seconds_since_boot": start_seconds,
            "window_duration_seconds": duration_seconds,
            "sample_period_seconds": STEP_SECONDS,
            "position_and_velocity": "NED [north,east,down] to ENU [east,north,up]",
            "attitude": "PX4 FRD-to-NED to project FLU-to-ENU",
            "angular_velocity": "FRD [forward,right,down] to FLU [forward,left,up]",
            "motor_input": "actuator_motors.control; normalized thrust, linearly interpolated",
            "motor_speed_reference": "esc_status RPM mapped by actuator_function"
                if esc_status is not None else None,
            "initial_motor_speed": "ESC RPM converted to rad/s"
                if esc_status is not None
                else "estimated from normalized thrust; not measured RPM",
            "vertical_ground_reference": (
                "median of the initial landed position samples"
                if ground_z is not None
                else "PX4 local NED origin assumed to match the simulation ground"
            ),
            "ground_reference_z_ned_meters": ground_z,
            "initial_height_above_ground_meters": initial_height,
            "maximum_observed_sample_gap_microseconds": {
                "vehicle_local_position": position.maximum_observed_gap_microseconds,
                "vehicle_attitude": attitude.maximum_observed_gap_microseconds,
                "vehicle_angular_velocity": angular_velocity.maximum_observed_gap_microseconds,
                "actuator_motors": motors.maximum_observed_gap_microseconds,
                "esc_status": esc_status.maximum_observed_gap_microseconds
                    if esc_status is not None else None,
            },
            "maximum_allowed_sample_gap_microseconds": {
                **maximum_sample_gaps,
                "esc_status": MAXIMUM_ESC_SAMPLE_GAP_MICROSECONDS
                    if esc_status is not None else None,
            },
            "sample_gap_override_microseconds": maximum_sample_gap_microseconds,
        },
        "reference_samples": reference_samples,
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="PX4 ULog에서 개인정보를 뺀 재생 시나리오와 참조 궤적을 만듭니다."
    )
    parser.add_argument("ulog", type=Path, help="원본 ULog 경로")
    parser.add_argument("output", type=Path, help="새로 만들 JSON 경로; 기존 파일은 덮어쓰지 않음")
    parser.add_argument("--start", type=float, default=12.05, help="로그 시작 후 구간 시작 시각(초)")
    parser.add_argument("--duration", type=float, default=4.0, help="재생 구간 길이(초, 60Hz 배수)")
    parser.add_argument("--source-id", help="공개 Flight Review 로그 ID; 출처 링크와 이용 조건 기록용")
    parser.add_argument(
        "--maximum-sample-gap-ms",
        type=float,
        help="모든 필수 주제의 보간 허용 간격을 지정합니다. 기본값은 주제별 제한입니다.",
    )
    args = parser.parse_args()

    try:
        if not args.ulog.is_file():
            raise ValueError("원본 ULog 파일을 찾을 수 없습니다.")
        if args.output.exists():
            raise ValueError("출력 파일이 이미 있습니다. 다른 경로를 지정해 주세요.")
        if (not math.isfinite(args.start) or not math.isfinite(args.duration)
                or args.start < 0.0 or args.duration <= 0.0):
            raise ValueError("시작 시각은 0 이상, 구간 길이는 0보다 커야 합니다.")
        maximum_sample_gap_microseconds = None
        if args.maximum_sample_gap_ms is not None:
            if (not math.isfinite(args.maximum_sample_gap_ms)
                    or args.maximum_sample_gap_ms <= 0.0):
                raise ValueError("표본 간격 허용치는 유한한 0 초과 값이어야 합니다.")
            maximum_sample_gap_microseconds = round(args.maximum_sample_gap_ms * 1000.0)
            if maximum_sample_gap_microseconds <= 0:
                raise ValueError("표본 간격 허용치는 0보다 커야 합니다.")
        scenario = prepare_scenario(
            args.ulog,
            args.start,
            args.duration,
            args.source_id,
            maximum_sample_gap_microseconds,
        )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(scenario, ensure_ascii=False, separators=(",", ":")) + "\n",
            encoding="utf-8",
        )
    except (OSError, ValueError, KeyError, IndexError) as error:
        parser.error(str(error))

    print(
        f"{args.output} 생성: {scenario['ticks']} Tick, "
        f"{len(scenario['reference_samples'])} 참조 표본"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
