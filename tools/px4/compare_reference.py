"""PX4 참조 궤적과 시뮬레이터 결과의 오차를 계산한다."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _read_vector(values: list[float], size: int, label: str) -> list[float]:
    if len(values) != size:
        raise ValueError(f"{label}은(는) {size}개 값이어야 합니다.")
    vector = [float(value) for value in values]
    if not all(math.isfinite(value) for value in vector):
        raise ValueError(f"{label}에 유한하지 않은 값이 있습니다.")
    return vector


def _find_drone(sample: dict, name: str) -> dict:
    for drone in sample["drones"]:
        if drone["name"] == name:
            return drone["state"]
    raise ValueError(f"시뮬레이션 궤적에 '{name}' 드론이 없습니다.")


def _orientation_error_radians(actual: list[float], reference: list[float]) -> float:
    actual = _read_vector(actual, 4, "시뮬레이션 자세")
    reference = _read_vector(reference, 4, "참조 자세")
    actual_length = math.hypot(*actual)
    reference_length = math.hypot(*reference)
    if actual_length == 0.0 or reference_length == 0.0:
        raise ValueError("자세 쿼터니언은 0일 수 없습니다.")
    cosine = abs(sum(
        (a / actual_length) * (b / reference_length)
        for a, b in zip(actual, reference)
    ))
    return 2.0 * math.acos(min(1.0, cosine))


def compare_reference(scenario: dict, result: dict) -> dict:
    """같은 시각의 참조 표본과 시뮬레이션 궤적에서 네 가지 RMSE를 구한다."""
    if scenario.get("dynamics") != "rigid_body":
        raise ValueError("물리 비교에는 rigid_body 시나리오가 필요합니다.")
    physical_model = result.get("physical_model")
    if not physical_model:
        raise ValueError("실행 결과에 강체 물리 모델 설정이 없습니다.")

    reference_samples = scenario["reference_samples"]
    trajectory = result["trajectory"]
    if not reference_samples or len(reference_samples) != len(trajectory):
        raise ValueError("참조 표본과 시뮬레이션 궤적의 개수가 다르거나 비어 있습니다.")

    drone_name = scenario["drones"][0]["name"]
    initial_position = _read_vector(
        _find_drone(trajectory[0], drone_name)["position_meters"],
        3,
        "초기 위치",
    )
    squared_errors = [0.0, 0.0, 0.0, 0.0]

    for reference, sample in zip(reference_samples, trajectory):
        if not math.isclose(reference["t"], sample["time_seconds"], abs_tol=1.0e-9):
            raise ValueError("참조 표본과 시뮬레이션 궤적의 시각이 서로 다릅니다.")
        state = _find_drone(sample, drone_name)
        position = _read_vector(state["position_meters"], 3, "시뮬레이션 위치")
        velocity = _read_vector(
            state["velocity_meters_per_second"], 3, "시뮬레이션 속도"
        )
        angular_velocity = _read_vector(
            state["angular_velocity_radians_per_second"], 3, "시뮬레이션 각속도"
        )
        position_reference = _read_vector(reference["position_delta_enu"], 3, "참조 위치")
        velocity_reference = _read_vector(reference["velocity_enu"], 3, "참조 속도")
        angular_velocity_reference = _read_vector(
            reference["angular_velocity_flu"], 3, "참조 각속도"
        )
        position_error = [
            position[i] - initial_position[i] - position_reference[i]
            for i in range(3)
        ]
        velocity_error = [
            velocity[i] - velocity_reference[i] for i in range(3)
        ]
        angular_velocity_error = [
            angular_velocity[i] - angular_velocity_reference[i]
            for i in range(3)
        ]
        squared_errors[0] += sum(value * value for value in position_error)
        squared_errors[1] += sum(value * value for value in velocity_error)
        squared_errors[3] += sum(value * value for value in angular_velocity_error)

        attitude_error = _orientation_error_radians(
            state["orientation_quaternion_wxyz"],
            reference["orientation_wxyz"],
        )
        squared_errors[2] += attitude_error * attitude_error

    count = len(reference_samples)
    rmse = [math.sqrt(total / count) for total in squared_errors]
    metadata = scenario.get("reference_metadata", {})
    relative_start = reference_samples[0]["t"]
    relative_end = reference_samples[-1]["t"]
    start_since_boot = metadata.get("window_start_seconds_since_boot")
    return {
        "drone": drone_name,
        "outcome": result["outcome"],
        "sample_count": count,
        "duration_seconds": relative_end - relative_start,
        "reference_source": metadata.get("source", "출처 정보 없음"),
        "window_start_seconds_since_boot": (
            None if start_since_boot is None else start_since_boot + relative_start
        ),
        "window_end_seconds_since_boot": (
            None if start_since_boot is None else start_since_boot + relative_end
        ),
        "sample_period_seconds": metadata.get("sample_period_seconds"),
        "maximum_observed_sample_gap_microseconds": metadata.get(
            "maximum_observed_sample_gap_microseconds", {}
        ),
        "maximum_allowed_sample_gap_microseconds": metadata.get(
            "maximum_allowed_sample_gap_microseconds", {}
        ),
        "sample_gap_override_microseconds": metadata.get(
            "sample_gap_override_microseconds"
        ),
        "physical_model": physical_model,
        "position_rmse_meters": rmse[0],
        "velocity_rmse_meters_per_second": rmse[1],
        "attitude_rmse_degrees": math.degrees(rmse[2]),
        "angular_velocity_rmse_radians_per_second": rmse[3],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="참조 궤적과 시뮬레이션 결과의 위치·속도·자세·각속도 오차를 계산합니다."
    )
    parser.add_argument("scenario", type=Path, help="참조 표본이 포함된 입력 시나리오 JSON")
    parser.add_argument("result", type=Path, help="화면 없는 실행 결과 JSON")
    args = parser.parse_args()

    try:
        scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
        result = json.loads(args.result.read_text(encoding="utf-8"))
        metrics = compare_reference(scenario, result)
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))

    print(f"대상 드론: {metrics['drone']}")
    print(f"실행 결과: {metrics['outcome']} (시나리오 판정이며 물리 정확도 판정은 아님)")
    print(f"비교 표본: {metrics['sample_count']}개, {metrics['duration_seconds']:.3f}초")
    print(f"참조 자료: {metrics['reference_source']}")
    if metrics["window_start_seconds_since_boot"] is not None:
        print(
            "로그 구간: "
            f"{metrics['window_start_seconds_since_boot']:.3f}~"
            f"{metrics['window_end_seconds_since_boot']:.3f}초 (부팅 후)"
        )
    if metrics["sample_period_seconds"] is not None:
        print(f"재표본 간격: {metrics['sample_period_seconds'] * 1000.0:.3f}ms")
    gap_labels = (
        ("vehicle_local_position", "위치·속도"),
        ("vehicle_attitude", "자세"),
        ("vehicle_angular_velocity", "각속도"),
        ("actuator_motors", "모터 명령"),
    )
    observed_gaps = metrics["maximum_observed_sample_gap_microseconds"]
    allowed_gaps = metrics["maximum_allowed_sample_gap_microseconds"]
    if observed_gaps:
        print("참조 표본 간격 최대: " + ", ".join(
            f"{label} {observed_gaps[name] / 1000.0:.3f}/"
            f"{allowed_gaps[name] / 1000.0:.3f}ms"
            for name, label in gap_labels
            if name in observed_gaps and name in allowed_gaps
        ) + " (관측/허용)")
    if metrics["sample_gap_override_microseconds"] is not None:
        print(
            "표본 간격 완화 옵션: "
            f"{metrics['sample_gap_override_microseconds'] / 1000.0:.3f}ms"
        )
    model = metrics["physical_model"]
    inertia = ", ".join(
        f"{value:.5g}" for value in model["principal_inertia_kg_m2"]
    )
    print(
        "물리 설정: "
        f"질량 {model['mass_kg']:.4g}kg, 관성 [{inertia}]kg·m², "
        f"추력 계수 {model['thrust_coefficient_newtons_per_speed_squared']:.5g}, "
        f"토크 비 {model['moment_to_thrust_ratio']:.5g}, "
        f"모터 응답 {model['motor_time_constant_up_seconds']:.4g}/"
        f"{model['motor_time_constant_down_seconds']:.4g}s"
    )
    print("오차 RMSE")
    print(f"  위치:     {metrics['position_rmse_meters']:.4f} m")
    print(f"  속도:     {metrics['velocity_rmse_meters_per_second']:.4f} m/s")
    print(f"  자세:     {metrics['attitude_rmse_degrees']:.4f}°")
    print(f"  각속도:   {metrics['angular_velocity_rmse_radians_per_second']:.4f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
