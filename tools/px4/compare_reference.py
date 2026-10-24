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
    return {
        "drone": drone_name,
        "outcome": result["outcome"],
        "sample_count": count,
        "duration_seconds": reference_samples[-1]["t"] - reference_samples[0]["t"],
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
    print("오차 RMSE")
    print(f"  위치:     {metrics['position_rmse_meters']:.4f} m")
    print(f"  속도:     {metrics['velocity_rmse_meters_per_second']:.4f} m/s")
    print(f"  자세:     {metrics['attitude_rmse_degrees']:.4f}°")
    print(f"  각속도:   {metrics['angular_velocity_rmse_radians_per_second']:.4f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
