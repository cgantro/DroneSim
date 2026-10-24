"""ULog 변환기가 큰 표본 공백을 놓치지 않는지 확인한다."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from prepare_reference import (
    TopicSamples,
    _load_topics,
    _initial_height_above_ground,
    _measured_motor_speeds,
    _project_attitude,
    _slerp,
    _state_at,
)


class TopicSamplesTests(unittest.TestCase):
    def test_start_height_uses_initial_ground_samples_not_ned_origin(self) -> None:
        position = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 100_000, 200_000, 300_000, 400_000, 500_000],
            "z": [564.70, 564.71, 564.69, 564.45, 564.20, 564.10],
            "z_reset_counter": [7, 7, 7, 7, 7, 7],
        }), "vehicle_local_position", 250_000)
        landed = TopicSamples(SimpleNamespace(data={
            "timestamp": [100_000, 200_000, 300_000, 400_000, 500_000],
            "landed": [True, True, False, False, False],
        }), "vehicle_land_detected", 2_000_000)

        height, ground_z, sample_count = _initial_height_above_ground(
            position, landed, 400_000, 500_000)

        self.assertAlmostEqual(ground_z, 564.70)
        self.assertAlmostEqual(height, 0.50)
        self.assertEqual(sample_count, 2)

    def test_start_height_rejects_vertical_origin_reset_in_comparison_window(self) -> None:
        position = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 100_000, 200_000, 300_000, 400_000],
            "z": [0.0, 0.0, 0.0, 0.1, 0.2],
            "z_reset_counter": [2, 2, 2, 2, 3],
        }), "vehicle_local_position", 250_000)
        landed = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 100_000, 200_000, 300_000, 400_000],
            "landed": [True, True, False, False, False],
        }), "vehicle_land_detected", 2_000_000)

        with self.assertRaisesRegex(ValueError, "수직 위치 원점이 바뀌어"):
            _initial_height_above_ground(position, landed, 300_000, 400_000)

    def test_esc_rpm_uses_px4_motor_function_mapping_and_interpolates(self) -> None:
        data = {"timestamp": [0, 10_000], "esc_count": [4, 4],
                "esc_online_flags": [15, 15]}
        functions = [103, 101, 104, 102]
        first_rpms = [3_000, 6_000, 4_000, 5_000]
        second_rpms = [5_000, 8_000, 6_000, 7_000]
        for slot, function in enumerate(functions):
            data[f"esc[{slot}].actuator_function"] = [function, function]
            data[f"esc[{slot}].esc_rpm"] = [first_rpms[slot], second_rpms[slot]]
        esc_status = TopicSamples(
            SimpleNamespace(data=data), "esc_status", 20_000)

        speeds = _measured_motor_speeds(esc_status, 5_000, 1_000.0)

        expected_rpms_by_motor = [7_000, 6_000, 4_000, 5_000]
        for speed, rpm in zip(speeds, expected_rpms_by_motor):
            self.assertAlmostEqual(speed, rpm * (2.0 * 3.141592653589793 / 60.0))

    def test_esc_rpm_rejects_missing_online_motor(self) -> None:
        data = {"timestamp": [0, 10_000], "esc_count": [4, 4],
                "esc_online_flags": [15, 7]}
        for slot in range(4):
            data[f"esc[{slot}].actuator_function"] = [101 + slot, 101 + slot]
            data[f"esc[{slot}].esc_rpm"] = [3_000, 3_000]
        esc_status = TopicSamples(
            SimpleNamespace(data=data), "esc_status", 20_000)

        with self.assertRaisesRegex(ValueError, "연결이 끊긴 ESC"):
            _measured_motor_speeds(esc_status, 5_000, 1_000.0)

    def test_esc_rpm_rejects_a_motor_mapping_change_inside_the_sample_gap(self) -> None:
        data = {"timestamp": [0, 10_000], "esc_count": [4, 4],
                "esc_online_flags": [15, 15]}
        for slot in range(4):
            data[f"esc[{slot}].actuator_function"] = [101 + slot, 101 + slot]
            data[f"esc[{slot}].esc_rpm"] = [3_000, 3_000]
        data["esc[0].actuator_function"][1] = 102
        data["esc[1].actuator_function"][1] = 101
        esc_status = TopicSamples(
            SimpleNamespace(data=data), "esc_status", 20_000)

        with self.assertRaisesRegex(ValueError, "연결 순서가 바뀝니다"):
            _measured_motor_speeds(esc_status, 5_000, 1_000.0)

    def test_topic_specific_limits_can_be_overridden_for_a_known_log(self) -> None:
        log = SimpleNamespace(data_list=[
            SimpleNamespace(name=name, multi_id=0, data={"timestamp": [0, 100_000]})
            for name in (
                "vehicle_local_position",
                "vehicle_attitude",
                "vehicle_angular_velocity",
                "actuator_motors",
            )
        ])

        default_topics = _load_topics(log)
        overridden_topics = _load_topics(log, maximum_gap_override_microseconds=120_000)

        self.assertEqual(
            [topic.maximum_gap_microseconds for topic in default_topics],
            [250_000, 20_000, 20_000, 20_000],
        )
        self.assertEqual(
            [topic.maximum_gap_microseconds for topic in overridden_topics],
            [120_000, 120_000, 120_000, 120_000],
        )

    def test_interpolates_normal_samples_and_rejects_a_large_gap(self) -> None:
        samples = TopicSamples(
            SimpleNamespace(data={"timestamp": [0, 5_000, 10_000], "value": [0.0, 5.0, 10.0]}),
            topic_name="vehicle_attitude",
            maximum_gap_microseconds=20_000,
        )

        self.assertEqual(samples.scalar("value", 0), 0.0)
        self.assertEqual(samples.scalar("value", 2_500), 2.5)

        sparse_samples = TopicSamples(
            SimpleNamespace(data={
                "timestamp": [0, 700_000, 800_000],
                "value": [0.0, 1.0, 2.0],
            }),
            topic_name="vehicle_attitude",
            maximum_gap_microseconds=20_000,
        )
        with self.assertRaisesRegex(ValueError, r"vehicle_attitude.*700\.0ms.*20\.0ms"):
            sparse_samples.scalar("value", 350_000)

    def test_state_at_converts_ned_frd_samples_to_enu_flu(self) -> None:
        position = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 10_000],
            "x": [10.0, 10.0], "y": [20.0, 20.0], "z": [30.0, 30.0],
            "vx": [4.0, 4.0], "vy": [5.0, 5.0], "vz": [6.0, 6.0],
            "xy_valid": [True, True], "z_valid": [True, True],
            "v_xy_valid": [True, True], "v_z_valid": [True, True],
        }), "vehicle_local_position", 250_000)
        attitude = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 10_000],
            "q[0]": [1.0, 1.0], "q[1]": [0.0, 0.0],
            "q[2]": [0.0, 0.0], "q[3]": [0.0, 0.0],
        }), "vehicle_attitude", 20_000)
        angular_velocity = TopicSamples(SimpleNamespace(data={
            "timestamp": [0, 10_000],
            "xyz[0]": [7.0, 7.0], "xyz[1]": [8.0, 8.0],
            "xyz[2]": [9.0, 9.0],
        }), "vehicle_angular_velocity", 20_000)

        state = _state_at(position, attitude, angular_velocity, 5_000)

        self.assertEqual(state["position_enu"], [20.0, 10.0, -30.0])
        self.assertEqual(state["velocity_enu"], [5.0, 4.0, -6.0])
        self.assertEqual(state["angular_velocity_flu"], [7.0, -8.0, -9.0])
        self.assertAlmostEqual(state["orientation_wxyz"][0], 2 ** -0.5)
        self.assertAlmostEqual(state["orientation_wxyz"][3], 2 ** -0.5)

    def test_slerp_treats_opposite_quaternion_signs_as_same_rotation(self) -> None:
        midpoint = _slerp([1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0], 0.5)

        self.assertEqual(midpoint, [1.0, 0.0, 0.0, 0.0])

    def test_project_attitude_converts_px4_axis_rotations(self) -> None:
        half_turn_component = 2 ** -0.5
        cases = (
            ([half_turn_component, half_turn_component, 0.0, 0.0],
             [0.5, 0.5, 0.5, 0.5]),
            ([half_turn_component, 0.0, half_turn_component, 0.0],
             [0.5, 0.5, -0.5, 0.5]),
            ([half_turn_component, 0.0, 0.0, half_turn_component],
             [1.0, 0.0, 0.0, 0.0]),
        )

        for px4_orientation, expected in cases:
            with self.subTest(px4_orientation=px4_orientation):
                projected = _project_attitude(px4_orientation)
                orientation_dot = abs(sum(
                    actual_component * expected_component
                    for actual_component, expected_component in zip(projected, expected)
                ))
                self.assertAlmostEqual(orientation_dot, 1.0)


if __name__ == "__main__":
    unittest.main()
