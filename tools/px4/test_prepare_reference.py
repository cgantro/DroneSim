"""ULog 변환기가 큰 표본 공백을 놓치지 않는지 확인한다."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from prepare_reference import TopicSamples, _slerp, _state_at


class TopicSamplesTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
