from __future__ import annotations

import unittest

from compare_reference import compare_reference


class CompareReferenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = {
            "drones": [{"name": "x500"}],
            "reference_samples": [{
                "t": 0.0,
                "position_delta_enu": [0.0, 0.0, 0.0],
                "velocity_enu": [1.0, 0.0, 0.0],
                "orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
                "angular_velocity_flu": [0.0, 0.0, 0.5],
            }],
        }
        self.result = {
            "outcome": "passed",
            "trajectory": [{
                "time_seconds": 0.0,
                "drones": [{"name": "x500", "state": {
                    "position_meters": [10.0, 20.0, 2.0],
                    "velocity_meters_per_second": [1.0, 0.0, 0.0],
                    "orientation_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "angular_velocity_radians_per_second": [0.0, 0.0, 0.5],
                }}],
            }],
        }

    def test_identical_states_have_zero_error(self) -> None:
        metrics = compare_reference(self.scenario, self.result)

        self.assertEqual(metrics["sample_count"], 1)
        self.assertEqual(metrics["position_rmse_meters"], 0.0)
        self.assertEqual(metrics["velocity_rmse_meters_per_second"], 0.0)
        self.assertEqual(metrics["attitude_rmse_degrees"], 0.0)
        self.assertEqual(metrics["angular_velocity_rmse_radians_per_second"], 0.0)

    def test_reports_position_speed_attitude_and_rate_errors(self) -> None:
        state = self.result["trajectory"][0]["drones"][0]["state"]
        self.scenario["reference_samples"][0]["position_delta_enu"] = [1.0, 0.0, 0.0]
        state["velocity_meters_per_second"] = [2.0, 0.0, 0.0]
        state["orientation_quaternion_wxyz"] = [2 ** -0.5, 0.0, 0.0, 2 ** -0.5]
        state["angular_velocity_radians_per_second"] = [0.0, 0.0, 1.5]

        metrics = compare_reference(self.scenario, self.result)

        self.assertEqual(metrics["position_rmse_meters"], 1.0)
        self.assertEqual(metrics["velocity_rmse_meters_per_second"], 1.0)
        self.assertAlmostEqual(metrics["attitude_rmse_degrees"], 90.0)
        self.assertEqual(metrics["angular_velocity_rmse_radians_per_second"], 1.0)

    def test_quaternion_sign_does_not_change_attitude_error(self) -> None:
        state = self.result["trajectory"][0]["drones"][0]["state"]
        state["orientation_quaternion_wxyz"] = [-1.0, 0.0, 0.0, 0.0]

        metrics = compare_reference(self.scenario, self.result)

        self.assertEqual(metrics["attitude_rmse_degrees"], 0.0)

    def test_different_sample_times_are_rejected(self) -> None:
        self.result["trajectory"][0]["time_seconds"] = 0.1

        with self.assertRaisesRegex(ValueError, "시각이 서로 다릅니다"):
            compare_reference(self.scenario, self.result)


if __name__ == "__main__":
    unittest.main()
