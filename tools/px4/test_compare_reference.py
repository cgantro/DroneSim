from __future__ import annotations

import unittest

from compare_reference import compare_reference


class CompareReferenceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = {
            "dynamics": "rigid_body",
            "drones": [{"name": "x500"}],
            "reference_metadata": {
                "source": "시험 자료",
                "window_start_seconds_since_boot": 12.0,
                "sample_period_seconds": 1.0 / 60.0,
                "maximum_observed_sample_gap_microseconds": {
                    "vehicle_local_position": 100_000,
                },
                "maximum_allowed_sample_gap_microseconds": {
                    "vehicle_local_position": 250_000,
                },
                "sample_gap_override_microseconds": None,
            },
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
            "physical_model": {
                "mass_kg": 2.0,
                "principal_inertia_kg_m2": [0.02, 0.02, 0.04],
                "thrust_coefficient_newtons_per_speed_squared": 0.00001,
                "moment_to_thrust_ratio": 0.016,
                "motor_time_constant_up_seconds": 0.0125,
                "motor_time_constant_down_seconds": 0.025,
                "maximum_motor_speed_radians_per_second": 1000.0,
                "customized": False,
            },
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
        self.assertEqual(metrics["reference_source"], "시험 자료")
        self.assertEqual(metrics["window_start_seconds_since_boot"], 12.0)
        self.assertEqual(metrics["window_end_seconds_since_boot"], 12.0)
        self.assertEqual(metrics["physical_model"]["mass_kg"], 2.0)
        self.assertEqual(
            metrics["maximum_observed_sample_gap_microseconds"][
                "vehicle_local_position"
            ],
            100_000,
        )
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

    def test_compares_esc_motor_speed_telemetry_when_present(self) -> None:
        self.scenario["reference_samples"][0]["motor_speeds_radians_per_second"] = [
            100.0, 200.0, 300.0, 400.0]
        self.result["trajectory"][0]["drones"][0]["state"][
            "motor_speeds_radians_per_second"] = [100.0, 210.0, 300.0, 390.0]

        metrics = compare_reference(self.scenario, self.result)

        self.assertAlmostEqual(
            metrics["motor_speed_rmse_radians_per_second"], 50.0 ** 0.5)

    def test_different_sample_times_are_rejected(self) -> None:
        self.result["trajectory"][0]["time_seconds"] = 0.1

        with self.assertRaisesRegex(ValueError, "시각이 서로 다릅니다"):
            compare_reference(self.scenario, self.result)

    def test_rejects_kinematic_scenario(self) -> None:
        self.scenario["dynamics"] = "kinematic"

        with self.assertRaisesRegex(ValueError, "rigid_body"):
            compare_reference(self.scenario, self.result)

    def test_rejects_result_without_physical_model(self) -> None:
        self.result["physical_model"] = None

        with self.assertRaisesRegex(ValueError, "강체 물리 모델"):
            compare_reference(self.scenario, self.result)


if __name__ == "__main__":
    unittest.main()
