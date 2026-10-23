"""ULog 변환기가 큰 표본 공백을 놓치지 않는지 확인한다."""

from __future__ import annotations

import unittest
from types import SimpleNamespace

from prepare_reference import TopicSamples


class TopicSamplesTests(unittest.TestCase):
    def test_interpolates_normal_samples_and_rejects_a_large_gap(self) -> None:
        samples = TopicSamples(
            SimpleNamespace(data={"timestamp": [0, 5_000, 10_000], "value": [0.0, 5.0, 10.0]}),
            topic_name="vehicle_attitude",
            maximum_gap_microseconds=20_000,
        )

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
            sparse_samples.scalar("value", 700_000)


if __name__ == "__main__":
    unittest.main()
