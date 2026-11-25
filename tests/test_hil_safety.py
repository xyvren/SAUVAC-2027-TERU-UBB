import unittest

from simulasi_auv.hil_safety import validate_hil_pulses, validate_hil_sensors


class HilSafetyTests(unittest.TestCase):
    def test_accepts_neutral_and_boundary_pulses(self):
        self.assertEqual(
            validate_hil_pulses([1425, 1500, 1575, 1500, 1500, 1500]),
            [1425, 1500, 1575, 1500, 1500, 1500],
        )

    def test_rejects_wrong_length_and_out_of_limit_values(self):
        for pulses in ([1500] * 5, [1500] * 7, [1424, 1500, 1500, 1500, 1500, 1500],
                       [1576, 1500, 1500, 1500, 1500, 1500]):
            with self.subTest(pulses=pulses):
                with self.assertRaises(ValueError):
                    validate_hil_pulses(pulses)

    def test_rejects_non_integer_or_non_numeric_values(self):
        for value in (True, "1500", 1500.5, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    validate_hil_pulses([value, 1500, 1500, 1500, 1500, 1500])

    def test_validates_virtual_sensor_bounds_and_shape(self):
        sensors = {
            "depth_cm": 50,
            "pitch": 10,
            "roll": -10,
            "gyro_dps": [0, 0, 100],
        }
        self.assertEqual(validate_hil_sensors(sensors)["depth_cm"], 50.0)
        for invalid in (
            {**sensors, "depth_cm": 141},
            {**sensors, "pitch": 91},
            {**sensors, "gyro_dps": [0, 0]},
            {**sensors, "gyro_dps": [0, 0, float("nan")]},
        ):
            with self.subTest(sensors=invalid):
                with self.assertRaises(ValueError):
                    validate_hil_sensors(invalid)


if __name__ == "__main__":
    unittest.main()
