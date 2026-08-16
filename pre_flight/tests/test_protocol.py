import unittest

from lib.protocol import packet_sun, packet_vector, parse_packet


class ProtocolTests(unittest.TestCase):
    def test_ignores_unrelated_logs(self):
        self.assertIsNone(parse_packet("BMI270 startup chatter"))

    def test_parses_magnetometer(self):
        packet = parse_packet("PF,MAG,123,1,2,3,-0.1,-0.2,0.3")
        self.assertIsNotNone(packet)
        self.assertEqual(packet_vector(packet, "raw"), (1.0, 2.0, 3.0))
        self.assertEqual(packet_vector(packet, "body"), (-0.1, -0.2, 0.3))

    def test_parses_sun(self):
        packet = parse_packet("PF,SUN,99," + ",".join(str(value) for value in range(16)))
        self.assertIsNotNone(packet)
        self.assertEqual(packet_sun(packet), tuple(range(16)))


if __name__ == "__main__":
    unittest.main()
