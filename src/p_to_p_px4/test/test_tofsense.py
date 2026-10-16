import unittest
from p_to_p_px4.tofsense import (TofHeightKalman, TofSenseMDecoder, TofReceiptClock,
                                 decode_tof_ipc, distance_sensor_v1,
                                 tof_ipc_packet)
from p_to_p_px4.range_preview import crc_x25


def frame(distance=.3, status=0):
    data = bytearray(400)
    data[:4] = b'\x57\x01\xff\x00'
    data[4:8] = (1000).to_bytes(4, 'little')
    data[8] = 64
    for index in range(64):
        offset = 9 + 6*index
        data[offset:offset+3] = int(distance*1e6).to_bytes(3, 'little', signed=True)
        data[offset+3] = status
    data[-1] = sum(data[:-1]) & 255
    return data


class TestTofSense(unittest.TestCase):
    def test_height_kalman_reduces_static_noise(self):
        estimator = TofHeightKalman()
        raw = []
        filtered = []
        for index in range(90):
            height = .6 + (.02 if index % 2 else -.02)
            raw.append(height)
            filtered.append(estimator.update(index / 15, height))
        self.assertLess(max(filtered[-30:])-min(filtered[-30:]),
                        (max(raw)-min(raw))/2)
        self.assertAlmostEqual(sum(filtered[-30:])/30, .6, delta=.01)

    def test_height_kalman_tracks_ramp_and_rejects_isolated_spike(self):
        estimator = TofHeightKalman()
        for index in range(30):
            height = .4 + .2*index/15
            output = estimator.update(index/15, height)
        self.assertAlmostEqual(output, height, delta=.03)
        self.assertIsNone(estimator.update(2., 2.0))
        output = estimator.update(31/15, .4+.2*31/15)
        self.assertIsNotNone(output)
        self.assertAlmostEqual(output, .4+.2*31/15, delta=.03)

    def test_height_kalman_reacquires_real_step_and_gap(self):
        estimator = TofHeightKalman()
        for index in range(30):
            estimator.update(index/15, .4)
        self.assertIsNone(estimator.update(2., 1.2))
        self.assertIsNone(estimator.update(31/15, 1.2))
        self.assertAlmostEqual(estimator.update(32/15, 1.2), 1.2)
        self.assertAlmostEqual(estimator.update(3., .7), .7)
        self.assertIsNone(estimator.update(3., .8))
        self.assertIsNone(estimator.update(3.1, float('nan')))
        self.assertAlmostEqual(estimator.update(3.2, .8), .8)

    def test_ipc_round_trip(self):
        receipt, ticks, distance = decode_tof_ipc(tof_ipc_packet(1234567890, 42, .215))
        self.assertAlmostEqual(receipt, 1.234567890)
        self.assertEqual(ticks, 42)
        self.assertAlmostEqual(distance, .215, places=6)
    def test_distance_sensor_packet(self):
        packet = distance_sensor_v1(.213, 9, 1234)
        self.assertEqual(packet[0], 254)
        self.assertEqual(packet[5], 132)
        self.assertEqual(int.from_bytes(packet[14:16], 'little'), 21)
        self.assertEqual(packet[17], 77)
        self.assertEqual(packet[18], 25)
        self.assertEqual(packet[19], 3)
        self.assertEqual(crc_x25(packet[1:-2], 85), int.from_bytes(packet[-2:], 'little'))

    def test_distance_sensor_rejects_invalid(self):
        with self.assertRaises(ValueError):
            distance_sensor_v1(0., 0, 0)

    def test_decoder_rejects_sub_mavlink_minimum_and_recovers(self):
        decoder = TofSenseMDecoder()
        for distance in (.015, .019, .019999):
            self.assertEqual(decoder.feed(frame(distance=distance)), [])
        self.assertEqual(decoder.range_rejected, 3)
        rows = decoder.feed(frame(distance=.02) + frame(distance=.072))
        self.assertEqual(rows, [(1., .02), (1., .072)])
        for ticks, distance in rows:
            self.assertTrue(distance_sensor_v1(distance, 0, 0))
    def test_clock_warmup_and_reboot(self):
        clock = TofReceiptClock()
        for i in range(14):
            self.assertIsNone(clock.stamp(i*.067, 100+i*.067, i*.067))
        self.assertAlmostEqual(clock.stamp(.938,100.938,.938),100.938)
        self.assertIsNone(clock.stamp(0.,101.,1.))
        self.assertTrue(clock.reset)

    def test_clock_ros_jump(self):
        clock = TofReceiptClock()
        for i in range(15):
            clock.stamp(i*.067, 100+i*.067, i*.067)
        self.assertIsNone(clock.stamp(1.005,200.,1.005))
        self.assertTrue(clock.reset)

    def test_sensor_ticks_not_real_time(self):
        clock = TofReceiptClock()
        for i in range(20):
            stamp = clock.stamp(i*.027,100+i*.067,i*.067)
        self.assertAlmostEqual(stamp,100+19*.067)

    def test_fragmentation_and_units(self):
        decoder = TofSenseMDecoder()
        data = frame()
        self.assertEqual(decoder.feed(b'noise'+data[:100]), [])
        self.assertEqual(decoder.feed(data[100:]), [(1., .3)])
        self.assertEqual((decoder.frames_seen, decoder.frames_valid), (1, 1))

    def test_crc_recovery(self):
        decoder = TofSenseMDecoder()
        bad = frame(); bad[10] ^= 1
        self.assertEqual(decoder.feed(bad+frame()), [(1., .3)])
        self.assertEqual(decoder.crc_errors, 1)
        self.assertEqual((decoder.frames_seen, decoder.frames_valid), (2, 1))

    def test_invalid_status(self):
        decoder = TofSenseMDecoder()
        self.assertEqual(decoder.feed(frame(status=255)), [])
        self.assertEqual(decoder.range_rejected, 1)

    def test_out_of_range(self):
        decoder = TofSenseMDecoder()
        self.assertEqual(decoder.feed(frame(distance=5.)), [])
        self.assertEqual(decoder.range_rejected, 1)

    def test_wrong_mode(self):
        data = frame()
        data[8] = 63
        data[-1] = sum(data[:-1]) & 255
        decoder = TofSenseMDecoder()
        self.assertEqual(decoder.feed(data), [])
        self.assertEqual(decoder.mode_rejected, 1)
