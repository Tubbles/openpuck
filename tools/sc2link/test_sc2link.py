#!/usr/bin/env python3
"""Host checks of sc2link_test.py, no dongle needed.

Run: python3 tools/sc2link/test_sc2link.py
"""

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sc2link_test as sc2  # noqa: E402


def run(model, words):
    return [model.word(word) for word in words]


def session_model():
    model = sc2.Model()
    run(model, sc2.HELLO)
    return model


class StructLayouts(unittest.TestCase):
    def test_sizes_on_this_host(self):
        if struct.calcsize("@P") != 8:
            self.skipTest("the expected sizes are those of 64-bit Linux")
        self.assertEqual(struct.calcsize("@llHHi"), 24)
        self.assertEqual(sc2.INPUT_EVENT.size, 24)
        self.assertEqual(sc2.FF_EFFECT.size, 48)
        self.assertEqual(sc2.FF_RUMBLE_OFFSET, 16)
        self.assertEqual(sc2.INPUT_ABSINFO.size, 24)
        self.assertEqual(sc2.INPUT_ID.size, 8)

    def test_ioctl_numbers(self):
        if struct.calcsize("@P") != 8:
            self.skipTest("EVIOCSFF carries the 64-bit struct size")
        self.assertEqual(sc2.EVIOCSFF, 0x40304580)
        self.assertEqual(sc2.EVIOCGID, 0x80084502)
        self.assertEqual(sc2.eviocgabs(sc2.ABS_BRAKE), 0x8018454A)
        self.assertEqual(sc2.eviocgname(256), 0x81004506)
        self.assertEqual(sc2.EVIOCSCLOCKID, 0x400445A0)


class Encoder(unittest.TestCase):
    def test_protocol_example(self):
        # docs/PROTOCOL.md: COMMAND 0x3AF6 becomes 82 03 02 F6.
        self.assertEqual(sc2.encode_command(3, 2, -10), [0x3AF6])
        self.assertEqual(sc2.report_command(3, 2, -10).hex(), "820302f6")

    def test_word_counts(self):
        counts = {"command": 1, "script": 2, "rumble": 3, "pulse": 4,
                  "sweep": 4, "tone": 5}
        for name, message in sc2.MESSAGES.items():
            for values in sc2.boundary_cases(message):
                words = message.encode(*values)
                self.assertEqual(len(words), counts[name])
                self.assertTrue(all(0 <= word <= 0xFFFF for word in words))
                self.assertEqual(sc2.payload_words(words[0]), len(words) - 1)

    def test_control_words(self):
        self.assertEqual(sc2.encode_bootloader(), [0x0500, 0x5546])
        self.assertEqual(sc2.encode_mode(13), [0x060D, 0x4D0D])
        self.assertEqual(sc2.encode_rumble_stop(), [0x6001])
        for word in (sc2.NOP, sc2.BYE, sc2.CHECKSUM):
            self.assertEqual(sc2.payload_words(word), 0)

    def test_report_lengths(self):
        # SDL's *_REPORT_BYTES, report id included.
        lengths = {"command": 4, "script": 4, "rumble": 10, "pulse": 8,
                   "sweep": 9, "tone": 10}
        for name, message in sc2.MESSAGES.items():
            for values in sc2.boundary_cases(message):
                self.assertEqual(len(message.report(*values)), lengths[name])

    def test_reports_at_full_range(self):
        self.assertEqual(sc2.report_rumble(0xFFFF, 0x1234).hex(),
                         "8004ffffffff00341200")
        self.assertEqual(sc2.report_rumble(0, 0).hex(), "80000000000000000000")
        self.assertEqual(sc2.report_pulse(3, 0xFFFF, 1, 0x8000).hex(),
                         "8103ffff01000080")
        self.assertEqual(sc2.report_sweep(1, -128, 0xFFFF, 2, 3).hex(),
                         "840180ffff02000300")
        self.assertEqual(sc2.report_tone(2, 127, 0xFFFF, 4, 5, 0xFF).hex(),
                         "83027fffff04000500ff")
        self.assertEqual(sc2.report_script(1, 0xFF, -1).hex(), "8501ffff")

    def test_boundary_cases(self):
        tone = sc2.MESSAGES["tone"]
        cases = sc2.boundary_cases(tone)
        # The all-minimum case, then a middle and a maximum per field.
        self.assertEqual(len(cases), 1 + 2 * len(tone.fields))
        for index, field in enumerate(tone.fields):
            values = {case[index] for case in cases}
            self.assertEqual(values,
                             {field.minimum, field.middle, field.maximum})

    def test_parse_fields(self):
        pulse = sc2.MESSAGES["pulse"]
        self.assertEqual(sc2.parse_fields(pulse, ["3", "0x10", "0", "65535"]),
                         (3, 16, 0, 65535))
        with self.assertRaises(ValueError):
            sc2.parse_fields(pulse, ["4", "0", "0", "0"])
        with self.assertRaises(ValueError):
            sc2.parse_fields(pulse, ["1", "0", "0"])


class Decoder(unittest.TestCase):
    """The model must rebuild every report the encoder's words describe."""

    def test_round_trip_every_boundary(self):
        for message in sc2.MESSAGES.values():
            for values in sc2.boundary_cases(message):
                words = message.encode(*values)
                payload = words[1:]
                for index, word in enumerate(payload):
                    self.assertTrue(sc2.payload_valid(words[0], index, word))
                self.assertEqual(sc2.build_report(words[0], payload),
                                 message.report(*values),
                                 sc2.describe_case(message, values))

    def test_checksum_of_queued_report(self):
        model = session_model()
        words = sc2.encode_tone(3, -20, 160, 500, 4, 200)
        run(model, words)
        outcome = model.word(sc2.CHECKSUM)
        expected = sc2.crc16(sc2.report_tone(3, -20, 160, 500, 4, 200))
        self.assertEqual(outcome.echo, expected)

    def test_checksum_without_controller(self):
        model = sc2.Model(linked=False)
        run(model, sc2.HELLO)
        run(model, sc2.encode_rumble(1, 2))
        self.assertEqual(model.word(sc2.CHECKSUM).echo, 0xFFFF)

    def test_crc_check_value(self):
        self.assertEqual(sc2.crc16(b"123456789"), 0x29B1)
        self.assertEqual(sc2.crc16(b""), 0xFFFF)


class State(unittest.TestCase):
    def test_ack_state_fields(self):
        state = sc2.ack_state(0x1FF, True, 4)
        self.assertEqual(state, 0xFFC1)
        self.assertEqual(sc2.decode_state(state), {
            "counter": 0xFF, "session": 1, "expected": 4, "version": 1})

    def test_plain_rumble_outside_a_session(self):
        model = sc2.Model()
        for word in (0x1010, 0x0000, 0xFFFF, sc2.HELLO[1]):
            self.assertFalse(model.word(word).accepted)
        self.assertEqual(model.counter, 0)

    def test_hello(self):
        model = sc2.Model(counter=0xFE)
        outcomes = run(model, sc2.HELLO)
        self.assertTrue(all(outcome.accepted for outcome in outcomes))
        self.assertEqual([sc2.decode_state(outcome.state)["expected"]
                          for outcome in outcomes], [3, 2, 1, 0])
        self.assertEqual(sc2.decode_state(outcomes[-1].state)["session"], 1)
        # The counter wraps.
        self.assertEqual(sc2.decode_state(outcomes[-1].state)["counter"], 2)

    def test_broken_hello_starts_over(self):
        model = sc2.Model()
        run(model, sc2.HELLO[:2])
        self.assertFalse(model.word(0x1234).accepted)
        # A first HELLO word right after a mismatch begins a new HELLO.
        run(model, sc2.HELLO[:2])
        self.assertTrue(model.word(sc2.HELLO[0]).accepted)
        self.assertEqual(model.hello, 1)

    def test_expected_counts_down(self):
        model = session_model()
        outcomes = run(model, sc2.encode_tone(1, 0, 1, 2, 3, 4))
        self.assertEqual([sc2.decode_state(outcome.state)["expected"]
                          for outcome in outcomes], [4, 3, 2, 1, 0])
        self.assertEqual([outcome.echo for outcome in outcomes],
                         sc2.encode_tone(1, 0, 1, 2, 3, 4))

    def test_bye(self):
        model = session_model()
        outcome = model.word(sc2.BYE)
        self.assertEqual(sc2.decode_state(outcome.state)["session"], 0)
        self.assertFalse(model.word(0x1010).accepted)


class Framing(unittest.TestCase):
    def assert_rejected(self, words):
        model = session_model()
        outcomes = run(model, words)
        last = outcomes[-1]
        self.assertTrue(last.accepted)
        self.assertEqual(last.echo, words[-1])
        self.assertEqual(sc2.decode_state(last.state)["session"], 0)
        return model

    def test_rejected_headers(self):
        for word in (0xE000, 0x8001, 0x1F00, 0x0001, 0x0300, 0x0501,
                     0x060E, 0x6002, 0x4100):
            with self.subTest(word=hex(word)):
                self.assert_rejected([word])

    def test_rejected_payloads(self):
        self.assert_rejected([sc2.header(sc2.OP_SCRIPT), 0x0100])
        self.assert_rejected(sc2.encode_tone(0, 0, 1, 2, 3, 4)[:4] + [0x100])
        self.assert_rejected([0x0500, 0x5547])
        self.assert_rejected([0x0603, 0x4D04])

    def test_hello_after_a_lost_count(self):
        # A HELLO sent into an open message completes it, then its third
        # word is an undefined CONTROL sub that ends the session, and the
        # fourth is plain rumble. The client sees session 0 and starts over.
        model = session_model()
        run(model, sc2.encode_pulse(1, 2, 3, 4)[:2])
        outcomes = run(model, sc2.HELLO)
        self.assertEqual([outcome.accepted for outcome in outcomes],
                         [True, True, True, False])
        self.assertEqual(sc2.decode_state(outcomes[2].state)["session"], 0)
        outcomes = run(model, sc2.HELLO)
        self.assertEqual(sc2.decode_state(outcomes[-1].state)["session"], 1)

    def test_single_word_message_inside_a_long_one_is_payload(self):
        model = session_model()
        click = sc2.encode_command(3, 2, -10)[0]
        run(model, sc2.encode_pulse(3, click, 0, 1))
        self.assertEqual(model.queued, sc2.report_pulse(3, click, 0, 1))

    def test_nop_only_at_a_header(self):
        model = session_model()
        run(model, sc2.encode_pulse(1, 0, 0, 0))
        self.assertEqual(model.queued, sc2.report_pulse(1, 0, 0, 0))
        self.assertEqual(model.word(sc2.NOP).state >> 4 & 7, 0)

    def test_reboot_commands(self):
        model = session_model()
        run(model, sc2.encode_bootloader())
        self.assertEqual(model.reboot, "bootloader")
        model = session_model()
        run(model, sc2.encode_mode(1))
        self.assertEqual(model.reboot, 1)


class CommandLine(unittest.TestCase):
    def parse(self, *argv):
        return sc2.build_parser().parse_args(list(argv))

    def test_subcommands(self):
        self.assertTrue(self.parse("info", "--watch").watch)
        self.assertEqual(self.parse("mode", "13").number, 13)
        args = self.parse("--phys", "usb-x", "send", "tone", "3", "-6",
                          "160", "500", "0", "0", "--hold", "1")
        self.assertEqual((args.phys, args.message, args.hold),
                         ("usb-x", "tone", 1.0))
        self.assertEqual(args.fields, ["3", "-6", "160", "500", "0", "0"])
        for command in ("hello", "bye", "selftest", "bootloader", "knock"):
            self.assertEqual(self.parse(command).command, command)

    def test_errors(self):
        with self.assertRaises(SystemExit):
            self.parse("send", "noise")
        with self.assertRaises(SystemExit):
            self.parse()


if __name__ == "__main__":
    unittest.main()
