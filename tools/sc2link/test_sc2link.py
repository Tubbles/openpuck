#!/usr/bin/env python3
"""Host checks of sc2link_test.py, no dongle needed.

Run: python3 tools/sc2link/test_sc2link.py
"""

import contextlib
import io
import math
import os
import re
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sc2link_test as sc2  # noqa: E402

FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "..", "OpenPuck")


def firmware_source(name):
    with open(os.path.join(FIRMWARE, name), encoding="utf-8") as file:
        return file.read()


def firmware_defines(source):
    """The #defines of plain integers, unsigned ones included."""
    return {name: int(value, 0) for name, value in
            re.findall(r"^#define (\w+) (0x[0-9A-Fa-f]+|\d+)u?$", source,
                       re.M)}


def run(model, words):
    return [model.word(word) for word in words]


def send(model, data_words):
    """Session words with the toggles a client gives them."""
    words, _ = sc2.stamp(data_words, model.toggle)
    return run(model, words)


def session_model(linked=True):
    model = sc2.Model(linked=linked)
    run(model, sc2.HELLO)
    return model


# The stop reports by report id and side: "81 02", "82 02", "81 05", "82 05".
STOP = {report[:2].hex(" "): report for _, report in sc2.STOP_ENTRIES}


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
        # docs/PROTOCOL.md: COMMAND data 0x52F6 becomes 82 02 02 F6, a click
        # on both pads.
        self.assertEqual(sc2.encode_command(2, 2, -10), [0x52F6])
        self.assertEqual(sc2.report_command(2, 2, -10).hex(), "820202f6")

    def test_layouts(self):
        # The bit layouts of the message table in docs/PROTOCOL.md.
        self.assertEqual(sc2.encode_command(5, 7, -1), [0x7DFF])
        self.assertEqual(sc2.encode_script(1, 0xFF, -1), [0x09FF, 0x00FF])
        self.assertEqual(sc2.encode_script(5, 0, -128), [0x0D80, 0x0000])
        self.assertEqual(sc2.encode_rumble(0xFFFF, 0x1234),
                         [0x1002, 0x7FFF, 0x1234])
        self.assertEqual(sc2.encode_pulse(3, 0xFFFF, 1, 0x8000),
                         [0x1B05, 0x7FFF, 0x0001, 0x0000])
        self.assertEqual(sc2.encode_pulse(5, 0, 0, 0),
                         [0x1D00, 0x0000, 0x0000, 0x0000])
        self.assertEqual(sc2.encode_sweep(1, -128, 0xFFFF, 2, 0x8003),
                         [0x2180, 0x7FFF, 0x0002, 0x0003, 0x0005])
        self.assertEqual(sc2.encode_sweep(4, 0, 0, 0, 0),
                         [0x2400, 0x0000, 0x0000, 0x0000, 0x0000])
        self.assertEqual(sc2.encode_tone(2, 127, 0xFFFF, 4, 0x8005, 0xFF),
                         [0x2A7F, 0x7FFF, 0x0004, 0x0005, 0x05FF])
        self.assertEqual(sc2.encode_tone(5, 0, 0, 0, 0, 0),
                         [0x2D00, 0x0000, 0x0000, 0x0000, 0x0000])

    def test_every_side(self):
        # Sides 0 to 5 reach the report of every message that has one, with
        # every other field at its minimum and at its maximum.
        for name in ("command", "script", "pulse", "sweep", "tone"):
            message = sc2.MESSAGES[name]
            others = message.fields[1:]
            for side in range(sc2.SIDE_MAX + 1):
                for rest in (tuple(field.minimum for field in others),
                             tuple(field.maximum for field in others)):
                    values = (side,) + rest
                    with self.subTest(case=sc2.describe_case(message, values)):
                        report = message.report(*values)
                        self.assertEqual(report[1], side)
                        model = session_model()
                        send(model, message.encode(*values))
                        self.assertTrue(model.active)
                        self.assertEqual(model.queued, report)
                        self.assertEqual(model.relayed[-1], report)

    def test_word_counts(self):
        counts = {"command": 1, "script": 2, "rumble": 3, "pulse": 4,
                  "sweep": 5, "tone": 5}
        for name, message in sc2.MESSAGES.items():
            for values in sc2.boundary_cases(message):
                words = message.encode(*values)
                self.assertEqual(len(words), counts[name])
                self.assertTrue(all(0 <= word <= sc2.DATA for word in words))
                self.assertEqual(sc2.payload_words(words[0]), len(words) - 1)

    def test_control_words(self):
        self.assertEqual(sc2.encode_bootloader(), [0x0500, 0x5546])
        self.assertEqual(sc2.encode_mode(13), [0x060D, 0x4D0D])
        self.assertEqual(sc2.encode_rumble_stop(), [0x1004])
        for word in (sc2.NOP, sc2.BYE, sc2.CHECKSUM):
            self.assertEqual(sc2.payload_words(word), 0)

    def test_stamp(self):
        self.assertEqual(sc2.stamp([0x0400, 0x0400, 0x0200], 0),
                         ([0x8400, 0x0400, 0x8200], 1))

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
            sc2.parse_fields(pulse, ["6", "0", "0", "0"])
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
        outcomes = send(model, sc2.encode_tone(3, -20, 160, 500, 4, 200)
                        + [sc2.CHECKSUM])
        expected = sc2.crc16(sc2.report_tone(3, -20, 160, 500, 4, 200))
        self.assertEqual(outcomes[-1].echo, expected)

    def test_checksum_without_controller(self):
        model = session_model(linked=False)
        outcomes = send(model, sc2.encode_rumble(1, 2) + [sc2.CHECKSUM])
        self.assertEqual(outcomes[-1].echo, 0xFFFF)

    def test_session_end_checksum(self):
        # A grip train, BYE, a new session: CHECKSUM covers the last report
        # of the end's stop, the grip OFF. The start leaves it alone.
        for linked, expected in ((True, sc2.crc16(STOP["82 05"])),
                                 (False, 0xFFFF)):
            model = session_model(linked)
            send(model, sc2.encode_pulse(3, 20000, 20000, 1000) + [sc2.BYE])
            run(model, sc2.HELLO)
            self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, expected)
            # A tick keeps nothing running, so a rejected word's stop
            # queues nothing: the CRC of no report.
            send(model, sc2.encode_command(1, 1, 0) + [0x3800])
            run(model, sc2.HELLO)
            self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, 0xFFFF)

    def test_crc_check_value(self):
        self.assertEqual(sc2.crc16(b"123456789"), 0x29B1)
        self.assertEqual(sc2.crc16(b""), 0xFFFF)


RUMBLE_OFF = sc2.report_rumble(0, 0)


def rounds(names):
    """The stop reports named, in turn STOP_REPEATS times."""
    return [STOP[name] for name in names] * sc2.STOP_REPEATS


def stop(names, rumble=False):
    """What a stop queues: the 0x80 zero, three times while rumble was on,
    then the reports named; nothing with neither."""
    if not names and not rumble:
        return []
    return [RUMBLE_OFF] * (sc2.STOP_REPEATS if rumble else 1) + rounds(names)


class Stop(unittest.TestCase):
    """What a session's start and end queue for the controller."""

    # A message of each class that plays 400 ms: 40 ms periods, 400 ms tones.
    PLAYS = {
        sc2.PAD_PULSE: sc2.encode_pulse(1, 2000, 38000, 10),
        sc2.GRIP_PULSE: sc2.encode_pulse(4, 2000, 38000, 10),
        sc2.PAD_TONE: sc2.encode_tone(0, -10, 160, 400, 0, 0),
        sc2.GRIP_TONE: sc2.encode_sweep(5, -10, 400, 60, 200),
    }
    # The stop reports each class sends.
    SENDS = {
        sc2.PAD_PULSE: ["81 02", "82 02"],
        sc2.GRIP_PULSE: ["81 05", "82 05"],
        sc2.PAD_TONE: ["81 02", "82 02"],
        sc2.GRIP_TONE: ["81 05", "82 05"],
    }
    ENDS = {"BYE": lambda model: send(model, [sc2.BYE]),
            "a rejected word": lambda model: send(model, [0x3800]),
            "the timeout": lambda model: model.end_session()}

    def play(self, data_words, clock=0.0, model=None):
        """A session that sent the message at `clock` seconds, its relays
        cleared."""
        if model is None:
            model = session_model()
        model.clock = clock
        send(model, data_words)
        model.relayed.clear()
        return model

    def assert_edge(self, model, clock, names, rumble=False):
        """BYE at `clock` seconds queues the stop of the reports named, the
        next session's start repeats it, rumble aside, and the end after
        that queues nothing. CHECKSUM shows each end's last report."""
        expected = stop(names, rumble)
        model.clock = clock
        send(model, [sc2.BYE])
        self.assertEqual(model.relayed, expected)
        model.relayed.clear()
        run(model, sc2.HELLO)
        self.assertEqual(model.relayed, stop(names))
        model.relayed.clear()
        checksum = send(model, [sc2.CHECKSUM])[0].echo
        self.assertEqual(checksum,
                         sc2.crc16(expected[-1] if expected else b""))
        send(model, [sc2.BYE])
        run(model, sc2.HELLO)
        self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, 0xFFFF)
        self.assertEqual(model.relayed, [])

    def lose_the_end(self, data_words):
        """A session that plays at 0 s and whose end at 0.1 s finds no link,
        so its stop is lost, then a session started at 0.2 s."""
        model = self.play(data_words)
        model.linked = False
        model.clock = 0.1
        send(model, [sc2.BYE])
        model.linked = True
        model.clock = 0.2
        run(model, sc2.HELLO)
        return model

    def test_entries(self):
        self.assertEqual([(classes, report.hex(" "))
                          for classes, report in sc2.STOP_ENTRIES], [
            (sc2.PAD_PULSE | sc2.PAD_TONE, "81 02 01 00 01 00 01 00"),
            (sc2.PAD_PULSE | sc2.PAD_TONE, "82 02 00 00"),
            (sc2.GRIP_PULSE | sc2.GRIP_TONE, "81 05 01 00 01 00 01 00"),
            (sc2.GRIP_PULSE | sc2.GRIP_TONE, "82 05 00 00")])
        self.assertEqual(sc2.STOP_REPEATS, 3)

    def test_firmware_queues_the_same(self):
        source = firmware_source("mode_sc2link.cpp")
        defines = firmware_defines(source)
        table = re.search(r"SC2_STOP\[\] = \{(.*?)\n\};", source, re.S)
        entries = []
        for classes, length, body in re.findall(
                r"\{\s*([\w\s|]+?),\s*(\d+),\s*\{([^}]*)\}\s*\}",
                table.group(1)):
            mask = 0
            for name in classes.split("|"):
                mask |= defines[name.strip()]
            items = [item.strip() for item in body.split(",")]
            values = [defines[item] if item in defines else int(item, 0)
                      for item in items]
            self.assertEqual(len(values), int(length))
            entries.append((mask, bytes(values)))
        self.assertEqual(entries, list(sc2.STOP_ENTRIES))
        for name, value in (("SC2_PAD_PULSE", sc2.PAD_PULSE),
                            ("SC2_GRIP_PULSE", sc2.GRIP_PULSE),
                            ("SC2_PAD_TONE", sc2.PAD_TONE),
                            ("SC2_GRIP_TONE", sc2.GRIP_TONE),
                            ("SC2_CLASSES", 4),
                            ("SC2_RADIO_MARGIN_MS", sc2.RADIO_MARGIN_MS),
                            ("SC2_CMD_CLICK", sc2.COMMAND_CLICK),
                            ("SC2_SIDE_BOTH_PADS", 2),
                            ("SC2_PROTOCOL_VERSION", sc2.PROTOCOL_VERSION),
                            ("SC2_SIDE_MAX", sc2.SIDE_MAX)):
            self.assertEqual(defines[name], value, name)
        haptics = firmware_defines(firmware_source("haptics.h"))
        self.assertEqual(haptics["RUMBLE_STOP_REPS"], sc2.STOP_REPEATS)

    def test_relay_budget_holds_every_class_running(self):
        # The firmware reserves room for a rumble stop and the whole list,
        # each STOP_REPEATS times, and the ring holds that with a report.
        source = firmware_source("mode_sc2link.cpp")
        self.assertIn("#define SC2_STOP_ENTRIES ((1 + SC2_STOP_REPORTS) * "
                      "RUMBLE_STOP_REPS)", source)
        budget = (1 + len(sc2.STOP_ENTRIES)) * sc2.STOP_REPEATS
        ring = firmware_defines(firmware_source("haptics.cpp"))["RELAY_QLEN"]
        self.assertLessEqual(budget + 1, ring - 1)
        model = self.play(sc2.encode_rumble(0x1234, 0))
        for data_words in self.PLAYS.values():
            self.play(data_words, model=model)
        send(model, [sc2.BYE])
        self.assertEqual(len(model.relayed), budget)
        # The repeat at the next edge, without rumble, takes fewer.
        model.relayed.clear()
        run(model, sc2.HELLO)
        self.assertEqual(len(model.relayed), budget - 2)

    def test_the_zero_comes_with_every_stop_that_sends(self):
        # Once with rumble off, ahead of the reports, at the stop and at its
        # repeat; never in a stop that sends nothing.
        for bit, data_words in self.PLAYS.items():
            with self.subTest(bit=bit):
                model = self.play(data_words)
                model.clock = 0.1
                send(model, [sc2.BYE])
                run(model, sc2.HELLO)
                send(model, [sc2.BYE])
                stops = [model.relayed[:7], model.relayed[7:]]
                self.assertEqual(stops, [stop(self.SENDS[bit])] * 2)
                self.assertEqual(model.relayed.count(RUMBLE_OFF), 2)
        model = session_model()
        send(model, [sc2.BYE])
        self.assertEqual(model.relayed, [])

    def test_grip_tones_send_both_grip_reports(self):
        for data_words in (sc2.encode_tone(3, 0, 65, 2000, 0, 0),
                           sc2.encode_tone(4, 0, 65, 2000, 0, 0),
                           sc2.encode_sweep(5, 0, 2000, 60, 200),
                           sc2.encode_command(5, 6, 0)):
            with self.subTest(words=[hex(word) for word in data_words]):
                self.assert_edge(self.play(data_words), 1.0,
                                 ["81 05", "82 05"])

    def test_a_stop_is_repeated_at_the_next_edge_only(self):
        # Whatever ends the session, and however long after the effect
        # ended the next edge comes.
        for name, end in self.ENDS.items():
            with self.subTest(end=name):
                model = self.play(self.PLAYS[sc2.PAD_PULSE])
                model.clock = 0.1
                end(model)
                self.assertEqual(model.relayed, stop(["81 02", "82 02"]))
                model.relayed.clear()
                model.clock = 3600.0
                run(model, sc2.HELLO)
                self.assertEqual(model.relayed, stop(["81 02", "82 02"]))
                model.relayed.clear()
                end(model)
                run(model, sc2.HELLO)
                end(model)
                self.assertEqual(model.relayed, [])

    def test_a_lost_stop_goes_out_at_the_next_edge(self):
        # The end had no link: the start stops the tone still running, and
        # the end after it repeats that.
        model = self.lose_the_end(self.PLAYS[sc2.PAD_TONE])
        self.assertEqual(model.relayed, stop(["81 02", "82 02"]))
        model.relayed.clear()
        model.clock = 1.0
        send(model, [sc2.BYE])
        self.assertEqual(model.relayed, stop(["81 02", "82 02"]))

    def test_a_new_effect_in_between_keeps_the_repeat(self):
        # A new effect on one side does not end an older one on another, so
        # the repeat stays whatever is queued after the start's stop. A new
        # pad tone that is still running at the end is stopped as usual and
        # the next edge repeats that.
        model = self.lose_the_end(self.PLAYS[sc2.PAD_TONE])
        self.play(sc2.encode_tone(1, 0, 160, 10, 0, 0), 0.3, model)
        self.assert_edge(model, 0.45, ["81 02", "82 02"])
        # Done by 1 s, or of another class: the end still repeats.
        for report in (sc2.encode_tone(1, 0, 160, 10, 0, 0),
                       sc2.encode_sweep(5, 0, 10, 60, 200)):
            with self.subTest(report=report):
                model = self.lose_the_end(self.PLAYS[sc2.PAD_TONE])
                self.play(report, 0.3, model)
                model.clock = 1.0
                send(model, [sc2.BYE])
                self.assertEqual(model.relayed, stop(["81 02", "82 02"]))

    def test_nothing_played(self):
        # No report at any session edge, and CHECKSUM shows none.
        for name, end in self.ENDS.items():
            with self.subTest(end=name):
                model = sc2.Model()
                run(model, sc2.HELLO)
                send(model, [sc2.NOP])
                end(model)
                self.assertFalse(model.active)
                run(model, sc2.HELLO)
                self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, 0xFFFF)
                self.assertEqual(model.relayed, [])

    def test_reports_that_keep_nothing_running(self):
        # OFF, tick, click, a train without on time, a script and rumble
        # that was switched off again need no stop.
        for data_words in (sc2.encode_command(2, 0, 0),
                           sc2.encode_command(5, 1, -10),
                           sc2.encode_command(2, 2, -10),
                           sc2.encode_pulse(2, 0, 0xFFFF, 0xFFFF),
                           sc2.encode_script(2, 13, -10),
                           sc2.encode_rumble(0x1234, 0)
                           + sc2.encode_rumble_stop()):
            with self.subTest(words=[hex(word) for word in data_words]):
                self.assert_edge(self.play(data_words), 0.0, [])

    def test_each_class_alone_still_running(self):
        for bit, data_words in self.PLAYS.items():
            for clock in (0.0, 0.549):
                with self.subTest(bit=bit, clock=clock):
                    self.assert_edge(self.play(data_words), clock,
                                     self.SENDS[bit])

    def test_each_class_alone_already_finished(self):
        # 400 ms and the 150 ms margin after the message.
        for bit, data_words in self.PLAYS.items():
            for clock in (0.55, 60.0):
                with self.subTest(bit=bit, clock=clock):
                    self.assert_edge(self.play(data_words), clock, [])

    def test_every_session_end_stops_a_running_class(self):
        for name, end in self.ENDS.items():
            with self.subTest(end=name):
                model = self.play(self.PLAYS[sc2.GRIP_PULSE])
                model.clock = 0.5
                end(model)
                self.assertEqual(model.relayed, stop(["81 05", "82 05"]))
                self.assertEqual(model.queued, STOP["82 05"])

    def test_all_classes_running(self):
        model = self.play(sc2.encode_rumble(0x1234, 0))
        for data_words in self.PLAYS.values():
            self.play(data_words, model=model)
        self.assert_edge(model, 0.1, ["81 02", "82 02", "81 05", "82 05"],
                         rumble=True)

    def test_a_long_train_ends_far_in_the_future(self):
        # 131070 us periods 65535 times: more microseconds than 32 bits hold.
        train = sc2.report_pulse(0, 0xFFFF, 0xFFFF, 0xFFFF)
        self.assertGreater(131070 * 65535, 2 ** 32)
        self.assertEqual(sc2.play_ms(train), 8589673)
        # Running 2 h later, done 8589673 ms and the margin after it.
        for clock, names in ((7200.0, ["81 02", "82 02"]),
                             (8589.822, ["81 02", "82 02"]),
                             (8589.823, [])):
            with self.subTest(clock=clock):
                model = self.play(sc2.encode_pulse(0, 0xFFFF, 0xFFFF,
                                                   0xFFFF))
                self.assert_edge(model, clock, names)

    def test_effects_of_no_known_end_run_until_a_stop(self):
        # A tone or sweep of 0 ms, an undefined 0x82 command.
        for data_words, names in (
                (sc2.encode_tone(1, 0, 160, 0, 0, 0), ["81 02", "82 02"]),
                (sc2.encode_sweep(3, 0, 0, 60, 200), ["81 05", "82 05"]),
                (sc2.encode_command(2, 3, 0), ["81 02", "82 02"]),
                (sc2.encode_command(4, 7, 0), ["81 05", "82 05"])):
            with self.subTest(words=[hex(word) for word in data_words]):
                self.assert_edge(self.play(data_words), 1e6, names)

    def test_a_shorter_effect_keeps_the_longer_end(self):
        model = self.play(sc2.encode_tone(0, 0, 160, 2000, 0, 0))
        self.play(sc2.encode_tone(1, 0, 160, 10, 0, 0), 0.1, model)
        self.assert_edge(model, 1.0, ["81 02", "82 02"])
        # The other way round the later end counts too.
        model = self.play(sc2.encode_tone(0, 0, 160, 10, 0, 0))
        self.play(sc2.encode_tone(1, 0, 160, 2000, 0, 0), 0.1, model)
        self.assert_edge(model, 2.249, ["81 02", "82 02"])

    def test_a_running_rumble_stops_three_times(self):
        # Rumble alone: the zero three times, and no repeat.
        model = self.play(sc2.encode_rumble(0x1234, 0))
        self.assert_edge(model, 0.0, [], rumble=True)
        # Plain rumble before a HELLO counts the same.
        send(model, [sc2.BYE])
        model.relayed.clear()
        model.word(0x0100)
        run(model, sc2.HELLO)
        self.assertEqual(model.relayed, [sc2.report_plain_rumble(0x0100)]
                         + [RUMBLE_OFF] * 3)

    def test_nothing_without_a_controller(self):
        model = session_model(linked=False)
        send(model, sc2.encode_rumble(1, 2)
             + sc2.encode_tone(0, 0, 160, 0, 0, 0) + [sc2.BYE])
        self.assertEqual(model.relayed, [])

    def test_play_ms(self):
        self.assertEqual(sc2.play_ms(sc2.report_pulse(0, 2, 40, 6)), 1)
        self.assertEqual(sc2.play_ms(sc2.report_pulse(0, 1000, 0, 0)), 1)
        self.assertEqual(sc2.play_ms(sc2.report_pulse(3, 0, 5000, 9)), 0)
        self.assertEqual(sc2.play_ms(sc2.report_pulse(5, 30000, 30000, 15)),
                         900)
        self.assertEqual(sc2.play_ms(sc2.report_tone(3, 0, 65, 2000, 0, 0)),
                         2000)
        self.assertEqual(sc2.play_ms(sc2.report_sweep(0, 0, 0xFFFF, 1, 2)),
                         0xFFFF)
        self.assertEqual(sc2.play_ms(sc2.report_tone(0, 0, 65, 0, 0, 0)),
                         math.inf)
        self.assertEqual(sc2.play_ms(sc2.report_sweep(0, 0, 0, 1, 2)),
                         math.inf)
        self.assertEqual([sc2.play_ms(sc2.report_command(2, command, 0))
                          for command in range(8)],
                         [0, 0, 0] + [math.inf] * 5)
        self.assertEqual(sc2.play_ms(sc2.report_script(2, 13, 0)), 0)
        self.assertEqual(sc2.play_ms(sc2.report_rumble(0xFFFF, 0)), 0)
        self.assertEqual([sc2.stop_class(sc2.report_pulse(side, 1, 1, 1))
                          for side in range(6)],
                         [sc2.PAD_PULSE] * 3 + [sc2.GRIP_PULSE] * 3)
        self.assertEqual([sc2.stop_class(sc2.report_command(side, 3, 0))
                          for side in range(6)],
                         [sc2.PAD_TONE] * 3 + [sc2.GRIP_TONE] * 3)


class State(unittest.TestCase):
    def test_ack_state_fields(self):
        state = sc2.ack_state(0x1FF, True, 4)
        self.assertEqual(state, 0xFFC2)
        self.assertEqual(sc2.decode_state(state), {
            "counter": 0xFF, "session": 1, "expected": 4, "version": 2})

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
        data_words = sc2.encode_tone(1, 0, 1, 2, 3, 4)
        outcomes = send(model, data_words)
        self.assertEqual([sc2.decode_state(outcome.state)["expected"]
                          for outcome in outcomes], [4, 3, 2, 1, 0])
        # The echo is the whole word, toggle included.
        self.assertEqual([outcome.echo for outcome in outcomes],
                         sc2.stamp(data_words, 0)[0])

    def test_bye(self):
        model = session_model()
        outcome = send(model, [sc2.BYE])[0]
        self.assertEqual(sc2.decode_state(outcome.state)["session"], 0)
        self.assertFalse(model.word(0x1010).accepted)


class Copies(unittest.TestCase):
    """A word that arrives twice must change nothing but the traffic."""

    def test_every_message_with_every_word_twice(self):
        for message in sc2.MESSAGES.values():
            for values in sc2.boundary_cases(message):
                with self.subTest(case=sc2.describe_case(message, values)):
                    self.assert_copies_ignored(message.encode(*values))
        self.assert_copies_ignored(sc2.encode_rumble_stop())
        self.assert_copies_ignored([sc2.NOP, sc2.CHECKSUM])

    def assert_copies_ignored(self, data_words):
        clean, twice = session_model(), session_model()
        words, _ = sc2.stamp(data_words + [sc2.CHECKSUM], clean.toggle)
        expected = run(clean, words)
        outcomes = []
        for word in words:
            counter = twice.counter
            outcomes.append(twice.word(word))
            copy = twice.word(word)
            self.assertTrue(copy.copy)
            self.assertFalse(copy.accepted)
            self.assertEqual(twice.counter, counter + 1)
            self.assertEqual(copy.state, outcomes[-1].state)
        self.assertEqual(outcomes, expected)
        self.assertEqual(twice.queued, clean.queued)

    def test_first_session_word_must_flip_the_toggle(self):
        model = session_model()
        outcome = model.word(sc2.CHECKSUM)
        self.assertTrue(outcome.copy)
        self.assertTrue(send(model, [sc2.CHECKSUM])[0].accepted)

    def test_hello_with_every_word_twice(self):
        model = sc2.Model()
        for word in sc2.HELLO:
            self.assertTrue(model.word(word).accepted)
            self.assertTrue(model.word(word).copy)
        self.assertEqual(model.counter, 4)
        self.assertTrue(model.active)

    def assert_end_word_copied(self, data_words, end_word):
        model = session_model()
        outcomes = send(model, data_words)
        self.assertEqual(outcomes[-1].echo, end_word)
        self.assertFalse(model.active)
        counter, state = model.counter, model.state()
        copy = model.word(end_word)
        self.assertTrue(copy.copy)
        self.assertEqual((model.counter, copy.state), (counter, state))
        # After another word the same word is plain rumble again.
        self.assertFalse(model.word(0x1010).accepted)
        repeat = model.word(end_word)
        self.assertFalse(repeat.accepted or repeat.copy)

    def test_bye_twice_at_both_toggles(self):
        # 0x8200 as plain rumble drives the left motor at 0x82.
        self.assert_end_word_copied([sc2.BYE], 0x8200)
        self.assert_end_word_copied([sc2.NOP, sc2.BYE], 0x0200)

    def test_rejected_word_twice(self):
        self.assert_end_word_copied([0x3800], 0xB800)
        self.assert_end_word_copied([sc2.NOP, 0x3800], 0x3800)

    def test_rejected_hello_word_twice(self):
        # A rejected first HELLO word starts a HELLO, and its copy is a copy
        # under both rules.
        model = session_model()
        send(model, [sc2.NOP])
        self.assertTrue(model.word(sc2.HELLO[0]).accepted)
        self.assertTrue(model.word(sc2.HELLO[0]).copy)
        run(model, sc2.HELLO[1:])
        self.assertTrue(model.active)


class Framing(unittest.TestCase):
    def assert_rejected(self, data_words):
        model = session_model()
        outcomes = send(model, data_words)
        last = outcomes[-1]
        self.assertTrue(last.accepted)
        self.assertEqual(last.echo & sc2.DATA, data_words[-1])
        self.assertEqual(sc2.decode_state(last.state)["session"], 0)
        return model

    def test_rejected_headers(self):
        for word in (0x3000, 0x3800, 0x0700, 0x0001, 0x0300, 0x0501,
                     0x060E, 0x1008, 0x1005, 0x1400, 0x1808, 0x1880):
            with self.subTest(word=hex(word)):
                self.assert_rejected([word])

    def test_sides_6_and_7_are_rejected(self):
        for opcode in (sc2.OP_COMMAND, sc2.OP_SCRIPT, sc2.OP_PULSE,
                       sc2.OP_SWEEP, sc2.OP_TONE):
            for side in (6, 7):
                word = sc2.header(opcode, side << 8)
                with self.subTest(word=hex(word)):
                    self.assertIsNone(sc2.payload_words(word))
                    # Nothing played, so the end's stop queued nothing.
                    model = self.assert_rejected([word])
                    self.assertEqual(model.queued, b"")

    def test_rejected_payloads(self):
        self.assert_rejected([sc2.header(sc2.OP_SCRIPT), 0x0100])
        self.assert_rejected(sc2.encode_tone(0, 0, 1, 2, 3, 4)[:4] + [0x800])
        self.assert_rejected(sc2.encode_sweep(0, 0, 1, 2, 3)[:4] + [0x8])
        self.assert_rejected([0x0500, 0x5547])
        self.assert_rejected([0x0603, 0x4D04])

    def test_hello_words_are_rejected_headers(self):
        # With the other toggle a HELLO word in a session is rejected and
        # counts as the first word of a new HELLO.
        for word in sc2.HELLO:
            self.assertIsNone(sc2.payload_words(word))
        model = session_model()
        send(model, [sc2.NOP])
        self.assertEqual(model.toggle, 1)
        outcomes = run(model, sc2.HELLO)
        self.assertTrue(all(outcome.accepted for outcome in outcomes))
        self.assertTrue(model.active)

    def test_single_word_message_inside_a_long_one_is_payload(self):
        model = session_model()
        click = sc2.encode_command(3, 2, -10)[0]
        send(model, sc2.encode_pulse(3, click, 0, 1))
        self.assertEqual(model.queued, sc2.report_pulse(3, click, 0, 1))

    def test_nop_only_at_a_header(self):
        model = session_model()
        send(model, sc2.encode_pulse(1, 0, 0, 0))
        self.assertEqual(model.queued, sc2.report_pulse(1, 0, 0, 0))
        self.assertEqual(send(model, [sc2.NOP])[0].state >> 4 & 7, 0)

    def test_reboot_commands(self):
        model = session_model()
        send(model, sc2.encode_bootloader())
        self.assertEqual(model.reboot, "bootloader")
        model = session_model()
        send(model, sc2.encode_mode(1))
        self.assertEqual(model.reboot, 1)


class FakeLink:
    """Link's interface over the Model. The session times out before the
    words whose send index is in timeouts, and refused words are never
    taken, as with a relay that stays full."""

    def __init__(self, timeouts=(), refused=()):
        self.model = sc2.Model()
        self.timeouts = set(timeouts)
        self.refused = set(refused)
        self.sent = []

    def state(self):
        return self.model.state()

    def quiet(self, seconds):
        if seconds >= sc2.SESSION_TIMEOUT_S and self.model.active:
            self.model.end_session()
        self.model.hello = 0

    def send(self, word, in_session=False):
        if len(self.sent) in self.timeouts:
            self.model.end_session()
        refused = len(self.sent) in self.refused
        self.sent.append(word)
        outcome = sc2.Outcome(False) if refused else self.model.word(word)
        if outcome.accepted:
            return sc2.Ack(word, outcome.echo, outcome.state, 0, 0, 1)
        # Link.send resends a session word only while the session bit is
        # set, and each resend is a copy or refused again.
        if not (in_session and not self.model.active):
            self.sent += [word] * sc2.MAX_RESENDS
        return None


class Client(unittest.TestCase):
    PULSE = sc2.encode_pulse(3, 1000, 1000, 3)

    def run_client(self, link, *messages):
        client = sc2.Client(link)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(client.hello())
            return [client.message(data, "test") for data in messages]

    def test_a_session_lost_mid_message_restarts_it(self):
        # The session times out before the third pulse word, which the
        # dongle then sees as plain rumble.
        link = FakeLink(timeouts={6})
        acks = self.run_client(link, self.PULSE, [sc2.CHECKSUM])
        self.assertEqual(len(acks[0]), 4)
        words, _ = sc2.stamp(self.PULSE, 0)
        # Sent once, not resent while the session bit was clear.
        self.assertEqual(link.sent[:8],
                         list(sc2.HELLO) + words[:3] + [sc2.HELLO[0]])
        self.assertEqual(link.sent[7:15], list(sc2.HELLO) + words)
        self.assertEqual(acks[1][0].echo,
                         sc2.crc16(sc2.report_pulse(3, 1000, 1000, 3)))

    def test_a_session_found_gone_before_a_message(self):
        link = FakeLink()
        client = sc2.Client(link)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(client.hello())
            link.quiet(sc2.SILENCE_S)
            acks = client.message([sc2.CHECKSUM], "test")
        self.assertIsNotNone(acks)
        self.assertEqual(link.sent[4:], list(sc2.HELLO) + [0x8400])

    def test_a_missing_acknowledgement_stops_the_message(self):
        link = FakeLink(refused={5})
        acks = self.run_client(link, self.PULSE)
        self.assertIsNone(acks[0])
        # The refused word was resent, nothing after it was sent.
        self.assertEqual(len(link.sent), 4 + 2 + sc2.MAX_RESENDS)

    def test_a_rejected_message_is_tried_twice(self):
        link = FakeLink()
        acks = self.run_client(link, [0x3800])
        self.assertIsNone(acks[0])
        self.assertEqual(link.sent.count(0xB800), 2)

    def test_bye_ends_the_session_without_a_restart(self):
        link = FakeLink()
        acks = self.run_client(link, [sc2.BYE])
        self.assertEqual(len(acks[0]), 1)
        self.assertEqual(len(link.sent), 5)


class ModelAcks:
    def __init__(self, dongle):
        self.dongle = dongle

    @property
    def state(self):
        return self.dongle.state()

    @property
    def echo(self):
        return self.dongle.echo

    def counter(self):
        return self.dongle.counter

    def refresh(self):
        pass


class ModelLink:
    """Link with the Model as the dongle, for whole subcommands."""

    linked = True
    version = sc2.PROTOCOL_VERSION
    made = []

    def __init__(self, dongle_nodes):
        self.dongle = sc2.Model(counter=200, linked=self.linked,
                                version=self.version)
        self.acks = ModelAcks(self.dongle)
        self.round_trips_ns = []
        self.resends = 0
        self.made.append(self)

    def close(self):
        pass

    def state(self):
        return self.dongle.state()

    def quiet(self, seconds):
        if seconds >= sc2.SESSION_TIMEOUT_S:
            if self.dongle.active:
                self.dongle.end_session()
            self.dongle.hello = 0

    def send(self, word, in_session=False):
        outcome = self.dongle.word(word)
        if not outcome.accepted:
            return None
        self.round_trips_ns.append(1000000)
        return sc2.Ack(word, outcome.echo, outcome.state, 1000000, 0, 1)

    def send_plain(self, word, listen_s=0.1):
        return not self.dongle.word(word).accepted

    def gone(self, seconds):
        return self.dongle.reboot is not None


class RecordingDongle(sc2.Model):
    """The Model, logging (time, report) for each report it queues for the
    controller."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.log = []

    def _relay(self, report):
        queued = len(self.relayed)
        super()._relay(report)
        self.log += [(self.clock, entry) for entry in self.relayed[queued:]]


class TimedLink(ModelLink):
    """ModelLink over a RecordingDongle, each word taking one resend timeout
    and each listen or silence its length."""

    def __init__(self, dongle_nodes):
        super().__init__(dongle_nodes)
        self.dongle = RecordingDongle(counter=200, linked=self.linked,
                                      version=self.version)
        self.acks = ModelAcks(self.dongle)

    def quiet(self, seconds):
        self.dongle.clock += seconds
        super().quiet(seconds)

    def send(self, word, in_session=False):
        ack = super().send(word, in_session)
        self.dongle.clock += sc2.RESEND_AFTER_S
        return ack

    def send_plain(self, word, listen_s=0.1):
        taken = super().send_plain(word, listen_s)
        self.dongle.clock += listen_s
        return taken


# How long the controller plays a report, from its fields and the meanings in
# docs/PROTOCOL.md 9.5.1. Where the sources leave it open: forever.
TICK_S = 0.018
CLICK_S = 0.024
# The stock firmware's scripts as reverse engineered (mwdmwd/sc26re): up to
# 3 s, no stop. Other ids play nothing.
SCRIPT_IDS = range(1, 17)
SCRIPT_S = 3.0
# The dongle refreshes a rumble until the next 0x80 or session edge, and the
# controller plays each 0x80 for 100 ms (sc26re).
RUMBLE_HOLD_S = 0.1


def effect_seconds(report):
    """Seconds a report other than 0x80 plays."""
    kind = report[0]
    if kind == 0x81:
        _, _, on_us, off_us, repeat = struct.unpack("<BBHHH", report)
        # No on time plays nothing, no repeat may still play once.
        return max(repeat, 1) * (on_us + off_us) / 1e6 if on_us else 0.0
    if kind == 0x82:
        return {0: 0.0, 1: TICK_S, 2: CLICK_S}.get(report[2], math.inf)
    if kind == 0x85:
        return SCRIPT_S if report[2] in SCRIPT_IDS else 0.0
    duration_ms = struct.unpack_from("<H", report, 5 if kind == 0x83 else 3)[0]
    return duration_ms / 1000 if duration_ms else math.inf


def effects(log):
    """(report, seconds) for each report in a RecordingDongle's log."""
    for index, (start, report) in enumerate(log):
        if report[0] != 0x80:
            yield report, effect_seconds(report)
        elif report[1]:
            ends = [time for time, later in log[index + 1:]
                    if later[0] == 0x80]
            yield report, (ends[0] - start + RUMBLE_HOLD_S if ends
                           else math.inf)


class Effects(unittest.TestCase):
    def test_effect_seconds(self):
        # The selftest's old copy case: 65535 pulses of 65.5 ms on and off.
        self.assertGreater(effect_seconds(
            sc2.report_pulse(0, 0xFFFF, 0xFFFF, 0xFFFF)), 8000)
        self.assertEqual(effect_seconds(
            sc2.report_pulse(0, 0, 0xFFFF, 0xFFFF)), 0.0)
        self.assertEqual(effect_seconds(
            sc2.report_pulse(1, 0xFFFF, 0, 0)), 0.065535)
        self.assertEqual(effect_seconds(
            sc2.report_tone(0, -128, 0, 0, 0, 0)), math.inf)
        self.assertEqual(effect_seconds(
            sc2.report_sweep(0, -128, 300, 0, 0)), 0.3)
        self.assertEqual(effect_seconds(sc2.report_command(0, 7, -128)),
                         math.inf)
        self.assertEqual(effect_seconds(sc2.report_script(0, 5, -128)),
                         SCRIPT_S)

    def test_rumble_lasts_until_the_next(self):
        on, off = sc2.report_rumble(1, 0), sc2.report_rumble(0, 0)
        self.assertEqual(list(effects([(1.0, on), (1.5, off)])),
                         [(on, 0.5 + RUMBLE_HOLD_S)])
        self.assertEqual(list(effects([(1.0, on)])), [(on, math.inf)])
        # A session edge stops it with its own 0x80.
        dongle = RecordingDongle()
        run(dongle, sc2.HELLO)
        send(dongle, sc2.encode_rumble(1, 0))
        dongle.clock = 0.25
        send(dongle, [sc2.BYE])
        self.assertEqual([seconds for report, seconds in effects(dongle.log)
                          if report[0] == 0x80],
                         [0.25 + RUMBLE_HOLD_S])


class Subcommands(unittest.TestCase):
    """Each subcommand from argv to exit code, the Model as the dongle."""

    def run_main(self, *argv, linked=True, base=ModelLink,
                 version=sc2.PROTOCOL_VERSION):
        nodes = types.SimpleNamespace(
            prefix="usb-test", xpad=types.SimpleNamespace(path="/x"),
            pads=types.SimpleNamespace(path="/p"))
        link = type("Link", (base,), {"linked": linked, "version": version})
        output = io.StringIO()
        with mock.patch.object(sc2, "choose_dongle", return_value=nodes), \
                mock.patch.object(sc2, "controller_linked",
                                  return_value=linked), \
                mock.patch.object(sc2, "Link", link), \
                mock.patch.object(sc2.time, "sleep"), \
                contextlib.redirect_stdout(output):
            code = sc2.main(list(argv))
        return code, output.getvalue()

    def test_selftest_passes(self):
        for linked in (True, False):
            code, output = self.run_main("selftest", linked=linked)
            self.assertEqual(code, 0, output)
            self.assertNotIn("FAIL", output)
            self.assertEqual("CHECK by hand" in output, linked)

    def test_selftest_effects_end_within_a_second(self):
        TimedLink.made.clear()
        code, output = self.run_main("selftest", base=TimedLink)
        self.assertEqual(code, 0, output)
        played = list(effects(TimedLink.made[0].dongle.log))
        self.assertGreater(len(played), 50)
        for report, seconds in played:
            with self.subTest(report=report.hex(" ")):
                self.assertLessEqual(seconds, 1.0)

    def test_send_every_message(self):
        for message in sc2.MESSAGES.values():
            values = [str(field.maximum) for field in message.fields]
            code, output = self.run_main("send", message.name, *values)
            self.assertEqual(code, 0, output)
            self.assertIn(" match", output)
        code, output = self.run_main("send", "rumble", "stop")
        self.assertEqual(code, 0, output)

    def test_session_commands(self):
        for argv in (["hello"], ["bye"], ["bootloader"], ["mode", "13"]):
            code, output = self.run_main(*argv)
            self.assertEqual(code, 0, output)
        code, output = self.run_main("mode", "14")
        self.assertEqual(code, 1, output)

    def test_send_refuses_another_version(self):
        ModelLink.made.clear()
        code, output = self.run_main("send", "command", "5", "2", "-10",
                                     version=1)
        self.assertEqual(code, 1, output)
        self.assertIn("protocol version 1, this tool version 2", output)
        # Refused at the first HELLO word's acknowledgement.
        dongle = ModelLink.made[0].dongle
        self.assertEqual((dongle.counter, dongle.hello, dongle.relayed),
                         (201, 1, []))

    def test_selftest_refuses_another_version(self):
        ModelLink.made.clear()
        with self.assertRaises(SystemExit) as raised:
            self.run_main("selftest", version=1)
        self.assertIn("protocol version 1, this tool version 2",
                      str(raised.exception.code))
        # Only the plain rumble sent before the HELLO reached the relay.
        dongle = ModelLink.made[0].dongle
        self.assertFalse(dongle.active)
        self.assertEqual({report[0] for report in dongle.relayed}, {0x80})


class Discovery(unittest.TestCase):
    def test_hid_nodes_carry_the_hid_version(self):
        # As seen on Linux 7.3: only the xpad node reports bcdDevice.
        prefix = "usb-0000:00:14.0-2"
        nodes = [sc2.Node(f"/dev/input/event{index}", "", prefix + suffix,
                          0x045E, 0x028E, version)
                 for index, (suffix, version) in enumerate(
                     (("/input0", 0x5C21), ("/input1", 0x0111),
                      ("/input2", 0x0111), ("/input3", 0x0111)))]
        dongles = sc2.find_dongles(nodes)
        self.assertEqual(len(dongles), 1)
        self.assertEqual(dongles[0].pads.path, "/dev/input/event2")


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
