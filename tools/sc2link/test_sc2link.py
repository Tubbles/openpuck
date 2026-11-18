#!/usr/bin/env python3
"""Host checks of sc2link_test.py, no dongle needed.

Run: python3 tools/sc2link/test_sc2link.py
"""

import contextlib
import io
import math
import os
import struct
import sys
import types
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sc2link_test as sc2  # noqa: E402


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
        # docs/PROTOCOL.md: COMMAND data 0x4AF6 becomes 82 02 02 F6, a click
        # on both pads.
        self.assertEqual(sc2.encode_command(2, 2, -10), [0x4AF6])
        self.assertEqual(sc2.report_command(2, 2, -10).hex(), "820202f6")

    def test_layouts(self):
        # The bit layouts of the message table in docs/PROTOCOL.md.
        self.assertEqual(sc2.encode_script(1, 0xFF, -1), [0x09FF, 0x00FF])
        self.assertEqual(sc2.encode_rumble(0xFFFF, 0x1234),
                         [0x1002, 0x7FFF, 0x1234])
        self.assertEqual(sc2.encode_pulse(3, 0xFFFF, 1, 0x8000),
                         [0x1B05, 0x7FFF, 0x0001, 0x0000])
        self.assertEqual(sc2.encode_sweep(1, -128, 0xFFFF, 2, 0x8003),
                         [0x2180, 0x7FFF, 0x0002, 0x0003, 0x0005])
        self.assertEqual(sc2.encode_tone(2, 127, 0xFFFF, 4, 0x8005, 0xFF),
                         [0x2A7F, 0x7FFF, 0x0004, 0x0005, 0x05FF])

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
        outcomes = send(model, sc2.encode_tone(3, -20, 160, 500, 4, 200)
                        + [sc2.CHECKSUM])
        expected = sc2.crc16(sc2.report_tone(3, -20, 160, 500, 4, 200))
        self.assertEqual(outcomes[-1].echo, expected)

    def test_checksum_without_controller(self):
        model = session_model(linked=False)
        outcomes = send(model, sc2.encode_rumble(1, 2) + [sc2.CHECKSUM])
        self.assertEqual(outcomes[-1].echo, 0xFFFF)

    def test_session_end_queues_the_stop(self):
        # A message, BYE, a new session: CHECKSUM covers the end's OFF,
        # since a session start queues the same stop but leaves CHECKSUM.
        self.assertEqual(sc2.STOP_REPORT.hex(), "82030000")
        for linked, expected in ((True, sc2.crc16(sc2.STOP_REPORT)),
                                 (False, 0xFFFF)):
            model = session_model(linked)
            send(model, sc2.encode_pulse(3, 20000, 20000, 1000) + [sc2.BYE])
            run(model, sc2.HELLO)
            self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, expected)
            # A rejected word ends the session the same way.
            send(model, sc2.encode_command(1, 1, 0) + [0x3800])
            run(model, sc2.HELLO)
            self.assertEqual(send(model, [sc2.CHECKSUM])[0].echo, expected)

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
                     0x060E, 0x6000, 0x0C00, 0x1008, 0x1005, 0x1C00,
                     0x2400, 0x2C00):
            with self.subTest(word=hex(word)):
                self.assert_rejected([word])

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

    def __init__(self, dongle_nodes):
        self.dongle = sc2.Model(counter=200, linked=self.linked)
        self.acks = ModelAcks(self.dongle)
        self.round_trips_ns = []
        self.resends = 0

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
    """The Model, logging (time, report) for each report it relays to the
    controller and (time, None) where a session starts or ends, which stops
    rumble."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.clock = 0.0
        self.log = []

    def word(self, word):
        active = self.active
        outcome = super().word(word)
        if not (active or outcome.accepted or outcome.copy):
            self.log.append((self.clock, sc2.report_rumble(
                (word >> 8) * 257, (word & 0xFF) * 257)))
        if self.active and not active:
            self.log.append((self.clock, None))
        return outcome

    def end_session(self):
        self.log.append((self.clock, None))
        super().end_session()

    def _run(self, last):
        if sc2.opcode_of(self.head) != sc2.OP_CONTROL:
            self.log.append(
                (self.clock, sc2.build_report(self.head, self.payload)))
        return super()._run(last)


class TimedLink(ModelLink):
    """ModelLink over a RecordingDongle, each word taking one resend timeout
    and each listen or silence its length."""

    made = []

    def __init__(self, dongle_nodes):
        super().__init__(dongle_nodes)
        self.dongle = RecordingDongle(counter=200, linked=self.linked)
        self.acks = ModelAcks(self.dongle)
        self.made.append(self)

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
        if report is None:
            continue
        if report[0] != 0x80:
            yield report, effect_seconds(report)
        elif report[1]:
            ends = [time for time, later in log[index + 1:]
                    if later is None or later[0] == 0x80]
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
        self.assertEqual(list(effects([(1.0, on), (1.25, None)])),
                         [(on, 0.25 + RUMBLE_HOLD_S)])
        self.assertEqual(list(effects([(1.0, on)])), [(on, math.inf)])


class Subcommands(unittest.TestCase):
    """Each subcommand from argv to exit code, the Model as the dongle."""

    def run_main(self, *argv, linked=True, base=ModelLink):
        nodes = types.SimpleNamespace(
            prefix="usb-test", xpad=types.SimpleNamespace(path="/x"),
            pads=types.SimpleNamespace(path="/p"))
        link = type("Link", (base,), {"linked": linked})
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
