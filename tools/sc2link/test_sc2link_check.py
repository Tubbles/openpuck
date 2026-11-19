#!/usr/bin/env python3
"""Host checks of sc2link_check.py, no dongle needed.

The interactive flow runs against a fake console (virtual time, a simulated
person who performs each instruction) and the protocol Model as the dongle,
whose relayed reports a fake motion recorder turns into vibration.

Run: python3 -m unittest discover -s tools/sc2link -p "test_*.py"
"""

import collections
import dataclasses
import errno
import heapq
import itertools
import json
import math
import os
import re
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sc2link_check as check  # noqa: E402
import sc2link_test as sc2  # noqa: E402
from test_sc2link import TimedLink, effect_seconds, effects  # noqa: E402

PROTOCOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "..", "docs", "PROTOCOL.md")
SAMPLE_NS = int(check.SAMPLE_PERIOD_S * 1e9)
TICK_NS = 10_000_000
GYRO_CODES = (sc2.ABS_RY, sc2.ABS_THROTTLE, sc2.ABS_RUDDER)
ACCEL_CODES = (sc2.ABS_X, sc2.ABS_Z, sc2.ABS_RX)
FLAT = {sc2.ABS_X: 0, sc2.ABS_Z: 0, sc2.ABS_RX: check.ACCEL_COUNTS_PER_G}


def signed16(value):
    return value - 0x10000 if value & 0x8000 else value


def snapshot(time_s=0.0, source="controls", buttons=0, left_stick=(0, 0),
             right_stick=(0, 0), triggers=(0, 0), left_pad=(0, 0),
             right_pad=(0, 0), accel=(0, 0, 1), gyro=(0, 0, 0),
             connected=True):
    return check.Snapshot(int(time_s * 1e9), source, connected, buttons,
                          left_stick, right_stick, triggers, left_pad,
                          right_pad, accel, gyro)


def feed_all(watcher, snapshots):
    for item in snapshots:
        outcome = watcher.feed(item)
        if outcome is not None:
            return outcome
    return None


def ticks(start_s, end_s, step_s=0.01, **fields):
    count = int(round((end_s - start_s) / step_s))
    return [snapshot(start_s + index * step_s, **fields)
            for index in range(count)]


# ---- the simulated person ---------------------------------------------------

def bit_frames(bits):
    controls = {sc2.ABS_THROTTLE: signed16(bits & 0xFFFF),
                sc2.ABS_RUDDER: signed16(bits >> 16 & 0xFFFF)}
    pads = {sc2.ABS_WHEEL: (1 if bits & check.PADS_LEFT_TOUCH else 0)
            | (2 if bits & check.PADS_RIGHT_TOUCH else 0)}
    return controls, pads


ACTION_SPACING_S = 1.5


class Person:
    """Performs every instruction right unless actions says otherwise:
    {step key: [action, ...]}, each ACTION_SPACING_S after the one before:
    "right", bits pressed instead, "skip" or "fail" (the s or f key),
    "interrupt" (Ctrl+C) or "unplug" (a node gone). held bits stay active
    throughout, as a stuck sensor would."""

    def __init__(self, actions=None, held=0):
        self.actions = actions or {}
        self.held = held

    def perform(self, step):
        frames = []
        for index, action in enumerate(self.actions.get(step.key,
                                                        ["right"])):
            frames += [(index * ACTION_SPACING_S + offset_s, role, changes)
                       for offset_s, role, changes in self.act(step, action)]
        return frames

    def act(self, step, action):
        if action in ("skip", "fail"):
            return [(0.5, "key", action[0])]
        if action == "interrupt":
            raise KeyboardInterrupt
        if action == "unplug":
            raise OSError(errno.ENODEV, "No such device")
        if action != "right":
            return self.press(action)
        performer = {check.BitStep: lambda: self.press(step.expected),
                     check.StickStep: lambda: stick_frames(step),
                     check.TriggerStep: lambda: self.trigger(step),
                     check.PadStep: lambda: self.pad(step),
                     check.TurnStep: lambda: turn_frames(step)}
        return performer[type(step)]()

    def buttons(self, bits):
        return bit_frames(bits | self.held)

    def press(self, bits):
        on_controls, on_pads = self.buttons(bits)
        off_controls, off_pads = self.buttons(0)
        return [(0.2, "controls", on_controls), (0.201, "pads", on_pads),
                (0.4, "controls", off_controls), (0.401, "pads", off_pads)]

    def trigger(self, step):
        if step.target == "released":
            return []
        code = sc2.ABS_GAS if step.trigger == "left" else sc2.ABS_BRAKE
        amount = 0.95 if step.target == "full" else 0.5
        click = check.BIT["LTriggerClick" if step.trigger == "left"
                          else "RTriggerClick"]
        on = {code: round(amount * check.TRIGGER_FULL_SCALE),
              **self.buttons(click if step.target == "full" else 0)[0]}
        off = {code: 0, **self.buttons(0)[0]}
        return [(0.2, "controls", on), (0.8, "controls", off)]

    def pad(self, step):
        x_code, y_code, touched, name = {
            "left": (sc2.ABS_X, sc2.ABS_Z, 1, "LPadTouch"),
            "right": (sc2.ABS_RY, sc2.ABS_THROTTLE, 2, "RPadTouch")}[step.pad]
        x, y = {"left": (-0.8, 0), "right": (0.8, 0), "top": (0, 0.8),
                "bottom": (0, -0.8)}[step.edge]
        return [(0.2, "pads", {x_code: round(x * check.PAD_FULL_SCALE),
                               y_code: round(y * check.PAD_FULL_SCALE),
                               sc2.ABS_WHEEL: touched}),
                (0.201, "controls", self.buttons(check.BIT[name])[0]),
                (0.5, "pads", {sc2.ABS_WHEEL: 0}),
                (0.501, "controls", self.buttons(0)[0])]


def stick_frames(step):
    if step.target == "centre":
        return []
    x_code, y_code = {"left": (sc2.ABS_X, sc2.ABS_Z),
                      "right": (sc2.ABS_RX, sc2.ABS_RY)}[step.stick]
    x, y = {"left": (-1, 0), "right": (1, 0), "up": (0, 1),
            "down": (0, -1)}[step.direction]
    amount = (0.95 if step.target == "full" else 0.5) \
        * check.STICK_FULL_SCALE
    return [(0.2, "controls", {x_code: round(x * amount),
                               y_code: round(y * amount)}),
            (0.8, "controls", {x_code: 0, y_code: 0})]


def turn_frames(step):
    """180 deg/s for 0.5 s, held 1 s in the pose, and back."""
    rate = round(180 / check.GYRO_DEGREES_PER_COUNT)
    still = {code: 0 for code in GYRO_CODES}
    pose = dict(FLAT)
    if step.exposes:
        axis, sign = step.exposes
        pose = {code: 0 for code in ACCEL_CODES}
        pose[ACCEL_CODES[axis]] = sign * check.ACCEL_COUNTS_PER_G
    turn = [(0.2 + index * 0.004, "motion",
             {GYRO_CODES[step.axis]: step.sign * rate})
            for index in range(125)]
    back = [(1.7 + index * 0.004, "motion",
             {GYRO_CODES[step.axis]: -step.sign * rate})
            for index in range(125)]
    return (turn + [(0.7, "motion", {**still, **pose})] + back
            + [(2.2, "motion", {**still, **FLAT})])


def base_values(held=0):
    controls, pads = bit_frames(held)
    return {
        "controls": {sc2.ABS_MISC: 0, **controls, sc2.ABS_X: 0, sc2.ABS_Z: 0,
                     sc2.ABS_RX: 0, sc2.ABS_RY: 0, sc2.ABS_WHEEL: 1,
                     sc2.ABS_GAS: 0, sc2.ABS_BRAKE: 0},
        "pads": {sc2.ABS_X: 0, sc2.ABS_Z: 0, sc2.ABS_RX: 0, sc2.ABS_RY: 0,
                 sc2.ABS_THROTTLE: 0, sc2.ABS_RUDDER: 0, **pads,
                 sc2.ABS_GAS: 0, sc2.ABS_BRAKE: 0},
        "motion": {**FLAT, **{code: 0 for code in GYRO_CODES}},
    }


class FakeConsole:
    """Virtual time: a controls frame every 10 ms and the person's frames
    as each instruction asks. keys are (seconds, key) typed at that time,
    answers the keys section 6's questions get, in order."""

    def __init__(self, person, answers=(), keys=()):
        self.person = person
        self.values = base_values(person.held)
        self.answers = list(answers)
        self.time_ns = 0
        self.queue = []
        self.order = itertools.count()
        self.lines = []
        self.next_tick_ns = 0
        self.counter = 0
        self.key_calls = 0
        for seconds, key in keys:
            self.push(int(seconds * 1e9), "key", key)

    def push(self, time_ns, role, changes):
        heapq.heappush(self.queue, (time_ns, next(self.order), role, changes))

    def initial_values(self):
        return {role: dict(values) for role, values in self.values.items()}

    def clock_ns(self):
        return self.time_ns

    def say(self, text):
        self.lines.append(text)

    def show_step(self, step, text):
        self.lines.append(text)
        for offset_s, role, changes in self.person.perform(step):
            self.push(self.time_ns + int(offset_s * 1e9), role, changes)

    def events(self, timeout_s):
        end = self.time_ns + int(timeout_s * 1e9)
        frames, keys = [], []
        while True:
            queued = self.queue[0][0] if self.queue else math.inf
            if queued <= min(end, self.next_tick_ns):
                time_ns, _, role, changes = heapq.heappop(self.queue)
                if role == "key":
                    keys.append(changes)
                else:
                    frames.append((role, time_ns, changes))
            elif self.next_tick_ns <= end:
                self.counter += 1
                frames.append(("controls", self.next_tick_ns,
                               {sc2.ABS_MISC: self.counter}))
                self.next_tick_ns += TICK_NS
            else:
                break
        self.time_ns = end
        return frames, keys

    def key(self, timeout_s):
        self.time_ns += int(timeout_s * 1e9)
        self.key_calls += 1
        if self.key_calls > 1000:
            raise AssertionError("the check waits for a key the script "
                                 "does not give")
        if not self.answers:
            return None
        answer = self.answers.pop(0)
        if answer == "interrupt":
            raise KeyboardInterrupt
        return answer

    def flush_keys(self):
        pass


# ---- the dongle and the controller's motion ---------------------------------

def playing(log):
    """(start, end, period, on) in ns of what the Model relayed: rumble
    until the next 0x80 or session edge, a pulse train until its end or the
    next report or session edge (every stop candidate stops it here)."""
    intervals = []
    for index, (time_s, report) in enumerate(log):
        if report is None:
            continue
        start = int(time_s * 1e9)
        later = [int(time * 1e9) for time, other in log[index + 1:]
                 if other is None or other[0] != 0x80 or report[0] == 0x80]
        if report[0] == 0x80:
            if report[1]:
                intervals.append((start, later[0] if later else math.inf,
                                  0, 0))
        elif report[0] == 0x81:
            _, _, on_us, off_us, repeat = struct.unpack("<BBHHH", report)
            if on_us:
                end = start + max(repeat, 1) * (on_us + off_us) * 1000
                if later:
                    end = min(end, later[0])
                intervals.append((start, end, (on_us + off_us) * 1000,
                                  on_us * 1000))
        else:
            seconds = effect_seconds(report)
            if seconds and math.isfinite(seconds):
                intervals.append((start, start + int(seconds * 1e9), 0, 0))
    return intervals


def playing_at(intervals, time_ns):
    return any(start <= time_ns < end
               and (not period or (time_ns - start) % period < on)
               for start, end, period, on in intervals)


def report_at(log, start_ns):
    return next(report for time_s, report in log
                if report is not None and int(time_s * 1e9) == start_ns)


def unstoppable(log):
    """As playing, but a pulse train always plays to its commanded end."""
    result = []
    for start, end, period, on in playing(log):
        if period:
            _, _, on_us, off_us, repeat = struct.unpack(
                "<BBHHH", report_at(log, start))
            end = start + max(repeat, 1) * (on_us + off_us) * 1000
        result.append((start, end, period, on))
    return result


def rumble_tail(tail_s):
    """As playing, with each rumble going on tail_s after its end."""
    def intervals(log):
        return [(start, end + int(tail_s * 1e9)
                 if report_at(log, start)[0] == 0x80 else end, period, on)
                for start, end, period, on in playing(log)]
    return intervals


def invisible(kind):
    """As playing, without what reports of kind play."""
    def intervals(log):
        return [interval for interval in playing(log)
                if report_at(log, interval[0])[0] != kind]
    return intervals


class FakeRecorder:
    """Motion samples on the controller's grid: noise of a count, and a
    vibration of 300 counts while the Model relayed something playing, as
    intervals (playing by default) says."""

    def __init__(self, dongle, vibrating=True, intervals=None):
        self.dongle = dongle
        self.vibrating = vibrating
        self.intervals = intervals

    def samples(self, start_ns, end_ns):
        intervals = ((self.intervals or playing)(self.dongle.log)
                     if self.vibrating else [])
        intervals = [interval for interval in intervals
                     if interval[1] >= start_ns and interval[0] <= end_ns]
        result = []
        for index in range(-(-start_ns // SAMPLE_NS), end_ns // SAMPLE_NS + 1):
            time_ns = index * SAMPLE_NS
            noise = (index * 2654435761) % 3 - 1
            shake = (300 if index % 2 else -300) \
                if playing_at(intervals, time_ns) else 0
            result.append(check.MotionSample(
                time_ns, (shake + noise, 0, check.ACCEL_COUNTS_PER_G),
                (shake, noise, 0)))
        return result


def model_haptics(vibrating=True, intervals=None, link_type=TimedLink):
    def open_haptics():
        link = link_type(None)
        return check.HapticParts(sc2.Client(link),
                                 FakeRecorder(link.dongle, vibrating,
                                              intervals),
                                 lambda: int(link.dongle.clock * 1e9),
                                 lambda: None)
    return open_haptics


class GoneLink(TimedLink):
    """Works for words_before_gone words, then every call fails as a
    removed node does."""

    words_before_gone = 60

    def __init__(self, dongle_nodes):
        super().__init__(dongle_nodes)
        self.words = 0

    def gone_check(self):
        if self.words >= self.words_before_gone:
            raise OSError(errno.ENODEV, "No such device")

    def send(self, word, in_session=False):
        self.gone_check()
        self.words += 1
        return super().send(word, in_session)

    def state(self):
        self.gone_check()
        return super().state()

    def quiet(self, seconds):
        self.gone_check()
        super().quiet(seconds)


class LostStopLink(TimedLink):
    """Loses every word once a rumble plays: each of Link.send's resends
    reaches the dongle and refreshes its session timer, none is taken or
    acknowledged. The session times out SESSION_TIMEOUT_S after the last
    word the dongle saw."""

    def __init__(self, dongle_nodes):
        super().__init__(dongle_nodes)
        self.last_word_s = 0.0

    def rumbling(self):
        return any(report is not None and report[0] == 0x80 and report[1]
                   for _, report in self.dongle.log)

    def send(self, word, in_session=False):
        if not self.rumbling():
            self.last_word_s = self.dongle.clock
            return super().send(word, in_session)
        for _ in range(sc2.MAX_RESENDS + 1):
            self.last_word_s = self.dongle.clock
            self.dongle.clock += sc2.RESEND_AFTER_S
        return None

    def quiet(self, seconds):
        end_s = self.dongle.clock + seconds
        if (self.dongle.active
                and end_s - self.last_word_s >= sc2.SESSION_TIMEOUT_S):
            self.dongle.clock = self.last_word_s + sc2.SESSION_TIMEOUT_S
            self.dongle.end_session()
            self.dongle.hello = 0
        self.dongle.clock = end_s


def right_answers():
    keys = {place: key for key, place in check.ANSWERS.items()}
    return [" "] + [keys[place] for _, place in check.LOCATE_EFFECTS]


def all_steps():
    return (len(check.DIGITAL_STEPS) + len(check.ANALOG_STEPS)
            + len(check.TURN_STEPS) + 1 + 1 + len(check.MEASURED_EFFECTS)
            + len(check.STOP_CANDIDATES) + len(check.LOCATE_EFFECTS))


class ScriptedRun(unittest.TestCase):
    """The whole check from start to report, as a person would run it."""

    def run_check(self, person=None, answers=None, keys=(), vibrating=True,
                  intervals=None, link_type=TimedLink):
        console = FakeConsole(person or Person(),
                              right_answers() if answers is None else answers,
                              keys)
        TimedLink.made.clear()
        with tempfile.TemporaryDirectory() as directory:
            stem = os.path.join(directory, "report")
            code = check.run_check(console, model_haptics(
                vibrating, intervals, link_type), stem, {"prefix": "test"})
            with open(stem + ".json", encoding="utf-8") as file:
                report = json.load(file)
            with open(stem + ".txt", encoding="utf-8") as file:
                summary = file.read()
        steps = {step["key"]: step for step in report["steps"]}
        return code, report, steps, summary, console

    def assert_results(self, steps, expected):
        """Every step passes, a stop probe is observed, unless expected
        says otherwise."""
        for key, step in steps.items():
            default = ("observed" if key.startswith(check.STOP_PREFIX)
                       else "pass")
            with self.subTest(step=key):
                self.assertEqual(step["result"], expected.get(key, default),
                                 step["reason"])

    def stop_probes(self, steps):
        return {key: step for key, step in steps.items()
                if key.startswith(check.STOP_PREFIX)}

    def test_every_step_passes(self):
        code, report, steps, summary, _ = self.run_check()
        self.assertEqual(len(report["steps"]), all_steps())
        self.assert_results(steps, {})
        self.assertEqual(code, 0, summary)
        self.assertEqual(report["result"], "PASS")
        self.assertIn("Result: PASS", summary)
        self.assertEqual(steps["pulse side 0"]["observation"]["pulses"], 5)
        self.assertEqual(steps["steer right"]["observation"]["pose"]
                         ["angles_deg"], [0.0, 90.0, 0.0])
        self.assertEqual(steps["press A (0x1)"]["seen"], [])
        self.assertTrue(report["haptics"]["calibration"]["detectable"])
        # The Model's dongle cuts the train with every candidate.
        probes = self.stop_probes(steps)
        self.assertEqual(len(probes), len(check.STOP_CANDIDATES))
        for key, step in probes.items():
            self.assertTrue(step["reason"].startswith("stopped early at"),
                            step["reason"])
            self.assertIn(f"  {key[len(check.STOP_PREFIX):]}: "
                          f"{step['reason']}", summary)
        self.assertIn("Stop probes, experiments outside the result:",
                      summary)

    def test_stop_probes_are_observations_not_verdicts(self):
        # A controller where no candidate ends the train, as docs/PROTOCOL.md
        # 9.5.1 says of 82 03 00 00.
        code, report, steps, summary, _ = self.run_check(
            intervals=unstoppable)
        self.assert_results(steps, {})
        for step in self.stop_probes(steps).values():
            self.assertTrue(step["reason"].startswith(
                "ran on to the commanded end"), step["reason"])
            self.assertFalse(step["observation"]["stopped_early"])
        self.assertEqual(report["result"], "PASS")
        self.assertEqual(code, 0, summary)

    def test_rumble_tails_of_the_safety_model(self):
        # Each rumble playing RUMBLE_TAIL_S past its stop, as the bound has.
        _, _, steps, summary, _ = self.run_check(
            intervals=rumble_tail(check.RUMBLE_TAIL_S))
        self.assert_results(steps, {})
        self.assertIn("Result: PASS", summary)

    def test_an_effect_the_sensors_cannot_see_is_not_measurable(self):
        _, report, steps, _, _ = self.run_check(intervals=invisible(0x84))
        sweeps = {f"sweep side {side}": "skipped" for side in range(4)}
        self.assert_results(steps, sweeps)
        for key in sweeps:
            self.assertTrue(steps[key]["reason"].startswith(
                "not measurable: no activity"), steps[key]["reason"])
        self.assertEqual(report["result"], "PASS")

    def test_the_dongle_disappears_during_the_haptics(self):
        code, report, steps, summary, _ = self.run_check(link_type=GoneLink)
        self.assertEqual(report["result"], "INCOMPLETE")
        self.assertEqual(report["error"],
                         repr(OSError(errno.ENODEV, "No such device")))
        failed = [step for step in report["steps"]
                  if step["result"] == "fail"]
        self.assertEqual(len(failed), 1)
        self.assertEqual(failed[0]["section"], check.SECTIONS[4])
        self.assertTrue(failed[0]["reason"].startswith(
            "stopped by an error: OSError"), failed[0]["reason"])
        self.assertIn("link_log", report["haptics"])
        self.assertIn("Result: INCOMPLETE", summary)
        self.assertEqual(code, 1)

    def test_a_node_disappears_during_a_step(self):
        person = Person({"press Y (0x8)": ["unplug"]})
        code, report, steps, summary, _ = self.run_check(person)
        self.assertEqual(report["result"], "INCOMPLETE")
        self.assertEqual(steps["press Y (0x8)"]["result"], "fail")
        self.assertIn("No such device", steps["press Y (0x8)"]["reason"])
        self.assertIn("FAIL press Y (0x8): stopped by an error", summary)
        self.assertEqual(code, 1)

    def test_every_effect_ends_within_a_second(self):
        # The guard of test_sc2link.py, over what the check relayed.
        self.run_check()
        played = list(effects(TimedLink.made[0].dongle.log))
        self.assertGreater(len(played), 40)
        for report, seconds in played:
            with self.subTest(report=report.hex(" ")):
                self.assertLessEqual(seconds, 1.0)

    def test_a_slip_then_the_right_input_passes(self):
        person = Person({"press Y (0x8)": [check.BIT["X"], "right"]})
        code, _, steps, _, console = self.run_check(person)
        self.assert_results(steps, {})
        self.assertEqual(steps["press Y (0x8)"]["seen"],
                         [{"input": "the X button", "count": 1}])
        self.assertIn("I see: the X button. Waiting for: the Y button.",
                      console.lines)
        self.assertEqual(code, 0)

    def test_a_wrong_input_then_f_fails_with_what_was_seen(self):
        person = Person({"press A (0x1)": [check.BIT["B"], check.BIT["B"],
                                           "fail"]})
        code, _, steps, summary, console = self.run_check(person)
        self.assert_results(steps, {"press A (0x1)": "fail"})
        self.assertEqual(steps["press A (0x1)"]["seen"],
                         [{"input": "the B button", "count": 2}])
        reason = ("f pressed: doing exactly this, not seen; seen instead: "
                  "the B button (2x)")
        self.assertEqual(steps["press A (0x1)"]["reason"], reason)
        self.assertIn(f"FAIL press A (0x1): {reason}", summary)
        # The step itself never gave up: the right button still passes the
        # next step.
        self.assertEqual(steps["press B (0x2)"]["result"], "pass")
        self.assertEqual(code, 1)

    def test_nothing_seen_then_f_fails_with_an_empty_list(self):
        # Nothing happens for the first step; the reminder names both keys.
        person = Person({"press A (0x1)": []})
        code, _, steps, _, console = self.run_check(person, keys=[(25, "f")])
        self.assert_results(steps, {"press A (0x1)": "fail"})
        self.assertEqual(steps["press A (0x1)"]["seen"], [])
        self.assertEqual(steps["press A (0x1)"]["reason"],
                         "f pressed: doing exactly this, not seen; no other "
                         "input seen")
        self.assertIn("Still waiting for: Press the A button and let go. "
                      + check.KEY_LEGEND, console.lines)
        self.assertEqual(code, 1)

    def test_a_skipped_step(self):
        person = Person({"press R5 (0x100)": ["skip"]})
        code, _, steps, _, _ = self.run_check(person)
        self.assert_results(steps, {"press R5 (0x100)": "skipped"})
        self.assertEqual(code, 0)

    def test_haptics_the_sensors_cannot_see(self):
        code, report, steps, summary, _ = self.run_check(vibrating=False)
        measured = {effect.key for effect in check.MEASURED_EFFECTS}
        measured |= {check.STOP_PREFIX + candidate.key
                     for candidate in check.STOP_CANDIDATES}
        self.assert_results(steps, {key: "skipped"
                                    for key in measured | {"calibration"}})
        for key in measured:
            self.assertEqual(steps[key]["reason"], "not measurable")
        self.assertIn("not measurable", steps["calibration"]["reason"])
        self.assertFalse(report["haptics"]["calibration"]["detectable"])
        self.assertEqual(len(report["steps"]), all_steps())
        self.assertEqual(code, 0, summary)

    def test_a_stuck_sensor_is_a_finding(self):
        # The left grip reads touched from the start; s records it as stuck.
        person = Person(held=check.BIT["LGrip"])
        code, report, steps, _, _ = self.run_check(person, keys=[(25, "s")])
        self.assertEqual(report["findings"],
                         [{"section": check.SECTIONS[0],
                           "stuck_active": ["LGrip"]}])
        self.assert_results(steps, {"touch LGrip": "fail"})
        self.assertIn("stuck", steps["touch LGrip"]["reason"])
        self.assertEqual(code, 1)

    def test_ctrl_c_leaves_a_partial_report(self):
        person = Person({"press X (0x4)": ["interrupt"]})
        code, report, steps, summary, _ = self.run_check(person)
        self.assertTrue(report["interrupted"])
        self.assertEqual(report["result"], "INCOMPLETE")
        self.assertEqual(list(steps), ["press A (0x1)", "press B (0x2)"])
        self.assertIn("stopped with Ctrl+C", summary)
        self.assertEqual(code, 1)

    def test_ctrl_c_during_haptics_ends_the_session(self):
        code, report, steps, _, _ = self.run_check(answers=[" ",
                                                            "interrupt"])
        self.assertTrue(report["interrupted"])
        self.assertIn("calibration", steps)
        dongle = TimedLink.made[0].dongle
        self.assertFalse(dongle.active)
        self.assertEqual(dongle.end_word & sc2.DATA, sc2.BYE)
        self.assertEqual(code, 1)


# ---- the parts --------------------------------------------------------------

class Coverage(unittest.TestCase):
    def test_every_documented_bit_has_a_step(self):
        expected = 0
        for step in check.DIGITAL_STEPS:
            self.assertFalse(expected & step.expected, step.key)
            expected |= step.expected
        documented = sum(mask for mask, _ in sc2.BUTTONS)
        documented |= check.PADS_LEFT_TOUCH | check.PADS_RIGHT_TOUCH
        self.assertEqual(expected, documented)

    def test_the_bit_table_is_the_protocol_documents(self):
        if not os.path.exists(PROTOCOL):
            self.skipTest("docs/PROTOCOL.md is not beside the tool")
        with open(PROTOCOL, encoding="utf-8") as file:
            text = file.read()
        table = text.split("### 8.1 Button bitfield")[1].split("\n## ")[0]
        masks = {int(mask, 16) for mask in
                 re.findall(r"^0x([0-9a-fA-F]{8}) ", table, re.MULTILINE)}
        self.assertEqual(masks, {mask for mask, _ in sc2.BUTTONS})
        self.assertIn("bit 0 left pad touched, bit 1 right pad touched", text)

    def test_every_word_type_and_side_is_measured(self):
        kinds = collections.defaultdict(set)
        for effect in check.MEASURED_EFFECTS:
            for words in effect.messages:
                report = sc2.build_report(words[0], list(words[1:]))
                kinds[report[0]].add(report[1])
        self.assertEqual(set(kinds), {0x80, 0x81, 0x82, 0x83, 0x84, 0x85})
        for kind in (0x81, 0x82, 0x83, 0x84, 0x85):
            self.assertEqual(kinds[kind], {0, 1, 2, 3}, hex(kind))
        self.assertEqual({effect.side for effect, _ in check.LOCATE_EFFECTS},
                         {0, 1, 2, 3})

    def test_constructors_refuse_effects_over_a_second(self):
        for make in (lambda: check.pulse_effect("x", 0, 30000, 30000, 17),
                     lambda: check.tone_effect("x", 0, 0, 160, 0),
                     lambda: check.tone_effect("x", 0, 0, 160, 1001),
                     lambda: check.sweep_effect("x", 0, 0, 0, 100, 200),
                     lambda: check.command_effect("x", 0, 3, 0),
                     lambda: check.script_effect("x", 0, 1, 0),
                     lambda: check.rumble_effect("x", 1, 1, 0.5),
                     lambda: check.rumble_effect("x", 1, 1, 0.031),
                     lambda: check.stop_candidate(
                         "x", sc2.encode_pulse(0, 0xFFFF, 0xFFFF, 0xFFFF))):
            with self.assertRaises(ValueError):
                make()
        self.assertAlmostEqual(check.STOP_TRAIN.total_s, 0.9)

    def test_measured_frequencies_stay_off_the_sampling_blind_spot(self):
        # Samples 4.032 ms apart see a vibration at f through
        # |sin(pi f T)|, zero at 248 Hz.
        def sensitivity(frequency):
            return abs(math.sin(math.pi * frequency * check.SAMPLE_PERIOD_S))
        for effect in check.MEASURED_EFFECTS:
            report = sc2.build_report(effect.messages[0][0],
                                      list(effect.messages[0][1:]))
            if report[0] == 0x84:
                low, high = struct.unpack_from("<HH", report, 5)
            elif report[0] == 0x83:
                low = high = struct.unpack_from("<H", report, 3)[0]
            else:
                continue
            with self.subTest(effect=effect.key):
                for frequency in range(low, high + 1):
                    self.assertGreater(sensitivity(frequency), 0.5, frequency)


class Decoding(unittest.TestCase):
    def test_snapshot_reads_the_decoders_fields(self):
        values = {
            "controls": {sc2.ABS_THROTTLE: -32768, sc2.ABS_RUDDER: 0x1234,
                         sc2.ABS_X: -1000, sc2.ABS_Z: 2000, sc2.ABS_RX: 3000,
                         sc2.ABS_RY: -4000, sc2.ABS_WHEEL: 1,
                         sc2.ABS_GAS: -1, sc2.ABS_BRAKE: 0x4000},
            "pads": {sc2.ABS_X: 100, sc2.ABS_Z: -200, sc2.ABS_RY: 300,
                     sc2.ABS_THROTTLE: -400, sc2.ABS_WHEEL: 3},
            "motion": {sc2.ABS_X: 1, sc2.ABS_Z: 2, sc2.ABS_RX: 3,
                       sc2.ABS_RY: 4, sc2.ABS_THROTTLE: 5, sc2.ABS_RUDDER: 6},
        }
        got = check.snapshot_from(values, "controls", 7)
        controls = sc2.decode_controls(values["controls"])
        pads = sc2.decode_pads(values["pads"])
        motion = sc2.decode_motion(values["motion"])

        def raw(fractions, scale):
            return tuple(round(value * scale) for value in fractions)
        self.assertEqual(got.buttons, 0x12348000 | check.PADS_LEFT_TOUCH
                         | check.PADS_RIGHT_TOUCH)
        self.assertTrue(controls["buttons"].startswith("0x12348000"))
        self.assertEqual(pads["touch"], "left+right")
        self.assertEqual(raw(got.left_stick, check.STICK_FULL_SCALE),
                         controls["left_stick"])
        self.assertEqual(raw(got.right_stick, check.STICK_FULL_SCALE),
                         controls["right_stick"])
        self.assertEqual(raw(got.triggers, check.TRIGGER_FULL_SCALE),
                         controls["triggers"])
        self.assertEqual(raw(got.left_pad, check.PAD_FULL_SCALE),
                         pads["left_pad"][:2])
        self.assertEqual(raw(got.right_pad, check.PAD_FULL_SCALE),
                         pads["right_pad"][:2])
        self.assertEqual(raw(got.accel, check.ACCEL_COUNTS_PER_G),
                         motion["accel"])
        self.assertEqual(raw(got.gyro, 1 / check.GYRO_DEGREES_PER_COUNT),
                         motion["gyro"])
        self.assertTrue(got.connected)

    def test_classification(self):
        self.assertEqual(check.stick_reading((-0.9, 0.1)),
                         ("full", "left", False))
        self.assertEqual(check.stick_reading((0.3, 0.5)),
                         ("half", "up", False))
        self.assertEqual(check.stick_reading((0.85, 0.45)),
                         ("full", "right", True))
        self.assertEqual(check.stick_reading((0.1, -0.05))[0], "centre")
        self.assertEqual(check.stick_reading((0.0, -0.17))[:2],
                         ("between", "down"))
        self.assertEqual([check.trigger_zone(value)
                          for value in (0.05, 0.15, 0.5, 0.9)],
                         ["released", "between", "half", "full"])
        self.assertEqual([check.pad_region(position) for position in
                          ((-0.8, 0.1), (0.8, -0.2), (0.1, 0.7), (0, -0.9),
                           (0.2, 0.3), (0.7, 0.7))],
                         ["left edge", "right edge", "top edge",
                          "bottom edge", "middle", "corner"])

    def test_other_input(self):
        stuck = check.Stuck()
        self.assertEqual(check.other_input(
            snapshot(buttons=check.BIT["B"] | check.BIT["LGrip"]),
            check.TOUCH_BITS, (), stuck), "the B button")
        self.assertEqual(check.other_input(
            snapshot(buttons=check.BIT["LGrip"], right_stick=(0.1, -0.5)),
            check.TOUCH_BITS, ("left stick",), stuck),
            "the right stick pushed down")
        self.assertEqual(check.other_input(
            snapshot(left_stick=(0.9, 0), triggers=(0.25, 0)),
            check.TOUCH_BITS, ("left stick",), stuck), "")
        self.assertEqual(check.other_input(
            snapshot(buttons=check.BIT["B"]), 0, (),
            check.Stuck(check.BIT["B"])), "")


class Watchers(unittest.TestCase):
    STUCK = check.Stuck()

    def test_a_press_with_its_companions(self):
        step = check.press("LPadClick", "")
        touch = check.BIT["LPadTouch"] | check.PADS_LEFT_TOUCH
        frames = (ticks(0, 0.1, buttons=touch)
                  + ticks(0.1, 0.2, buttons=touch | step.expected)
                  + ticks(0.2, 0.3, buttons=touch) + ticks(0.3, 0.5))
        outcome = feed_all(check.BitWatcher(step, self.STUCK), frames)
        self.assertTrue(outcome.matched)
        self.assertEqual(outcome.observation["seen"], ["LPadClick"])
        self.assertEqual(outcome.observation["companions_seen"],
                         ["LPadTouch", "PadsLeftTouch"])

    def test_another_bit_is_wrong_at_once(self):
        step = check.press("A", "")
        frames = ticks(0, 0.1, buttons=check.BIT["A"] | check.BIT["B"])
        outcome = feed_all(check.BitWatcher(step, self.STUCK), frames[:1])
        self.assertFalse(outcome.matched)
        self.assertEqual(outcome.signature, "the B button")

    def test_both_copies_of_a_pad_touch(self):
        step = check.DIGITAL_STEPS[24]
        self.assertEqual(step.key, "touch LPadTouch+PadsLeftTouch")
        # The pads node's frame comes a little after the controls frame.
        frames = [snapshot(0.100, buttons=check.BIT["LPadTouch"]),
                  snapshot(0.101, "pads", buttons=step.expected),
                  snapshot(0.300, buttons=check.PADS_LEFT_TOUCH),
                  snapshot(0.301, "pads")] + ticks(0.31, 0.5)
        self.assertTrue(feed_all(check.BitWatcher(step, self.STUCK),
                                 frames).matched)
        frames = ticks(0, 0.1, buttons=check.BIT["LPadTouch"]) \
            + ticks(0.1, 0.3)
        outcome = feed_all(check.BitWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature, "a touch on the left "
                         "trackpad without the pads node's left trackpad "
                         "touch")

    def test_a_stick_pushed_and_let_go(self):
        step = check.StickStep("", "", "left", "full", "left")
        frames = (ticks(0, 0.5, left_stick=(-0.95, 0.05))
                  + ticks(0.5, 0.6, left_stick=(-0.4, 0)) + ticks(0.6, 0.7))
        outcome = feed_all(check.StickWatcher(step, self.STUCK), frames)
        self.assertTrue(outcome.matched)
        self.assertEqual(outcome.observation["position"], [-0.95, 0.05])

    def test_stick_verdicts(self):
        full = check.StickStep("", "", "left", "full", "left")
        half = check.StickStep("", "", "left", "half", "up")
        cases = [
            (full, ("full", "right", False, 0.3), "the left stick pushed "
             "right"),
            (full, ("full", "left", True, 0.3), "the left stick pushed "
             "left at an angle"),
            (full, ("half", "left", False, 0.5), None),
            (full, ("half", "left", False, 1.0), "the left stick pushed "
             "only part way left"),
            (half, ("full", "up", False, 0.3), "the left stick pushed all "
             "the way up"),
            (half, ("half", "up", False, 0.29), None),
            (half, ("half", "up", False, 0.3), ""),
        ]
        for step, reading, expected in cases:
            with self.subTest(reading=reading):
                self.assertEqual(check.stick_verdict(step, *reading),
                                 expected)

    def test_a_stick_resting_off_centre(self):
        step = check.StickStep("", "", "right", "centre")
        frames = ticks(0, 1.5, right_stick=(0.3, 0.0))
        outcome = feed_all(check.StickWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature,
                         "the right stick rests off centre (right)")
        self.assertTrue(feed_all(check.StickWatcher(step, self.STUCK),
                                 ticks(0, 1)).matched)

    def test_a_trigger_pulled_too_far_for_halfway(self):
        step = check.TriggerStep("", "", "left", "half")
        frames = ticks(0, 0.5, triggers=(0.95, 0),
                       buttons=check.BIT["LTriggerClick"])
        outcome = feed_all(check.TriggerWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature, "a left trigger click")
        frames = ticks(0, 0.5, triggers=(0.95, 0))
        outcome = feed_all(check.TriggerWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature, "the left trigger pulled all the "
                         "way")

    def test_a_pad_touch_on_the_wrong_side_of_the_pad(self):
        step = check.PadStep("", "", "right", "left")
        touched = check.PADS_RIGHT_TOUCH | check.BIT["RPadTouch"]
        frames = ticks(0, 0.2, buttons=touched, right_pad=(0.8, 0.0)) \
            + ticks(0.2, 0.4)
        outcome = feed_all(check.PadWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature,
                         "the right trackpad touched at the right edge")
        # A thumb on the other pad is allowed, a click there is not.
        frames = ticks(0, 0.2, buttons=check.PADS_LEFT_TOUCH)
        self.assertIsNone(feed_all(check.PadWatcher(step, self.STUCK), frames))
        frames = ticks(0, 0.2, buttons=check.BIT["LPadClick"])
        outcome = feed_all(check.PadWatcher(step, self.STUCK), frames)
        self.assertEqual(outcome.signature, "a left trackpad click")


class Motion(unittest.TestCase):
    REST = check.RestReference((1.0, -2.0, 0.5), (0.0, 0.0, 1.0))

    def turn(self, axis, degrees_per_s, pose, bias=(1.0, -2.0, 0.5)):
        rate = [bias[0], bias[1], bias[2]]
        rate[axis] += degrees_per_s
        turning = [snapshot(0.1 + index * 0.004, "motion", gyro=tuple(rate))
                   for index in range(125)]
        held = [snapshot(0.6 + index * 0.004, "motion", gyro=bias,
                         accel=pose) for index in range(150)]
        flat = [snapshot(1.2 + index * 0.004, "motion", gyro=bias)
                for index in range(150)]
        return turning + held + flat

    def test_integration_uses_the_timestamps(self):
        self.assertEqual(check.integrate([0, 0, 0], (90, -10, 0), 500_000_000),
                         [45.0, -5.0, 0.0])

    def test_a_steering_turn(self):
        step = check.TURN_STEPS[0]
        watcher = check.TurnWatcher(step, self.REST)
        outcome = feed_all(watcher, self.turn(1, 180, (-1.0, 0.0, 0.0)))
        self.assertTrue(outcome.matched, outcome.signature)
        angles = outcome.observation["pose"]["angles_deg"]
        self.assertAlmostEqual(angles[1], 90.0, delta=1.0)
        self.assertEqual(outcome.observation["pose"]["accel_g"],
                         [-1.0, 0.0, 0.0])

    def test_a_turn_the_wrong_way(self):
        step = check.TURN_STEPS[0]
        outcome = feed_all(check.TurnWatcher(step, self.REST),
                           self.turn(1, -180, (1.0, 0.0, 0.0)))
        self.assertEqual(outcome.signature, "a steering turn to the left "
                         "(gyroscope Y-)")
        outcome = feed_all(check.TurnWatcher(step, self.REST),
                           self.turn(0, 180, (0.0, 1.0, 0.0)))
        self.assertEqual(outcome.signature, "the far edge lifted (gyroscope "
                         "X+)")

    def test_turn_verdicts(self):
        step = check.TURN_STEPS[1]
        self.assertEqual(check.turn_verdict([90, 10, -20], step), "")
        self.assertEqual(check.turn_verdict([150, 0, 0], step),
                         "more than a quarter turn")
        self.assertEqual(check.turn_verdict([80, 50, 0], step),
                         "a turn about two axes at once")

    def test_the_accelerometer_verdict(self):
        poses = {"steer right": (-0.95, 0.1, 0.2), "tilt up": (0.0, 0.8, 0.5)}
        verdict, checks = check.accel_verdict((0.02, -0.03, 1.04), poses)
        self.assertEqual(verdict, "pass", checks)
        verdict, checks = check.accel_verdict((0.0, 0.0, -1.0), poses)
        self.assertEqual(verdict, "fail")
        self.assertEqual(checks["lying flat"]["result"], "fail")
        verdict, checks = check.accel_verdict((0.0, 0.0, 1.0),
                                              {"steer right": (-1, 0, 0)})
        self.assertEqual(verdict, "skipped")
        self.assertEqual(checks["held pose 'tilt up'"]["reason"],
                         "not measured")

    def test_rest_and_flat(self):
        still = [snapshot(index * 0.01, gyro=(1, 1, 1), accel=(0, 0.1, 1))
                 for index in range(101)]
        outcome = feed_all(check.RestWatcher(1.0), still)
        self.assertEqual(outcome.observation["gyro_bias_deg_s"],
                         (1.0, 1.0, 1.0))
        tilted = [snapshot(index * 0.01, gyro=(1, 1, 1), accel=(0, 1, 0))
                  for index in range(101)]
        self.assertIsNone(feed_all(check.RestWatcher(0.5, self.REST), tilted))


class Policy(unittest.TestCase):
    def test_other_inputs_are_counted_in_first_seen_order(self):
        seen = []
        for signature in ("the B button", "the X button", "the B button"):
            seen = check.count_seen(seen, signature)
        self.assertEqual(seen, [{"input": "the B button", "count": 2},
                                {"input": "the X button", "count": 1}])
        self.assertEqual(check.declared_fault(seen),
                         "f pressed: doing exactly this, not seen; seen "
                         "instead: the B button (2x); the X button (1x)")

    def test_step_targets(self):
        self.assertEqual(check.step_target(check.DIGITAL_STEPS[0]),
                         "the A button")
        self.assertEqual(check.step_target(check.ANALOG_STEPS[1]),
                         "the left stick all the way to the left")
        self.assertEqual(check.step_target(check.TURN_STEPS[0]),
                         "a steering turn to the right")

    def test_step_neutral(self):
        self.assertEqual(check.step_neutral(check.DIGITAL_STEPS[0])[1],
                         check.TOUCH_BITS)
        self.assertEqual(check.step_neutral(check.DIGITAL_STEPS[-1])[1], 0)
        centre = check.ANALOG_STEPS[0]
        self.assertEqual(check.step_neutral(centre)[2], ("left stick",))


def vibration(start_s, end_s, samples_s=(0.0, 2.0), on_s=None, period_s=None,
              amplitude=300):
    """Samples with noise and a vibration from start_s to end_s, in pulses
    of on_s every period_s when given."""
    result = []
    first, last = (int(seconds * 1e9) // SAMPLE_NS for seconds in samples_s)
    for index in range(first, last):
        time_ns = index * SAMPLE_NS
        offset = time_ns / 1e9 - start_s
        active = start_s <= time_ns / 1e9 < end_s and (
            period_s is None or offset % period_s < on_s)
        shake = (amplitude if index % 2 else -amplitude) if active else 0
        noise = (index * 2654435761) % 3 - 1
        result.append(check.MotionSample(time_ns, (shake + noise, 0, 16384),
                                         (noise, 0, 0)))
    return result


class Measurement(unittest.TestCase):
    def test_envelope_is_centred(self):
        points = [(index * 4_000_000, 10.0 if 10 <= index < 20 else 0.0)
                  for index in range(30)]
        smooth = check.envelope(points, 0.012)
        span = check.active_span(smooth, 1.0, 0, 10**9)
        self.assertEqual(span, (9 * 4_000_000, 20 * 4_000_000))

    def test_calibration(self):
        noise = vibration(9, 9, (0.0, 1.0))
        result = check.calibrate(noise, vibration(1.2, 1.5, (1.0, 2.0)),
                                 int(1.2e9), int(1.5e9))
        self.assertTrue(result["detectable"])
        self.assertEqual(result["channel"], "accel")
        quiet = check.calibrate(noise, vibration(9, 9, (1.0, 2.0)),
                                int(1.2e9), int(1.5e9))
        self.assertFalse(quiet["detectable"])
        # The short calibration rumble: 25 ms and a stop's round trip.
        short = check.calibrate(noise, vibration(1.18, 1.235, (1.0, 2.0)),
                                int(1.21e9), int(1.405e9))
        self.assertTrue(short["detectable"])
        # One bump is not a level.
        bump = check.calibrate(noise, vibration(1.2, 1.205, (1.0, 2.0)),
                               int(1.2e9), int(1.5e9))
        self.assertFalse(bump["detectable"])

    def threshold(self):
        noise = vibration(9, 9, (0.0, 1.0))
        return check.noise_threshold(check.envelope(check.differences(
            noise, "accel")))

    def test_a_tone_of_the_commanded_length(self):
        samples = vibration(0.3, 0.6)
        measured = check.measure_effect(samples, "accel", self.threshold(),
                                        int(0.3e9), int(0.6e9), 0.3)
        self.assertEqual(check.effect_verdict(measured), ("pass", ""))
        self.assertAlmostEqual(measured["duration_ms"], 300, delta=15)

    def test_a_tone_too_long_and_not_quiet_after(self):
        samples = vibration(0.3, 0.8)
        measured = check.measure_effect(samples, "accel", self.threshold(),
                                        int(0.3e9), int(0.6e9), 0.3)
        verdict, reason = check.effect_verdict(measured)
        self.assertEqual(verdict, "fail")
        self.assertIn("commanded 300.0 ms", reason)
        self.assertIn("not quiet", reason)

    def test_pulses_are_counted(self):
        samples = vibration(0.3, 0.7, on_s=0.03, period_s=0.08)
        measured = check.measure_effect(samples, "accel", self.threshold(),
                                        int(0.3e9), int(0.7e9), 0.35, 5,
                                        0.025)
        self.assertEqual(measured["pulses"], 5)
        self.assertEqual(check.effect_verdict(measured), ("pass", ""))

    def rings(self, edges_s, ring_s=0.008):
        """A voice coil ringing ring_s after each step of the drive."""
        samples = vibration(9, 9)
        return [dataclasses.replace(sample, accel=(
            (300 if index % 2 else -300) + sample.accel[0], 0, 16384))
            if any(0 <= sample.time_ns / 1e9 - edge < ring_s
                   for edge in edges_s) else sample
            for index, sample in enumerate(samples)]

    def test_a_train_rendered_as_sc26re_renders_it(self):
        # +gain on, -gain off, back to 0 after the last period: an edge at
        # each change, the last at repeat periods.
        effect = check.pulse_effect("x", 0, 30000, 50000, 5)
        edges = [0.3 + period * 0.08 + offset for period in range(5)
                 for offset in (0.0, 0.03)] + [0.3 + 5 * 0.08]
        measured = check.measure_effect(
            self.rings(edges), "accel", self.threshold(), int(0.3e9),
            int(0.3e9 + effect.total_s * 1e9), effect.active_s,
            effect.pulses, effect.pulse_gap_s)
        self.assertEqual(measured["pulses"], 6)
        # The last ring and half the envelope's window after the last edge.
        self.assertAlmostEqual(measured["offset_ms"], 414, delta=5)
        self.assertEqual(check.effect_verdict(measured), ("pass", ""))
        # One edge more, which also ends past the last period, disagrees
        # with the command.
        measured = check.measure_effect(
            self.rings(edges + [0.76]), "accel", self.threshold(),
            int(0.3e9), int(0.3e9 + effect.total_s * 1e9), effect.active_s,
            effect.pulses, effect.pulse_gap_s)
        verdict, reason = check.effect_verdict(measured)
        self.assertEqual(verdict, "fail")
        self.assertIn("7 pulses, commanded 5", reason)
        self.assertIn("commanded 350.0 to 400.0 ms", reason)

    def rumble(self, tail_s):
        """A rumble from its word at 0.27 s, acknowledged at 0.3 s, stopped
        25 ms later by a word acknowledged at 0.355 s, as fire() records it,
        playing tail_s past the stop word."""
        stop_ns = int(0.355e9)
        samples = vibration(0.27, 0.325 + tail_s)
        return check.measure_effect(
            samples, "accel", self.threshold(), int(0.3e9),
            stop_ns + int(check.RUMBLE_AFTER_STOP_S * 1e9), 0.055)

    def test_a_rumble_within_its_tail(self):
        for tail_s in (0.0, 0.06, 0.1):
            with self.subTest(tail_s=tail_s):
                self.assertEqual(check.effect_verdict(self.rumble(tail_s)),
                                 ("pass", ""))
        verdict, reason = check.effect_verdict(self.rumble(0.25))
        self.assertEqual(verdict, "fail")
        self.assertIn("commanded 55.0 to 195.0 ms", reason)

    def test_an_effect_without_activity_is_not_measurable(self):
        measured = check.measure_effect(vibration(9, 9), "accel",
                                        self.threshold(), int(0.3e9),
                                        int(0.6e9), 0.3)
        verdict, reason = check.effect_verdict(measured)
        self.assertEqual(verdict, "skipped")
        self.assertTrue(reason.startswith("not measurable: no activity "
                                          "above the noise threshold"))

    def test_nothing_where_nothing_should_play(self):
        samples = vibration(9, 9)
        measured = check.measure_effect(samples, "accel", self.threshold(),
                                        int(0.3e9), int(0.3e9), 0.0)
        self.assertEqual(check.effect_verdict(measured), ("pass", ""))
        measured = check.measure_effect(vibration(0.3, 0.33), "accel",
                                        self.threshold(), int(0.3e9),
                                        int(0.3e9), 0.0)
        self.assertEqual(check.effect_verdict(measured)[0], "fail")

    def test_stops_are_observed(self):
        threshold = self.threshold()
        arguments = ("accel", threshold, int(0.3e9), int(0.6e9), int(1.2e9),
                     check.STOP_TRAIN.active_s)
        verdict, reason, measured = check.measure_stop(vibration(0.3, 0.62),
                                                       *arguments)
        self.assertEqual(verdict, "observed")
        self.assertTrue(reason.startswith("stopped early at 3"), reason)
        self.assertTrue(measured["stopped_early"])
        verdict, reason, measured = check.measure_stop(vibration(0.3, 1.2),
                                                       *arguments)
        self.assertEqual(verdict, "observed")
        self.assertTrue(reason.startswith("ran on to the commanded end"),
                        reason)
        self.assertFalse(measured["stopped_early"])
        unseen = check.measure_stop(vibration(9, 9), *arguments)
        self.assertEqual(unseen[0], "skipped")


def event_pipe(events):
    """A non-blocking pipe holding struct input_event records."""
    read_fd, write_fd = os.pipe()
    os.set_blocking(read_fd, False)
    os.write(write_fd, b"".join(sc2.INPUT_EVENT.pack(*event)
                                for event in events))
    os.close(write_fd)
    return read_fd


class Nodes(unittest.TestCase):
    """The glue between event nodes and frames, over pipes."""

    EVENTS = [(1, 0, sc2.EV_ABS, sc2.ABS_X, 5),
              (1, 0, sc2.EV_ABS, sc2.ABS_WHEEL, 1),
              (1, 0, sc2.EV_SYN, sc2.SYN_REPORT, 0),
              (1, 4000, sc2.EV_ABS, sc2.ABS_X, 6),
              (1, 4000, sc2.EV_SYN, sc2.SYN_REPORT, 0)]

    def test_frames_from_events(self):
        fd = event_pipe(self.EVENTS)
        console = object.__new__(check.TerminalConsole)
        console.role_of, console.kernel_time = {fd: "pads"}, {fd: True}
        console.pending, console.resyncing = {fd: {}}, set()
        try:
            frames = console.read_frames(fd)
        finally:
            os.close(fd)
        self.assertEqual(frames, [
            ("pads", 1_000_000_000, {sc2.ABS_X: 5, sc2.ABS_WHEEL: 1}),
            ("pads", 1_004_000_000, {sc2.ABS_X: 6})])

    def test_the_motion_recorder_keeps_every_frame(self):
        recorder = object.__new__(check.MotionRecorder)
        recorder.fd = event_pipe(self.EVENTS)
        recorder.kernel_time, recorder.values = True, {sc2.ABS_RX: 16384}
        recorder.recorded, recorder.dropped = [], 0
        recorder.lock, recorder.running = check.threading.Lock(), True
        recorder.thread = check.threading.Thread(target=recorder.run)
        recorder.thread.start()
        deadline = check.time.monotonic() + 2
        while (len(recorder.samples(0, 2 * 10**9)) < 2
               and check.time.monotonic() < deadline):
            check.time.sleep(0.01)
        recorder.stop()
        self.assertEqual(recorder.samples(0, 2 * 10**9), [
            check.MotionSample(1_000_000_000, (5, 0, 16384), (0, 0, 0)),
            check.MotionSample(1_004_000_000, (6, 0, 16384), (0, 0, 0))])


class Safety(unittest.TestCase):
    def test_ctrl_c_inside_a_message_waits_for_the_timeout(self):
        link = TimedLink(None)
        parts = check.HapticParts(sc2.Client(link), None,
                                  lambda: int(link.dongle.clock * 1e9),
                                  lambda: None)
        runner = check.HapticRunner(parts, print)
        runner.in_message = True
        sent = len(link.dongle.log)
        runner.stop_safely()
        self.assertEqual(len(link.dongle.log), sent)
        self.assertGreaterEqual(link.dongle.clock, sc2.SILENCE_S)

    def test_a_lost_rumble_stop_still_ends_within_a_second(self):
        # Every resend of the stop refreshes the dongle's session timer;
        # the timeout's stop comes only after the last of them.
        link = LostStopLink(None)
        parts = check.HapticParts(sc2.Client(link), None,
                                  lambda: int(link.dongle.clock * 1e9),
                                  lambda: None)
        runner = check.HapticRunner(parts, print)
        self.assertTrue(runner.send([sc2.NOP], "NOP"))
        self.assertIsNone(runner.fire(check.CALIBRATION_EFFECT))
        log = link.dongle.log
        start = next(index for index, (_, report) in enumerate(log)
                     if report is not None and report[0] == 0x80)
        end = next(time for time, report in log[start + 1:]
                   if report is None or report[0] == 0x80)
        seconds = end - log[start][0] + check.RUMBLE_AFTER_STOP_S
        self.assertGreater(seconds, 0.9)
        self.assertLessEqual(seconds, check.MAX_EFFECT_S)
        self.assertLessEqual(seconds, check.rumble_worst_s(
            check.CALIBRATION_EFFECT.stop_after_s) + 1e-9)
        self.assertAlmostEqual(check.LOST_WORD_S, (sc2.MAX_RESENDS + 1)
                               * sc2.RESEND_AFTER_S)


if __name__ == "__main__":
    unittest.main()
