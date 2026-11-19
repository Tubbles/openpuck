"""The interactive controller check behind `sc2link_test.py check`.

It walks a person through the whole Steam Controller (2026) as an OpenPuck
dongle in SC2 Link mode presents it, one plain instruction at a time:

1. Digital inputs: every bit of the controls node's buttons and the pads
   node's touch copies (docs/PROTOCOL.md 8.1 and 9.5), one at a time.
2. Analog inputs, roughly: sticks, triggers and trackpad positions.
3. Gyroscope: three quarter turns starting flat on the table.
4. Accelerometer: gravity at rest and in the held poses of section 3, one
   verdict and no extra steps.
5. Haptics measured by the controller's own motion sensors, controller flat
   and hands off: every word type and side, then candidate stops of a pulse
   train. Skipped when a calibration rumble does not show above the noise;
   an effect that shows nothing is "not measurable", not a fault. The stop
   probes are experiments: "observed", outside the overall result.
6. Haptics located by the person: one effect per side value, one key each.

An input step passes when it sees the expected input, after anything else
has been let go. The tool never fails a step by itself: other input is shown
("I see: ...") and the step keeps waiting. f records a fault ("I am doing
exactly this and the tool does not see it"), s skips (the control is missing
or broken, or the person cannot do it). Every step keeps the other inputs
seen while waiting, with counts, so a swapped mapping reads in the report.
Ctrl+C ends the session the way the tool's other commands do; Ctrl+C or a
dongle that disappears still writes the report: STEM.json with every step and
the observation that decided it, STEM.txt a summary, STEM from --report or
sc2link-check-<UTC time> in the working directory.

Raw value conventions (from sources, the check is what verifies them): stick
and trackpad X grows to the right and Y upwards, as SDL's triton driver
negates Y into its down-positive axes; a full trigger pull reads near 0x8000
(OpenPuck/triton.h); the accelerometer counts 16384 per g and the gyroscope
2000/32768 deg/s per count (SDL). Motion axes form a right-handed frame, X to
the right, Y away from the player, Z out of the face, so lying flat face up
reads +1 g on Z, and a rotation is positive counter-clockwise seen from the
axis' positive end: SDL maps (x, z, -y) onto its right, up and toward-the-
player axes. Bit 0x40, named View in docs/PROTOCOL.md, is the physical Menu
button and 0x4000 the View button (OpenPuck/triton.h). L4 and R4 are the
upper back buttons (SDL's paddle 1). Every haptic effect the check sends ends
within a second by construction (report_seconds).
"""

import collections
import contextlib
import dataclasses
import datetime
import io
import json
import math
import os
import select
import statistics
import struct
import sys
import termios
import threading
import time
import tty

import sc2link_test as sc2

# ---- units ------------------------------------------------------------------

# The controller produces a state about every 4.032 ms (SDL's triton driver).
SAMPLE_PERIOD_S = 0.004032
STICK_FULL_SCALE = 32767
# A full trigger pull reads near half the u16 range (OpenPuck/triton.h).
TRIGGER_FULL_SCALE = 0x8000
PAD_FULL_SCALE = 32768
# 2 g and 2000 deg/s full scale (SDL's triton driver).
ACCEL_COUNTS_PER_G = 16384
GYRO_DEGREES_PER_COUNT = 2000 / 32768

# ---- thresholds, as fractions of full scale unless named otherwise ----------

STICK_CENTRE_MAX = 0.15
STICK_FULL_MIN = 0.80
STICK_OTHER_AXIS_MAX = 0.40
HALF_MIN = 0.20
TRIGGER_RELEASED_MAX = 0.10
TRIGGER_FULL_MIN = 0.80
PAD_EDGE_MIN = 0.50
PAD_OTHER_AXIS_MAX = 0.40
# Another stick or trigger moved this far is a wrong input, less is a resting
# finger.
OTHER_ANALOG_ACTIVE_MIN = 0.30
# A resting control that moves less than this counts as steady.
STEADY_SPAN = 0.03

GYRO_TURN_MIN_DEG = 45
GYRO_TURN_MAX_DEG = 135
GYRO_OTHER_AXIS_MAX_DEG = 45
STILL_RATE_DEG_S = 10
FLAT_RETURN_DEG = 20
ACCEL_FLAT_TOLERANCE_G = 0.10
ACCEL_POSE_MIN_G = 0.5

DURATION_TOLERANCE_S = 0.030
DURATION_TOLERANCE_FRACTION = 0.15
PULSE_MIN_SAMPLES_PER_PERIOD = 3
# The motion node samples at about 248/s, so anything above 124 Hz aliases:
# the check measures the envelope of the sample-to-sample change.
ENVELOPE_WINDOW_S = 0.012
NOISE_PERCENTILE = 0.99
NOISE_MARGIN = 2.0
MINIMUM_ACTIVITY = 1.0
DETECTABLE_RATIO = 2.0
# The calibration's level: the envelope sustained this long, so that one
# bump does not make effects look measurable.
CALIBRATION_SUSTAIN_S = 0.020
ONSET_EARLY_S = 0.050

# ---- timing -----------------------------------------------------------------

POLL_S = 0.05
SETTLE_S = 0.05
NEUTRAL_HOLD_S = 0.3
NEUTRAL_PROMPT_S = 1.0
ANALOG_HOLD_S = 0.3
PART_WAY_HOLD_S = 1.0
REST_HOLD_S = 0.5
REST_STEADY_S = 1.0
STILL_HOLD_S = 0.5
REST_S = 1.0
REMIND_AFTER_S = 20.0
NOISE_S = 1.0
PRE_ROLL_S = 0.3
QUIET_AFTER_S = 1.0
AFTER_MARGIN_S = 0.2
SETTLE_TRIES = 10
STOP_AT_S = 0.3
MAX_EFFECT_S = 1.0

SKIP_KEY = "s"
FAIL_KEY = "f"
REPLAY_KEY = "r"
STEP_KEYS = (SKIP_KEY, FAIL_KEY)
KEY_LEGEND = ("f: I am doing exactly this and the tool does not see it. s: "
              "this control is missing or broken, or I cannot do this.")

SECTIONS = ("1. Digital inputs", "2. Analog inputs", "3. Gyroscope",
            "4. Accelerometer", "5. Haptics measured by the motion sensors",
            "6. Haptics located by hand")
TABLE_PROMPT = ("Put the controller down on the table and take your hands "
                "off it.")
FLAT_PROMPT = ("Lay the controller flat on the table, face up, with the grips "
               "toward you, and take your hands off it.")
INTRODUCTION = (
    "SC2 Link controller check. One instruction at a time: the check moves on "
    "by itself when it sees the right input, and shows anything else it "
    "sees. Keys during a step: " + KEY_LEGEND + " Ctrl+C stops (the report "
    "is still written).")

# ---- the documented bits ----------------------------------------------------

# The pads node's copies of the trackpad touch bits, after the 32 button bits.
PADS_LEFT_TOUCH = 1 << 32
PADS_RIGHT_TOUCH = 1 << 33
BIT_NAMES = {**dict(sc2.BUTTONS), PADS_LEFT_TOUCH: "PadsLeftTouch",
             PADS_RIGHT_TOUCH: "PadsRightTouch"}
BIT = {name: mask for mask, name in BIT_NAMES.items()}
# Bits 30 and 31 are undocumented: seen active they are a wrong input too.
ALL_BITS = 0xFFFFFFFF | PADS_LEFT_TOUCH | PADS_RIGHT_TOUCH
TOUCH_BITS = sum(BIT[name] for name in (
    "RStickTouch", "RPadTouch", "LStickTouch", "LPadTouch", "RGrip", "LGrip",
    "PadsLeftTouch", "PadsRightTouch"))
LABELS = {
    "A": "the A button", "B": "the B button", "X": "the X button",
    "Y": "the Y button", "QAM": "the Quick Access button",
    "R3": "a right stick click", "View": "the Menu button",
    "R4": "the R4 back button", "R5": "the R5 back button",
    "RB": "the right bumper", "Down": "d-pad down", "Right": "d-pad right",
    "Left": "d-pad left", "Up": "d-pad up", "Menu": "the View button",
    "L3": "a left stick click", "Steam": "the Steam button",
    "L4": "the L4 back button", "L5": "the L5 back button",
    "LB": "the left bumper", "RStickTouch": "a touch on the right stick",
    "RPadTouch": "a touch on the right trackpad",
    "RPadClick": "a right trackpad click",
    "RTriggerClick": "a right trigger click",
    "LStickTouch": "a touch on the left stick",
    "LPadTouch": "a touch on the left trackpad",
    "LPadClick": "a left trackpad click",
    "LTriggerClick": "a left trigger click",
    "RGrip": "a touch on the right grip", "LGrip": "a touch on the left grip",
    "PadsLeftTouch": "the pads node's left trackpad touch",
    "PadsRightTouch": "the pads node's right trackpad touch",
}


def set_bits(bits):
    return [1 << index for index in range(bits.bit_length())
            if bits >> index & 1]


def bit_names(bits):
    return [BIT_NAMES.get(mask, f"bit {mask.bit_length() - 1}")
            for mask in set_bits(bits)]


def describe_bits(bits):
    return " and ".join(LABELS.get(name, name) for name in bit_names(bits))


# ---- steps ------------------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class BitStep:
    """expected bits must all go active and inactive again; companions may."""
    key: str
    instruction: str
    expected: int
    companions: int = 0


@dataclasses.dataclass(frozen=True)
class StickStep:
    key: str
    instruction: str
    stick: str
    target: str
    direction: str = ""


@dataclasses.dataclass(frozen=True)
class TriggerStep:
    key: str
    instruction: str
    trigger: str
    target: str


@dataclasses.dataclass(frozen=True)
class PadStep:
    key: str
    instruction: str
    pad: str
    edge: str


@dataclasses.dataclass(frozen=True)
class TurnStep:
    """axis and sign of the gyroscope's expected turn; exposes the
    accelerometer axis and sign gravity shows on in the held pose."""
    key: str
    instruction: str
    axis: int
    sign: int
    exposes: tuple = None


def press(name, instruction):
    """A button held in the hands, which may touch any capacitive sensor."""
    return BitStep(f"press {name} (0x{BIT[name]:x})", instruction, BIT[name],
                   TOUCH_BITS)


def touch(names, instruction):
    """A capacitive sensor alone, the controller lying on the table."""
    return BitStep("touch " + "+".join(names),
                   "With the controller lying on the table, " + instruction,
                   sum(BIT[name] for name in names))


DIGITAL_STEPS = (
    press("A", "Press the A button and let go."),
    press("B", "Press the B button and let go."),
    press("X", "Press the X button and let go."),
    press("Y", "Press the Y button and let go."),
    press("Up", "Press up on the d-pad and let go."),
    press("Down", "Press down on the d-pad and let go."),
    press("Left", "Press left on the d-pad and let go."),
    press("Right", "Press right on the d-pad and let go."),
    press("LB", "Press the left bumper (above the left trigger) and let go."),
    press("RB", "Press the right bumper (above the right trigger) and let "
          "go."),
    press("LTriggerClick", "Pull the left trigger all the way in and let go."),
    press("RTriggerClick", "Pull the right trigger all the way in and let "
          "go."),
    press("L4", "Press the upper back button on the left grip (L4) and let "
          "go."),
    press("L5", "Press the lower back button on the left grip (L5) and let "
          "go."),
    press("R4", "Press the upper back button on the right grip (R4) and let "
          "go."),
    press("R5", "Press the lower back button on the right grip (R5) and let "
          "go."),
    press("L3", "Press the left stick straight down until it clicks, and let "
          "go."),
    press("R3", "Press the right stick straight down until it clicks, and "
          "let go."),
    press("Menu", "Press the View button (the left of the two small buttons "
          "in the middle) and let go."),
    press("View", "Press the Menu button (the right of the two small buttons "
          "in the middle) and let go."),
    press("Steam", "Press the Steam button and let go."),
    press("QAM", "Press the Quick Access button (three dots) and let go."),
    press("LPadClick", "Press the left trackpad down until it clicks, and let "
          "go."),
    press("RPadClick", "Press the right trackpad down until it clicks, and "
          "let go."),
    touch(("LPadTouch", "PadsLeftTouch"), "touch the left trackpad lightly "
          "with one finger, without clicking, then lift the finger."),
    touch(("RPadTouch", "PadsRightTouch"), "touch the right trackpad lightly "
          "with one finger, without clicking, then lift the finger."),
    touch(("LStickTouch",), "rest a fingertip on top of the left stick "
          "without moving it, then lift it."),
    touch(("RStickTouch",), "rest a fingertip on top of the right stick "
          "without moving it, then lift it."),
    touch(("LGrip",), "touch the left grip (the handle your left hand holds) "
          "with your fingers, then take them away."),
    touch(("RGrip",), "touch the right grip (the handle your right hand "
          "holds) with your fingers, then take them away."),
)

TOWARD = {"left": "to the left", "right": "to the right", "up": "up",
          "down": "down"}


def stick_steps(stick, half_direction):
    name = f"{stick} stick"
    return (
        StickStep(f"{name} centre", f"Let go of the {name} so it rests in "
                  "the middle.", stick, "centre"),
        *(StickStep(f"{name} {direction}", f"Push the {name} all the way "
                    f"{TOWARD[direction]}, hold it a moment, then let go.",
                    stick, "full", direction)
          for direction in ("left", "right", "up", "down")),
        StickStep(f"{name} half {half_direction}", f"Push the {name} about "
                  f"halfway {TOWARD[half_direction]}, hold it there a moment, "
                  "then let go.", stick, "half", half_direction),
    )


def trigger_steps(trigger):
    name = f"{trigger} trigger"
    return (
        TriggerStep(f"{name} released", f"Let go of the {name} completely.",
                    trigger, "released"),
        TriggerStep(f"{name} half", f"Pull the {name} about halfway, hold it "
                    "there a moment, then let go.", trigger, "half"),
        TriggerStep(f"{name} full", f"Pull the {name} all the way, hold it a "
                    "moment, then let go.", trigger, "full"),
    )


PAD_PLACES = {"left": "near its left edge, halfway up",
              "right": "near its right edge, halfway up",
              "top": "near its top edge, in the middle",
              "bottom": "near its bottom edge, in the middle"}


def pad_steps(pad):
    return tuple(
        PadStep(f"{pad} trackpad {edge}", f"Touch the {pad} trackpad "
                f"{PAD_PLACES[edge]}, hold the finger there a moment, then "
                "lift it.", pad, edge)
        for edge in ("left", "right", "top", "bottom"))


ANALOG_STEPS = (stick_steps("left", "up") + stick_steps("right", "right")
                + trigger_steps("left") + trigger_steps("right")
                + pad_steps("left") + pad_steps("right"))

TURN_STEPS = (
    TurnStep("steer right", "Pick the controller up and turn it a quarter "
             "turn to the right like a steering wheel, so the right grip "
             "points down at the table. Hold it still there for a second, "
             "then lay it flat again.", axis=1, sign=1, exposes=(0, -1)),
    TurnStep("tilt up", "Lift the far edge (the one with the triggers) a "
             "quarter turn, so the controller stands on its grips facing "
             "you. Hold it still there for a second, then lay it flat again.",
             axis=0, sign=1, exposes=(1, 1)),
    TurnStep("compass left", "Leave the controller flat on the table and "
             "turn it a quarter turn to the left (counter-clockwise seen from "
             "above), like a compass needle. Hold it still for a second, then "
             "let go.", axis=2, sign=1),
)
# What a turn about each gyroscope axis and sign looks like from the player.
TURN_LABELS = {(1, 1): "a steering turn to the right",
               (1, -1): "a steering turn to the left",
               (0, 1): "the far edge lifted", (0, -1): "the near edge lifted",
               (2, 1): "a turn to the left on the table",
               (2, -1): "a turn to the right on the table"}


# ---- what the nodes say -----------------------------------------------------

ROLES = ("controls", "pads", "motion")


@dataclasses.dataclass(frozen=True)
class Snapshot:
    """All three nodes after one frame of source. Sticks, triggers and pads
    as fractions of full scale, accel in g, gyro in deg/s."""
    time_ns: int
    source: str
    connected: bool
    buttons: int
    left_stick: tuple
    right_stick: tuple
    triggers: tuple
    left_pad: tuple
    right_pad: tuple
    accel: tuple
    gyro: tuple


def scaled(node, codes, scale, convert=int):
    return tuple(convert(node.get(code, 0)) / scale for code in codes)


def snapshot_from(values, source, time_ns):
    controls, pads, motion = (values[role] for role in ROLES)
    touched = pads.get(sc2.ABS_WHEEL, 0)
    buttons = (sc2.unsigned(controls.get(sc2.ABS_THROTTLE, 0))
               | sc2.unsigned(controls.get(sc2.ABS_RUDDER, 0)) << 16
               | (PADS_LEFT_TOUCH if touched & 1 else 0)
               | (PADS_RIGHT_TOUCH if touched & 2 else 0))
    return Snapshot(
        time_ns, source, bool(controls.get(sc2.ABS_WHEEL, 0) & 1), buttons,
        scaled(controls, (sc2.ABS_X, sc2.ABS_Z), STICK_FULL_SCALE),
        scaled(controls, (sc2.ABS_RX, sc2.ABS_RY), STICK_FULL_SCALE),
        scaled(controls, (sc2.ABS_GAS, sc2.ABS_BRAKE), TRIGGER_FULL_SCALE,
               sc2.unsigned),
        scaled(pads, (sc2.ABS_X, sc2.ABS_Z), PAD_FULL_SCALE),
        scaled(pads, (sc2.ABS_RY, sc2.ABS_THROTTLE), PAD_FULL_SCALE),
        scaled(motion, (sc2.ABS_X, sc2.ABS_Z, sc2.ABS_RX),
               ACCEL_COUNTS_PER_G),
        scaled(motion, (sc2.ABS_RY, sc2.ABS_THROTTLE, sc2.ABS_RUDDER),
               1 / GYRO_DEGREES_PER_COUNT))


def stick_position(snapshot, stick):
    return snapshot.left_stick if stick == "left" else snapshot.right_stick


def trigger_value(snapshot, trigger):
    return snapshot.triggers[0 if trigger == "left" else 1]


def pad_position(snapshot, pad):
    return snapshot.left_pad if pad == "left" else snapshot.right_pad


REST_LIMITS = {"left stick": STICK_CENTRE_MAX,
               "right stick": STICK_CENTRE_MAX,
               "left trigger": TRIGGER_RELEASED_MAX,
               "right trigger": TRIGGER_RELEASED_MAX}


def analog_amounts(snapshot):
    return (("left stick", max(map(abs, snapshot.left_stick))),
            ("right stick", max(map(abs, snapshot.right_stick))),
            ("left trigger", snapshot.triggers[0]),
            ("right trigger", snapshot.triggers[1]))


@dataclasses.dataclass(frozen=True)
class Stuck:
    """Controls found active before a section and recorded as stuck."""
    bits: int = 0
    analog: frozenset = frozenset()


@dataclasses.dataclass(frozen=True)
class Active:
    bits: int = 0
    analog: tuple = ()
    connected: bool = True

    def __bool__(self):
        return bool(self.bits or self.analog or not self.connected)

    def describe(self):
        parts = [] if self.connected else ["the controller is not connected"]
        if self.bits:
            parts.append(describe_bits(self.bits))
        parts += [f"the {name} is not at rest" for name in self.analog]
        return ", ".join(parts)

    def names(self):
        return bit_names(self.bits) + list(self.analog)


def active_controls(snapshot, stuck, ignore_bits=0, ignore_analog=()):
    bits = snapshot.buttons & ALL_BITS & ~ignore_bits & ~stuck.bits
    analog = tuple(name for name, amount in analog_amounts(snapshot)
                   if amount >= REST_LIMITS[name] and name not in ignore_analog
                   and name not in stuck.analog)
    return Active(bits, analog, snapshot.connected)


# ---- classification ---------------------------------------------------------

def stick_reading(position):
    """(zone, direction, angled): zone centre, between, half or full."""
    x, y = position
    major, minor = max(abs(x), abs(y)), min(abs(x), abs(y))
    if abs(x) >= abs(y):
        direction = "right" if x > 0 else "left"
    else:
        direction = "up" if y > 0 else "down"
    if major < STICK_CENTRE_MAX:
        zone = "centre"
    elif major >= STICK_FULL_MIN:
        zone = "full"
    elif major >= HALF_MIN:
        zone = "half"
    else:
        zone = "between"
    return zone, direction, minor >= STICK_OTHER_AXIS_MAX


def trigger_zone(value):
    if value < TRIGGER_RELEASED_MAX:
        return "released"
    if value >= TRIGGER_FULL_MIN:
        return "full"
    return "half" if value >= HALF_MIN else "between"


def pad_region(position):
    x, y = position
    if max(abs(x), abs(y)) < PAD_EDGE_MIN:
        return "middle"
    if min(abs(x), abs(y)) >= PAD_OTHER_AXIS_MAX:
        return "corner"
    if abs(x) >= abs(y):
        return "right edge" if x > 0 else "left edge"
    return "top edge" if y > 0 else "bottom edge"


def other_input(snapshot, allowed_bits, tested, stuck):
    """What the person did besides the step's own control, or ''."""
    bits = snapshot.buttons & ALL_BITS & ~allowed_bits & ~stuck.bits
    if bits:
        return describe_bits(bits)
    for name, amount in analog_amounts(snapshot):
        if (name in tested or name in stuck.analog
                or amount < OTHER_ANALOG_ACTIVE_MIN):
            continue
        if name.endswith("stick"):
            _, direction, _ = stick_reading(
                stick_position(snapshot, name.split()[0]))
            return f"the {name} pushed {direction}"
        return f"the {name} pulled"
    return ""


def stick_verdict(step, zone, direction, angled, seconds):
    """None to keep waiting, '' when the step's position is held, else what
    was held instead."""
    name = f"the {step.stick} stick"
    if zone in ("centre", "between") or seconds < ANALOG_HOLD_S:
        return None
    if direction != step.direction:
        return f"{name} pushed {direction}"
    if angled:
        return f"{name} pushed {direction} at an angle"
    if zone == step.target:
        return ""
    if zone == "full":
        return f"{name} pushed all the way {direction}"
    if seconds >= PART_WAY_HOLD_S:
        return f"{name} pushed only part way {direction}"
    return None


def trigger_verdict(step, zone, seconds):
    name = f"the {step.trigger} trigger"
    if zone in ("released", "between") or seconds < ANALOG_HOLD_S:
        return None
    if zone == step.target:
        return ""
    if zone == "full":
        return f"{name} pulled all the way"
    if seconds >= PART_WAY_HOLD_S:
        return f"{name} pulled only part way"
    return None


def integrate(angles, rate, gap_ns):
    """Degrees after gap_ns at rate deg/s: the rate holds until the next
    sample, since the kernel sends no frame for unchanged values."""
    return [angle + value * gap_ns / 1e9 for angle, value in zip(angles, rate)]


def turn_verdict(angles, step):
    """'' for a quarter turn about the step's axis alone, else what it was."""
    main = angles[step.axis] * step.sign
    others = max(abs(angle) for index, angle in enumerate(angles)
                 if index != step.axis)
    if (GYRO_TURN_MIN_DEG <= main <= GYRO_TURN_MAX_DEG
            and others < GYRO_OTHER_AXIS_MAX_DEG):
        return ""
    dominant = max(range(3), key=lambda index: abs(angles[index]))
    sign = 1 if angles[dominant] > 0 else -1
    if (dominant, sign) != (step.axis, step.sign):
        return (f"{TURN_LABELS[(dominant, sign)]} (gyroscope "
                f"{'XYZ'[dominant]}{'+' if sign > 0 else '-'})")
    if main > GYRO_TURN_MAX_DEG:
        return "more than a quarter turn"
    return "a turn about two axes at once"


def mean_vector(values):
    return tuple(statistics.fmean(axis) for axis in zip(*values))


def angle_between(first, second):
    lengths = math.hypot(*first) * math.hypot(*second)
    if not lengths:
        return 180.0
    cosine = sum(a * b for a, b in zip(first, second)) / lengths
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def rounded(value, digits=3):
    if isinstance(value, float):
        return round(value, digits)
    if isinstance(value, (list, tuple)):
        return [rounded(item, digits) for item in value]
    if isinstance(value, dict):
        return {key: rounded(item, digits) for key, item in value.items()}
    return value


def step_target(step):
    """What a step waits for, in the words of 'Waiting for: ...'."""
    if isinstance(step, BitStep):
        return describe_bits(step.expected)
    if isinstance(step, StickStep):
        if step.target == "centre":
            return f"the {step.stick} stick resting in the middle"
        amount = "all the way" if step.target == "full" else "about halfway"
        return f"the {step.stick} stick {amount} {TOWARD[step.direction]}"
    if isinstance(step, TriggerStep):
        return {"released": f"the {step.trigger} trigger released",
                "half": f"the {step.trigger} trigger pulled about halfway",
                "full": f"the {step.trigger} trigger pulled all the way"
                }[step.target]
    if isinstance(step, PadStep):
        return f"a touch near the {step.edge} edge of the {step.pad} trackpad"
    return TURN_LABELS[(step.axis, step.sign)]


def seen_text(seen):
    """The other inputs a step saw, with counts, for a reason."""
    return "; ".join(f"{entry['input']} ({entry['count']}x)"
                     for entry in seen)


def step_neutral(step):
    """The prompt, bits and analog controls a step's start may ignore."""
    if isinstance(step, BitStep):
        prompt = ("Let go of everything." if step.companions
                  else "Take your hands off the controller.")
        return prompt, step.companions, ()
    tested = ()
    if isinstance(step, StickStep) and step.target == "centre":
        tested = (f"{step.stick} stick",)
    if isinstance(step, TriggerStep) and step.target == "released":
        tested = (f"{step.trigger} trigger",)
    return "Let go of everything.", TOUCH_BITS, tested


def release_hint(step):
    """What must happen after other input before the expected one counts:
    the other control let go, which the neutral wait asks for itself, or a
    turn's controller laid flat again."""
    if isinstance(step, TurnStep):
        return " Lay the controller flat on the table again."
    return ""


# ---- watchers: each takes snapshots until it decides ------------------------

@dataclasses.dataclass(frozen=True)
class Outcome:
    """matched, or what was seen instead (signature, the same text for the
    same input), with the observation that decided it."""
    matched: bool
    signature: str = ""
    observation: dict = dataclasses.field(default_factory=dict)


class Hold:
    """How long one label has lasted, and the values seen meanwhile."""

    def __init__(self):
        self.label = None
        self.since_ns = 0
        self.values = []

    def update(self, label, time_ns, value):
        if label != self.label:
            self.label, self.since_ns, self.values = label, time_ns, []
        self.values.append(value)
        return (time_ns - self.since_ns) / 1e9


class Steady:
    """How long a value has stayed within STEADY_SPAN on every axis."""

    def __init__(self):
        self.since_ns = None
        self.low = self.high = ()

    def update(self, value, time_ns):
        value = tuple(value) if isinstance(value, (tuple, list)) else (value,)
        if self.since_ns is not None:
            low = tuple(map(min, self.low, value))
            high = tuple(map(max, self.high, value))
            if all(top - bottom <= STEADY_SPAN
                   for bottom, top in zip(low, high)):
                self.low, self.high = low, high
                return (time_ns - self.since_ns) / 1e9
        self.since_ns, self.low, self.high = time_ns, value, value
        return 0.0


def mean_value(values):
    if values and isinstance(values[0], tuple):
        return mean_vector(values)
    return statistics.fmean(values) if values else 0.0


def rest_outcome(hold, steady, time_ns, value, at_rest, signature):
    """A control left alone: at rest for REST_HOLD_S, or steady elsewhere for
    REST_STEADY_S."""
    seconds = hold.update(at_rest, time_ns, value)
    observation = {"value": rounded(mean_value(hold.values)),
                   "seconds": round(seconds, 3)}
    if at_rest:
        if seconds >= REST_HOLD_S:
            return Outcome(True, "", observation)
        return None
    if steady.update(value, time_ns) >= REST_STEADY_S:
        return Outcome(False, signature, observation)
    return None


class BitWatcher:
    """Expected bits went active and every relevant bit inactive again."""

    def __init__(self, step, stuck):
        self.step = step
        self.relevant = ALL_BITS & ~step.companions & ~stuck.bits
        self.seen = 0
        self.companions_seen = 0
        self.inactive_since = None

    def feed(self, snapshot):
        active = snapshot.buttons & self.relevant
        self.seen |= active
        self.companions_seen |= snapshot.buttons & self.step.companions
        unexpected = self.seen & ~self.step.expected
        if unexpected:
            return self.outcome(False, describe_bits(unexpected))
        if active or not self.seen:
            self.inactive_since = None
            return None
        # Frames of the two nodes arrive apart: settle before deciding.
        if self.inactive_since is None:
            self.inactive_since = snapshot.time_ns
        if snapshot.time_ns - self.inactive_since < SETTLE_S * 1e9:
            return None
        missing = self.step.expected & self.relevant & ~self.seen
        if missing:
            return self.outcome(False, f"{describe_bits(self.seen)} without "
                                f"{describe_bits(missing)}")
        return self.outcome(True)

    def outcome(self, matched, signature=""):
        return Outcome(matched, signature, {
            "seen": bit_names(self.seen),
            "companions_seen": bit_names(self.companions_seen)})


class StickWatcher:
    def __init__(self, step, stuck):
        self.step = step
        self.stuck = stuck
        self.name = f"{step.stick} stick"
        self.hold = Hold()
        self.steady = Steady()
        self.reached = None

    def feed(self, snapshot):
        position = stick_position(snapshot, self.step.stick)
        zone, direction, angled = stick_reading(position)
        if self.reached is not None:
            if zone == "centre":
                return Outcome(True, "", self.reached)
            return None
        other = other_input(snapshot, TOUCH_BITS, (self.name,), self.stuck)
        if other:
            return Outcome(False, other, {"position": rounded(position)})
        if self.step.target == "centre":
            return rest_outcome(self.hold, self.steady, snapshot.time_ns,
                                position, zone == "centre",
                                f"the {self.name} rests off centre "
                                f"({direction})")
        seconds = self.hold.update((zone, direction, angled),
                                   snapshot.time_ns, position)
        verdict = stick_verdict(self.step, zone, direction, angled, seconds)
        if verdict is None:
            return None
        observation = {"position": rounded(mean_vector(self.hold.values)),
                       "held_s": round(seconds, 3)}
        if verdict:
            return Outcome(False, verdict, observation)
        self.reached = observation
        return None


class TriggerWatcher:
    def __init__(self, step, stuck):
        self.step = step
        self.stuck = stuck
        self.name = f"{step.trigger} trigger"
        click = BIT["LTriggerClick" if step.trigger == "left"
                    else "RTriggerClick"]
        self.allowed = TOUCH_BITS | (click if step.target == "full" else 0)
        self.hold = Hold()
        self.steady = Steady()
        self.reached = None

    def feed(self, snapshot):
        value = trigger_value(snapshot, self.step.trigger)
        zone = trigger_zone(value)
        if self.reached is not None:
            if zone == "released":
                return Outcome(True, "", self.reached)
            return None
        other = other_input(snapshot, self.allowed, (self.name,), self.stuck)
        if other:
            return Outcome(False, other, {"value": round(value, 3)})
        if self.step.target == "released":
            return rest_outcome(self.hold, self.steady, snapshot.time_ns,
                                value, zone == "released",
                                f"the {self.name} does not return to "
                                "released")
        seconds = self.hold.update(zone, snapshot.time_ns, value)
        verdict = trigger_verdict(self.step, zone, seconds)
        if verdict is None:
            return None
        observation = {"value": round(statistics.fmean(self.hold.values), 3),
                       "held_s": round(seconds, 3)}
        if verdict:
            return Outcome(False, verdict, observation)
        self.reached = observation
        return None


class PadWatcher:
    """A touch on the step's pad, judged by its median position at lift.

    A thumb resting on the other pad is allowed: section 1 checks each touch
    bit alone, and position fields of the wrong pad show as a wrong position.
    """

    def __init__(self, step, stuck):
        self.step = step
        self.stuck = stuck
        left = step.pad == "left"
        self.touch_bit = PADS_LEFT_TOUCH if left else PADS_RIGHT_TOUCH
        self.allowed = TOUCH_BITS | BIT["LPadClick" if left else "RPadClick"]
        self.positions = []
        self.lifted_since = None

    def feed(self, snapshot):
        other = other_input(snapshot, self.allowed, (), self.stuck)
        if other:
            return Outcome(False, other, {})
        if snapshot.buttons & self.touch_bit:
            self.positions.append(pad_position(snapshot, self.step.pad))
            self.lifted_since = None
            return None
        if not self.positions:
            return None
        if self.lifted_since is None:
            self.lifted_since = snapshot.time_ns
        if snapshot.time_ns - self.lifted_since < SETTLE_S * 1e9:
            return None
        position = tuple(statistics.median(axis)
                         for axis in zip(*self.positions))
        region = pad_region(position)
        observation = {"position": rounded(position),
                       "samples": len(self.positions)}
        if region == f"{self.step.edge} edge":
            return Outcome(True, "", observation)
        return Outcome(False, f"the {self.step.pad} trackpad touched at the "
                       f"{region}", observation)


@dataclasses.dataclass(frozen=True)
class RestReference:
    """The controller at rest: gyroscope bias (deg/s), gravity (g)."""
    gyro_bias: tuple
    accel: tuple


class RestWatcher:
    """Still for some seconds, and with a reference flat like it too."""

    def __init__(self, seconds, reference=None):
        self.seconds = seconds
        self.reference = reference
        self.bias = reference.gyro_bias if reference else (0.0, 0.0, 0.0)
        self.since_ns = None
        self.gyro = []
        self.accel = []

    def feed(self, snapshot):
        rate = [value - bias for value, bias in zip(snapshot.gyro, self.bias)]
        if max(map(abs, rate)) >= STILL_RATE_DEG_S:
            self.since_ns, self.gyro, self.accel = None, [], []
            return None
        if self.since_ns is None:
            self.since_ns = snapshot.time_ns
        self.gyro.append(snapshot.gyro)
        self.accel.append(snapshot.accel)
        if snapshot.time_ns - self.since_ns < self.seconds * 1e9:
            return None
        accel = mean_vector(self.accel)
        if (self.reference is not None
                and angle_between(accel, self.reference.accel)
                > FLAT_RETURN_DEG):
            self.since_ns, self.gyro, self.accel = None, [], []
            return None
        return Outcome(True, "", {"gyro_bias_deg_s": mean_vector(self.gyro),
                                  "accel_g": accel})


class TurnWatcher:
    """Integrates the gyroscope from flat, judges the first pose held still
    at least GYRO_TURN_MIN_DEG away, then waits for flat again."""

    def __init__(self, step, rest):
        self.step = step
        self.rest = rest
        self.angles = [0.0, 0.0, 0.0]
        self.rate = (0.0, 0.0, 0.0)
        self.last_motion_ns = None
        self.largest_gap_ns = 0
        self.motion_samples = 0
        self.still_since = None
        # (time, accel) of the last STILL_HOLD_S while still.
        self.still_accel = collections.deque()
        self.pose = None

    def feed(self, snapshot):
        rate = tuple(value - bias for value, bias
                     in zip(snapshot.gyro, self.rest.gyro_bias))
        if snapshot.source == "motion":
            self.integrate(snapshot.time_ns, rate)
        if max(map(abs, rate)) >= STILL_RATE_DEG_S:
            self.still_since = None
            self.still_accel.clear()
            return None
        if self.still_since is None:
            self.still_since = snapshot.time_ns
        self.still_accel.append((snapshot.time_ns, snapshot.accel))
        while snapshot.time_ns - self.still_accel[0][0] > STILL_HOLD_S * 1e9:
            self.still_accel.popleft()
        if snapshot.time_ns - self.still_since < STILL_HOLD_S * 1e9:
            return None
        accel = mean_vector([accel for _, accel in self.still_accel])
        if self.pose is None:
            if max(map(abs, self.angles)) < GYRO_TURN_MIN_DEG:
                return None
            self.pose = {"angles_deg": rounded(self.angles, 1),
                         "accel_g": rounded(accel)}
            signature = turn_verdict(self.angles, self.step)
            if signature:
                return Outcome(False, signature, self.observation())
        if angle_between(accel, self.rest.accel) <= FLAT_RETURN_DEG:
            return Outcome(True, "", self.observation())
        return None

    def integrate(self, time_ns, rate):
        if self.last_motion_ns is not None:
            gap = time_ns - self.last_motion_ns
            self.largest_gap_ns = max(self.largest_gap_ns, gap)
            self.angles = integrate(self.angles, self.rate, gap)
        self.last_motion_ns = time_ns
        self.rate = rate
        self.motion_samples += 1

    def observation(self):
        return {"pose": self.pose, "motion_samples": self.motion_samples,
                "largest_gap_ms": round(self.largest_gap_ns / 1e6, 1)}


def watcher_for(step, stuck, rest):
    if isinstance(step, BitStep):
        return BitWatcher(step, stuck)
    if isinstance(step, StickStep):
        return StickWatcher(step, stuck)
    if isinstance(step, TriggerStep):
        return TriggerWatcher(step, stuck)
    if isinstance(step, PadStep):
        return PadWatcher(step, stuck)
    return TurnWatcher(step, rest)


# ---- the accelerometer ------------------------------------------------------

def gravity_check(accel, axis, sign, flat):
    if accel is None:
        return {"result": "skipped", "reason": "not measured"}
    value = accel[axis] * sign
    if flat:
        passed = abs(value - 1) <= ACCEL_FLAT_TOLERANCE_G
        expected = f"{'XYZ'[axis]} {1 - ACCEL_FLAT_TOLERANCE_G:.2f} to " \
                   f"{1 + ACCEL_FLAT_TOLERANCE_G:.2f} g"
    else:
        passed = value >= ACCEL_POSE_MIN_G
        expected = f"{'XYZ'[axis]} {'+' if sign > 0 else '-'}" \
                   f"{ACCEL_POSE_MIN_G} g or beyond"
    return {"result": "pass" if passed else "fail", "expected": expected,
            "accel_g": rounded(accel)}


def accel_verdict(rest_accel, poses):
    """One verdict: +1 g on Z lying flat, and gravity with the right sign on
    the axis each held pose of section 3 points down or up."""
    checks = {"lying flat": gravity_check(rest_accel, 2, 1, True)}
    for step in TURN_STEPS:
        if step.exposes:
            checks[f"held pose '{step.key}'"] = gravity_check(
                poses.get(step.key), *step.exposes, False)
    results = [check["result"] for check in checks.values()]
    if "fail" in results:
        verdict = "fail"
    else:
        verdict = "pass" if set(results) == {"pass"} else "skipped"
    return verdict, checks


# ---- haptic effects ---------------------------------------------------------

# How long the controller plays each report, from its fields
# (docs/PROTOCOL.md 9.5.1 and the reverse-engineered stock firmware,
# mwdmwd/sc26re): tick 18 ms, click 24 ms, scripts 1 to 16 up to 3 s and
# unstoppable, other script ids nothing, a tone or sweep of 0 ms forever.
COMMAND_SECONDS = {0: 0.0, 1: 0.018, 2: 0.024}
PLAYING_SCRIPTS = range(1, 17)
SCRIPT_SECONDS = 3.0
# The dongle resends a running 0x80 every 40 ms (docs/PROTOCOL.md 9.5.1) and
# the controller plays each for about 100 ms after it, so a rumble may go on
# for RUMBLE_AFTER_STOP_S after its stop was acknowledged.
RUMBLE_TAIL_S = 0.1
RUMBLE_REFRESH_S = 0.040
RUMBLE_AFTER_STOP_S = RUMBLE_REFRESH_S + RUMBLE_TAIL_S
# A word the dongle never acknowledges takes all of Link.send's resends, each
# of which may refresh the dongle's session timer, before the timeout starts.
LOST_WORD_S = (sc2.MAX_RESENDS + 1) * sc2.RESEND_AFTER_S
# A rumble's stop follows this long after it, with no NOP between, so that a
# stop the dongle never takes still ends the rumble within MAX_EFFECT_S.
RUMBLE_PLAY_S = 0.025
# Samples 4.032 ms apart see a vibration at f through |sin(pi f T)|, which is
# zero at 248 Hz: the measured sweeps stay where it is above 0.65.
SWEEP_START_HZ = 60
SWEEP_END_HZ = 110
PULSE_SIDES = {0: "right pad", 1: "left pad", 2: "both pads",
               3: "left internal actuator in the sc26re reading"}
OTHER_SIDES = {0: "left pad", 1: "right pad", 2: "both pads",
               3: "left internal actuator"}


def report_seconds(report):
    """The longest a report other than a running rumble plays."""
    kind = report[0]
    if kind == 0x80:
        return 0.0 if not report[1] else math.inf
    if kind == 0x81:
        _, _, on_us, off_us, repeat = struct.unpack("<BBHHH", report)
        return max(repeat, 1) * (on_us + off_us) / 1e6 if on_us else 0.0
    if kind == 0x82:
        return COMMAND_SECONDS.get(report[2], math.inf)
    if kind == 0x85:
        return SCRIPT_SECONDS if report[2] in PLAYING_SCRIPTS else 0.0
    if kind in (0x83, 0x84):
        duration_ms = struct.unpack_from("<H", report, 5 if kind == 0x83
                                         else 3)[0]
        return duration_ms / 1000 if duration_ms else math.inf
    return math.inf


def rumble_worst_s(hold_s):
    """The longest a rumble stopped hold_s after it plays: its stop lost
    through every resend, the session timeout's stop, then the tail."""
    return hold_s + LOST_WORD_S + sc2.SESSION_TIMEOUT_S + RUMBLE_AFTER_STOP_S


def bounded_messages(key, messages, rumble_s=0.0):
    """The messages as tuples, or ValueError when one could play longer than
    MAX_EFFECT_S. A rumble is stopped rumble_s later (rumble_worst_s)."""
    for words in messages:
        report = sc2.build_report(words[0], list(words[1:]))
        if report is None:
            continue
        seconds = report_seconds(report)
        if report[0] == 0x80 and report[1]:
            seconds = rumble_worst_s(rumble_s)
        if seconds > MAX_EFFECT_S:
            raise ValueError(f"{key}: {report.hex(' ')} plays up to "
                             f"{seconds} s, the limit is {MAX_EFFECT_S} s")
    return tuple(tuple(words) for words in messages)


@dataclasses.dataclass(frozen=True)
class Effect:
    """messages are data words, sent in order. From the command, active_s
    is the shortest activity the command allows, total_s the latest end."""
    key: str
    description: str
    messages: tuple
    active_s: float
    total_s: float
    pulses: int = 0
    pulse_gap_s: float = 0.0
    stop_after_s: float = 0.0
    side: int = -1


def rumble_effect(key, left, right, seconds=RUMBLE_PLAY_S):
    messages = bounded_messages(key, [sc2.encode_rumble(left, right)],
                                seconds)
    return Effect(key, f"0x80 rumble, left 0x{left:04x}, right 0x{right:04x}, "
                  f"stopped after {seconds * 1000:.0f} ms", messages, seconds,
                  seconds, stop_after_s=seconds)


def pulse_effect(key, side, on_us, off_us, repeat):
    """sc26re renders +gain while on, -gain while off and returns to zero
    after repeat whole periods: the activity ends between the last on time
    and the last period, with repeat or repeat + 1 edges seen as pulses."""
    if not on_us or not repeat:
        raise ValueError(f"{key}: a pulse without on time or repeat has no "
                         "known duration")
    messages = bounded_messages(
        key, [sc2.encode_pulse(side, on_us, off_us, repeat)])
    period = (on_us + off_us) / 1e6
    countable = period >= PULSE_MIN_SAMPLES_PER_PERIOD * SAMPLE_PERIOD_S
    return Effect(key, f"0x81 side {side} ({PULSE_SIDES[side]}), {on_us} us "
                  f"on, {off_us} us off, repeat {repeat}", messages,
                  (repeat - 1) * period + on_us / 1e6, repeat * period,
                  pulses=repeat if countable else 0,
                  pulse_gap_s=max(off_us / 2e6, 2.5 * SAMPLE_PERIOD_S),
                  side=side)


def command_effect(key, side, command, gain):
    messages = bounded_messages(key, [sc2.encode_command(side, command,
                                                         gain)])
    seconds = COMMAND_SECONDS[command]
    return Effect(key, f"0x82 side {side} ({OTHER_SIDES[side]}), command "
                  f"{command}, gain {gain} dB", messages, seconds, seconds,
                  side=side)


def script_effect(key, side, script, gain):
    messages = bounded_messages(key, [sc2.encode_script(side, script, gain)])
    return Effect(key, f"0x85 side {side} ({OTHER_SIDES[side]}), script "
                  f"{script} (plays nothing), gain {gain} dB", messages, 0.0,
                  0.0, side=side)


def sweep_effect(key, side, gain, duration_ms, start_hz, end_hz):
    messages = bounded_messages(key, [sc2.encode_sweep(
        side, gain, duration_ms, start_hz, end_hz)])
    return Effect(key, f"0x84 side {side} ({OTHER_SIDES[side]}), {start_hz} "
                  f"to {end_hz} Hz in {duration_ms} ms, gain {gain} dB",
                  messages, duration_ms / 1000, duration_ms / 1000, side=side)


def tone_effect(key, side, gain, frequency_hz, duration_ms):
    messages = bounded_messages(key, [sc2.encode_tone(
        side, gain, frequency_hz, duration_ms, 0, 0)])
    return Effect(key, f"0x83 side {side} ({OTHER_SIDES[side]}), "
                  f"{frequency_hz} Hz for {duration_ms} ms, gain {gain} dB",
                  messages, duration_ms / 1000, duration_ms / 1000, side=side)


@dataclasses.dataclass(frozen=True)
class StopCandidate:
    key: str
    messages: tuple
    ends_session: bool = False


def stop_candidate(key, *messages, ends_session=False):
    return StopCandidate(key, bounded_messages(key, messages), ends_session)


SIDE_VALUES = range(4)
# The bounded rumble of the selftest's boundary cases, also the calibration.
CALIBRATION_EFFECT = rumble_effect("rumble both", 0xFFFF, 0xFFFF)
MEASURED_EFFECTS = (
    CALIBRATION_EFFECT,
    rumble_effect("rumble left", 0xFFFF, 0),
    rumble_effect("rumble right", 0, 0xFFFF),
    *(pulse_effect(f"pulse side {side}", side, 30000, 50000, 5)
      for side in SIDE_VALUES),
    command_effect("tick side 2", 2, 1, 0),
    *(command_effect(f"click side {side}", side, 2, 0)
      for side in SIDE_VALUES),
    *(script_effect(f"script 0 side {side}", side, 0, 0)
      for side in SIDE_VALUES),
    *(sweep_effect(f"sweep side {side}", side, 0, 300, SWEEP_START_HZ,
                   SWEEP_END_HZ) for side in SIDE_VALUES),
    *(tone_effect(f"tone side {side}", side, 0, 160, 300)
      for side in SIDE_VALUES),
)
# A right pad train commanded for 0.9 s, and the stops to try on it at
# STOP_AT_S: the candidates of the design note's disputed section.
STOP_TRAIN = pulse_effect("stop test train", 0, 30000, 30000, 15)
# The stop probes' step keys, set apart in the summary.
STOP_PREFIX = "stop: "
STOP_CANDIDATES = (
    stop_candidate("0x82 OFF side 3, the session stop's OFF",
                   sc2.encode_command(3, 0, 0)),
    stop_candidate("0x82 OFF on the train's pad (side 1)",
                   sc2.encode_command(1, 0, 0)),
    stop_candidate("0x81 with on 0 on the train's side",
                   sc2.encode_pulse(0, 0, 0, 1)),
    stop_candidate("0x81 one 1 us pulse on the train's side",
                   sc2.encode_pulse(0, 1, 1, 1)),
    stop_candidate("proposed session stop: 0x80 zero, 0x81 both pads 1 us, "
                   "1 ms tone and sweep at minimum gain on both pads",
                   sc2.encode_rumble_stop(), sc2.encode_pulse(2, 1, 1, 1),
                   sc2.encode_tone(2, -128, 160, 1, 0, 0),
                   sc2.encode_sweep(2, -128, 1, 100, 200)),
    stop_candidate("session end (BYE), the dongle's own stop", [sc2.BYE],
                   ends_session=True),
)
# One effect per side value, in an order that does not give the answer away,
# with where it should be felt. The internal actuator sitting in the left
# grip is an assumption.
LOCATE_EFFECTS = tuple(
    (tone_effect(f"locate side {side}", side, 0, 160, 300), place)
    for side, place in ((1, "right trackpad"), (3, "left grip"),
                        (0, "left trackpad"), (2, "both sides")))
ANSWERS = {"1": "left grip", "2": "right grip", "3": "left trackpad",
           "4": "right trackpad", "5": "both sides", "0": "nothing felt"}
QUESTION = ("Where did you feel effect {number}? 1 left grip, 2 right grip, "
            "3 left trackpad, 4 right trackpad, 5 both sides, 0 nothing, "
            "r play it again, s skip.")


# ---- measuring haptics ------------------------------------------------------

@dataclasses.dataclass(frozen=True)
class MotionSample:
    """Raw counts of the motion node at one frame."""
    time_ns: int
    accel: tuple
    gyro: tuple


CHANNELS = ("accel", "gyro")


def motion_sample(values, time_ns):
    return MotionSample(
        time_ns,
        tuple(values.get(code, 0) for code in (sc2.ABS_X, sc2.ABS_Z,
                                               sc2.ABS_RX)),
        tuple(values.get(code, 0) for code in (sc2.ABS_RY, sc2.ABS_THROTTLE,
                                               sc2.ABS_RUDDER)))


def differences(samples, channel):
    """(time, size of the change since the previous sample) per sample."""
    return [(sample.time_ns, math.dist(getattr(previous, channel),
                                       getattr(sample, channel)))
            for previous, sample in zip(samples, samples[1:])]


def envelope(points, window_s=ENVELOPE_WINDOW_S):
    """Root mean square over a window centred on each point."""
    half = int(window_s * 1e9 / 2)
    result, low, high, total = [], 0, 0, 0.0
    for time_ns, _ in points:
        while high < len(points) and points[high][0] <= time_ns + half:
            total += points[high][1] ** 2
            high += 1
        while points[low][0] < time_ns - half:
            total -= points[low][1] ** 2
            low += 1
        result.append((time_ns, math.sqrt(max(total, 0.0) / (high - low))))
    return result


def noise_threshold(points):
    values = sorted(value for _, value in points)
    if not values:
        return MINIMUM_ACTIVITY
    return max(sc2.percentile(values, NOISE_PERCENTILE) * NOISE_MARGIN,
               MINIMUM_ACTIVITY)


def active_span(points, threshold, start_ns, end_ns):
    """The first and last time above threshold within the window, or None."""
    active = [time_ns for time_ns, value in points
              if start_ns <= time_ns <= end_ns and value > threshold]
    return (active[0], active[-1]) if active else None


def count_pulses(points, threshold, start_ns, end_ns, min_gap_ns):
    """Active stretches separated by at least min_gap_ns of quiet."""
    count, last_active = 0, None
    for time_ns, value in points:
        if not start_ns <= time_ns <= end_ns or value <= threshold:
            continue
        if last_active is None or time_ns - last_active >= min_gap_ns:
            count += 1
        last_active = time_ns
    return count


def milliseconds(nanoseconds):
    return round(nanoseconds / 1e6, 1)


def envelope_record(points, zero_ns):
    return [[milliseconds(time_ns - zero_ns), round(value, 2)]
            for time_ns, value in points]


def sustained_level(points, hold_ns):
    """The highest value the points stay at or above for hold_ns."""
    best = 0.0
    for index, (start_ns, _) in enumerate(points):
        stretch = [value for time_ns, value in points[index:]
                   if time_ns - start_ns <= hold_ns]
        if points[-1][0] - start_ns < hold_ns:
            break
        best = max(best, min(stretch))
    return best


def calibrate(noise, samples, command_ns, end_ns):
    """Per channel, the noise threshold and how far a known effect rises
    above it; the better channel decides whether effects are measurable.
    The effect runs from its command, at the earliest ONSET_EARLY_S before,
    to end_ns at the latest."""
    channels = {}
    for channel in CHANNELS:
        threshold = noise_threshold(envelope(differences(noise, channel)))
        inside = [(time_ns, value) for time_ns, value
                  in envelope(differences(samples, channel))
                  if command_ns - ONSET_EARLY_S * 1e9 <= time_ns <= end_ns]
        level = sustained_level(inside, int(CALIBRATION_SUSTAIN_S * 1e9))
        channels[channel] = {"threshold": round(threshold, 2),
                             "effect_level": round(level, 2),
                             "ratio": round(level / threshold, 2),
                             "noise_samples": len(noise),
                             "effect_samples": len(inside)}
    best = max(CHANNELS, key=lambda channel: channels[channel]["ratio"])
    return {"channels": channels, "channel": best,
            "threshold": channels[best]["threshold"],
            "detectable": channels[best]["ratio"] >= DETECTABLE_RATIO}


def duration_tolerance(seconds):
    return max(DURATION_TOLERANCE_S, DURATION_TOLERANCE_FRACTION * seconds)


def measure_effect(samples, channel, threshold, command_ns, end_ns, active_s,
                   pulses=0, pulse_gap_s=0.0):
    """Onset, offset and pulses of the activity after a command, and whether
    the second after the commanded end (plus tolerance) stayed quiet. The
    command allows any duration from active_s to its end, end_ns."""
    points = envelope(differences(samples, channel))
    early = int(ONSET_EARLY_S * 1e9)
    last_ns = samples[-1].time_ns if samples else command_ns
    span = active_span(points, threshold, command_ns - early, last_ns)
    tolerance_s = duration_tolerance(active_s)
    quiet_from = end_ns + int(tolerance_s * 1e9)
    quiet_to = quiet_from + int(QUIET_AFTER_S * 1e9)
    late = active_span(points, threshold, quiet_from, quiet_to)
    measured = {
        "channel": channel, "threshold": round(threshold, 2),
        "peak": round(max((value for time_ns, value in points
                           if time_ns >= command_ns - early), default=0.0), 2),
        "commanded_ms": round(active_s * 1000, 1),
        "commanded_until_ms": round(max(end_ns - command_ns,
                                        active_s * 1e9) / 1e6, 1),
        "tolerance_ms": round(tolerance_s * 1000, 1),
        "onset_ms": None, "offset_ms": None, "duration_ms": None,
        "pulses": None, "commanded_pulses": pulses or None,
        "quiet_after": late is None and last_ns >= quiet_to,
        "late_activity_ms": None if late is None
        else milliseconds(late[0] - command_ns),
        "recorded_until_ms": milliseconds(last_ns - command_ns),
        "envelope_ms_value": envelope_record(points, command_ns),
    }
    if span:
        measured.update(onset_ms=milliseconds(span[0] - command_ns),
                        offset_ms=milliseconds(span[1] - command_ns),
                        duration_ms=milliseconds(span[1] - span[0]))
    if span and pulses:
        measured["pulses"] = count_pulses(points, threshold, *span,
                                          int(pulse_gap_s * 1e9))
    return measured


def commanded_text(measured):
    shortest = measured["commanded_ms"]
    longest = measured["commanded_until_ms"]
    if longest == shortest:
        return f"commanded {shortest} ms"
    return f"commanded {shortest} to {longest} ms"


def effect_verdict(measured):
    """fail only for activity seen that disagrees with the command; nothing
    seen of an effect that should play is not measurable."""
    commanded = measured["commanded_ms"]
    if commanded and measured["onset_ms"] is None:
        return "skipped", (f"not measurable: no activity above the noise "
                           f"threshold (peak {measured['peak']}, threshold "
                           f"{measured['threshold']})")
    problems = []
    tolerance = measured["tolerance_ms"]
    if not commanded:
        if measured["onset_ms"] is not None:
            problems.append(f"activity from {measured['onset_ms']} ms where "
                            "nothing should play")
    else:
        if not (commanded - tolerance <= measured["duration_ms"]
                <= measured["commanded_until_ms"] + tolerance):
            problems.append(f"lasted {measured['duration_ms']} ms, "
                            + commanded_text(measured))
        # A train ends with one more edge, back to zero (pulse_effect).
        pulses = measured["commanded_pulses"]
        if pulses and measured["pulses"] not in (pulses, pulses + 1):
            problems.append(f"{measured['pulses']} pulses, commanded "
                            f"{pulses}")
    if not measured["quiet_after"]:
        late = measured["late_activity_ms"]
        problems.append("not quiet in the second after the commanded end ("
                        + (f"activity at {late} ms" if late is not None
                           else "recording too short") + ")")
    return ("fail", "; ".join(problems)) if problems else ("pass", "")


def measure_stop(samples, channel, threshold, train_ns, stop_ns,
                 train_end_ns, train_active_s):
    """What a train running at the stop did: stopped early, or ran on to its
    commanded end (from train_active_s, less the tolerance). An experiment
    with no expected answer: 'observed', or 'skipped' when the train was
    not seen before the stop."""
    points = envelope(differences(samples, channel))
    early = int(ONSET_EARLY_S * 1e9)
    last_ns = samples[-1].time_ns if samples else stop_ns
    before = active_span(points, threshold, train_ns - early, stop_ns)
    span = active_span(points, threshold, train_ns - early, last_ns)
    natural_end_ns = train_ns + int((train_active_s - duration_tolerance(
        train_active_s)) * 1e9)
    measured = {"channel": channel, "threshold": round(threshold, 2),
                "stop_at_ms": milliseconds(stop_ns - train_ns),
                "train_end_ms": milliseconds(train_end_ns - train_ns),
                "last_activity_ms": None if span is None
                else milliseconds(span[1] - train_ns),
                "stopped_early": None,
                "envelope_ms_value": envelope_record(points, train_ns)}
    if before is None:
        return "skipped", "the train was not detected before the stop", \
            measured
    last = measured["last_activity_ms"]
    measured["stopped_early"] = span[1] < natural_end_ns
    if measured["stopped_early"]:
        return "observed", (f"stopped early at {last} ms (the stop went "
                            f"out at {measured['stop_at_ms']} ms)"), measured
    return "observed", (f"ran on to the commanded end (last activity at "
                        f"{last} ms, commanded end "
                        f"{measured['train_end_ms']} ms)"), measured


# ---- the haptic link --------------------------------------------------------

@dataclasses.dataclass
class HapticParts:
    """A Client, a recorder of the motion node, the clock both share."""
    client: object
    recorder: object
    clock_ns: object
    close: object


@dataclasses.dataclass(frozen=True)
class Fired:
    """The command's acknowledgement, the latest end the command allows and
    the shortest activity it allows."""
    command_ns: int
    end_ns: int
    active_s: float


class HapticRunner:
    """Sends effects in one session, kept with NOPs, and records motion."""

    def __init__(self, parts, say):
        self.parts = parts
        self.client = parts.client
        self.link = parts.client.link
        self.recorder = parts.recorder
        self.clock_ns = parts.clock_ns
        self.say = say
        self.log = io.StringIO()
        self.in_message = False

    def send(self, words, label):
        """One message, or False. The Client's lines go to the report.

        After a word the dongle did not take, silence until the session
        times out (docs/PROTOCOL.md 9.5.1): the timeout stops the haptics,
        a running rumble included, where NOPs would keep it going.
        """
        with contextlib.redirect_stdout(self.log):
            self.in_message = True
            acks = self.client.message(list(words), label, show=False)
            self.in_message = False
        if acks is None:
            self.link.quiet(sc2.SILENCE_S)
        return acks is not None

    def keep(self, seconds, session=True):
        deadline = self.clock_ns() + int(seconds * 1e9)
        while self.clock_ns() < deadline:
            self.link.quiet(min(sc2.IDLE_NOP_S,
                                (deadline - self.clock_ns()) / 1e9))
            if session:
                self.send([sc2.NOP], "NOP")

    def samples(self, start_ns):
        return self.recorder.samples(start_ns, self.clock_ns())

    def fire(self, effect):
        """The effect's messages; a rumble also gets its stop, after which it
        may play RUMBLE_AFTER_STOP_S on. None when the dongle did not take a
        message."""
        for words in effect.messages:
            if not self.send(words, effect.key):
                return None
        command_ns = self.clock_ns()
        if not effect.stop_after_s:
            return Fired(command_ns, command_ns + int(effect.total_s * 1e9),
                         effect.active_s)
        self.link.quiet(effect.stop_after_s)
        if not self.send(sc2.encode_rumble_stop(), "rumble stop"):
            return None
        stop_ns = self.clock_ns()
        return Fired(command_ns, stop_ns + int(RUMBLE_AFTER_STOP_S * 1e9),
                     (stop_ns - command_ns) / 1e9)

    def settle(self, channel, threshold):
        """PRE_ROLL_S without activity before an effect, False if the
        controller kept moving."""
        for attempt in range(SETTLE_TRIES):
            start_ns = self.clock_ns()
            self.keep(PRE_ROLL_S)
            points = envelope(differences(self.samples(start_ns), channel))
            if all(value <= threshold for _, value in points):
                return True
            if attempt == 0:
                self.say("Keep your hands off the controller.")
        return False

    def run_effect(self, effect, channel, threshold):
        settled = self.settle(channel, threshold)
        pre_ns = self.clock_ns() - int(PRE_ROLL_S * 1e9)
        fired = self.fire(effect)
        if fired is None:
            return None, settled, []
        self.keep(max(fired.end_ns - self.clock_ns(), 0) / 1e9
                  + QUIET_AFTER_S + AFTER_MARGIN_S)
        return fired, settled, self.samples(pre_ns)

    def run_stop(self, candidate, channel, threshold):
        """The train, the candidate at STOP_AT_S, quiet to well past the
        train's commanded end. Returns (fired, stop_ns or None, settled,
        samples), stop_ns when the candidate's first word went out."""
        settled = self.settle(channel, threshold)
        pre_ns = self.clock_ns() - int(PRE_ROLL_S * 1e9)
        fired = self.fire(STOP_TRAIN)
        if fired is None:
            return None, None, settled, []
        self.keep(STOP_AT_S - (self.clock_ns() - fired.command_ns) / 1e9)
        sent_ns = self.clock_ns()
        sent = all(self.send(words, candidate.key)
                   for words in candidate.messages)
        stop_ns = sent_ns if sent else None
        self.keep((fired.end_ns - self.clock_ns()) / 1e9 + QUIET_AFTER_S
                  + AFTER_MARGIN_S, session=not candidate.ends_session)
        return fired, stop_ns, settled, self.samples(pre_ns)

    def stop_safely(self):
        """As a session end: BYE, whose end the dongle follows with its
        stop. Inside a message a word could complete it as payload, so
        silence lets the timeout drop it and stop instead."""
        if self.in_message:
            self.link.quiet(sc2.SILENCE_S)
        elif self.client.in_session():
            self.send([sc2.BYE], "BYE")

    def close(self):
        try:
            self.stop_safely()
        finally:
            self.parts.close()


# ---- the check --------------------------------------------------------------

def new_step(section, key, instruction):
    return {"section": section, "key": key, "instruction": instruction,
            "result": None, "reason": "", "observation": {}}


def count_seen(seen, signature):
    """seen with one more sighting of signature, in first-seen order."""
    counts = {entry["input"]: entry["count"] for entry in seen}
    counts[signature] = counts.get(signature, 0) + 1
    return [{"input": name, "count": count} for name, count in counts.items()]


def declared_fault(seen):
    """The reason of a step the person failed with f."""
    reason = "f pressed: doing exactly this, not seen; "
    if seen:
        return reason + "seen instead: " + seen_text(seen)
    return reason + "no other input seen"


THRESHOLD_NAMES = (
    "STICK_CENTRE_MAX", "STICK_FULL_MIN", "STICK_OTHER_AXIS_MAX", "HALF_MIN",
    "TRIGGER_RELEASED_MAX", "TRIGGER_FULL_MIN", "PAD_EDGE_MIN",
    "PAD_OTHER_AXIS_MAX", "OTHER_ANALOG_ACTIVE_MIN", "GYRO_TURN_MIN_DEG",
    "GYRO_TURN_MAX_DEG", "GYRO_OTHER_AXIS_MAX_DEG", "STILL_RATE_DEG_S",
    "FLAT_RETURN_DEG", "ACCEL_FLAT_TOLERANCE_G", "ACCEL_POSE_MIN_G",
    "DURATION_TOLERANCE_S", "DURATION_TOLERANCE_FRACTION",
    "PULSE_MIN_SAMPLES_PER_PERIOD", "ENVELOPE_WINDOW_S", "NOISE_PERCENTILE",
    "NOISE_MARGIN", "DETECTABLE_RATIO", "CALIBRATION_SUSTAIN_S", "STOP_AT_S",
    "QUIET_AFTER_S", "MAX_EFFECT_S")


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(
        timespec="seconds")


def new_report(dongle):
    return {"tool": "sc2link_test.py check", "started": utc_now(),
            "finished": None, "interrupted": False, "error": None,
            "dongle": dongle, "result": None,
            "thresholds": {name: globals()[name] for name in THRESHOLD_NAMES},
            "findings": [], "steps": [], "haptics": {}}


class Check:
    def __init__(self, console, open_haptics, report):
        self.console = console
        self.open_haptics = open_haptics
        self.report = report
        self.values = console.initial_values()
        self.snapshot = snapshot_from(self.values, "start",
                                      console.clock_ns())
        self.stuck = Stuck()
        self.rest = None
        self.haptics = None
        self.current = None

    def run(self):
        self.console.say(INTRODUCTION)
        self.run_steps(SECTIONS[0], DIGITAL_STEPS, TABLE_PROMPT)
        self.run_steps(SECTIONS[1], ANALOG_STEPS, TABLE_PROMPT)
        self.motion()
        self.haptic_sections()

    # -- input ----------------------------------------------------------------

    def next_snapshots(self, seconds=POLL_S):
        frames, keys = self.console.events(seconds)
        snapshots = []
        for role, time_ns, changes in frames:
            self.values[role].update(changes)
            self.snapshot = snapshot_from(self.values, role, time_ns)
            snapshots.append(self.snapshot)
        return snapshots, keys

    def watch(self, feed, waiting_for, keys=(SKIP_KEY,),
              hint="Press s to skip this step."):
        """The first outcome feed returns, or the first of keys pressed."""
        next_reminder = self.console.clock_ns() + int(REMIND_AFTER_S * 1e9)
        while True:
            snapshots, pressed = self.next_snapshots()
            for key in keys:
                if key in pressed:
                    return key
            for snapshot in snapshots:
                outcome = feed(snapshot)
                if outcome is not None:
                    return outcome
            if self.console.clock_ns() >= next_reminder:
                self.console.say(f"Still waiting for: {waiting_for} {hint}")
                next_reminder += int(REMIND_AFTER_S * 1e9)

    def wait_neutral(self, prompt, ignore_bits=0, ignore_analog=(),
                     section=None):
        """Until nothing counts as active for NEUTRAL_HOLD_S: None. s records
        what is active as stuck when section names one; for a step, s while
        something is active or f ends the wait and is returned."""
        started = self.console.clock_ns()
        next_reminder = started + int(REMIND_AFTER_S * 1e9)
        neutral_since, prompted = None, False
        while True:
            snapshots, keys = self.next_snapshots()
            for snapshot in snapshots:
                if active_controls(snapshot, self.stuck, ignore_bits,
                                   ignore_analog):
                    neutral_since = None
                elif neutral_since is None:
                    neutral_since = snapshot.time_ns
            now = self.console.clock_ns()
            if (neutral_since is not None
                    and now - neutral_since >= NEUTRAL_HOLD_S * 1e9):
                return None
            active = active_controls(self.snapshot, self.stuck, ignore_bits,
                                     ignore_analog)
            if section is None and FAIL_KEY in keys:
                return FAIL_KEY
            if SKIP_KEY in keys and active:
                if section is None:
                    return SKIP_KEY
                self.record_stuck(section, active)
                continue
            if active and not prompted and \
                    now - started >= NEUTRAL_PROMPT_S * 1e9:
                self.console.say(f"{prompt} Still active: "
                                 f"{active.describe()}.")
                prompted = True
            if active and now >= next_reminder:
                hint = ("Or press s to record them as stuck and go on."
                        if section else KEY_LEGEND)
                self.console.say(f"Still active: {active.describe()}. "
                                 f"{prompt} {hint}")
                next_reminder = now + int(REMIND_AFTER_S * 1e9)

    def record_stuck(self, section, active):
        if not (active.bits or active.analog):
            return
        self.report["findings"].append({"section": section,
                                        "stuck_active": active.names()})
        self.stuck = Stuck(self.stuck.bits | active.bits,
                           self.stuck.analog | frozenset(active.analog))
        self.console.say(f"Recorded as stuck: {active.describe()}.")

    def wait_rest(self, waiting_for):
        outcome = self.watch(RestWatcher(REST_S).feed, waiting_for)
        if outcome == SKIP_KEY:
            return None
        return RestReference(outcome.observation["gyro_bias_deg_s"],
                             outcome.observation["accel_g"])

    def wait_ready(self, step):
        """The neutral state a step starts from: None, or the key (s or f)
        that ended the wait."""
        if isinstance(step, TurnStep):
            prompt = "Lay the controller flat on the table and let go."
            outcome = self.watch(RestWatcher(STILL_HOLD_S, self.rest).feed,
                                 prompt, STEP_KEYS, KEY_LEGEND)
            return outcome if outcome in STEP_KEYS else None
        return self.wait_neutral(*step_neutral(step))

    # -- steps ----------------------------------------------------------------

    def begin_step(self, section, key, instruction):
        """A new step, the one in progress until it finishes."""
        self.current = new_step(section, key, instruction)
        return self.current

    def finish(self, result, verdict, reason=""):
        result["result"] = verdict
        result["reason"] = reason
        self.report["steps"].append(result)
        return result

    def fail_current(self, error):
        """The step in progress, if any, failed by error."""
        if self.current is not None and self.current["result"] is None:
            self.finish(self.current, "fail", f"stopped by an error: "
                        f"{error!r}")

    def skip(self, section, key, instruction, reason):
        return self.finish(new_step(section, key, instruction), "skipped",
                           reason)

    def begin_section(self, section, prompt):
        self.console.say(f"\n{section}\n{prompt}")
        self.wait_neutral(prompt, section=section)

    def run_steps(self, section, steps, prompt):
        self.begin_section(section, prompt)
        for number, step in enumerate(steps, 1):
            self.run_step(section, step, f"[{section[0]}.{number}/"
                          f"{len(steps)}]")

    def run_step(self, section, step, label):
        """Passes on the expected input; other input is shown and recorded
        in seen, and the step waits on. Only the person fails it, with f."""
        result = self.begin_step(section, step.key, step.instruction)
        result["seen"] = []
        stuck = getattr(step, "expected", 0) & self.stuck.bits
        if stuck:
            return self.finish(result, "fail", f"{describe_bits(stuck)} was "
                               "stuck active before the section")
        shown = False
        while True:
            outcome = self.wait_ready(step)
            if outcome == FAIL_KEY and not shown:
                continue  # a late f from the step before, not about this one
            if outcome is None:
                if not shown:
                    self.console.flush_keys()
                    self.console.show_step(step, f"{label} {step.instruction}")
                    shown = True
                outcome = self.watch(
                    watcher_for(step, self.stuck, self.rest).feed,
                    step.instruction, STEP_KEYS, KEY_LEGEND)
            if outcome == SKIP_KEY:
                return self.finish(result, "skipped", "skipped with s")
            if outcome == FAIL_KEY:
                return self.finish(result, "fail",
                                   declared_fault(result["seen"]))
            result["observation"] = outcome.observation
            if outcome.matched:
                return self.finish(result, "pass")
            result["seen"] = count_seen(result["seen"], outcome.signature)
            self.console.say(f"I see: {outcome.signature}. Waiting for: "
                             f"{step_target(step)}.{release_hint(step)}")

    # -- sections 3 and 4 -----------------------------------------------------

    def motion(self):
        gyro, accel = SECTIONS[2], SECTIONS[3]
        self.begin_section(gyro, FLAT_PROMPT)
        self.rest = self.wait_rest(FLAT_PROMPT)
        poses = {}
        for number, step in enumerate(TURN_STEPS, 1):
            if self.rest is None:
                self.skip(gyro, step.key, step.instruction,
                          "skipped while waiting for the controller to rest")
                continue
            result = self.run_step(gyro, step, f"[3.{number}/"
                                   f"{len(TURN_STEPS)}]")
            if result["result"] == "pass":
                poses[step.key] = result["observation"]["pose"]["accel_g"]
        verdict, checks = accel_verdict(
            self.rest.accel if self.rest else None, poses)
        result = new_step(accel, "gravity", "no action: measured at rest and "
                          "in the held poses of section 3")
        result["observation"] = {
            "checks": checks,
            "rest": None if self.rest is None else rounded(
                dataclasses.asdict(self.rest))}
        reasons = [f"{name}: {check.get('reason') or check['result']}"
                   for name, check in checks.items()
                   if check["result"] != "pass"]
        self.finish(result, verdict, "; ".join(reasons))
        self.console.say(f"\n{accel}: {verdict}")

    # -- sections 5 and 6 -----------------------------------------------------

    def haptic_sections(self):
        measured, located = SECTIONS[4], SECTIONS[5]
        self.begin_section(measured, FLAT_PROMPT)
        resting = self.wait_rest(FLAT_PROMPT) is not None
        try:
            self.haptics = HapticRunner(self.open_haptics(), self.console.say)
        except (OSError, SystemExit) as error:
            reason = f"no haptic link: {error}"
            self.skip_measured(measured, reason)
            self.skip_located(located, reason)
            return
        if resting:
            self.measured_haptics(measured)
        else:
            self.skip_measured(measured, "skipped while waiting for the "
                               "controller to rest")
        self.begin_section(located, TABLE_PROMPT)
        self.located_haptics(located)

    def skip_measured(self, section, reason, calibration=True):
        if calibration:
            self.skip(section, "calibration", CALIBRATION_EFFECT.description,
                      reason)
        for effect in MEASURED_EFFECTS:
            self.skip(section, effect.key, effect.description, reason)
        for candidate in STOP_CANDIDATES:
            self.skip(section, STOP_PREFIX + candidate.key, candidate.key,
                      reason)

    def skip_located(self, section, reason):
        for number, (effect, _) in enumerate(LOCATE_EFFECTS, 1):
            self.skip(section, effect.key, f"effect {number}", reason)

    def calibrate(self, section):
        """The noise at rest and one known effect: the channel and threshold
        to measure with, or None when effects are not measurable."""
        runner = self.haptics
        self.console.say("Measuring the noise at rest, then a short rumble. "
                         "Keep your hands off the controller.")
        result = self.begin_step(section, "calibration",
                                 CALIBRATION_EFFECT.description)
        runner.send([sc2.NOP], "NOP")
        start_ns = runner.clock_ns()
        runner.keep(NOISE_S)
        noise = runner.samples(start_ns)
        pre_ns = runner.clock_ns()
        runner.keep(PRE_ROLL_S)
        fired = runner.fire(CALIBRATION_EFFECT)
        if fired is None:
            self.finish(result, "fail", "the dongle did not take the rumble")
            return None
        runner.keep(QUIET_AFTER_S)
        calibration = calibrate(noise, runner.samples(pre_ns),
                                fired.command_ns, fired.end_ns)
        result["observation"] = calibration
        self.report["haptics"]["calibration"] = calibration
        if calibration["detectable"]:
            self.finish(result, "pass")
            return calibration
        best = calibration["channels"][calibration["channel"]]
        self.finish(result, "skipped", f"not measurable: the rumble rose "
                    f"{best['ratio']} times above the noise threshold, "
                    f"{DETECTABLE_RATIO} needed")
        return None

    def measured_haptics(self, section):
        calibration = self.calibrate(section)
        if calibration is None:
            self.console.say("The motion sensors do not show the haptics, so "
                             "the measured tests are skipped.")
            self.skip_measured(section, "not measurable", calibration=False)
            return
        channel, threshold = calibration["channel"], calibration["threshold"]
        total = len(MEASURED_EFFECTS) + len(STOP_CANDIDATES)
        for number, effect in enumerate(MEASURED_EFFECTS, 1):
            self.console.say(f"[5.{number}/{total}] {effect.key}")
            self.measure_one(section, effect, channel, threshold)
        for number, candidate in enumerate(STOP_CANDIDATES,
                                           len(MEASURED_EFFECTS) + 1):
            self.console.say(f"[5.{number}/{total}] {STOP_PREFIX}"
                             f"{candidate.key}")
            self.stop_one(section, candidate, channel, threshold)

    def measure_one(self, section, effect, channel, threshold):
        result = self.begin_step(section, effect.key, effect.description)
        fired, settled, samples = self.haptics.run_effect(effect, channel,
                                                          threshold)
        if fired is None:
            return self.finish(result, "fail", "the dongle did not take the "
                               "message")
        measured = measure_effect(samples, channel, threshold,
                                  fired.command_ns, fired.end_ns,
                                  fired.active_s, effect.pulses,
                                  effect.pulse_gap_s)
        measured["moving_before"] = not settled
        result["observation"] = measured
        verdict, reason = effect_verdict(measured)
        if not settled:
            reason = "the controller moved before the command; " + reason
        return self.finish(result, verdict, reason)

    def stop_one(self, section, candidate, channel, threshold):
        """An experiment: observed or skipped, never a fault."""
        result = self.begin_step(
            section, STOP_PREFIX + candidate.key, f"{STOP_TRAIN.description}, "
            f"then at {STOP_AT_S} s: {candidate.key}")
        fired, stop_ns, settled, samples = self.haptics.run_stop(
            candidate, channel, threshold)
        if fired is None:
            return self.finish(result, "skipped", "the dongle did not take "
                               "the train")
        if stop_ns is None:
            return self.finish(result, "observed", "the dongle did not take "
                               "the stop")
        verdict, reason, measured = measure_stop(
            samples, channel, threshold, fired.command_ns, stop_ns,
            fired.end_ns, fired.active_s)
        measured["moving_before"] = not settled
        result["observation"] = measured
        if not settled:
            reason = "the controller moved before the train; " + reason
        return self.finish(result, verdict, reason)

    def ask(self, keys, question):
        """One key of keys, keeping the session meanwhile."""
        self.console.flush_keys()
        self.console.say(question)
        next_reminder = time.monotonic() + REMIND_AFTER_S
        while True:
            key = self.console.key(sc2.IDLE_NOP_S)
            if key in keys:
                return key
            self.haptics.send([sc2.NOP], "NOP")
            if time.monotonic() >= next_reminder:
                self.console.say(question)
                next_reminder = time.monotonic() + REMIND_AFTER_S

    def located_haptics(self, section):
        runner = self.haptics
        self.console.say("Pick up the controller and hold it the way you "
                         "normally do.")
        self.ask({" "}, "Press the space bar when you hold it.")
        for number, (effect, place) in enumerate(LOCATE_EFFECTS, 1):
            result = self.begin_step(section, effect.key, f"effect {number}")
            taken = []
            while True:
                self.console.say(f"Effect {number}")
                taken.append(runner.fire(effect) is not None)
                runner.keep(effect.total_s + AFTER_MARGIN_S)
                key = self.ask(set(ANSWERS) | {REPLAY_KEY, SKIP_KEY},
                               QUESTION.format(number=number))
                if key != REPLAY_KEY:
                    break
            answer = ANSWERS.get(key)
            result["observation"] = {"effect": effect.description,
                                     "side": effect.side, "expected": place,
                                     "answer": answer, "plays": len(taken),
                                     "taken": taken}
            if key == SKIP_KEY:
                self.finish(result, "skipped", "skipped with s")
            elif not all(taken):
                self.finish(result, "fail", "the dongle did not take the "
                            f"effect; felt: {answer}")
            elif answer == place:
                self.finish(result, "pass")
            else:
                self.finish(result, "fail", f"felt: {answer}, sent to: "
                            f"{place}")

    def close_haptics(self):
        if self.haptics is not None:
            haptics, self.haptics = self.haptics, None
            try:
                haptics.close()
            finally:
                self.report["haptics"]["link_log"] = haptics.log.getvalue()


# ---- the report -------------------------------------------------------------

RESULTS = ("pass", "fail", "skipped", "observed")


def verdict_of(report):
    """Stop probes are observed, never pass or fail, so they stay out."""
    if report["interrupted"] or report["error"]:
        return "INCOMPLETE"
    results = [step["result"] for step in report["steps"]]
    return "FAIL" if "fail" in results else "PASS"


def stop_probe_lines(steps):
    probes = [step for step in steps if step["key"].startswith(STOP_PREFIX)]
    if not probes:
        return []
    return ["Stop probes, experiments outside the result:"] + [
        f"  {step['key'][len(STOP_PREFIX):]}: " + (
            step["reason"] if step["result"] == "observed"
            else f"{step['result']}, {step['reason']}")
        for step in probes]


def summary_lines(report):
    counts = collections.Counter(step["result"] for step in report["steps"])
    lines = [f"SC2 Link controller check, started {report['started']}",
             f"Result: {verdict_of(report)}, {counts['pass']} passed, "
             f"{counts['fail']} failed, {counts['skipped']} skipped, "
             f"{counts['observed']} observed"
             + (", stopped with Ctrl+C" if report["interrupted"] else "")
             + (f", error {report['error']}" if report["error"] else "")]
    for section in SECTIONS:
        steps = [step for step in report["steps"]
                 if step["section"] == section]
        if not steps:
            continue
        section_counts = collections.Counter(step["result"] for step in steps)
        lines.append(f"{section}: " + ", ".join(
            f"{section_counts[result]} {result}" for result in RESULTS
            if section_counts[result]))
        lines += [f"  {step['result'].upper()} {step['key']}: "
                  f"{step['reason']}"
                  for step in steps if step["result"] != "pass"
                  and not step["key"].startswith(STOP_PREFIX)]
    lines += stop_probe_lines(report["steps"])
    lines += [f"Finding in {finding['section']}: stuck active: "
              + ", ".join(finding["stuck_active"])
              for finding in report["findings"]]
    return lines


def write_report(report, stem):
    json_path = os.path.abspath(stem + ".json")
    text_path = os.path.abspath(stem + ".txt")
    with open(json_path, "w", encoding="utf-8") as file:
        json.dump(report, file, indent=1)
    with open(text_path, "w", encoding="utf-8") as file:
        file.write("\n".join(summary_lines(report)) + "\n")
    return json_path, text_path


def run_check(console, open_haptics, stem, dongle):
    """The whole check; the report is written however it ends. An OSError
    (the dongle gone, a node failing) fails the step in progress and ends
    the check as Ctrl+C does."""
    report = new_report(dongle)
    check = Check(console, open_haptics, report)
    try:
        check.run()
    except KeyboardInterrupt:
        report["interrupted"] = True
        console.say("\nStopped.")
    except OSError as error:
        report["error"] = repr(error)
        check.fail_current(error)
        console.say(f"\nStopped by an error: {error}")
    except Exception as error:
        report["error"] = repr(error)
        raise
    finally:
        # Closing a link that is gone fails too, and must not cost the report.
        with contextlib.suppress(KeyboardInterrupt, OSError):
            check.close_haptics()
        report["finished"] = utc_now()
        report["result"] = verdict_of(report)
        paths = write_report(report, stem)
        for line in summary_lines(report):
            console.say(line)
        console.say(f"Report: {paths[0]}\nSummary: {paths[1]}")
    return 0 if report["result"] == "PASS" else 1


# ---- the hardware -----------------------------------------------------------

def read_values(fd):
    return {code: sc2.abs_value(fd, code) for code in sc2.abs_codes(fd)}


class TerminalConsole:
    """The controls, pads and motion nodes and single keys from stdin."""

    def __init__(self, dongle, output=sys.stdout):
        self.output = output
        self.role_of, self.kernel_time, self.pending = {}, {}, {}
        self.resyncing = set()
        self.values = {}
        self.poller = select.poll()
        for role in ROLES:
            fd = sc2.open_node(getattr(dongle, role).path, os.O_RDONLY)
            self.role_of[fd] = role
            self.kernel_time[fd] = sc2.use_monotonic_clock(fd)
            self.pending[fd] = {}
            self.values[role] = read_values(fd)
            self.poller.register(fd, select.POLLIN)
        self.stdin = sys.stdin.fileno()
        self.stdin_open = True
        self.saved_terminal = None
        if os.isatty(self.stdin):
            self.saved_terminal = termios.tcgetattr(self.stdin)
            tty.setcbreak(self.stdin)
        self.poller.register(self.stdin, select.POLLIN)

    def close(self):
        if self.saved_terminal is not None:
            termios.tcsetattr(self.stdin, termios.TCSADRAIN,
                              self.saved_terminal)
        for fd in self.role_of:
            os.close(fd)

    def initial_values(self):
        return {role: dict(values) for role, values in self.values.items()}

    def clock_ns(self):
        return time.monotonic_ns()

    def say(self, text):
        print(text, file=self.output, flush=True)

    def show_step(self, step, text):
        self.say(text)

    def events(self, timeout_s):
        """Frames of all nodes in time order, and the keys typed."""
        frames, keys = [], []
        for fd, _ in self.poller.poll(int(timeout_s * 1000)):
            if fd == self.stdin:
                keys += self.read_keys()
            else:
                frames += self.read_frames(fd)
        frames.sort(key=lambda frame: frame[1])
        return frames, keys

    def read_frames(self, fd):
        read_ns = time.monotonic_ns()
        frames = []
        for event_ns, kind, code, value in sc2.read_events(fd):
            time_ns = event_ns if self.kernel_time[fd] else read_ns
            if kind == sc2.EV_SYN and code == sc2.SYN_DROPPED:
                self.resyncing.add(fd)
                self.pending[fd] = {}
            elif kind == sc2.EV_SYN and code == sc2.SYN_REPORT:
                if fd in self.resyncing:
                    self.resyncing.discard(fd)
                    self.pending[fd] = read_values(fd)
                frames.append((self.role_of[fd], time_ns, self.pending[fd]))
                self.pending[fd] = {}
            elif kind == sc2.EV_ABS and fd not in self.resyncing:
                self.pending[fd][code] = value
        return frames

    def read_keys(self):
        data = os.read(self.stdin, 64)
        if not data:
            self.stdin_open = False
            self.poller.unregister(self.stdin)
        return [character for character in data.decode("utf-8",
                                                       "replace").lower()
                if character not in "\r\n"]

    def key(self, timeout_s):
        if not self.stdin_open:
            time.sleep(timeout_s)
            return None
        if not select.select([self.stdin], [], [], timeout_s)[0]:
            return None
        keys = self.read_keys()
        return keys[0] if keys else None

    def flush_keys(self):
        if self.saved_terminal is not None:
            termios.tcflush(self.stdin, termios.TCIFLUSH)


class MotionRecorder:
    """Every frame of the motion node, read by a thread, so that sending
    words cannot overflow the node's small evdev buffer."""

    def __init__(self, path):
        self.fd = sc2.open_node(path, os.O_RDONLY)
        self.kernel_time = sc2.use_monotonic_clock(self.fd)
        self.values = read_values(self.fd)
        self.recorded = []
        self.dropped = 0
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        poller = select.poll()
        poller.register(self.fd, select.POLLIN)
        resyncing = False
        while self.running:
            if not poller.poll(50):
                continue
            read_ns = time.monotonic_ns()
            try:
                events = sc2.read_events(self.fd)
            except OSError:
                return
            for event_ns, kind, code, value in events:
                if kind == sc2.EV_SYN and code == sc2.SYN_DROPPED:
                    resyncing = True
                    self.dropped += 1
                elif kind == sc2.EV_SYN and code == sc2.SYN_REPORT:
                    if resyncing:
                        resyncing = False
                        self.values = read_values(self.fd)
                    sample = motion_sample(
                        self.values, event_ns if self.kernel_time else read_ns)
                    with self.lock:
                        self.recorded.append(sample)
                elif kind == sc2.EV_ABS and not resyncing:
                    self.values[code] = value

    def samples(self, start_ns, end_ns):
        with self.lock:
            return [sample for sample in self.recorded
                    if start_ns <= sample.time_ns <= end_ns]

    def stop(self):
        self.running = False
        self.thread.join(1.0)
        os.close(self.fd)


def default_stem():
    return "sc2link-check-" + datetime.datetime.now(
        datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")


def command_check(args):
    dongle = sc2.choose_dongle(args.phys)

    def open_haptics():
        link = sc2.Link(dongle)
        try:
            recorder = MotionRecorder(dongle.motion.path)
        except BaseException:
            link.close()
            raise

        def close():
            recorder.stop()
            link.close()
        return HapticParts(sc2.Client(link), recorder, time.monotonic_ns,
                           close)

    description = {"prefix": dongle.prefix,
                   "nodes": {role: node.path for role, node in dongle.nodes()}}
    console = TerminalConsole(dongle)
    try:
        return run_check(console, open_haptics, args.report or default_stem(),
                         description)
    finally:
        console.close()
