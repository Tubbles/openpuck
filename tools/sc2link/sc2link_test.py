#!/usr/bin/env python3
"""Test the SC2 Link word protocol of an OpenPuck dongle from a Linux host.

Works through the event nodes the mode creates (docs/PROTOCOL.md section 9.5):
words go out as xpad rumble on the Xbox 360 node and come back acknowledged on
the pads node. Needs read and write access to /dev/input/event* (root, or the
input group) and nothing beyond the Python 3 standard library. The ioctl
numbers follow the generic Linux encoding used on x86 and ARM.

A session ends 500 ms after the last word, so every command that needs one
opens its own after 600 ms of silence.
"""

import argparse
import binascii
import collections
import dataclasses
import errno
import fcntl
import glob
import math
import os
import re
import select
import struct
import sys
import time

# ---- the dongle -------------------------------------------------------------

XPAD_VENDOR = 0x045E
XPAD_PRODUCT = 0x028E
# bcdDevice of SC2 Link, the evdev version of the xpad node.
XPAD_VERSION = 0x5C21
NODE_SUFFIXES = {
    "xpad": "/input0",
    "controls": "/input1",
    "pads": "/input2",
    "motion": "/input3",
}

# ---- evdev ------------------------------------------------------------------

IOC_WRITE = 1
IOC_READ = 2

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03
EV_FF = 0x15
SYN_REPORT = 0
SYN_DROPPED = 3
FF_RUMBLE = 0x50
FF_GAIN = 0x60
CLOCK_MONOTONIC = 1

ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05
ABS_THROTTLE = 0x06
ABS_RUDDER = 0x07
ABS_WHEEL = 0x08
ABS_GAS = 0x09
ABS_BRAKE = 0x0A
ABS_HAT0X = 0x10
ABS_HAT0Y = 0x11
ABS_MISC = 0x28
ABS_COUNT = 0x40
ABS_NAMES = {
    ABS_X: "ABS_X", ABS_Y: "ABS_Y", ABS_Z: "ABS_Z", ABS_RX: "ABS_RX",
    ABS_RY: "ABS_RY", ABS_RZ: "ABS_RZ", ABS_THROTTLE: "ABS_THROTTLE",
    ABS_RUDDER: "ABS_RUDDER", ABS_WHEEL: "ABS_WHEEL", ABS_GAS: "ABS_GAS",
    ABS_BRAKE: "ABS_BRAKE", ABS_HAT0X: "ABS_HAT0X", ABS_HAT0Y: "ABS_HAT0Y",
    ABS_MISC: "ABS_MISC",
}
KEY_NAMES = {
    0x130: "BTN_A", 0x131: "BTN_B", 0x133: "BTN_X", 0x134: "BTN_Y",
    0x136: "BTN_TL", 0x137: "BTN_TR", 0x13A: "BTN_SELECT",
    0x13B: "BTN_START", 0x13C: "BTN_MODE", 0x13D: "BTN_THUMBL",
    0x13E: "BTN_THUMBR", 0x2C0: "BTN_TRIGGER_HAPPY1",
    0x2C1: "BTN_TRIGGER_HAPPY2", 0x2C2: "BTN_TRIGGER_HAPPY3",
    0x2C3: "BTN_TRIGGER_HAPPY4",
}

# struct input_event: struct timeval, then type, code, value. 24 bytes on
# 64-bit Linux, 16 on 32-bit.
INPUT_EVENT = struct.Struct("@llHHi")
# struct input_absinfo: value, minimum, maximum, fuzz, flat, resolution.
INPUT_ABSINFO = struct.Struct("@6i")
# struct input_id: bustype, vendor, product, version.
INPUT_ID = struct.Struct("@4H")
# struct ff_effect with its largest union member, ff_periodic_effect, whose
# custom_data pointer aligns the union: 48 bytes on 64-bit Linux.
FF_EFFECT = struct.Struct("@HhHHHHH0PHHhhHHHHHIP")
# type, id, direction, trigger button and interval, replay length and delay.
FF_EFFECT_HEADER = struct.Struct("@HhHHHHH")
# ff_rumble_effect's strong and weak magnitudes open the union.
FF_RUMBLE_OFFSET = struct.calcsize("@HhHHHHH0P")
FF_RUMBLE = struct.Struct("@HH")
FF_RUMBLE_TYPE = 0x50


def ioc(direction, number, size):
    return direction << 30 | size << 16 | ord("E") << 8 | number


EVIOCGID = ioc(IOC_READ, 0x02, INPUT_ID.size)
EVIOCSFF = ioc(IOC_WRITE, 0x80, FF_EFFECT.size)
EVIOCSCLOCKID = ioc(IOC_WRITE, 0xA0, struct.calcsize("@i"))


def eviocgname(length):
    return ioc(IOC_READ, 0x06, length)


def eviocgphys(length):
    return ioc(IOC_READ, 0x07, length)


def eviocgbit(event_type, length):
    return ioc(IOC_READ, 0x20 + event_type, length)


def eviocgabs(code):
    return ioc(IOC_READ, 0x40 + code, INPUT_ABSINFO.size)


# ---- the word protocol ------------------------------------------------------

PROTOCOL_VERSION = 1
SESSION_TIMEOUT_S = 0.5
# A client waits this long before its HELLO so any earlier session has ended.
SILENCE_S = SESSION_TIMEOUT_S + 0.1
# Stop-and-wait: with no acknowledgement after this, read ACK_STATE and resend.
RESEND_AFTER_S = 0.030
MAX_RESENDS = 10
IDLE_NOP_S = 0.2
MODE_MAX = 13

# In a session bit 15 of a word is a toggle that flips with each new word,
# bits 14 to 0 are the data. The words below are data.
TOGGLE_SHIFT = 15
DATA = 0x7FFF
# A header with data bit 14 set is a COMMAND, otherwise bits 13 to 11 hold the
# opcode.
COMMAND_BIT = 0x4000
OP_CONTROL = 0
OP_SCRIPT = 1
OP_RUMBLE = 2
OP_PULSE = 3
OP_SWEEP = 4
OP_TONE = 5
OP_COMMAND = 8

CONTROL_NOP = 0x00
CONTROL_BYE = 0x02
CONTROL_CHECKSUM = 0x04
CONTROL_BOOTLOADER = 0x05
CONTROL_MODE = 0x06
BOOTLOADER_CONFIRM = 0x5546
MODE_CONFIRM = 0x4D00

# Fixed words outside the toggle: their toggle is 0, so the first session
# word carries 1.
HELLO = (0x0102, 0x0201, 0x0103, 0x0301)
NOP = CONTROL_NOP << 8
BYE = CONTROL_BYE << 8
CHECKSUM = CONTROL_CHECKSUM << 8
# Outside a session these four, each within 500 ms, reboot into the UF2
# bootloader.
KNOCK = (0x5546, 0x32B0, 0x4F4B, 0x2121)


def header(opcode, bits=0):
    if opcode == OP_COMMAND:
        return COMMAND_BIT | bits
    return opcode << 11 | bits


def side_gain(side, gain):
    return side << 8 | (gain & 0xFF)


def high(value, at):
    """Bit 15 of a 16-bit field, moved to bit `at` of another word."""
    return (value >> 15) << at


def low(value):
    return value & DATA


def encode_command(side, command, gain):
    return [header(OP_COMMAND, command << 10 | side_gain(side, gain))]


def encode_script(side, script, gain):
    return [header(OP_SCRIPT, side_gain(side, gain)), script]


def encode_rumble(left, right):
    return [header(OP_RUMBLE, high(left, 1) | high(right, 0)), low(left),
            low(right)]


def encode_rumble_stop():
    return [header(OP_RUMBLE, 4)]


def encode_pulse(side, on_us, off_us, repeat):
    tops = high(on_us, 2) | high(off_us, 1) | high(repeat, 0)
    return [header(OP_PULSE, side << 8 | tops), low(on_us), low(off_us),
            low(repeat)]


def encode_sweep(side, gain, duration_ms, start_hz, end_hz):
    tops = high(duration_ms, 2) | high(start_hz, 1) | high(end_hz, 0)
    return [header(OP_SWEEP, side_gain(side, gain)), low(duration_ms),
            low(start_hz), low(end_hz), tops]


def encode_tone(side, gain, frequency_hz, duration_ms, lfo_hz, lfo_depth):
    tops = high(frequency_hz, 10) | high(duration_ms, 9) | high(lfo_hz, 8)
    return [header(OP_TONE, side_gain(side, gain)), low(frequency_hz),
            low(duration_ms), low(lfo_hz), tops | lfo_depth]


def stamp(data_words, toggle):
    """Session words for data sent after a word with this toggle.

    Returns the words and the toggle of the last one.
    """
    words = []
    for data in data_words:
        toggle ^= 1
        words.append(toggle << TOGGLE_SHIFT | data)
    return words, toggle


def encode_bootloader():
    return [CONTROL_BOOTLOADER << 8, BOOTLOADER_CONFIRM]


def encode_mode(mode):
    return [CONTROL_MODE << 8 | mode, MODE_CONFIRM | mode]


# The controller reports, report id first, as SDL's controller_structs.h lays
# them out.
def report_command(side, command, gain):
    return struct.pack("<BBBb", 0x82, side, command, gain)


def report_script(side, script, gain):
    return struct.pack("<BBBb", 0x85, side, script, gain)


def report_rumble(left, right):
    kind = 4 if left or right else 0
    return struct.pack("<BBHHbHb", 0x80, kind, max(left, right), left, 0,
                       right, 0)


def report_pulse(side, on_us, off_us, repeat):
    return struct.pack("<BBHHH", 0x81, side, on_us, off_us, repeat)


def report_sweep(side, gain, duration_ms, start_hz, end_hz):
    return struct.pack("<BBbHHH", 0x84, side, gain, duration_ms, start_hz,
                       end_hz)


def report_tone(side, gain, frequency_hz, duration_ms, lfo_hz, lfo_depth):
    return struct.pack("<BBbHHHB", 0x83, side, gain, frequency_hz,
                       duration_ms, lfo_hz, lfo_depth)


# The OFF command a session's start and end send, meant for both pads. Side 3
# of 0x82 is the left internal actuator, so it does not stop the pads.
# CHECKSUM covers the one a session end queues.
STOP_REPORT = report_command(3, 0, 0)


def crc16(data):
    """CRC-16/CCITT-FALSE, the CHECKSUM command's."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


@dataclasses.dataclass(frozen=True)
class Field:
    name: str
    minimum: int
    maximum: int

    @property
    def middle(self):
        return (self.minimum + self.maximum + 1) // 2


def word_field(name):
    return Field(name, 0, 0xFFFF)


# The side byte names other actuators per report (docs/PROTOCOL.md 9.5.1).
# Pulse: 0 right pad, 1 left pad, 2 both pads. The others: 0 left pad, 1 right
# pad, 2 both pads, 3 left internal actuator. None is silent.
SIDE = Field("side", 0, 3)
GAIN = Field("gain", -128, 127)
COMMAND = Field("command", 0, 7)
ON_US = word_field("on_us")
DURATION_MS = word_field("duration_ms")


@dataclasses.dataclass(frozen=True)
class Message:
    name: str
    fields: tuple
    encode: object
    report: object


MESSAGES = {message.name: message for message in (
    Message("command", (SIDE, COMMAND, GAIN),
            encode_command, report_command),
    Message("script", (SIDE, Field("script", 0, 0xFF), GAIN),
            encode_script, report_script),
    Message("rumble", (word_field("left"), word_field("right")),
            encode_rumble, report_rumble),
    Message("pulse", (SIDE, ON_US, word_field("off_us"),
                      word_field("repeat")),
            encode_pulse, report_pulse),
    Message("sweep", (SIDE, GAIN, DURATION_MS,
                      word_field("start_hz"), word_field("end_hz")),
            encode_sweep, report_sweep),
    Message("tone", (SIDE, GAIN, word_field("frequency_hz"),
                     DURATION_MS, word_field("lfo_hz"),
                     Field("lfo_depth", 0, 0xFF)),
            encode_tone, report_tone),
)}


def boundary_cases(message):
    """Every field at its minimum, middle and maximum, one field at a time.

    The other fields stay at their minimum.
    """
    base = tuple(field.minimum for field in message.fields)
    cases = [base]
    for index, field in enumerate(message.fields):
        for value in (field.middle, field.maximum):
            case = base[:index] + (value,) + base[index + 1:]
            if case not in cases:
                cases.append(case)
    return cases


def describe_case(message, values):
    changed = [f"{field.name}={value}"
               for field, value in zip(message.fields, values)
               if value != field.minimum]
    return f"{message.name} " + (", ".join(changed) or "all minimum")


def ack_state(counter, session, expected):
    return ((counter & 0xFF) << 8 | (0x80 if session else 0)
            | (expected & 7) << 4 | PROTOCOL_VERSION)


def decode_state(state):
    return {
        "counter": state >> 8 & 0xFF,
        "session": state >> 7 & 1,
        "expected": state >> 4 & 7,
        "version": state & 0xF,
    }


def format_state(state):
    fields = decode_state(state)
    return (f"counter {fields['counter']:3d} session {fields['session']} "
            f"expected {fields['expected']} v{fields['version']}")


def opcode_of(head):
    return OP_COMMAND if head & COMMAND_BIT else head >> 11 & 7


# Header bits that must be zero, per opcode.
UNUSED_BITS = {OP_COMMAND: 0x2000, OP_SCRIPT: 0x0400, OP_RUMBLE: 0x07F8,
               OP_PULSE: 0x04F8, OP_SWEEP: 0x0400, OP_TONE: 0x0400}
PAYLOAD_WORDS = {OP_COMMAND: 0, OP_SCRIPT: 1, OP_RUMBLE: 2, OP_PULSE: 3,
                 OP_SWEEP: 4, OP_TONE: 4}


def payload_words(head):
    """Payload words after a header, None for a header the dongle rejects."""
    opcode = opcode_of(head)
    if opcode == OP_CONTROL:
        sub, argument = head >> 8 & 7, head & 0xFF
        if sub in (CONTROL_NOP, CONTROL_BYE, CONTROL_CHECKSUM):
            return None if argument else 0
        if sub == CONTROL_BOOTLOADER:
            return None if argument else 1
        if sub == CONTROL_MODE:
            return 1 if argument <= MODE_MAX else None
        return None
    if opcode not in UNUSED_BITS or head & UNUSED_BITS[opcode]:
        return None
    if opcode == OP_RUMBLE and head & 4:
        return None if head & 3 else 0
    return PAYLOAD_WORDS[opcode]


def payload_valid(head, index, data):
    opcode = opcode_of(head)
    if opcode == OP_CONTROL:
        if head >> 8 & 7 == CONTROL_MODE:
            return data == MODE_CONFIRM | (head & 0xFF)
        return data == BOOTLOADER_CONFIRM
    if opcode == OP_SCRIPT:
        return data <= 0xFF
    if opcode == OP_SWEEP and index == 3:
        return data <= 7
    if opcode == OP_TONE and index == 3:
        return data <= 0x7FF
    return True


def signed8(value):
    return value - 0x100 if value & 0x80 else value


def join(tops, at, low_bits):
    """A 16-bit field: bit 15 from bit `at` of tops, the rest from low_bits."""
    return (tops >> at & 1) << 15 | low_bits


def build_report(head, payload):
    """The report a complete message queues, None for CONTROL."""
    opcode = opcode_of(head)
    side, gain = head >> 8 & 3, signed8(head & 0xFF)
    if opcode == OP_COMMAND:
        return report_command(side, head >> 10 & 7, gain)
    if opcode == OP_SCRIPT:
        return report_script(side, payload[0], gain)
    if opcode == OP_RUMBLE:
        if head & 4:
            return report_rumble(0, 0)
        return report_rumble(join(head, 1, payload[0]),
                             join(head, 0, payload[1]))
    if opcode == OP_PULSE:
        return report_pulse(side, *(join(head, 2 - index, payload[index])
                                    for index in range(3)))
    if opcode == OP_SWEEP:
        return report_sweep(side, gain,
                            *(join(payload[3], 2 - index, payload[index])
                              for index in range(3)))
    if opcode == OP_TONE:
        return report_tone(side, gain,
                           *(join(payload[3], 10 - index, payload[index])
                             for index in range(3)),
                           payload[3] & 0xFF)
    return None


@dataclasses.dataclass
class Outcome:
    """A word taken (accepted), a copy acknowledged again, or neither."""
    accepted: bool
    echo: int = 0
    state: int = 0
    copy: bool = False


class Model:
    """What the dongle does with each word, its timeouts and knock aside.

    linked says whether a report reaches the relay: without a linked
    controller the dongle drops it, and CHECKSUM then answers 0xFFFF.
    """

    def __init__(self, counter=0, linked=True):
        self.counter = counter
        self.linked = linked
        self.active = False
        self.hello = 0
        self.head = 0
        self.payload = []
        self.need = 0
        self.toggle = 0
        # The word that ended the last session, BYE or a rejected one,
        # until another word arrives.
        self.end_word = None
        self.echo = 0
        self.queued = b""
        self.reboot = None

    def state(self):
        expected = self.need if self.active else (
            len(HELLO) - self.hello if self.hello else 0)
        return ack_state(self.counter, self.active, expected)

    def end_session(self):
        """BYE, a rejected word or the timeout, with its stop."""
        self.active = False
        self.need = 0
        self.queued = STOP_REPORT if self.linked else b""

    def word(self, word):
        if self._copy(word):
            return Outcome(False, self.echo, self.state(), copy=True)
        if self.active:
            outcome = self._session_word(word)
            if not self.active:
                self.end_word = word
            return outcome
        self.end_word = None
        if not self._hello_step(word):
            return Outcome(False)
        if self.hello == len(HELLO):
            self.active = True
            self.hello = 0
            self.need = 0
            self.toggle = HELLO[-1] >> TOGGLE_SHIFT
        return self._accept(word)

    def _session_word(self, word):
        if self.need:
            return self._payload(word)
        need = payload_words(word & DATA)
        if need is None:
            return self._reject(word)
        self.head, self.payload, self.need = word & DATA, [], need
        self.toggle = word >> TOGGLE_SHIFT
        return self._accept(self._run(word) if not need else word)

    def _copy(self, word):
        if self.active:
            return word >> TOGGLE_SHIFT == self.toggle
        if word == self.end_word:
            return True
        return bool(self.hello) and word == HELLO[self.hello - 1]

    def _hello_step(self, word):
        if word != HELLO[self.hello]:
            self.hello = 0
        if word != HELLO[self.hello]:
            return False
        self.hello += 1
        return True

    def _payload(self, word):
        if not payload_valid(self.head, len(self.payload), word & DATA):
            return self._reject(word)
        self.payload.append(word & DATA)
        self.toggle = word >> TOGGLE_SHIFT
        self.need -= 1
        return self._accept(self._run(word) if not self.need else word)

    def _reject(self, word):
        self.end_session()
        self._hello_step(word)
        return self._accept(word)

    def _run(self, last):
        opcode, sub = opcode_of(self.head), self.head >> 8 & 7
        report = build_report(self.head, self.payload)
        if opcode != OP_CONTROL:
            self.queued = report if self.linked else b""
            return last
        if sub == CONTROL_BYE:
            self.end_session()
        elif sub == CONTROL_CHECKSUM:
            return crc16(self.queued)
        elif sub == CONTROL_BOOTLOADER:
            self.reboot = "bootloader"
        elif sub == CONTROL_MODE:
            self.reboot = self.head & 0xFF
        return last

    def _accept(self, echo):
        self.counter = (self.counter + 1) & 0xFF
        self.echo = echo
        return Outcome(True, echo, self.state())


def parse_fields(message, texts):
    """Field values from the command line, range checked."""
    if len(texts) != len(message.fields):
        names = " ".join(field.name.upper() for field in message.fields)
        raise ValueError(f"{message.name} takes {names}")
    values = []
    for field, text in zip(message.fields, texts):
        value = int(text, 0)
        if not field.minimum <= value <= field.maximum:
            raise ValueError(f"{field.name} must be {field.minimum} to "
                             f"{field.maximum}, not {value}")
        values.append(value)
    return tuple(values)


# ---- node discovery ---------------------------------------------------------

@dataclasses.dataclass
class Node:
    path: str
    name: str
    phys: str
    vendor: int
    product: int
    version: int


@dataclasses.dataclass
class Dongle:
    prefix: str
    xpad: Node
    controls: Node
    pads: Node
    motion: Node

    def nodes(self):
        return (("xpad", self.xpad), ("controls", self.controls),
                ("pads", self.pads), ("motion", self.motion))


def ioctl_string(fd, request_for_length):
    buffer = bytearray(256)
    length = fcntl.ioctl(fd, request_for_length(len(buffer)), buffer, True)
    return bytes(buffer[:max(length - 1, 0)]).split(b"\0")[0].decode(
        "utf-8", "replace")


def read_node(path):
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
    try:
        identity = bytearray(INPUT_ID.size)
        fcntl.ioctl(fd, EVIOCGID, identity, True)
        _, vendor, product, version = INPUT_ID.unpack(identity)
        name = ioctl_string(fd, eviocgname)
        try:
            phys = ioctl_string(fd, eviocgphys)
        except OSError:
            phys = ""
        return Node(path, name, phys, vendor, product, version)
    finally:
        os.close(fd)


def event_number(path):
    match = re.search(r"(\d+)$", path)
    return int(match.group(1)) if match else 0


def scan_nodes():
    nodes, refused = [], []
    for path in sorted(glob.glob("/dev/input/event*"), key=event_number):
        try:
            nodes.append(read_node(path))
        except OSError as error:
            refused.append((path, error))
    return nodes, refused


def is_sc2link_node(node):
    return (node.vendor == XPAD_VENDOR and node.product == XPAD_PRODUCT
            and node.version == XPAD_VERSION)


def find_dongles(nodes):
    """Nodes of one USB device share the phys prefix, the interface differs."""
    ours = [node for node in nodes if is_sc2link_node(node)]
    # usbhid reports the HID version (0x0111) on its nodes, not bcdDevice.
    hid_nodes = [node for node in nodes
                 if (node.vendor, node.product) == (XPAD_VENDOR, XPAD_PRODUCT)]
    dongles = []
    for xpad in ours:
        if not xpad.phys.endswith(NODE_SUFFIXES["xpad"]):
            continue
        prefix = xpad.phys[:-len(NODE_SUFFIXES["xpad"])]
        found = {"xpad": xpad}
        for role in ("controls", "pads", "motion"):
            for node in hid_nodes:
                if node.phys == prefix + NODE_SUFFIXES[role]:
                    found[role] = node
        if len(found) == 4:
            dongles.append(Dongle(prefix, **found))
    return dongles


def choose_dongle(phys_prefix):
    nodes, refused = scan_nodes()
    dongles = find_dongles(nodes)
    if phys_prefix:
        dongles = [dongle for dongle in dongles
                   if dongle.prefix == phys_prefix]
    if len(dongles) == 1:
        return dongles[0]
    if dongles:
        prefixes = ", ".join(dongle.prefix for dongle in dongles)
        raise SystemExit(f"several SC2 Link dongles ({prefixes}): choose one "
                         "with --phys")
    hint = ""
    if refused:
        hint = (f"; {len(refused)} event nodes could not be opened "
                f"({refused[0][1].strerror}): run as root or join the input "
                "group")
    raise SystemExit("no SC2 Link dongle found: an xpad node 045e:028e "
                     "version 5c21 and HID nodes on interfaces 1 to 3 of "
                     f"the same device{hint}")


def open_node(path, flags):
    try:
        return os.open(path, flags | os.O_NONBLOCK | os.O_CLOEXEC)
    except OSError as error:
        raise SystemExit(f"{path}: {error.strerror}") from error


def abs_value(fd, code):
    info = bytearray(INPUT_ABSINFO.size)
    fcntl.ioctl(fd, eviocgabs(code), info, True)
    return INPUT_ABSINFO.unpack(info)[0]


def abs_codes(fd):
    bits = bytearray(ABS_COUNT // 8)
    fcntl.ioctl(fd, eviocgbit(EV_ABS, len(bits)), bits, True)
    return [code for code in range(ABS_COUNT)
            if bits[code // 8] >> (code % 8) & 1]


def use_monotonic_clock(fd):
    try:
        fcntl.ioctl(fd, EVIOCSCLOCKID, struct.pack("@i", CLOCK_MONOTONIC))
        return True
    except OSError:
        return False


def read_events(fd):
    """Whole events waiting on fd: (time ns, type, code, value)."""
    events = []
    while True:
        try:
            data = os.read(fd, INPUT_EVENT.size * 64)
        except BlockingIOError:
            return events
        if not data:
            return events
        for offset in range(0, len(data) - INPUT_EVENT.size + 1,
                            INPUT_EVENT.size):
            seconds, microseconds, kind, code, value = \
                INPUT_EVENT.unpack_from(data, offset)
            events.append((seconds * 1_000_000_000 + microseconds * 1000,
                           kind, code, value))


def write_event(fd, kind, code, value):
    os.write(fd, INPUT_EVENT.pack(0, 0, kind, code, value))


# ---- talking to the dongle --------------------------------------------------

class AckTracker:
    """ACK_ECHO and ACK_STATE as the pads node's frames report them."""

    def __init__(self, fd):
        self.fd = fd
        self.kernel_time = use_monotonic_clock(fd)
        self.poller = select.poll()
        self.poller.register(fd, select.POLLIN)
        self.resyncing = False
        self.frames = collections.deque()
        self.refresh()

    def refresh(self):
        """The kernel's current values, which may run ahead of read events."""
        self.echo = abs_value(self.fd, ABS_GAS) & 0xFFFF
        self.state = abs_value(self.fd, ABS_BRAKE) & 0xFFFF
        self.pending = [self.echo, self.state]

    def counter(self):
        return self.state >> 8

    def poll(self, timeout_s):
        timeout_ms = max(0, int(timeout_s * 1000 + 0.999))
        if not self.poller.poll(timeout_ms):
            return
        read_ns = time.monotonic_ns()
        for event_ns, kind, code, value in read_events(self.fd):
            self._handle(event_ns if self.kernel_time else read_ns, kind,
                         code, value)

    def _handle(self, event_ns, kind, code, value):
        if kind == EV_SYN and code == SYN_DROPPED:
            self.resyncing = True
        elif kind == EV_SYN and code == SYN_REPORT:
            if self.resyncing:
                self.resyncing = False
                self.refresh()
            elif self.pending != [self.echo, self.state]:
                self.echo, self.state = self.pending
                self.frames.append((event_ns, self.echo, self.state))
        elif kind == EV_ABS and code == ABS_GAS:
            self.pending[0] = value & 0xFFFF
        elif kind == EV_ABS and code == ABS_BRAKE:
            self.pending[1] = value & 0xFFFF

    def wait_counter(self, before, deadline_ns):
        """The first frame whose counter is not before, or None at deadline."""
        while True:
            while self.frames:
                frame = self.frames.popleft()
                if frame[2] >> 8 != before:
                    return frame
            remaining = deadline_ns - time.monotonic_ns()
            if remaining <= 0:
                return None
            self.poll(remaining / 1e9)


@dataclasses.dataclass
class Ack:
    word: int
    echo: int
    state: int
    round_trip_ns: int
    resends: int
    advance: int


class Link:
    """One FF effect on the xpad node for words, the pads node for acks."""

    def __init__(self, dongle):
        self.dongle = dongle
        self.xpad_fd = open_node(dongle.xpad.path, os.O_RDWR)
        self.pads_fd = open_node(dongle.pads.path, os.O_RDONLY)
        self.effect_id = -1
        # ff-memless scales every magnitude by the gain.
        write_event(self.xpad_fd, EV_FF, FF_GAIN, 0xFFFF)
        self._upload(0)
        self.acks = AckTracker(self.pads_fd)
        self.round_trips_ns = []
        self.resends = 0

    def close(self):
        for fd in (self.xpad_fd, self.pads_fd):
            try:
                os.close(fd)
            except OSError:
                pass

    def _upload(self, word):
        effect = bytearray(FF_EFFECT.size)
        FF_EFFECT_HEADER.pack_into(effect, 0, FF_RUMBLE_TYPE, self.effect_id,
                                   0, 0, 0, 0xFFFF, 0)
        FF_RUMBLE.pack_into(effect, FF_RUMBLE_OFFSET, word & 0xFF00,
                            (word & 0xFF) << 8)
        fcntl.ioctl(self.xpad_fd, EVIOCSFF, effect, True)
        self.effect_id = struct.unpack_from("@h", effect, 2)[0]

    def write_word(self, word):
        """One rumble packet: the upload rearms, the play sends it now."""
        self._upload(word)
        write_event(self.xpad_fd, EV_FF, self.effect_id, 1)

    def drain(self):
        self.acks.poll(0)
        self.acks.frames.clear()

    def state(self):
        """ACK_STATE now: the events read, then the kernel's value."""
        self.drain()
        self.acks.refresh()
        return self.acks.state

    def send(self, word, in_session=False):
        """Stop-and-wait: one word, its acknowledgement or None.

        A session word is not resent once the session bit is clear: outside
        a session it would be rumble.
        """
        self.drain()
        before = self.acks.counter()
        for resend in range(MAX_RESENDS + 1):
            sent_ns = time.monotonic_ns()
            self.write_word(word)
            deadline = sent_ns + int(RESEND_AFTER_S * 1e9)
            frame = self.acks.wait_counter(before, deadline)
            if frame is None:
                self.acks.refresh()
                if self.acks.counter() != before:
                    frame = (time.monotonic_ns(), self.acks.echo,
                             self.acks.state)
                elif in_session and not self.acks.state >> 7 & 1:
                    self.resends += resend
                    return None
            if frame is not None:
                round_trip = max(frame[0] - sent_ns, 0)
                self.round_trips_ns.append(round_trip)
                self.resends += resend
                return Ack(word, frame[1], frame[2], round_trip, resend,
                           (frame[2] >> 8) - before & 0xFF)
        self.resends += MAX_RESENDS
        return None

    def send_plain(self, word, listen_s=0.1):
        """A word expected to stay unacknowledged: True if none came."""
        self.drain()
        before = self.acks.counter()
        self.write_word(word)
        deadline = time.monotonic_ns() + int(listen_s * 1e9)
        return self.acks.wait_counter(before, deadline) is None

    def quiet(self, seconds):
        deadline = time.monotonic() + seconds
        while (remaining := deadline - time.monotonic()) > 0:
            self.acks.poll(remaining)
        self.acks.frames.clear()

    def gone(self, seconds):
        """True once the dongle's nodes disappear (it rebooted)."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                fcntl.ioctl(self.xpad_fd, EVIOCGID, bytearray(INPUT_ID.size),
                            True)
            except OSError as error:
                if error.errno == errno.ENODEV:
                    return True
                raise
            time.sleep(0.05)
        return False


def format_ack(ack, label=""):
    if ack is None:
        return f"{label}no acknowledgement"
    resent = f", {ack.resends} resends" if ack.resends else ""
    jump = f", counter +{ack.advance}" if ack.advance != 1 else ""
    return (f"{label}0x{ack.word:04x}  ack echo 0x{ack.echo:04x} "
            f"{format_state(ack.state)}  {ack.round_trip_ns / 1e6:6.2f} ms"
            f"{resent}{jump}")


def acknowledged(ack, word):
    """The counter moved by exactly one and the echo is this word's.

    CHECKSUM's echo is the checksum, so only its counter is checked.
    """
    return (ack is not None and ack.advance == 1
            and (ack.echo == word or word & DATA == CHECKSUM))


class Client:
    """A session kept as docs/PROTOCOL.md tells a client to keep one."""

    def __init__(self, link):
        self.link = link
        self.toggle = 0

    def in_session(self):
        return bool(decode_state(self.link.state())["session"])

    def hello(self, wait=True):
        """HELLO, after silence when a session or a HELLO may be running."""
        state = decode_state(self.link.state())
        if wait or state["session"] or state["expected"]:
            self.link.quiet(SILENCE_S)
        for index, word in enumerate(HELLO):
            ack = self.link.send(word)
            print(format_ack(ack, f"HELLO {index + 1}/4 "))
            if not acknowledged(ack, word):
                return False
        state = decode_state(ack.state)
        if not state["session"] or state["version"] != PROTOCOL_VERSION:
            print(f"no session: {format_state(ack.state)}")
            return False
        self.toggle = HELLO[-1] >> TOGGLE_SHIFT
        return True

    def message(self, data_words, label, show=True):
        """One message, word by word. Its acknowledgements, or None.

        A session found gone, before the message or in an acknowledgement,
        is opened again and the message sent again from its start, once.
        """
        for attempt in range(2):
            if not self.in_session() and not self.hello(wait=False):
                return None
            acks, lost = self.words(data_words, label, show)
            if not lost:
                return acks
            print("the session ended: HELLO, then the message from its "
                  "start")
        return None

    def words(self, data_words, label, show):
        """The acknowledgements, or None, and whether the session was lost."""
        words, _ = stamp(data_words, self.toggle)
        acks = []
        for index, word in enumerate(words):
            ack = self.link.send(word, in_session=True)
            if show or not acknowledged(ack, word):
                print(format_ack(ack, f"{label} {index + 1}/{len(words)} "))
            if ack is None:
                return None, not self.in_session()
            if not acknowledged(ack, word):
                return None, False
            if not ack.state >> 7 & 1 and word & DATA != BYE:
                return None, True
            self.toggle = word >> TOGGLE_SHIFT
            acks.append(ack)
        return acks, False


def controller_linked(dongle):
    fd = open_node(dongle.controls.path, os.O_RDONLY)
    try:
        return bool(abs_value(fd, ABS_WHEEL) & 1)
    finally:
        os.close(fd)


# ---- decoding the nodes -----------------------------------------------------

BUTTONS = (
    (0x1, "A"), (0x2, "B"), (0x4, "X"), (0x8, "Y"), (0x10, "QAM"),
    (0x20, "R3"), (0x40, "View"), (0x80, "R4"), (0x100, "R5"),
    (0x200, "RB"), (0x400, "Down"), (0x800, "Right"), (0x1000, "Left"),
    (0x2000, "Up"), (0x4000, "Menu"), (0x8000, "L3"), (0x10000, "Steam"),
    (0x20000, "L4"), (0x40000, "L5"), (0x80000, "LB"),
    (0x100000, "RStickTouch"), (0x200000, "RPadTouch"),
    (0x400000, "RPadClick"), (0x800000, "RTriggerClick"),
    (0x1000000, "LStickTouch"), (0x2000000, "LPadTouch"),
    (0x4000000, "LPadClick"), (0x8000000, "LTriggerClick"),
    (0x10000000, "RGrip"), (0x20000000, "LGrip"),
)
EXPECTED_CODES = {
    "controls": {ABS_MISC, ABS_THROTTLE, ABS_RUDDER, ABS_X, ABS_Z, ABS_RX,
                 ABS_RY, ABS_WHEEL, ABS_GAS, ABS_BRAKE},
    "pads": {ABS_X, ABS_Z, ABS_RX, ABS_RY, ABS_THROTTLE, ABS_RUDDER,
             ABS_WHEEL, ABS_GAS, ABS_BRAKE},
    "motion": {ABS_X, ABS_Z, ABS_RX, ABS_RY, ABS_THROTTLE, ABS_RUDDER},
}


def unsigned(value):
    return value & 0xFFFF


def decode_controls(values):
    get = values.get
    buttons = (unsigned(get(ABS_THROTTLE, 0))
               | unsigned(get(ABS_RUDDER, 0)) << 16)
    names = "+".join(name for bit, name in BUTTONS if buttons & bit)
    status = unsigned(get(ABS_WHEEL, 0))
    battery = (f"{status >> 4 & 0x7F}%, charge state {status >> 1 & 7}"
               if status >> 11 & 1 else "battery unknown")
    linked = "connected" if status & 1 else "no controller"
    return {
        "counter": unsigned(get(ABS_MISC, 0)),
        "buttons": f"0x{buttons:08x} {names}".rstrip(),
        "left_stick": (get(ABS_X, 0), get(ABS_Z, 0)),
        "right_stick": (get(ABS_RX, 0), get(ABS_RY, 0)),
        "triggers": (unsigned(get(ABS_GAS, 0)), unsigned(get(ABS_BRAKE, 0))),
        "status": f"{linked}, {battery}",
    }


def decode_pads(values):
    get = values.get
    touch = get(ABS_WHEEL, 0)
    return {
        "left_pad": (get(ABS_X, 0), get(ABS_Z, 0), unsigned(get(ABS_RX, 0))),
        "right_pad": (get(ABS_RY, 0), get(ABS_THROTTLE, 0),
                      unsigned(get(ABS_RUDDER, 0))),
        "touch": "+".join(name for bit, name in ((1, "left"), (2, "right"))
                          if touch & bit) or "none",
        "ack_echo": f"0x{unsigned(get(ABS_GAS, 0)):04x}",
        "ack_state": format_state(unsigned(get(ABS_BRAKE, 0))),
    }


def decode_motion(values):
    get = values.get
    return {
        "accel": (get(ABS_X, 0), get(ABS_Z, 0), get(ABS_RX, 0)),
        "gyro": (get(ABS_RY, 0), get(ABS_THROTTLE, 0), get(ABS_RUDDER, 0)),
    }


def decode_xpad(values):
    return {ABS_NAMES.get(code, f"ABS_{code:#x}"): value
            for code, value in sorted(values.items())}


DECODERS = {"xpad": decode_xpad, "controls": decode_controls,
            "pads": decode_pads, "motion": decode_motion}


def format_fields(fields):
    return "  ".join(f"{name}={value}" for name, value in fields.items())


# ---- subcommands ------------------------------------------------------------

def command_info(args):
    dongle = choose_dongle(args.phys)
    descriptors, values = {}, {}
    for role, node in dongle.nodes():
        fd = open_node(node.path, os.O_RDONLY)
        descriptors[role] = fd
        codes = abs_codes(fd)
        values[role] = {code: abs_value(fd, code) for code in codes}
        print(f"{role:8s} {node.path}  \"{node.name}\"  {node.phys}  "
              f"{node.vendor:04x}:{node.product:04x} v{node.version:04x}")
        print("         " + " ".join(ABS_NAMES.get(code, f"ABS_{code:#x}")
                                     for code in codes))
        if role in EXPECTED_CODES and set(codes) != EXPECTED_CODES[role]:
            print("         unexpected ABS codes for this node")
        print("         " + format_fields(DECODERS[role](values[role])))
    if args.watch:
        watch(descriptors, values)
    for fd in descriptors.values():
        os.close(fd)
    return 0


def watch(descriptors, values):
    """Changed fields every 100 ms and reports per second once a second."""
    poller = select.poll()
    role_of = {}
    for role, fd in descriptors.items():
        poller.register(fd, select.POLLIN)
        role_of[fd] = role
    keys = {role: {} for role in descriptors}
    frames = collections.Counter()
    shown = {role: DECODERS[role](values[role]) for role in descriptors}
    next_print = next_rate = time.monotonic()
    print("watching, ctrl-c to stop")
    try:
        while True:
            for fd, _ in poller.poll(20):
                role = role_of[fd]
                for _, kind, code, value in read_events(fd):
                    if kind == EV_ABS:
                        values[role][code] = value
                    elif kind == EV_KEY:
                        name = KEY_NAMES.get(code, f"KEY_{code:#x}")
                        keys[role][name] = value
                    elif kind == EV_SYN and code == SYN_REPORT:
                        frames[role] += 1
            now = time.monotonic()
            if now >= next_print:
                next_print = now + 0.1
                for role in descriptors:
                    fields = DECODERS[role](values[role])
                    fields.update(keys[role])
                    changed = {name: value for name, value in fields.items()
                               if name != "counter"
                               and shown[role].get(name) != value}
                    if changed:
                        print(f"{role:8s} {format_fields(changed)}")
                    shown[role] = fields
            if now >= next_rate:
                elapsed = now - next_rate + 1.0
                next_rate = now + 1.0
                if frames:
                    print("reports/s: " + "  ".join(
                        f"{role} {frames[role] / elapsed:.0f}"
                        for role in descriptors))
                frames.clear()
    except KeyboardInterrupt:
        print()


def with_client(args, body):
    link = Link(choose_dongle(args.phys))
    try:
        return body(Client(link))
    finally:
        link.close()


def command_hello(args):
    def body(client):
        if not client.hello():
            return 1
        print("session open; it ends 500 ms after the last word")
        return 0
    return with_client(args, body)


def command_bye(args):
    def body(client):
        if not client.hello():
            return 1
        acks = client.message([BYE], "BYE")
        return 0 if acks and not decode_state(acks[-1].state)["session"] else 1
    return with_client(args, body)


def hold_session(client, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        client.link.quiet(min(IDLE_NOP_S,
                              max(deadline - time.monotonic(), 0)))
        if client.message([NOP], "NOP", show=False) is None:
            return False
    return True


def command_send(args):
    message = MESSAGES[args.message]
    if message.name == "rumble" and args.fields == ["stop"]:
        values, words = (0, 0), encode_rumble_stop()
    else:
        try:
            values = parse_fields(message, args.fields)
        except ValueError as error:
            raise SystemExit(f"send {message.name}: {error}") from error
        words = message.encode(*values)
    expected = message.report(*values)

    def body(client):
        # A missing acknowledgement ends the command: the rest would be
        # rumble. The session then times out and the dongle stops.
        if not client.hello() or client.message(words, message.name) is None:
            return 1
        acks = client.message([CHECKSUM], "CHECKSUM")
        if acks is None:
            return 1
        checksum = acks[-1].echo
        report = " ".join(f"{byte:02x}" for byte in expected)
        match = "match" if checksum == crc16(expected) else "MISMATCH"
        print(f"report {report}  crc 0x{crc16(expected):04x}  dongle "
              f"0x{checksum:04x} {match}")
        if args.hold and not hold_session(client, args.hold):
            return 1
        if client.message([BYE], "BYE") is None:
            return 1
        return 0 if checksum == crc16(expected) else 1
    return with_client(args, body)


def reboot_command(args, words, what):
    def body(client):
        if not client.hello():
            return 1
        acks = client.message(words, what)
        if acks is None:
            return 1
        if not decode_state(acks[-1].state)["session"]:
            print("rejected: the session ended")
            return 1
        if client.link.gone(3.0):
            print(f"the dongle left the bus: rebooting into {what}")
            return 0
        print("the dongle is still there after 3 s")
        return 1
    return with_client(args, body)


def command_bootloader(args):
    return reboot_command(args, encode_bootloader(), "bootloader")


def command_mode(args):
    if not 0 <= args.number <= 0xFF:
        raise SystemExit("mode number must be 0 to 255")
    return reboot_command(args, encode_mode(args.number),
                          f"mode {args.number}")


def command_knock(args):
    def body(client):
        link = client.link
        link.quiet(SILENCE_S)
        for word in KNOCK:
            link.write_word(word)
            print(f"knock 0x{word:04x}")
            link.quiet(0.1)
        if link.gone(3.0):
            print("the dongle left the bus: rebooting into the bootloader")
            return 0
        print("the dongle is still there after 3 s")
        return 1
    return with_client(args, body)


class SelfTest:
    """Each word checked against Model, which predicts its acknowledgement."""

    def __init__(self, link, linked):
        self.link = link
        self.model = Model(link.acks.counter(), linked)
        self.results = []
        self.words = 0
        # The client's toggle, which the cases follow or break on purpose.
        self.toggle = 0

    def stamp(self, data_words):
        words, self.toggle = stamp(data_words, self.toggle)
        return words

    def step(self, word):
        """Problems with one word, empty when the dongle did as predicted."""
        in_session = self.model.active
        predicted = self.model.word(word)
        if not predicted.accepted:
            if self.link.send_plain(word):
                return []
            what = "a copy" if predicted.copy else "plain rumble"
            return [f"0x{word:04x}, {what}, moved the counter"]
        ack = self.link.send(word, in_session)
        self.words += 1
        if ack is None:
            return [f"0x{word:04x}: no acknowledgement"]
        problems = []
        if ack.advance != 1:
            problems.append(f"0x{word:04x}: counter advanced by {ack.advance}")
        if ack.echo != predicted.echo:
            problems.append(f"0x{word:04x}: echo 0x{ack.echo:04x}, expected "
                            f"0x{predicted.echo:04x}")
            if word & DATA == CHECKSUM:
                problems.append("expected report: " + (
                    self.model.queued.hex(" ") or "none queued"))
        if ack.state != predicted.state:
            problems.append(f"0x{word:04x}: {format_state(ack.state)}, "
                            f"expected {format_state(predicted.state)}")
        return problems

    def restart(self):
        """After a surprise: let the session time out and open a new one."""
        self.link.quiet(SILENCE_S)
        self.link.acks.refresh()
        self.model = Model(self.link.acks.counter(), self.model.linked)
        self.model.end_session()
        for word in HELLO:
            self.model.word(word)
            if self.link.send(word) is None:
                raise SystemExit("result: FAIL, no session after a failed "
                                 "case")
        self.toggle = self.model.toggle

    def case(self, name, words, quiet=False):
        problems = []
        for word in words:
            problems += self.step(word)
            if problems:
                break
        self.record(name, problems, quiet)
        if problems:
            self.restart()
        return not problems

    def record(self, name, problems, quiet=False):
        self.results.append((name, problems))
        if problems or not quiet:
            print(f"{'FAIL' if problems else 'PASS'} {name}"
                  + "".join(f"\n     {problem}" for problem in problems))

    def hello(self, name="HELLO opens a session", words=HELLO):
        passed = self.case(name, words)
        self.toggle = HELLO[-1] >> TOGGLE_SHIFT
        return passed


def selftest_plain(test):
    test.link.quiet(SILENCE_S)
    test.link.acks.refresh()
    test.model = Model(test.link.acks.counter(), test.model.linked)
    problems = []
    # Both motors at 1 of 255 for the 100 ms the tool listens, then off.
    for word in (0x0101, 0x0000):
        problems += test.step(word)
    test.record("plain rumble outside a session is not acknowledged",
                problems)
    if problems:
        test.link.quiet(SILENCE_S)
        test.link.acks.refresh()
        test.model = Model(test.link.acks.counter(), test.model.linked)


# Each effect the selftest starts ends within a second. Commands above CLICK
# are undefined for 0x82. A tone or sweep plays for its duration, a tone of
# 0 ms forever (reverse engineered). 300 ms still sets the high byte.
SELFTEST_RANGES = {COMMAND: Field("command", 0, 2),
                   DURATION_MS: Field("duration_ms", 1, 300)}
# The value of a field that plays least: OFF, no on time (a pulse without one
# plays nothing), the shortest tone or sweep.
QUIETEST = {SIDE: 0, GAIN: GAIN.minimum, COMMAND: 0, ON_US: 0,
            DURATION_MS: 1}


def selftest_message(message):
    """The message with the field ranges the selftest keeps to."""
    return dataclasses.replace(message, fields=tuple(
        SELFTEST_RANGES.get(field, field) for field in message.fields))


def selftest_messages(test):
    started = time.monotonic()
    words_before = test.words
    for message in map(selftest_message, MESSAGES.values()):
        failed = 0
        cases = boundary_cases(message)
        for values in cases:
            words = test.stamp(message.encode(*values) + [CHECKSUM])
            if not test.case(describe_case(message, values), words,
                             quiet=True):
                failed += 1
        if message.name == "rumble":
            failed += not test.case(
                "rumble stop", test.stamp(encode_rumble_stop() + [CHECKSUM]),
                quiet=True)
        if not failed:
            print(f"PASS {message.name}: {len(cases)} messages, every field "
                  "at minimum, middle and maximum")
    elapsed = time.monotonic() - started
    return (test.words - words_before) / elapsed if elapsed else 0.0


def doubled(words):
    return [word for word in words for _ in range(2)]


def quiet_maximum(field):
    """The quietest value where a field has one, else its maximum."""
    return QUIETEST.get(field, field.maximum)


def selftest_copies(test):
    """Every word twice: each copy is acknowledged again, not taken."""
    for message in MESSAGES.values():
        values = tuple(map(quiet_maximum, message.fields))
        data_words = message.encode(*values) + [CHECKSUM]
        if message.name == "rumble":
            data_words += encode_rumble_stop()
        test.case(f"{message.name} with every word sent twice",
                  doubled(test.stamp(data_words)))
    # BYE with toggle 1 is 0x8200, as plain rumble 0x82 on the left motor.
    # A copy and plain rumble look the same on the pads node.
    data_words = [NOP, BYE] if test.toggle else [BYE]
    words = test.stamp(data_words)
    test.case("BYE with toggle 1 sent twice",
              words[:-1] + doubled(words[-1:]))
    if test.model.linked:
        print("CHECK by hand: no rumble after the doubled BYE.")
    test.hello("HELLO after it opens a new session")
    test.case("a rejected word sent twice", doubled(test.stamp([header(7)])))
    test.link.quiet(SILENCE_S)
    test.link.acks.refresh()
    test.model.end_session()
    test.model.counter = test.link.acks.counter()
    test.hello("HELLO with every word sent twice", doubled(HELLO))


def selftest_framing(test):
    rejected = (
        ("reserved opcode 7 ends the session", [header(7)]),
        ("a non-zero unused header bit ends the session",
         [header(OP_PULSE, 8)]),
        ("an undefined CONTROL sub ends the session", [0x0700]),
        ("a non-zero unused payload bit ends the session",
         [header(OP_SCRIPT), 0x0100]),
    )
    for name, data_words in rejected:
        test.case(name, test.stamp(data_words))
        test.hello("HELLO after it opens a new session")
    # A click on both pads sent after a PULSE header is the off time of a
    # pulse that plays nothing.
    click = encode_command(2, 2, -10)[0]
    test.case("a one-word message inside a longer one is payload",
              test.stamp(encode_pulse(0, 0, click, 0) + [CHECKSUM]))


def time_out(test, name):
    """Silence until the session ends, then the state it left."""
    test.link.quiet(SILENCE_S)
    test.link.acks.refresh()
    state = test.link.acks.state
    test.model.end_session()
    problems = []
    if state != test.model.state():
        problems.append(f"{format_state(state)}, expected "
                        f"{format_state(test.model.state())}")
    test.record(name, problems)
    test.model.counter = state >> 8


def selftest_truncated(test):
    tone = test.stamp(encode_tone(0, -128, 1000, 10, 0, 0))
    if not test.case("a truncated message waits for its payload", tone[:3]):
        return
    time_out(test, "the session timeout drops it")
    test.hello("HELLO after the timeout opens a new session")
    test.case("CHECKSUM shows the timeout's stop, not the dropped message",
              test.stamp([CHECKSUM]))


def selftest_stop(test):
    """NOPs hold a session, silence ends it with its stop.

    Nothing plays meanwhile: an effect that lasts until the stop would run
    longer than a second, and the stop's OFF does not reach the pads.
    """
    problems = []
    for _ in range(5):
        test.link.quiet(IDLE_NOP_S)
        problems += test.step(test.stamp([NOP])[0])
    test.record("NOPs keep the session for 1 s", problems)
    if problems:
        test.restart()
        return
    time_out(test, "the session times out")
    test.hello("HELLO after the timeout opens a new session")
    test.case("CHECKSUM shows the OFF report the timeout queued",
              test.stamp([CHECKSUM]))


def percentile(sorted_values, fraction):
    """Nearest rank."""
    return sorted_values[max(0, math.ceil(len(sorted_values) * fraction) - 1)]


def command_selftest(args):
    dongle = choose_dongle(args.phys)
    linked = controller_linked(dongle)
    print(f"dongle {dongle.prefix}: xpad {dongle.xpad.path}, pads "
          f"{dongle.pads.path}")
    if not linked:
        print("no controller linked: the dongle drops every report, so each "
              "CHECKSUM must read 0xffff")

    def body(client):
        link = client.link
        test = SelfTest(link, linked)
        selftest_plain(test)
        if not test.hello():
            print("result: FAIL, no session")
            return 1
        rate = selftest_messages(test)
        selftest_copies(test)
        selftest_framing(test)
        selftest_truncated(test)
        selftest_stop(test)
        test.case("clean up: BYE ends the session", test.stamp([BYE]))
        failed = sum(1 for _, problems in test.results if problems)
        times = sorted(link.round_trips_ns)
        print(f"words acknowledged: {len(times)}, resends: {link.resends}")
        if times:
            print("round trip ms: min {:.2f}, median {:.2f}, p95 {:.2f}, "
                  "max {:.2f}".format(times[0] / 1e6,
                                      percentile(times, 0.5) / 1e6,
                                      percentile(times, 0.95) / 1e6,
                                      times[-1] / 1e6))
        print(f"words per second, message cases: {rate:.0f}")
        if failed:
            print(f"result: FAIL, {failed} of {len(test.results)} cases")
            return 1
        print(f"result: PASS, {len(test.results)} cases")
        return 0
    return with_client(args, body)


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--phys", help="phys prefix of the dongle to use, "
                        "when several are plugged in")
    commands = parser.add_subparsers(dest="command", required=True)
    info = commands.add_parser("info", help="list the nodes and decode them")
    info.add_argument("--watch", action="store_true",
                      help="keep printing changes and reports per second")
    info.set_defaults(handler=command_info)
    commands.add_parser("hello", help="open a session").set_defaults(
        handler=command_hello)
    commands.add_parser("bye", help="open a session and close it with BYE"
                        ).set_defaults(handler=command_bye)
    send = commands.add_parser(
        "send", help="send one message in a session",
        description="Fields: " + "; ".join(
            f"{message.name} " + " ".join(field.name.upper()
                                          for field in message.fields)
            for message in MESSAGES.values()) + "; rumble stop. SIDE of a "
        "pulse: 0 right pad, 1 left pad, 2 both pads. Of the others: 0 left "
        "pad, 1 right pad, 2 both pads, 3 left internal actuator.")
    send.add_argument("message", choices=sorted(MESSAGES))
    send.add_argument("fields", nargs="*", help="field values, 0x for hex")
    send.add_argument("--hold", type=float, default=0.0, metavar="SECONDS",
                      help="keep the session this long before BYE")
    send.set_defaults(handler=command_send)
    commands.add_parser("selftest", help="check every message, framing and "
                        "timeout rule").set_defaults(handler=command_selftest)
    commands.add_parser("bootloader", help="reboot into the UF2 bootloader "
                        "with the session command").set_defaults(
        handler=command_bootloader)
    mode = commands.add_parser("mode", help="store a mode and reboot into it")
    mode.add_argument("number", type=int)
    mode.set_defaults(handler=command_mode)
    commands.add_parser("knock", help="reboot into the UF2 bootloader with "
                        "the out-of-session knock").set_defaults(
        handler=command_knock)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return args.handler(args)
    except OSError as error:
        if error.errno != errno.ENODEV:
            raise
        print("the dongle disappeared", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
