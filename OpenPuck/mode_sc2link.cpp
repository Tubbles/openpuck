// One controller as an Xbox 360 pad on interface 0, which Linux xpad binds,
// and as three HID Multi-axis nodes on interfaces 1 to 3, which hid-generic
// binds. setup() adds the WebUSB panel after them as interface 4. The rumble
// packets of interface 0 also carry the word protocol of docs/PROTOCOL.md,
// acknowledged on the pads node.
#include "mode_sc2link.h"
#include "mode_xinput.h"
#include "gamepad_util.h"
#include "triton.h"
#include "board_config.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "fault_diag.h"
#include "usb_tx.h"
#include "usb_app_drivers.h"
#include "usb_mount.h" // modeSwitchReboot()
#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <string.h>

SC2LinkController g_sc2LinkCtl;

#define SC2_PACKET_SIZE 32
#define SC2_EP_IN 0x81
#define SC2_EP_OUT 0x02
#define SC2_CONN_MS 1200u
// The Xbox report reads through byte 27, and rf_link passes on no shorter
// state report.
#define SC2_STATE_MIN_LEN 28
// rf_link releases input with a synthetic neutral state this long after the
// last reply. It is no RF state, so it does not advance the sample counter.
#define SC2_LIVE_MS 300u

#define SC2_CONTROLS_LEN 20
#define SC2_CONTROLS_STATUS_AT 14
#define SC2_PADS_LEN 18
#define SC2_PADS_ACK_AT 14
#define SC2_MOTION_LEN 12

// The word protocol. A word is one rumble packet, strong byte high. In a
// session bit 15 is a toggle that flips with each new word, so a copy of the
// last word is told from the next, and bits 14 to 0 are the data.
#define SC2_TOGGLE_SHIFT 15
#define SC2_DATA 0x7FFF
#define SC2_PROTOCOL_VERSION 1
#define SC2_SESSION_MS 500u
// SDL refreshes a running rumble this often: the controller stops one that
// is not refreshed within about 50 ms, and no hop before the dongle resends.
#define SC2_RUMBLE_RESEND_MS 40u
// Only a plain rumble flood fills it: a session client has one word in flight.
#define SC2_WORD_RING 16
#define SC2_MAX_PAYLOAD 4
// A controller report with its id, the longest being 0x80 and 0x83.
#define SC2_REPORT_MAX 10
// The command report that stops the effects of 0x81 to 0x85: command 0 is
// HAPTIC_TYPE_OFF in SDL's controller_structs.h, side 3 both pads, gain 0.
static const uint8_t SC2_HAPTIC_OFF[] = { 0x82, 3, 0, 0 };
// A session's start and end each queue a rumble stop burst and as many OFF
// reports, since the relay is not acknowledged.
#define SC2_STOP_ENTRIES (2 * RUMBLE_STOP_REPS)

// A header with data bit 14 set is a COMMAND, whose 13 bits leave no room for
// a 3-bit opcode in one word. Otherwise the opcode is in bits 13 to 11.
#define SC2_COMMAND_BIT 0x4000
enum Sc2Op {
	SC2_OP_CONTROL,
	SC2_OP_SCRIPT,
	SC2_OP_RUMBLE,
	SC2_OP_PULSE,
	SC2_OP_SWEEP,
	SC2_OP_TONE,
	// 6 and 7 are reserved.
	SC2_OP_COMMAND = 8,
};
// CONTROL subs, header bits 10 to 8.
#define SC2_CTL_NOP 0x00
#define SC2_CTL_BYE 0x02
// A test aid: its ACK_ECHO is the CRC of the last queued report.
#define SC2_CTL_CHECKSUM 0x04
#define SC2_CTL_BOOTLOADER 0x05
#define SC2_CTL_MODE 0x06
// The commands that reboot take effect only with this payload word, so a
// client that lost count does not reboot the dongle with one stray word.
#define SC2_BOOT_CONFIRM 0x5546
// ORed with the mode number from the header.
#define SC2_MODE_CONFIRM 0x4D00

// Each is an undefined CONTROL header, so no message starts with one. Their
// toggle is 0, so the first word of a session carries 1.
static const uint16_t SC2_HELLO[] = { 0x0102, 0x0201, 0x0103, 0x0301 };
static const uint8_t SC2_HELLO_WORDS = sizeof SC2_HELLO / sizeof SC2_HELLO[0];

// Offsets in a 0x45 or 0x42 state: [id][seq][buttons u32][lt][rt][lx][ly]
// [rx][ry][left pad x, y, pressure][right pad x, y, pressure][IMU time u32]
// [accel x, y, z][gyro x, y, z], 16-bit fields little endian. 0x47 puts a
// 16-bit trackpad time in front of the pads and shortens the IMU time.
#define SC2_REP_BUTTONS 2
#define SC2_REP_TRIGGERS 6
#define SC2_REP_STICKS 10
#define SC2_REP_PADS 18
#define SC2_REP_PADS_47 20
#define SC2_REP_IMU 34
#define SC2_REP_FULL_LEN 46

// Interface 0 of a genuine wired pad, its 0x21 descriptor included, without
// the audio, plugin and security interfaces that follow it there: Linux binds
// none of them, and their endpoints are needed for the HID nodes. The OUT
// interval is 1 ms instead of 8 because it bounds how fast a host can rumble.
// clang-format off
static const uint8_t SC2_XBOX_ITF[] = {
	// interface 0, 2 endpoints, class FF/5D/01
	0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x5D, 0x01, 0x00,
	// vendor descriptor 0x21, naming both endpoints
	0x11, 0x21, 0x00, 0x01, 0x01, 0x25, 0x81, 0x14, 0x00, 0x00, 0x00,
	0x00, 0x13, 0x02, 0x08, 0x00, 0x00,
	// interrupt IN 0x81 and OUT 0x02, 32 bytes, bInterval 1
	0x07, 0x05, 0x81, 0x03, 0x20, 0x00, 0x01,
	0x07, 0x05, 0x02, 0x03, 0x20, 0x00, 0x01,
};
// clang-format on
#define SC2_XBOX_EP_IN_AT 26
#define SC2_XBOX_EP_OUT_AT 33
static_assert(sizeof SC2_XBOX_ITF == 40, "SC2 Link interface 0 is 40 bytes");

// Each node is one Multi-axis application without report IDs, so old and new
// kernels alike make one input device per interface. Multi-axis because
// hid-input gives Gamepad and Joystick axes a fuzz of range/256, which would
// drop small changes of these 16-bit fields. Buttons travel as axes too: a
// Multi-axis button maps to BTN_MISC + n and reaches BTN_LEFT. No node has both
// X and Y or all of Rx, Ry and Rz, so SDL takes none of them for a joystick or
// an accelerometer, and no node repeats a usage, which old and new kernels
// resolve differently.
// clang-format off
static const uint8_t SC2_CONTROLS_DESC[] = {
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x08,		// Usage (Multi-axis Controller)
	0xA1, 0x01,		// Collection (Application)
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x09, 0x40,		//   Usage (Vx)      ABS_MISC
	0x09, 0x36,		//   Usage (Slider)  ABS_THROTTLE
	0x09, 0x37,		//   Usage (Dial)    ABS_RUDDER
	0x09, 0x30,		//   Usage (X)       ABS_X
	0x09, 0x32,		//   Usage (Z)       ABS_Z
	0x09, 0x33,		//   Usage (Rx)      ABS_RX
	0x09, 0x34,		//   Usage (Ry)      ABS_RY
	0x09, 0x38,		//   Usage (Wheel)   ABS_WHEEL
	0x95, 0x08,		//   Report Count (8)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x05, 0x02,		//   Usage Page (Simulation Controls)
	0x09, 0xC4,		//   Usage (Accelerator)  ABS_GAS
	0x09, 0xC5,		//   Usage (Brake)        ABS_BRAKE
	0x95, 0x02,		//   Report Count (2)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0xC0			// End Collection
};

static const uint8_t SC2_PADS_DESC[] = {
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x08,		// Usage (Multi-axis Controller)
	0xA1, 0x01,		// Collection (Application)
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x09, 0x30,		//   Usage (X)       ABS_X
	0x09, 0x32,		//   Usage (Z)       ABS_Z
	0x09, 0x33,		//   Usage (Rx)      ABS_RX
	0x09, 0x34,		//   Usage (Ry)      ABS_RY
	0x09, 0x36,		//   Usage (Slider)  ABS_THROTTLE
	0x09, 0x37,		//   Usage (Dial)    ABS_RUDDER
	0x09, 0x38,		//   Usage (Wheel)   ABS_WHEEL
	0x95, 0x07,		//   Report Count (7)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0x05, 0x02,		//   Usage Page (Simulation Controls)
	0x09, 0xC4,		//   Usage (Accelerator)  ABS_GAS
	0x09, 0xC5,		//   Usage (Brake)        ABS_BRAKE
	0x95, 0x02,		//   Report Count (2)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0xC0			// End Collection
};

static const uint8_t SC2_MOTION_DESC[] = {
	0x05, 0x01,		// Usage Page (Generic Desktop)
	0x09, 0x08,		// Usage (Multi-axis Controller)
	0xA1, 0x01,		// Collection (Application)
	0x16, 0x00, 0x80,	//   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,	//   Logical Maximum (32767)
	0x75, 0x10,		//   Report Size (16)
	0x09, 0x30,		//   Usage (X)       ABS_X
	0x09, 0x32,		//   Usage (Z)       ABS_Z
	0x09, 0x33,		//   Usage (Rx)      ABS_RX
	0x09, 0x34,		//   Usage (Ry)      ABS_RY
	0x09, 0x36,		//   Usage (Slider)  ABS_THROTTLE
	0x09, 0x37,		//   Usage (Dial)    ABS_RUDDER
	0x95, 0x06,		//   Report Count (6)
	0x81, 0x02,		//   Input (Data,Var,Abs)
	0xC0			// End Collection
};
// clang-format on
static_assert(sizeof SC2_CONTROLS_DESC == 45, "controls descriptor length");
static_assert(sizeof SC2_PADS_DESC == 43, "pads descriptor length");
static_assert(sizeof SC2_MOTION_DESC == 31, "motion descriptor length");

// Adafruit sizes every HID endpoint at CFG_TUD_HID_EP_BUFSIZE (64). This
// layout uses 32 like interface 0, which still holds the largest report.
class Sc2HidInterface : public Adafruit_USBD_HID {
    public:
	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override
	{
		uint16_t length = Adafruit_USBD_HID::getInterfaceDescriptor(
			itfnum, buf, bufsize);
		uint16_t offset = 0;
		while (buf && offset + 7 <= length && buf[offset] >= 2) {
			if (buf[offset + 1] == TUSB_DESC_ENDPOINT) {
				buf[offset + 4] = SC2_PACKET_SIZE;
				buf[offset + 5] = 0;
			}
			offset += buf[offset];
		}
		return length;
	}
};

// The descriptor is fixed, so it must come out as interface 0 with the
// endpoints it names. A genuine pad's OUT endpoint is 0x02, so 0x01 stays
// unused.
class Sc2XboxInterface : public Adafruit_USBD_Interface {
    public:
	uint16_t getInterfaceDescriptor(uint8_t itfnum, uint8_t *buf,
					uint16_t bufsize) override
	{
		(void)itfnum;
		if (!buf)
			return sizeof SC2_XBOX_ITF;
		if (bufsize < sizeof SC2_XBOX_ITF)
			return 0;
		uint8_t number = TinyUSBDevice.allocInterface(1);
		uint8_t in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
		TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		uint8_t out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);
		if (number != 0 || in != SC2_EP_IN || out != SC2_EP_OUT)
			return 0;
		memcpy(buf, SC2_XBOX_ITF, sizeof SC2_XBOX_ITF);
		return sizeof SC2_XBOX_ITF;
	}
	bool begin()
	{
		return TinyUSBDevice.addInterface(*this);
	}
};

static Sc2XboxInterface g_sc2XboxItf;
static Sc2HidInterface g_sc2Controls, g_sc2Pads, g_sc2Motion;

struct Sc2Word {
	uint16_t value;
	unsigned long ms;
};

struct Sc2Usb {
	// The usbd task publishes endpoint state, the loop owns the reports.
	volatile bool inUse;
	volatile bool txPending;
	// TinyUSB ignores xfer_cb's return value, so the loop drain must
	// remember and recover a failed OUT rearm.
	volatile bool outRearm;
	uint8_t rhport;
	uint8_t latest[XINPUT_REPORT_LEN];
	CFG_TUD_MEM_ALIGN uint8_t inBuf[SC2_PACKET_SIZE];
	CFG_TUD_MEM_ALIGN uint8_t outBuf[SC2_PACKET_SIZE];
	// Rumble words in arrival order, both ends under PRIMASK.
	Sc2Word words[SC2_WORD_RING];
	volatile uint8_t wordHead, wordTail;
	// After a bus reset the host can no longer stop the rumble it started.
	volatile bool rumbleStop;
};

static Sc2Usb g_sc2Usb;
// Loop only: the bond whose input feeds every interface.
static int8_t g_sc2Bond = -1;

struct Sc2Battery {
	bool known;
	uint8_t chargeState, percent;
};

// Loop only. The controls report is kept whole for the battery reports that
// arrive between states, the pads report for the acknowledgements.
static uint16_t g_sc2Counter;
static Sc2Battery g_sc2Battery;
static uint8_t g_sc2ControlsReport[SC2_CONTROLS_LEN];
static uint8_t g_sc2PadsReport[SC2_PADS_LEN];

// Loop only: the word protocol.
struct Sc2Session {
	bool active;
	// HELLO words matched in a row outside a session, and when the last
	// one arrived.
	uint8_t hello;
	unsigned long helloMs;
	unsigned long wordMs;
	// The message being received, data bits only, and how many payload
	// words are to come.
	uint16_t header;
	uint16_t payload[SC2_MAX_PAYLOAD];
	uint8_t have, need;
	// The toggle of the last word taken in the session.
	uint8_t toggle;
	// ACK_STATE's counter of accepted words, and ACK_ECHO.
	uint8_t counter;
	uint16_t echo;
	// The last 0x80 a RUMBLE built, resent while it rumbles.
	uint8_t rumble[HAPTIC_RUMBLE_LEN];
	bool rumbleOn;
	unsigned long rumbleMs;
	// The report the last report-building message or session end queued,
	// for CHECKSUM. Length 0 when it was dropped.
	uint8_t report[SC2_REPORT_MAX];
	uint8_t reportLength;
};

static Sc2Session g_sc2Session;
// Loop only: the latest plain rumble word, relayed once per loop.
static uint16_t g_sc2PlainWord;
static bool g_sc2PlainPending;

// Only before attach or after TinyUSB has dropped the endpoints.
static void sc2UsbClear(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	bool stop = g_sc2Usb.inUse;
	memset(&g_sc2Usb, 0, sizeof g_sc2Usb);
	g_sc2Usb.latest[1] = XINPUT_REPORT_LEN;
	g_sc2Usb.rumbleStop = stop;
	__set_PRIMASK(pm);
}

static void sc2Init(void)
{
	sc2UsbClear();
}

static bool sc2Deinit(void)
{
	sc2UsbClear();
	return true;
}

static void sc2Reset(uint8_t rhport)
{
	(void)rhport;
	sc2UsbClear();
}

static uint16_t sc2Open(uint8_t rhport, const tusb_desc_interface_t *itf,
			uint16_t max_len)
{
	if (g_usbMode != MODE_SC2LINK || !itf || g_sc2Usb.inUse ||
	    max_len < sizeof SC2_XBOX_ITF ||
	    memcmp(itf, SC2_XBOX_ITF, sizeof SC2_XBOX_ITF) != 0)
		return 0;
	const uint8_t *body = (const uint8_t *)itf;
	const tusb_desc_endpoint_t *in =
		(const tusb_desc_endpoint_t *)(body + SC2_XBOX_EP_IN_AT);
	const tusb_desc_endpoint_t *out =
		(const tusb_desc_endpoint_t *)(body + SC2_XBOX_EP_OUT_AT);
	if (!usbd_edpt_open(rhport, in))
		return 0;
	if (!usbd_edpt_open(rhport, out)) {
		usbd_edpt_close(rhport, SC2_EP_IN);
		return 0;
	}
	g_sc2Usb.rhport = rhport;
	g_sc2Usb.inUse = true;
	g_sc2Usb.txPending = true;
	g_sc2Usb.outRearm = !usbd_edpt_xfer(rhport, SC2_EP_OUT, g_sc2Usb.outBuf,
					    sizeof g_sc2Usb.outBuf);
	return sizeof SC2_XBOX_ITF;
}

static bool sc2Control(uint8_t rhport, uint8_t stage,
		       const tusb_control_request_t *request)
{
	(void)rhport;
	(void)stage;
	(void)request;
	return false;
}

// xpad's rumble packet: 00 08 00 strong weak 00 00 00.
static bool sc2RumblePacket(const uint8_t *data, uint32_t length,
			    uint8_t *strong, uint8_t *weak)
{
	if (length != 8 || data[0] != 0 || data[1] != 8 || data[2] != 0 ||
	    data[5] != 0 || data[6] != 0 || data[7] != 0)
		return false;
	*strong = data[3];
	*weak = data[4];
	return true;
}

// A host that reaches the pad only through its event node (a TV app, a desktop
// user without usbfs access) has no panel. For it, four rumble packets in a
// row outside a session, each within 500 ms of the one before, reboot into
// the UF2 bootloader. The first three still rumble. Inside a session the
// BOOTLOADER command does the same.
static const uint8_t SC2_BOOT_KNOCK[][2] = {
	{ 0x55, 0x46 },
	{ 0x32, 0xB0 },
	{ 0x4F, 0x4B },
	{ 0x21, 0x21 },
};
static const uint8_t SC2_BOOT_KNOCK_STEPS =
	sizeof SC2_BOOT_KNOCK / sizeof SC2_BOOT_KNOCK[0];
#define SC2_BOOT_KNOCK_GAP_MS 500u
// Long enough for the rumble stop burst to leave on the following RF polls.
#define SC2_BOOT_DELAY_MS 50u

// Loop only: SC2_REBOOT_NONE, SC2_REBOOT_UF2 or the mode to reboot into.
#define SC2_REBOOT_NONE 0xFF
#define SC2_REBOOT_UF2 0xFE
static uint8_t g_sc2Reboot = SC2_REBOOT_NONE;

static bool sc2KnockStep(uint8_t step, uint8_t strong, uint8_t weak)
{
	return strong == SC2_BOOT_KNOCK[step][0] &&
	       weak == SC2_BOOT_KNOCK[step][1];
}

// Loop only, with the time the packet arrived. True for the packet that
// completes the sequence.
static bool sc2BootKnock(uint8_t strong, uint8_t weak, unsigned long now)
{
	static uint8_t matched;
	static unsigned long lastMs;
	if (matched && (now - lastMs > SC2_BOOT_KNOCK_GAP_MS ||
			!sc2KnockStep(matched, strong, weak)))
		matched = 0;
	if (!sc2KnockStep(matched, strong, weak))
		return false;
	lastMs = now;
	if (++matched < SC2_BOOT_KNOCK_STEPS)
		return false;
	matched = 0;
	return true;
}

// Every OUT packet arrives here, on the usbd task. The loop runs the protocol
// on its word, since the acknowledgement and the relay are loop work.
static void sc2OnOut(const uint8_t *data, uint32_t length)
{
	uint8_t strong, weak;
	if (!sc2RumblePacket(data, length, &strong, &weak))
		return;
	Sc2Word word = { (uint16_t)(strong << 8 | weak), millis() };
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	uint8_t head = g_sc2Usb.wordHead;
	uint8_t next = (uint8_t)((head + 1) % SC2_WORD_RING);
	// A flood is plain rumble, whose newest packet is the one that counts.
	if (next == g_sc2Usb.wordTail)
		g_sc2Usb.wordTail =
			(uint8_t)((g_sc2Usb.wordTail + 1) % SC2_WORD_RING);
	g_sc2Usb.words[head] = word;
	g_sc2Usb.wordHead = next;
	__set_PRIMASK(pm);
}

static bool sc2NextWord(Sc2Word *word)
{
	bool have = false;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (g_sc2Usb.wordTail != g_sc2Usb.wordHead) {
		*word = g_sc2Usb.words[g_sc2Usb.wordTail];
		g_sc2Usb.wordTail =
			(uint8_t)((g_sc2Usb.wordTail + 1) % SC2_WORD_RING);
		have = true;
	}
	__set_PRIMASK(pm);
	return have;
}

static bool sc2Xfer(uint8_t rhport, uint8_t ep, xfer_result_t result,
		    uint32_t length)
{
	if (!g_sc2Usb.inUse || rhport != g_sc2Usb.rhport)
		return false;
	if (ep == SC2_EP_IN) {
		if (result != XFER_RESULT_SUCCESS)
			g_sc2Usb.txPending = true;
		return true;
	}
	if (ep != SC2_EP_OUT)
		return false;
	if (result == XFER_RESULT_SUCCESS)
		sc2OnOut(g_sc2Usb.outBuf, length);
	g_sc2Usb.outRearm = !usbd_edpt_xfer(rhport, ep, g_sc2Usb.outBuf,
					    sizeof g_sc2Usb.outBuf);
	return true;
}

static const usbd_class_driver_t g_sc2Driver = {
#if CFG_TUSB_DEBUG >= 2
	.name = "SC2LINK",
#endif
	.init = sc2Init,
	.deinit = sc2Deinit,
	.reset = sc2Reset,
	.open = sc2Open,
	.control_xfer_cb = sc2Control,
	.xfer_cb = sc2Xfer,
	.sof = NULL
};

const usbd_class_driver_t *sc2LinkClassDriver(void)
{
	return &g_sc2Driver;
}

// usbTxPump calls this from the loop with the USB priority-inversion guard.
static void sc2XboxDrain(void)
{
	if (!tud_mounted() || !g_sc2Usb.inUse)
		return;
	uint8_t rhport = g_sc2Usb.rhport;
	if (g_sc2Usb.outRearm && !usbd_edpt_busy(rhport, SC2_EP_OUT) &&
	    usbd_edpt_claim(rhport, SC2_EP_OUT)) {
		g_sc2Usb.outRearm = false;
		if (!usbd_edpt_xfer(rhport, SC2_EP_OUT, g_sc2Usb.outBuf,
				    sizeof g_sc2Usb.outBuf)) {
			g_sc2Usb.outRearm = true;
			usbd_edpt_release(rhport, SC2_EP_OUT);
		}
	}
	if (!g_sc2Usb.txPending || usbd_edpt_busy(rhport, SC2_EP_IN) ||
	    !usbd_edpt_claim(rhport, SC2_EP_IN))
		return;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	// A bus reset may have closed the endpoint since the check above.
	if (!g_sc2Usb.inUse) {
		__set_PRIMASK(pm);
		usbd_edpt_release(rhport, SC2_EP_IN);
		return;
	}
	memcpy(g_sc2Usb.inBuf, g_sc2Usb.latest, XINPUT_REPORT_LEN);
	g_sc2Usb.txPending = false;
	__set_PRIMASK(pm);
	if (!usbd_edpt_xfer(rhport, SC2_EP_IN, g_sc2Usb.inBuf,
			    XINPUT_REPORT_LEN)) {
		g_sc2Usb.txPending = true;
		usbd_edpt_release(rhport, SC2_EP_IN);
	}
}

// Bit 0 controller connected, bits 1 to 3 EChargeState, bits 4 to 10 battery
// percent, bit 11 battery known.
static uint16_t sc2Status(const Sc2Battery &battery)
{
	uint8_t percent = battery.percent > 127 ? 127 : battery.percent;
	uint16_t status = 1u | (uint16_t)((battery.chargeState & 7u) << 1) |
			  (uint16_t)(percent << 4);
	if (battery.known)
		status |= 1u << 11;
	return status;
}

static void sc2BuildControls(uint8_t out[SC2_CONTROLS_LEN], const uint8_t *rep,
			     uint16_t counter, uint16_t status)
{
	le16(out, (int16_t)counter);
	memcpy(out + 2, rep + SC2_REP_BUTTONS, 4);
	memcpy(out + 6, rep + SC2_REP_STICKS, 8);
	le16(out + SC2_CONTROLS_STATUS_AT, (int16_t)status);
	memcpy(out + 16, rep + SC2_REP_TRIGGERS, 4);
}

// False when the report ends before the pads. Leaves the acknowledgement
// fields at the end alone.
static bool sc2BuildPads(uint8_t out[SC2_PADS_LEN], const uint8_t *rep,
			 uint8_t length)
{
	uint8_t at = rep[0] == OPK_TRITON_REPORT_STATE_TIMESTAMP ?
			     SC2_REP_PADS_47 :
			     SC2_REP_PADS;
	if (length < at + 12)
		return false;
	uint32_t buttons = btnsOf(rep);
	memcpy(out, rep + at, 12);
	// A copy of the touch bits, since frames of different nodes reach a
	// client in no fixed order and a touch must arrive with its position.
	out[12] = (uint8_t)(((buttons & TB_LPADT) ? 1 : 0) |
			    ((buttons & TB_RPADT) ? 2 : 0));
	out[13] = 0;
	return true;
}

// Bits 15 to 8 accepted words mod 256, bit 7 session active, bits 6 to 4
// payload words still expected (HELLO words outside a session), bits 3 to 0
// protocol version.
static uint16_t sc2AckState(const Sc2Session &session)
{
	uint8_t expected = 0;
	if (session.active)
		expected = session.need;
	else if (session.hello)
		expected = (uint8_t)(SC2_HELLO_WORDS - session.hello);
	return (uint16_t)(session.counter << 8 | (session.active ? 0x80 : 0) |
			  (expected & 7) << 4 | SC2_PROTOCOL_VERSION);
}

static void sc2SendPads(void)
{
	le16(g_sc2PadsReport + SC2_PADS_ACK_AT, (int16_t)g_sc2Session.echo);
	le16(g_sc2PadsReport + SC2_PADS_ACK_AT + 2,
	     (int16_t)sc2AckState(g_sc2Session));
	usbTxHid(&g_sc2Pads, 0, g_sc2PadsReport, SC2_PADS_LEN);
}

// False when the report ends before the IMU.
static bool sc2BuildMotion(uint8_t out[SC2_MOTION_LEN], const uint8_t *rep,
			   uint8_t length)
{
	if (length < SC2_REP_FULL_LEN)
		return false;
	memcpy(out, rep + SC2_REP_IMU, SC2_MOTION_LEN);
	return true;
}

static void sc2PublishXbox(const uint8_t report[XINPUT_REPORT_LEN])
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	memcpy(g_sc2Usb.latest, report, XINPUT_REPORT_LEN);
	g_sc2Usb.txPending = true;
	__set_PRIMASK(pm);
}

// Releases every input with the connected bit clear. The counter keeps its
// value, and motion is left as is: nothing in it reads as held.
static void sc2PublishNeutral(void)
{
	uint8_t xbox[XINPUT_REPORT_LEN] = { 0, XINPUT_REPORT_LEN };
	sc2PublishXbox(xbox);
	memset(g_sc2ControlsReport + 2, 0, SC2_CONTROLS_LEN - 2);
	usbTxHid(&g_sc2Controls, 0, g_sc2ControlsReport, SC2_CONTROLS_LEN);
	memset(g_sc2PadsReport, 0, SC2_PADS_ACK_AT);
	sc2SendPads();
}

static bool sc2BondRecent(int slot, unsigned long now)
{
	return slot >= 0 && slot < NSLOT && g_slot[slot].used &&
	       g_connReplyMs[slot] != 0 &&
	       now - g_connReplyMs[slot] < SC2_CONN_MS;
}

// One controller: the first live bond, held through gaps shorter than
// SC2_CONN_MS and released with neutral reports after a longer one.
static void sc2UpdateBond(void)
{
	unsigned long now = millis();
	if (sc2BondRecent(g_sc2Bond, now))
		return;
	if (g_sc2Bond >= 0) {
		g_sc2Bond = -1;
		g_sc2Battery = Sc2Battery();
		sc2PublishNeutral();
	}
	for (int slot = 0; slot < NSLOT; slot++) {
		if (sc2BondRecent(slot, now)) {
			g_sc2Bond = (int8_t)slot;
			break;
		}
	}
}

// Plain rumble as in the Xbox mode. Its 2.5 s stuck-rumble watchdog is the
// one haptics.cpp already runs for every relayed 0x80, so none is kept here.
static void sc2RelayRumble(void)
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	bool stop = g_sc2Usb.rumbleStop;
	g_sc2Usb.rumbleStop = false;
	__set_PRIMASK(pm);
	bool pending = g_sc2PlainPending;
	g_sc2PlainPending = false;
	if (g_sc2Bond < 0)
		return;
	if (stop)
		hapticSteamRumble(0, 0, (uint8_t)g_sc2Bond);
	if (pending)
		hapticSteamRumble((uint16_t)(g_sc2PlainWord >> 8) * 257u,
				  (uint16_t)(g_sc2PlainWord & 0xFF) * 257u,
				  (uint8_t)g_sc2Bond);
}

static uint8_t sc2Op(uint16_t header)
{
	if (header & SC2_COMMAND_BIT)
		return SC2_OP_COMMAND;
	return (uint8_t)((header >> 11) & 7);
}

static uint8_t sc2ControlSub(uint16_t header)
{
	return (uint8_t)((header >> 8) & 7);
}

static int8_t sc2ControlWords(uint16_t header)
{
	uint8_t argument = (uint8_t)header;
	switch (sc2ControlSub(header)) {
	case SC2_CTL_NOP:
	case SC2_CTL_BYE:
	case SC2_CTL_CHECKSUM:
		return argument ? -1 : 0;
	case SC2_CTL_BOOTLOADER:
		// No UF2 mode without the Adafruit bootloader.
		return (argument || !OPK_HAS_ADAFRUIT_DFU) ? -1 : 1;
	case SC2_CTL_MODE:
		return modeValid(argument) ? 1 : -1;
	default:
		return -1;
	}
}

// The payload words that follow a header, or -1 for a header the parser
// rejects: a reserved opcode, an undefined CONTROL sub or a non-zero unused
// bit.
static int8_t sc2PayloadWords(uint16_t header)
{
	switch (sc2Op(header)) {
	case SC2_OP_CONTROL:
		return sc2ControlWords(header);
	case SC2_OP_COMMAND:
		return (header & 0x2000) ? -1 : 0;
	case SC2_OP_SCRIPT:
		return (header & 0x0400) ? -1 : 1;
	case SC2_OP_RUMBLE:
		if (header & 0x07F8)
			return -1;
		// Bit 2 is a stop: both speeds zero, no payload.
		if (header & 4)
			return (header & 3) ? -1 : 0;
		return 2;
	case SC2_OP_PULSE:
		return (header & 0x04F8) ? -1 : 3;
	case SC2_OP_SWEEP:
	case SC2_OP_TONE:
		return (header & 0x0400) ? -1 : 4;
	default:
		return -1;
	}
}

// A payload field narrower than its word keeps the unused bits zero, like a
// header, so a word taken out of step is more likely to be caught. A command
// that reboots takes only its confirmation word.
static bool sc2PayloadValid(uint16_t header, uint8_t index, uint16_t data)
{
	switch (sc2Op(header)) {
	case SC2_OP_CONTROL:
		if (sc2ControlSub(header) == SC2_CTL_MODE)
			return data == (SC2_MODE_CONFIRM | (header & 0xFF));
		return data == SC2_BOOT_CONFIRM;
	case SC2_OP_SCRIPT:
		return data <= 0xFF;
	case SC2_OP_SWEEP:
		return index != 3 || data <= 7;
	case SC2_OP_TONE:
		return index != 3 || data <= 0x7FF;
	default:
		return true;
	}
}

// A 16-bit field from 15-bit words: bit 15 is bit `at` of `high`, the rest is
// `low`.
static uint16_t sc2Join(uint16_t high, uint8_t at, uint16_t low)
{
	return (uint16_t)(((high >> at) & 1) << 15 | low);
}

// The controller report for a complete message, report id first, in the
// layout of SDL's controller_structs.h. Returns its length, 0 for CONTROL.
static uint8_t sc2BuildReport(uint16_t header, const uint16_t *payload,
			      uint8_t out[SC2_REPORT_MAX])
{
	uint8_t side = (uint8_t)((header >> 8) & 3);
	uint8_t gain = (uint8_t)(header & 0xFF);
	switch (sc2Op(header)) {
	case SC2_OP_COMMAND:
		out[0] = 0x82;
		out[1] = side;
		out[2] = (uint8_t)((header >> 10) & 7);
		out[3] = gain;
		return 4;
	case SC2_OP_SCRIPT:
		out[0] = 0x85;
		out[1] = side;
		out[2] = (uint8_t)payload[0];
		out[3] = gain;
		return 4;
	case SC2_OP_RUMBLE:
		out[0] = 0x80;
		if (header & 4)
			hapticRumbleReport(0, 0, out + 1);
		else
			hapticRumbleReport(sc2Join(header, 1, payload[0]),
					   sc2Join(header, 0, payload[1]),
					   out + 1);
		return 1 + HAPTIC_RUMBLE_LEN;
	case SC2_OP_PULSE:
		out[0] = 0x81;
		out[1] = side;
		for (uint8_t i = 0; i < 3; i++)
			le16(out + 2 + 2 * i,
			     (int16_t)sc2Join(header, (uint8_t)(2 - i),
					      payload[i]));
		return 8;
	case SC2_OP_SWEEP:
		out[0] = 0x84;
		out[1] = side;
		out[2] = gain;
		for (uint8_t i = 0; i < 3; i++)
			le16(out + 3 + 2 * i,
			     (int16_t)sc2Join(payload[3], (uint8_t)(2 - i),
					      payload[i]));
		return 9;
	case SC2_OP_TONE:
		out[0] = 0x83;
		out[1] = side;
		out[2] = gain;
		for (uint8_t i = 0; i < 3; i++)
			le16(out + 3 + 2 * i,
			     (int16_t)sc2Join(payload[3], (uint8_t)(10 - i),
					      payload[i]));
		out[9] = (uint8_t)payload[3];
		return 10;
	default:
		return 0;
	}
}

// Room in the relay for a session's stop after the report. A report the gates
// will drop needs none.
static bool sc2RelayRoom(void)
{
	return g_sc2Bond < 0 || haptic82Blocked(g_sc2Bond) ||
	       relayFree((uint8_t)g_sc2Bond) >= SC2_STOP_ENTRIES;
}

// The last word of a message that queues a report is accepted only while the
// relay has room, so a full queue holds the client back through the missing
// acknowledgement instead of evicting queued effects.
static bool sc2CanComplete(uint16_t header)
{
	return sc2Op(header) == SC2_OP_CONTROL || sc2RelayRoom();
}

// RUMBLE speeds go out verbatim: Steam has shaped them already. The rest is
// gated like Steam's own reports in the puck mode.
static bool sc2QueueReport(const uint8_t *report, uint8_t length)
{
	Sc2Session &session = g_sc2Session;
	bool rumble = report[0] == 0x80;
	if (rumble) {
		memcpy(session.rumble, report + 1, HAPTIC_RUMBLE_LEN);
		session.rumbleOn = session.rumble[0] != 0;
		session.rumbleMs = millis();
	}
	if (g_sc2Bond < 0)
		return false;
	uint8_t bond = (uint8_t)g_sc2Bond;
	if (rumble)
		return hapticRumbleQueue(session.rumble, bond);
	return !haptic82Blocked(bond) &&
	       relayEnqueue(report[0], report + 1, (uint8_t)(length - 1), true,
			    bond);
}

// Every kind of haptic on both sides: the rumble of 0x80 and, through OFF,
// the pulses, tones, sweeps and scripts. A stop passes the gates a stop of
// plain rumble passes. True when the OFF reports were queued.
static bool sc2StopHaptics(void)
{
	g_sc2Session.rumbleOn = false;
	if (g_sc2Bond < 0)
		return false;
	uint8_t bond = (uint8_t)g_sc2Bond;
	uint8_t stop[HAPTIC_RUMBLE_LEN];
	hapticRumbleReport(0, 0, stop);
	hapticRumbleQueue(stop, bond);
	if (!hapticLinkUp(bond))
		return false;
	bool queued = false;
	for (uint8_t i = 0; i < RUMBLE_STOP_REPS; i++)
		if (relayEnqueue(SC2_HAPTIC_OFF[0], SC2_HAPTIC_OFF + 1,
				 sizeof SC2_HAPTIC_OFF - 1, true, bond))
			queued = true;
	return queued;
}

// Stops every haptic and drops a partial message. Words are plain rumble
// again. The OFF report counts for CHECKSUM, so a client can tell from the
// next session that the stop was queued.
static void sc2EndSession(void)
{
	Sc2Session &session = g_sc2Session;
	session.active = false;
	session.need = 0;
	session.reportLength = sc2StopHaptics() ? sizeof SC2_HAPTIC_OFF : 0;
	memcpy(session.report, SC2_HAPTIC_OFF, session.reportLength);
}

static void sc2StartSession(unsigned long ms)
{
	Sc2Session &session = g_sc2Session;
	// Stray words before the HELLO were plain rumble at their byte values:
	// a session starts from silence, with the same stop as its end, which
	// leaves CHECKSUM to that end.
	g_sc2PlainPending = false;
	sc2StopHaptics();
	session.active = true;
	session.hello = 0;
	session.need = 0;
	session.toggle =
		(uint8_t)(SC2_HELLO[SC2_HELLO_WORDS - 1] >> SC2_TOGGLE_SHIFT);
	session.wordMs = ms;
}

// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection
// and no final XOR, which is Python's binascii.crc_hqx(data, 0xFFFF).
static uint16_t sc2Crc16(const uint8_t *bytes, uint8_t length)
{
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < length; i++) {
		crc ^= (uint16_t)(bytes[i] << 8);
		for (uint8_t bit = 0; bit < 8; bit++)
			crc = (crc & 0x8000) ? (uint16_t)(crc << 1 ^ 0x1021) :
					       (uint16_t)(crc << 1);
	}
	return crc;
}

// A complete message. Returns what ACK_ECHO shows for it.
static uint16_t sc2Run(uint16_t last)
{
	Sc2Session &session = g_sc2Session;
	if (sc2Op(session.header) == SC2_OP_CONTROL) {
		uint8_t sub = sc2ControlSub(session.header);
		if (sub == SC2_CTL_BYE)
			sc2EndSession();
		else if (sub == SC2_CTL_CHECKSUM)
			return sc2Crc16(session.report, session.reportLength);
		else if (sub == SC2_CTL_BOOTLOADER)
			g_sc2Reboot = SC2_REBOOT_UF2;
		else if (sub == SC2_CTL_MODE)
			g_sc2Reboot = (uint8_t)session.header;
		return last;
	}
	uint8_t report[SC2_REPORT_MAX];
	uint8_t length =
		sc2BuildReport(session.header, session.payload, report);
	session.reportLength = sc2QueueReport(report, length) ? length : 0;
	memcpy(session.report, report, session.reportLength);
	return last;
}

// Each accepted word sends a pads report: the counter is how a client tells
// this word's acknowledgement from the previous one.
static void sc2Accept(uint16_t echo)
{
	g_sc2Session.counter++;
	g_sc2Session.echo = echo;
	sc2SendPads();
}

// Four HELLO words in a row outside a session, each within the session
// timeout of the one before, start one. True for a HELLO word, which is
// taken instead of relayed.
static bool sc2HelloStep(uint16_t word, unsigned long ms)
{
	Sc2Session &session = g_sc2Session;
	if (session.hello && ms - session.helloMs > SC2_SESSION_MS)
		session.hello = 0;
	if (word != SC2_HELLO[session.hello])
		session.hello = 0;
	if (word != SC2_HELLO[session.hello])
		return false;
	session.hello++;
	session.helloMs = ms;
	return true;
}

// Boards without the Adafruit bootloader have no UF2 mode, so the panel's
// opcode 0x0C is off there too.
static bool sc2KnockWord(uint16_t word, unsigned long ms)
{
	return OPK_HAS_ADAFRUIT_DFU &&
	       sc2BootKnock((uint8_t)(word >> 8), (uint8_t)word, ms);
}

// A word the parser cannot take ends the session, so a client that lost
// count starts over with a HELLO. The word still counts as accepted, which
// tells the client at once, and it is then a word outside the session: it
// can be a knock step or the first word of a new HELLO, never rumble.
static void sc2Reject(uint16_t word, unsigned long ms)
{
	sc2EndSession();
	if (sc2KnockWord(word, ms))
		g_sc2Reboot = SC2_REBOOT_UF2;
	else
		sc2HelloStep(word, ms);
	sc2Accept(word);
}

static void sc2PayloadWord(uint16_t word, unsigned long ms)
{
	Sc2Session &session = g_sc2Session;
	uint16_t data = word & SC2_DATA;
	if (!sc2PayloadValid(session.header, session.have, data)) {
		sc2Reject(word, ms);
		return;
	}
	if (session.need == 1 && !sc2CanComplete(session.header))
		return;
	session.payload[session.have++] = data;
	session.toggle = (uint8_t)(word >> SC2_TOGGLE_SHIFT);
	uint16_t echo = word;
	if (!--session.need)
		echo = sc2Run(word);
	sc2Accept(echo);
}

static void sc2SessionWord(uint16_t word, unsigned long ms)
{
	Sc2Session &session = g_sc2Session;
	if (session.need) {
		sc2PayloadWord(word, ms);
		return;
	}
	uint16_t header = word & SC2_DATA;
	int8_t need = sc2PayloadWords(header);
	if (need < 0) {
		sc2Reject(word, ms);
		return;
	}
	if (!need && !sc2CanComplete(header))
		return;
	session.header = header;
	session.have = 0;
	session.need = (uint8_t)need;
	session.toggle = (uint8_t)(word >> SC2_TOGGLE_SHIFT);
	uint16_t echo = word;
	if (!need)
		echo = sc2Run(word);
	sc2Accept(echo);
}

// One upload and play can reach the dongle twice when an ff-memless timer
// tick falls between the two, and a client resends a word whose
// acknowledgement was slow. Such a copy is not taken again: in a session it
// carries the last toggle, outside it repeats the last HELLO word matched.
static bool sc2Copy(uint16_t word, unsigned long ms)
{
	const Sc2Session &session = g_sc2Session;
	if (session.active)
		return (word >> SC2_TOGGLE_SHIFT) == session.toggle;
	return session.hello && word == SC2_HELLO[session.hello - 1] &&
	       ms - session.helloMs <= SC2_SESSION_MS;
}

static void sc2OnWord(uint16_t word, unsigned long ms)
{
	Sc2Session &session = g_sc2Session;
	if (g_sc2Reboot != SC2_REBOOT_NONE)
		return;
	if (session.active)
		session.wordMs = ms;
	if (sc2Copy(word, ms)) {
		// The same acknowledgement again, in case the first was lost.
		sc2SendPads();
		return;
	}
	if (session.active) {
		sc2SessionWord(word, ms);
		return;
	}
	if (sc2KnockWord(word, ms)) {
		g_sc2Reboot = SC2_REBOOT_UF2;
		return;
	}
	if (sc2HelloStep(word, ms)) {
		if (session.hello == SC2_HELLO_WORDS)
			sc2StartSession(ms);
		sc2Accept(word);
		return;
	}
	g_sc2PlainWord = word;
	g_sc2PlainPending = true;
}

static void sc2DrainWords(void)
{
	Sc2Word word;
	for (uint8_t i = 0; i < SC2_WORD_RING && sc2NextWord(&word); i++)
		sc2OnWord(word.value, word.ms);
}

// The timeout ends the session of a client that crashed or left without BYE.
static void sc2SessionTask(void)
{
	Sc2Session &session = g_sc2Session;
	if (!session.active)
		return;
	unsigned long now = millis();
	if (now - session.wordMs > SC2_SESSION_MS) {
		sc2EndSession();
		sc2SendPads();
		return;
	}
	if (!session.rumbleOn || g_sc2Bond < 0 ||
	    now - session.rumbleMs < SC2_RUMBLE_RESEND_MS || !sc2RelayRoom())
		return;
	session.rumbleMs = now;
	hapticRumbleQueue(session.rumble, (uint8_t)g_sc2Bond);
}

// Stops the haptics a knock or a session started, then reboots the way the
// panel does: opcode 0x0C into the UF2 bootloader, opcode 0x03 into a mode.
static void sc2RebootTask(void)
{
	static bool stopped;
	static unsigned long stopMs;
	if (g_sc2Reboot == SC2_REBOOT_NONE)
		return;
	if (!stopped) {
		stopped = true;
		stopMs = millis();
		// Ending the session too keeps its rumble from being resent.
		sc2EndSession();
		return;
	}
	if (millis() - stopMs < SC2_BOOT_DELAY_MS)
		return;
	if (g_sc2Reboot == SC2_REBOOT_UF2) {
		faultDiagArmIntentionalReset();
		enterUf2Dfu();
	} else {
		modeSwitchReboot(g_sc2Reboot);
	}
}

static bool sc2StateReport(const uint8_t *rep, uint8_t length)
{
	return length >= SC2_STATE_MIN_LEN &&
	       (rep[0] == 0x45 || rep[0] == 0x42 ||
		rep[0] == OPK_TRITON_REPORT_STATE_TIMESTAMP);
}

static void sc2BeginNode(Sc2HidInterface &node, const uint8_t *desc,
			 uint16_t length)
{
	node.setReportDescriptor(desc, length);
	node.setPollInterval(1);
	node.begin();
}

void SC2LinkController::usbIdentity()
{
	// xpad binds an Xbox 360 interface only under a vendor in its table.
	USBDevice.setID(0x045E, 0x028E);
	// xpad reports this as the evdev version of its node, which is how a
	// client tells this mode from the Xbox mode (0x0114).
	USBDevice.setDeviceVersion(0x5C21);
	USBDevice.setManufacturerDescriptor("OpenPuck");
	USBDevice.setProductDescriptor("SC2 Link");
	// Class 0, not the Xbox mode's 0xFF: on a vendor-class device the USB
	// core skips interface-class matches unless the driver also names the
	// vendor, and usbhid matches on the interface class alone.
	tusb_desc_device_t *device =
		(tusb_desc_device_t *)tud_descriptor_device_cb();
	device->bDeviceClass = 0x00;
	device->bDeviceSubClass = 0x00;
	device->bDeviceProtocol = 0x00;
}

void SC2LinkController::begin()
{
	sc2UsbClear();
	usbIdentity();
	g_sc2XboxItf.begin();
	sc2BeginNode(g_sc2Controls, SC2_CONTROLS_DESC,
		     sizeof SC2_CONTROLS_DESC);
	sc2BeginNode(g_sc2Pads, SC2_PADS_DESC, sizeof SC2_PADS_DESC);
	sc2BeginNode(g_sc2Motion, SC2_MOTION_DESC, sizeof SC2_MOTION_DESC);
	usbTxRegisterDrain(sc2XboxDrain);
	// Meant to show protocol version 1 before any word, but on hardware the
	// host still read 0 until the first acknowledgement: a client checks the
	// version on an acknowledgement.
	sc2SendPads();
}

void SC2LinkController::onReport45(int slot, const uint8_t *rep, bool fresh,
				   uint8_t bodyTlen)
{
	if (!fresh || !rep || slot < 0 || slot != g_sc2Bond ||
	    !sc2StateReport(rep, bodyTlen))
		return;
	if (millis() - g_connReplyMs[slot] < SC2_LIVE_MS)
		g_sc2Counter++;
	sc2BuildControls(g_sc2ControlsReport, rep, g_sc2Counter,
			 sc2Status(g_sc2Battery));
	usbTxHid(&g_sc2Controls, 0, g_sc2ControlsReport, SC2_CONTROLS_LEN);
	uint8_t motion[SC2_MOTION_LEN];
	if (sc2BuildPads(g_sc2PadsReport, rep, bodyTlen))
		sc2SendPads();
	if (sc2BuildMotion(motion, rep, bodyTlen))
		usbTxHid(&g_sc2Motion, 0, motion, sizeof motion);
	uint8_t xbox[XINPUT_REPORT_LEN];
	xinputBuildReport(rep, xbox);
	sc2PublishXbox(xbox);
}

void SC2LinkController::onAuxReport(int slot, uint8_t rid, const uint8_t *data,
				    uint8_t n)
{
	// 0x43 starts with EChargeState and the battery percent.
	if (slot < 0 || slot != g_sc2Bond || rid != 0x43 || n < 2)
		return;
	g_sc2Battery.known = true;
	g_sc2Battery.chargeState = data[0];
	g_sc2Battery.percent = data[1];
	le16(g_sc2ControlsReport + SC2_CONTROLS_STATUS_AT,
	     (int16_t)sc2Status(g_sc2Battery));
	usbTxHid(&g_sc2Controls, 0, g_sc2ControlsReport, SC2_CONTROLS_LEN);
}

void SC2LinkController::task()
{
	sc2UpdateBond();
	sc2DrainWords();
	sc2SessionTask();
	sc2RelayRumble();
	sc2RebootTask();
}
