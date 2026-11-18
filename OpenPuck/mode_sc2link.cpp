// One controller as an Xbox 360 pad on interface 0, which Linux xpad binds,
// and as three HID Multi-axis nodes on interfaces 1 to 3, which hid-generic
// binds. setup() adds the WebUSB panel after them as interface 4.
#include "mode_sc2link.h"
#include "mode_xinput.h"
#include "triton.h"
#include "config.h"
#include "haptics.h"
#include "bonds.h"
#include "usb_tx.h"
#include "usb_app_drivers.h"
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
	volatile uint16_t rumbleLow, rumbleHigh;
	volatile bool rumblePending;
	// After a bus reset the host can no longer stop the rumble it started.
	volatile bool rumbleStop;
};

static Sc2Usb g_sc2Usb;
// Loop only: the bond whose input feeds every interface.
static int8_t g_sc2Bond = -1;

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

// Every OUT packet arrives here, on the usbd task. A parser for words carried
// in rumble packets goes in front of the plain rumble below.
static void sc2OnOut(const uint8_t *data, uint32_t length)
{
	uint8_t strong, weak;
	if (!sc2RumblePacket(data, length, &strong, &weak))
		return;
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	g_sc2Usb.rumbleLow = (uint16_t)strong * 257u;
	g_sc2Usb.rumbleHigh = (uint16_t)weak * 257u;
	g_sc2Usb.rumblePending = true;
	__set_PRIMASK(pm);
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

static void sc2PublishXbox(const uint8_t report[XINPUT_REPORT_LEN])
{
	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	memcpy(g_sc2Usb.latest, report, XINPUT_REPORT_LEN);
	g_sc2Usb.txPending = true;
	__set_PRIMASK(pm);
}

static void sc2PublishNeutral(void)
{
	uint8_t xbox[XINPUT_REPORT_LEN] = { 0, XINPUT_REPORT_LEN };
	sc2PublishXbox(xbox);
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
	bool pending = g_sc2Usb.rumblePending, stop = g_sc2Usb.rumbleStop;
	uint16_t low = g_sc2Usb.rumbleLow, high = g_sc2Usb.rumbleHigh;
	g_sc2Usb.rumblePending = false;
	g_sc2Usb.rumbleStop = false;
	__set_PRIMASK(pm);
	if (g_sc2Bond < 0)
		return;
	if (stop)
		hapticSteamRumble(0, 0, (uint8_t)g_sc2Bond);
	if (pending)
		hapticSteamRumble(low, high, (uint8_t)g_sc2Bond);
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
}

void SC2LinkController::onReport45(int slot, const uint8_t *rep, bool fresh,
				   uint8_t bodyTlen)
{
	if (!fresh || !rep || slot < 0 || slot != g_sc2Bond ||
	    !sc2StateReport(rep, bodyTlen))
		return;
	uint8_t xbox[XINPUT_REPORT_LEN];
	xinputBuildReport(rep, xbox);
	sc2PublishXbox(xbox);
}

void SC2LinkController::task()
{
	sc2UpdateBond();
	sc2RelayRumble();
}
