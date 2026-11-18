// mode_sc2link.h -- SC2 Link personality (MODE_SC2LINK): one controller as a
// plain Xbox 360 pad for Linux xpad, plus three HID Multi-axis interfaces that
// carry its raw state at full resolution to a client reading evdev.
#pragma once
#include "controllers.h"

class SC2LinkController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void task() override;
	void usbIdentity() override;
};

extern SC2LinkController g_sc2LinkCtl;
