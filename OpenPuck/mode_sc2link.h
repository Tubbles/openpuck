// mode_sc2link.h -- SC2 Link personality (MODE_SC2LINK): one controller as a
// plain Xbox 360 pad for Linux xpad, plus three HID Multi-axis interfaces that
// carry its raw state at full resolution to a client reading evdev.
#pragma once
#include "controllers.h"

// Test builds: boot into SC2 Link whatever cfg.bin holds, so a freshly flashed
// dongle needs no configuration step. Leaving the mode then takes a build
// without it. -DOPK_SC2LINK_FORCE=1 to enable.
#ifndef OPK_SC2LINK_FORCE
#define OPK_SC2LINK_FORCE 1
#endif

class SC2LinkController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void onAuxReport(int slot, uint8_t rid, const uint8_t *data,
			 uint8_t n) override;
	void task() override;
	void usbIdentity() override;
};

extern SC2LinkController g_sc2LinkCtl;
