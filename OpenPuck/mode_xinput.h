// Static wired Xbox 360 personality; one selected RF bond supplies input.
#pragma once
#include "controllers.h"
#include <stdint.h>

#define XINPUT_REPORT_LEN 20

// The Xbox 360 input report for a 0x45/0x42 state, under the active type's
// button config.
void xinputBuildReport(const uint8_t *rep, uint8_t report[XINPUT_REPORT_LEN]);

class XboxController : public IController {
    public:
	void begin() override;
	void onReport45(int slot, const uint8_t *rep, bool fresh,
			uint8_t bodyTlen) override;
	void task() override;
	bool dynamicMount() const override
	{
		return false;
	}
	void usbIdentity() override;
};
extern XboxController g_xboxCtl;
