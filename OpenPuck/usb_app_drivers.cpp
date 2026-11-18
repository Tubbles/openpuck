#include "usb_app_drivers.h"

extern "C" const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *count)
{
	// SC2 Link's interface 0 differs from the Xbox mode's only in the OUT
	// interval, so its driver goes first instead of relying on that one to
	// reject it. It opens nothing outside SC2 Link.
	static const usbd_class_driver_t drivers[] = {
		*sc2LinkClassDriver(),
		*xinputClassDriver(),
		*xboxOgClassDriver(),
	};

	*count = sizeof(drivers) / sizeof(drivers[0]);
	return drivers;
}
