/*
 * SkyWatch sim mobile — USB device bring-up.
 *
 * Registers a single CDC ACM class instance on top of the nRF52840 USB
 * device-next stack. The sim mobile multiplexes the Zephyr shell, log
 * backend, position JSON, and WARN text lines all over the same port
 * (/dev/ttyACM0 host-side), so a single CDC instance is sufficient —
 * unlike the controller which needs two (shell on ACM0, JSON on ACM1).
 *
 * Replaces `CDC_ACM_SERIAL_INITIALIZE_AT_BOOT` so we control the
 * enumeration order and the manufacturer / product strings.
 */
#include <zephyr/usb/usbd.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(usb_device, LOG_LEVEL_INF);

/* Custom USB init — registers cdc_acm_0 (shell + data, /dev/ttyACM0).
 * Replaces CDC_ACM_SERIAL_INITIALIZE_AT_BOOT. */
USBD_DEVICE_DEFINE(miniproject_usbd,
		   DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)),
		   0x2fe3, 0x0005);

USBD_DESC_LANG_DEFINE(usbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_mfr, "Zephyr Project");
USBD_DESC_PRODUCT_DEFINE(usbd_product, "Base Node Dual CDC");

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");

USBD_CONFIGURATION_DEFINE(usbd_fs_config,
			  0, 125, &fs_cfg_desc);

// runs at APPLICATION init priority — sets up USB descriptors and both CDC ACM interfaces
static int usb_device_init(void)
{
	int err;

	err = usbd_add_descriptor(&miniproject_usbd, &usbd_lang);
	if (err) {
		LOG_ERR("lang descriptor: %d", err);
		goto do_init;
	}

	err = usbd_add_descriptor(&miniproject_usbd, &usbd_mfr);
	if (err) {
		LOG_ERR("mfr descriptor: %d", err);
		goto do_init;
	}

	err = usbd_add_descriptor(&miniproject_usbd, &usbd_product);
	if (err) {
		LOG_ERR("product descriptor: %d", err);
		goto do_init;
	}

	err = usbd_add_configuration(&miniproject_usbd, USBD_SPEED_FS,
				     &usbd_fs_config);
	if (err) {
		LOG_ERR("add config: %d", err);
		goto do_init;
	}

	// cdc_acm_0 = board_cdc_acm_uart (shell / ttyACM0) — must succeed
	err = usbd_register_class(&miniproject_usbd, "cdc_acm_0",
				  USBD_SPEED_FS, 1);
	if (err) {
		LOG_ERR("register cdc_acm_0: %d", err);
		goto do_init;
	}

	// cdc_acm_1 = cdc_acm_uart0 (data / ttyACM1) — best-effort
	err = usbd_register_class(&miniproject_usbd, "cdc_acm_1",
				  USBD_SPEED_FS, 1);
	if (err) {
		LOG_WRN("register cdc_acm_1 failed (%d) — only ttyACM0", err);
	}
	// important: set IAD class triple so windows enumerates both ACM interfaces correctly
	usbd_device_set_code_triple(&miniproject_usbd, USBD_SPEED_FS,
				    USB_BCC_MISCELLANEOUS, 0x02, 0x01);

do_init:
	err = usbd_init(&miniproject_usbd);
	if (err) {
		LOG_ERR("usbd_init: %d", err);
		return 0;
	}

	err = usbd_enable(&miniproject_usbd);
	if (err) {
		LOG_ERR("usbd_enable: %d", err);
	}

	LOG_INF("USB device initialized");
	return 0;
}

SYS_INIT(usb_device_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
