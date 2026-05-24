/*
 * SkyWatch controller — clean-data-port TX wrapper.
 *
 * Sits on top of `cdc_acm_uart1` (ttyACM1, the JSON data port — distinct
 * from the shell/log port on ttyACM0). Multiple producers call
 * `usb_serial_println()` concurrently — collision detector, db_publisher,
 * missile/ghost workers — so the implementation serialises them under a
 * single mutex to keep JSON lines atomic on the wire. DTR-gated so we
 * silently drop writes when no host is reading.
 */
#ifndef USB_SERIAL_H
#define USB_SERIAL_H

int  usb_serial_init(void);

/* Print a JSON line (appends \n). Thread-safe. DTR-gated — drops on the
 * floor when the host hasn't asserted DTR on ttyACM1. */
void usb_serial_println(const char *str);

#endif /* USB_SERIAL_H */
