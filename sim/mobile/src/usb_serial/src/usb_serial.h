/*
 * SkyWatch sim mobile — USB-CDC TX helper (public API).
 *
 * Thin wrapper around the single CDC ACM port (host /dev/ttyACM0).
 * Used by codein.c to emit position JSON + WARN text lines.
 */
#ifndef USB_SERIAL_H
#define USB_SERIAL_H

int usb_serial_init(void);

/* Print a line (appends '\n'). Safe to call from multiple producers. */
void usb_serial_println(const char *str);

#endif /* USB_SERIAL_H */
