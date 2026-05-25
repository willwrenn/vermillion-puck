#ifndef USB_SERIAL_H
#define USB_SERIAL_H

int usb_serial_init(void);

// Print a JSON line (appends \n). Thread-safe.
void usb_serial_println(const char *str);

#endif // USB_SERIAL_H
