# usb_device

**Role:** Bring up the nRF52840 USB device stack with **two CDC ACM interfaces**
over one physical USB port.

**Files:** `src/usb_device.c`

**What it does:** Runs at `SYS_INIT(APPLICATION)` priority (so it happens
before `main()`). Registers two CDC ACM classes against the Zephyr USB
device-next stack:

| Class | Enumerates as | Wired in DTS as | Used for |
|---|---|---|---|
| `cdc_acm_0` | `/dev/ttyACM0` | `board_cdc_acm_uart` (board DTSI) | Zephyr shell + log output |
| `cdc_acm_1` | `/dev/ttyACM1` | `cdc_acm_uart0` (in `app.overlay`) | Data port (JSON in/out) |

Also sets the IAD class triple (`USB_BCC_MISCELLANEOUS / 0x02 / 0x01`) so
Windows enumerates both interfaces correctly.

**Do not touch unless:** you're changing how many ACM ports exist, the USB
PID/VID, or migrating off device-next. The shell/log/data split downstream of
this lib assumes exactly these two ports.

**Requires:** `CONFIG_USBD_CDC_ACM_CLASS=y`,
`CONFIG_CDC_ACM_SERIAL_INITIALIZE_AT_BOOT=n` (this lib replaces the default
init), and an `app.overlay` declaring `cdc_acm_uart0`.
