#!/bin/bash
# Host: reattach usbhid to the ZED 2i's HID sensor interface (2b03:f881) if nothing owns it.
# The ZED SDK detaches usbhid to use libusb and does not reattach it on exit, which removes
# /dev/hidraw*; the next start then fails with "sensors returned an invalid serial number".
# Driver usbfs means a running ZED SDK owns the interface: leave it alone.
for dev in /sys/bus/usb/devices/*; do
    [ "$(cat "$dev/idVendor" 2>/dev/null):$(cat "$dev/idProduct" 2>/dev/null)" = "2b03:f881" ] || continue
    for intf in "$dev"/*:1.*; do
        [ -e "$intf/driver" ] && continue
        echo -n "$(basename "$intf")" | sudo -n tee /sys/bus/usb/drivers/usbhid/bind >/dev/null \
            && echo "rebound ZED HID ($(basename "$intf"))"
    done
done
exit 0
