import usb.core
import usb.util
import libusb_package
import time


# ============================================================
# uDMX USB DEVICE
# ============================================================

VID = 0x16C0
PID = 0x05DC


class UDMX:
    """
    Python controller for a uDMX USB-DMX interface.
    """

    def __init__(self):

        print("Initializing uDMX...")

        # ----------------------------------------------------
        # Load libusb backend
        # ----------------------------------------------------

        self.backend = libusb_package.get_libusb1_backend()

        if self.backend is None:
            raise RuntimeError(
                "Could not load libusb backend.\n\n"
                "Run:\n"
                "python -m pip install --upgrade pyusb libusb-package"
            )

        print("libusb backend loaded.")

        # ----------------------------------------------------
        # Find uDMX
        # ----------------------------------------------------

        self.device = usb.core.find(
            backend=self.backend,
            idVendor=VID,
            idProduct=PID
        )

        if self.device is None:
            raise RuntimeError(
                "\n"
                "uDMX device was not found.\n\n"
                "Expected:\n"
                f"VID = 0x{VID:04X}\n"
                f"PID = 0x{PID:04X}\n\n"
                "Check that:\n"
                "1. uDMX is connected.\n"
                "2. The USB driver is installed.\n"
                "3. Device Manager detects the uDMX."
            )

        print("uDMX found!")

        print(
            f"VID: {self.device.idVendor:04X}"
        )

        print(
            f"PID: {self.device.idProduct:04X}"
        )

        # ----------------------------------------------------
        # Configure USB device
        # ----------------------------------------------------

        try:
            self.device.set_configuration()
        except usb.core.USBError:
            # Device may already be configured
            pass

        print("uDMX initialized successfully.")

    # ========================================================
    # SEND DMX CHANNELS
    # ========================================================

    def send_channels(self, start_channel, values):

        """
        Send a group of consecutive DMX channels.

        Example:

            send_channels(1, [255, 0, 0, 0])

        means:

            Channel 1 = 255
            Channel 2 = 0
            Channel 3 = 0
            Channel 4 = 0
        """

        # ----------------------------------------------------
        # Check channel number
        # ----------------------------------------------------

        if start_channel < 1 or start_channel > 512:
            raise ValueError(
                "DMX channel must be between 1 and 512."
            )

        # ----------------------------------------------------
        # Check values
        # ----------------------------------------------------

        for value in values:

            if value < 0 or value > 255:
                raise ValueError(
                    "DMX values must be between 0 and 255."
                )

        # ----------------------------------------------------
        # Check that we don't exceed channel 512
        # ----------------------------------------------------

        last_channel = start_channel + len(values) - 1

        if last_channel > 512:

            raise ValueError(
                "DMX data exceeds channel 512."
            )

        # ----------------------------------------------------
        # uDMX uses zero-based channel addressing
        # ----------------------------------------------------

        start_address = start_channel - 1

        number_of_channels = len(values)

        # ----------------------------------------------------
        # uDMX USB command
        #
        # bmRequestType = 0x40
        # bRequest      = 0x02
        # wValue        = number of channels
        # wIndex        = starting channel
        # data          = DMX values
        # ----------------------------------------------------

        try:

            self.device.ctrl_transfer(
                0x40,
                0x02,
                number_of_channels,
                start_address,
                bytes(values),
                timeout=5000
            )

        except usb.core.USBError as error:

            raise RuntimeError(
                f"Failed to send DMX data: {error}"
            )

    # ========================================================
    # SET ONE DMX CHANNEL
    # ========================================================

    def set_channel(self, channel, value):

        """
        Set a single DMX channel.
        """

        self.send_channels(
            channel,
            [value]
        )

    # ========================================================
    # BLACKOUT
    # ========================================================

    def blackout(self):

        """
        Turn all 512 DMX channels OFF.
        """

        self.send_channels(
            1,
            [0] * 512
        )

        print("All DMX channels OFF.")

    # ========================================================
    # TEST CHANNEL
    # ========================================================

    def test_channel(self, channel):

        """
        Set one channel to full brightness.
        """

        print(
            f"Testing DMX channel {channel}..."
        )

        self.set_channel(
            channel,
            255
        )

        time.sleep(8)

        self.set_channel(
            channel,
            0
        )

        print("Test complete.")


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    try:

        dmx = UDMX()

        print()
        print("==============================")
        print("uDMX DMX CONTROLLER")
        print("==============================")
        print()

        # Test DMX channel 1
        dmx.test_channel(1)

        # Turn everything off
        dmx.blackout()

        print()
        print("uDMX test finished successfully.")

    except Exception as error:

        print()
        print("==============================")
        print("ERROR")
        print("==============================")
        print()
        print(error)