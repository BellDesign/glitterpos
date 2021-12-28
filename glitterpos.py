"""glitter positioning system"""

import time
import gc
import math
# import adafruit_lsm9ds1
import adafruit_gps
import adafruit_rfm9x
import board
import busio
import digitalio
# import neopixel
# import rtc
from glitterpos_util import timestamp

# glitterpos_cfg.py should be unique to each box, and formatted as follows:
#
# MY_ID = 0 # must be a unique integer
# MAG_MIN = (-0.25046, -0.23506, -0.322)
# MAG_MAX = (0.68278, 0.70882, 0.59654)
# DECLINATION_RAD = 235.27 / 1000.0 # Black Rock City in radians
# #
# From the CircuitPython REPL, use `import calibrate` to find values for
# MAG_MIN and MAG_MAX.
from glitterpos_cfg import MY_ID, MAG_MIN, MAG_MAX, DECLINATION_RAD

# You can add fixed points here:
DEFAULT_BOX_COORDS = {
    # BOULDER_ID: (40.018258, -105.278457)
}

RADIO_FREQ_MHZ = 915.0
# CS = digitalio.DigitalInOut(board.D10)
# RESET = digitalio.DigitalInOut(board.D11)
# Feather M0 RFM9x
CS = digitalio.DigitalInOut(board.RFM9X_CS)
RESET = digitalio.DigitalInOut(board.RFM9X_RST)


class GlitterPOS:
    """glitter positioning system"""

    def __init__(self):
        """configure sensors, radio, blinkenlights"""

        # Our id and the dict for storing coords of other glitterpos_boxes:
        self.glitterpos_id = MY_ID
        self.glitterpos_boxes = DEFAULT_BOX_COORDS

        # Set the RTC to an obviously bogus time for debugging purposes:
        # time_struct takes: (tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec, tm_wday, tm_yday, tm_isdst)
        # rtc.RTC().datetime = time.struct_time((2000, 1, 1, 0, 0, 0, 0, 0, 0))
        # print("startup time: " + timestamp())
        self.time_set = False
        self.last_send = time.monotonic()

        # A tuple for our lat/long:
        self.coords = (0, 0)
        self.heading = 0.0

        # Status light on the board, we'll use to indicate GPS fix, etc.:
        # self.statuslight = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.005, auto_write=True)
        # self.statuslight.fill(RED)

        self.statuslight = digitalio.DigitalInOut(board.D13)
        self.statuslight.direction = digitalio.Direction.OUTPUT
        self.statuslight.value = False

        # # Neopixel ring:
        # self.pixels = neopixel.NeoPixel(board.A1, 16, brightness=0.01, auto_write=False)
        # self.startup_animation()
        time.sleep(2)

        self.init_radio()
        self.init_gps()


    def init_radio(self):
        """Set up RFM95."""
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
        self.rfm9x.tx_power = 18 # Default is 13 dB; the RFM95 goes up to 23 dB
        self.radio_tx('d', 'hello world')
        time.sleep(1)

    def init_gps(self):
        """Set up GPS module."""
        uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)
        gps = adafruit_gps.GPS(uart)
        time.sleep(1)

        # https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
        # Turn on the basic GGA and RMC info (what you typically want), then
        # set update to once a second:
        gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        gps.send_command(b'PMTK220,1000')

        self.gps = gps

    # def init_compass(self):
    #     """Set up LSM9DS1."""
    #     i2c = busio.I2C(board.SCL, board.SDA)
    #     self.compass = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
    #     time.sleep(1)

    def advance_frame(self):
        """
        Check the radio for new packets, poll GPS and compass data, send a
        radio packet if coordinates have changed (or if it's been a while), and
        update NeoPixel display.  Called in an infinite loop by code.py.

        To inspect the state of the system, initialize a new GlitterPOS object
        from the CircuitPython REPL, and call gp.advance_frame() manually.  You
        can then access the instance variables defined in __init__() and
        init_()* methods.
        """

        current = time.monotonic()
        self.radio_rx(timeout=0.5)
        new_gps_data = self.gps.update()
        # self.update_heading()
        # self.display_pixels()

        if not self.gps.has_fix:
            # Try again if we don't have a fix yet.
            self.statuslight.value = False
            print("GPS no fix")
            return

        # We want to send coordinates out either on new GPS data or roughly
        # every 15 seconds:
        if (not new_gps_data) and (current - self.last_send < 15):
            return

        # Set the RTC to GPS time (UTC):
        if new_gps_data and not self.time_set:
            # rtc.RTC().datetime = self.gps.timestamp_utc
            self.time_set = True

        gps_coords = (self.gps.latitude, self.gps.longitude)
        if gps_coords == self.coords:
            return

        self.coords = (self.gps.latitude, self.gps.longitude)

        self.statuslight.value = True
        print(':: ' + str(current))  # Print a separator line.
        # print(timestamp())
        send_packet = '{}:{}:{}:{}'.format(
            self.gps.latitude,
            self.gps.longitude,
            self.gps.speed_knots,
            self.heading
        )

        print('   quality: {}'.format(self.gps.fix_quality))
        print('   ' + str(gc.mem_free()) + " bytes free")

        # Send a location packet:
        self.radio_tx('l', send_packet)


    def radio_tx(self, msg_type, msg):
        """send a packet over radio with id prefix"""
        packet = 'e:' + msg_type + ':' + str(self.glitterpos_id) + ':' + msg
        print('   sending: ' + packet)

        # Blocking, max of 252 bytes:
        self.rfm9x.send(packet)
        self.last_send = time.monotonic()

    def radio_rx(self, timeout=0.5):
        """check radio for new packets, handle incoming data"""

        packet = self.rfm9x.receive()

        # If no packet was received during the timeout then None is returned:
        if packet is None:
            return

        packet = bytes(packet)
        # print(timestamp())
        print('   received signal strength: {0} dB'.format(self.rfm9x.rssi))
        print('   received (raw bytes): {0}'.format(packet))
        pieces = packet.split(b':')

        if pieces[0] != b'e' or len(pieces) < 5:
            print('   bogus packet, bailing out')
            return

        msg_type = pieces[1].format()
        sender_id = int(pieces[2].format())

        # A location message:
        if msg_type == 'l':
            sender_lat = float(pieces[3].format())
            sender_lon = float(pieces[4].format())
            self.glitterpos_boxes[sender_id] = (sender_lat, sender_lon)

        # packet_text = str(packet, 'ascii')
        # print('Received (ASCII): {0}'.format(packet_text))
