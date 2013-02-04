#!/usr/bin/env python
#
# File: $Id$
#
"""
Demo code to read the Adafruit TLS2561 breakout over the i2c bus on a
Raspberry Pi
"""

# system imports
#
from datetime import datetime
import time

# 3rd party imports
#
from Adafruit_I2C import Adafruit_I2C

##################################################################
##################################################################
#
class TSL2561(object):
    """
    A port of the Adafruit TSL2561 code...
    """
    VISIBLE      = 2 # channel 0 - channel 1
    INFRARED     = 1 # channel 1
    FULLSPECTRUM = 0 # channel 0

    # 3 i2c address options!
    #
    ADDR_LOW   = 0x29
    ADDR_FLOAT = 0x39
    ADDR_HIGH  = 0x49

    # Lux calculations differ slightly for CS package
    #
    PACKAGE_CS      = False
    PACKAGE_T_FN_CL = True

    READBIT = 0x01

    COMMAND_BIT = 0x80  # Must be 1
    CLEAR_BIT   = 0x40  # Clears any pending interrupt = write 1 to clear
    WORD_BIT    = 0x20  # 1 = read/write word = rather than byte
    BLOCK_BIT   = 0x10  # 1 = using block read/write

    CONTROL_POWERON  = 0x03
    CONTROL_POWEROFF = 0x00

    LUX_LUXSCALE      = 14      # Scale by 2^14
    LUX_RATIOSCALE    = 9       # Scale ratio by 2^9
    LUX_CHSCALE       = 10      # Scale channel values by 2^10
    LUX_CHSCALE_TINT0 = 0x7517  # 322/11 * 2^    LUX_CHSCALE
    LUX_CHSCALE_TINT1 = 0x0FE7  # 322/81 * 2^    LUX_CHSCALE

    # T, FN and CL package values
    #
    LUX_K1T = 0x0040  # 0.125 * 2^RATIO_SCALE
    LUX_B1T = 0x01f2  # 0.0304 * 2^LUX_SCALE
    LUX_M1T = 0x01be  # 0.0272 * 2^LUX_SCALE
    LUX_K2T = 0x0080  # 0.250 * 2^RATIO_SCALE
    LUX_B2T = 0x0214  # 0.0325 * 2^LUX_SCALE
    LUX_M2T = 0x02d1  # 0.0440 * 2^LUX_SCALE
    LUX_K3T = 0x00c0  # 0.375 * 2^RATIO_SCALE
    LUX_B3T = 0x023f  # 0.0351 * 2^LUX_SCALE
    LUX_M3T = 0x037b  # 0.0544 * 2^LUX_SCALE
    LUX_K4T = 0x0100  # 0.50 * 2^RATIO_SCALE
    LUX_B4T = 0x0270  # 0.0381 * 2^LUX_SCALE
    LUX_M4T = 0x03fe  # 0.0624 * 2^LUX_SCALE
    LUX_K5T = 0x0138  # 0.61 * 2^RATIO_SCALE
    LUX_B5T = 0x016f  # 0.0224 * 2^LUX_SCALE
    LUX_M5T = 0x01fc  # 0.0310 * 2^LUX_SCALE
    LUX_K6T = 0x019a  # 0.80 * 2^RATIO_SCALE
    LUX_B6T = 0x00d2  # 0.0128 * 2^LUX_SCALE
    LUX_M6T = 0x00fb  # 0.0153 * 2^LUX_SCALE
    LUX_K7T = 0x029a  # 1.3 * 2^RATIO_SCALE
    LUX_B7T = 0x0018  # 0.00146 * 2^LUX_SCALE
    LUX_M7T = 0x0012  # 0.00112 * 2^LUX_SCALE
    LUX_K8T = 0x029a  # 1.3 * 2^RATIO_SCALE
    LUX_B8T = 0x0000  # 0.000 * 2^LUX_SCALE
    LUX_M8T = 0x0000  # 0.000 * 2^LUX_SCALE

    # CS package values
    #
    LUX_K1C = 0x0043  # 0.130 * 2^RATIO_SCALE
    LUX_B1C = 0x0204  # 0.0315 * 2^LUX_SCALE
    LUX_M1C = 0x01ad  # 0.0262 * 2^LUX_SCALE
    LUX_K2C = 0x0085  # 0.260 * 2^RATIO_SCALE
    LUX_B2C = 0x0228  # 0.0337 * 2^LUX_SCALE
    LUX_M2C = 0x02c1  # 0.0430 * 2^LUX_SCALE
    LUX_K3C = 0x00c8  # 0.390 * 2^RATIO_SCALE
    LUX_B3C = 0x0253  # 0.0363 * 2^LUX_SCALE
    LUX_M3C = 0x0363  # 0.0529 * 2^LUX_SCALE
    LUX_K4C = 0x010a  # 0.520 * 2^RATIO_SCALE
    LUX_B4C = 0x0282  # 0.0392 * 2^LUX_SCALE
    LUX_M4C = 0x03df  # 0.0605 * 2^LUX_SCALE
    LUX_K5C = 0x014d  # 0.65 * 2^RATIO_SCALE
    LUX_B5C = 0x0177  # 0.0229 * 2^LUX_SCALE
    LUX_M5C = 0x01dd  # 0.0291 * 2^LUX_SCALE
    LUX_K6C = 0x019a  # 0.80 * 2^RATIO_SCALE
    LUX_B6C = 0x0101  # 0.0157 * 2^LUX_SCALE
    LUX_M6C = 0x0127  # 0.0180 * 2^LUX_SCALE
    LUX_K7C = 0x029a  # 1.3 * 2^RATIO_SCALE
    LUX_B7C = 0x0037  # 0.00338 * 2^LUX_SCALE
    LUX_M7C = 0x002b  # 0.00260 * 2^LUX_SCALE
    LUX_K8C = 0x029a  # 1.3 * 2^RATIO_SCALE
    LUX_B8C = 0x0000  # 0.000 * 2^LUX_SCALE
    LUX_M8C = 0x0000  # 0.000 * 2^LUX_SCALE

    REG_CONTROL          = 0x00
    REG_TIMING           = 0x01
    REG_THRESHHOLDL_LOW  = 0x02
    REG_THRESHHOLDL_HIGH = 0x03
    REG_THRESHHOLDH_LOW  = 0x04
    REG_THRESHHOLDH_HIGH = 0x05
    REG_INTERRUPT        = 0x06
    REG_CRC              = 0x08
    REG_ID               = 0x0A
    REG_CHAN0_LOW        = 0x0C
    REG_CHAN0_HIGH       = 0x0D
    REG_CHAN1_LOW        = 0x0E
    REG_CHAN1_HIGH       = 0x0F

    INTEG_TIME_13MS      = 0x00    # 13.7ms
    INTEG_TIME_101MS     = 0x01    # 101ms
    INTEG_TIME_402MS     = 0x02    # 402ms

    VALID_INTEG_TIMES = (INTEG_TIME_13MS, INTEG_TIME_101MS, INTEG_TIME_402MS)

    GAIN_0X          = 0x00    # No gain
    GAIN_16X         = 0x10    # 16x gain

    VALID_GAINS = (GAIN_0X, GAIN_16X)

    ##################################################################
    #
    def __init__(self, addr):
        """
        Need the address on which this TSL2561 resides on the i2c bus so we can
        talk to it.
        """
        self.addr = addr
        self.i2c = Adafruit_I2C(self.addr)
        self.initialized = False
        self.integration = self.INTEG_TIME_13MS
        self.gain = self.GAIN_16X

        self.set_timing(self.integration)
        self.set_gain(self.gain)
        return

    ##################################################################
    #
    def enable(self):
        """
        """
        # Writes on value to control register
        #
        self.i2c.write8(self.COMMAND_BIT | self.REG_CONTROL,
                        self.CONTROL_POWERON)
        return

    ##################################################################
    #
    def disable(self):
        """
        """
        # Writes off value to control register
        #
        self.i2c.write8(self.COMMAND_BIT | self.REG_CONTROL,
                        self.CONTROL_POWEROFF)
        return

    ##################################################################
    #
    def calculate_lux(self, ch0, ch1):
        """

        Arguments:
        - `ch0`: fullspectrum value
        - `ch1`: ir only value
        """

        # By default with no scaling ... integration time = 402ms
        #
        ch_scale = (1 << self.LUX_CHSCALE)

        # otherwise set ch_scale based on the integration time selected
        #
        if self.integration == self.INTEG_TIME_13MS:
            ch_scale = self.LUX_CHSCALE_TINT0
        elif self.integration == self.INTEG_TIME_101MS:
            ch_scale = self.LUX_CHSCALE_TINT1

        # Scale for gain (1x or 16x)
        #
        if self.gain != self.GAIN_0X:
            ch_scale = ch_scale << 4

        # scale the channel values
        #
        channel0 = (ch0 * ch_scale) >> self.LUX_CHSCALE
        channel1 = (ch1 * ch_scale) >> self.LUX_CHSCALE

        # find the ratio of the channel values (Channel1/Channel0)
        #
        ratio1 = 0
        if channel0 != 0:
            ratio1 = (channel1 << (self.LUX_RATIOSCALE+1)) / channel0

        # round the ratio value
        #
        ratio = (ratio1 + 1) >> 1
        b = 0
        m = 0

        # Lux calculations differ slightly for CS package
        #
        if self.PACKAGE_CS:
            if ((ratio >= 0) and (ratio <= self.LUX_K1C)):
                b=self.LUX_B1C
                m=self.LUX_M1C
            elif (ratio <= self.LUX_K2C):
                b=self.LUX_B2C
                m=self.LUX_M2C
            elif (ratio <= self.LUX_K3C):
                b=self.LUX_B3C
                m=self.LUX_M3C
            elif (ratio <= self.LUX_K4C):
                b=self.LUX_B4C
                m=self.LUX_M4C
            elif (ratio <= self.LUX_K5C):
                b=self.LUX_B5C
                m=self.LUX_M5C
            elif (ratio <= self.LUX_K6C):
                b=self.LUX_B6C
                m=self.LUX_M6C
            elif (ratio <= self.LUX_K7C):
                b=self.LUX_B7C
                m=self.LUX_M7C
            elif (ratio > self.LUX_K8C):
                b=self.LUX_B8C
                m=self.LUX_M8C
        else: # _must_ be a T/FN/CL package
            if ((ratio >= 0) and (ratio <= self.LUX_K1T)):
                b=self.LUX_B1T
                m=self.LUX_M1T
            elif (ratio <= self.LUX_K2T):
                b=self.LUX_B2T
                m=self.LUX_M2T
            elif (ratio <= self.LUX_K3T):
                b=self.LUX_B3T
                m=self.LUX_M3T
            elif (ratio <= self.LUX_K4T):
                b=self.LUX_B4T
                m=self.LUX_M4T
            elif (ratio <= self.LUX_K5T):
                b=self.LUX_B5T
                m=self.LUX_M5T
            elif (ratio <= self.LUX_K6T):
                b=self.LUX_B6T
                m=self.LUX_M6T
            elif (ratio <= self.LUX_K7T):
                b=self.LUX_B7T
                m=self.LUX_M7T
            elif (ratio > self.LUX_K8T):
                b=self.LUX_B8T
                m=self.LUX_M8T

        # do not allow negative lux value
        #
        temp = ((channel0 * b) - (channel1 * m))
        if temp < 0:
            temp = 0

        # round lsb (2^(LUX_SCALE-1))
        #
        temp += (1 << (self.LUX_LUXSCALE-1))

        # strip off fractional portion
        #
        lux = temp >> self.LUX_LUXSCALE

        # Signal I2C had no errors
        #
        return lux

    ##################################################################
    #
    def set_timing(self, integration = None):
        """
        Update the integration value as supplied in the argument. If the
        'integration' argument is not supplied, use the already set value.

        Arguments:
        - `integration`:
        """
        self.enable()
        if integration is not None:
            self.integration = integration
        self.i2c.write8(self.COMMAND_BIT | self.REG_TIMING,
                        self.integration | self.gain)
        self.disable()
        return

    ##################################################################
    #
    def set_gain(self, gain = None):
        """
        Update the gain value as supplied in the argument. If the
        'gain' argument is not supplied, use the already set value.

        Arguments:
        - `gain`:
        """
        self.enable()
        if gain is not None:
            self.gain = gain
        elif gain not in self.VALID_GAINS:
            raise ValueError("%d is not a valid gain (%s)" % \
                             (gain, repr(self.VALID_GAINS)))

        self.i2c.write8(self.COMMAND_BIT | self.REG_TIMING,
                        self.integration | self.gain)
        self.disable()
        return

    ##################################################################
    #
    def get_id(self):
        """
        Read the part number and silicon revision number for this device.
        It returns a tuple of (part number, revision number)
        """
        self.enable()
        id = self.i2c.readU8(self.COMMAND_BIT | self.REG_ID)
        self.disable()
        print "id is: %d, binary: %s" % (id,bin(id))
        return ((id & 0xf0) >> 4, (id & 0x0f))

    ##################################################################
    #
    def get_luminosity(self):
        """
        Read the full spectrum and ir only luminosities from the device and
        return a tuple of full spectrum, ir, and visible only to the caller.
        """
        # Enable the device by setting the control bit to 0x03
        #
        self.enable()

        # Wait x ms for ADC to complete
        #
        if self.integration == self.INTEG_TIME_13MS:
            start_time = datetime.now()
            time.sleep(0.014)
        elif self.integration == self.INTEG_TIME_101MS:
            start_time = datetime.now()
            time.sleep(0.102)
        else:
            start_time = datetime.now()
            time.sleep(0.402);

        chan1 = self.i2c.readU16(self.COMMAND_BIT | self.WORD_BIT |
                                self.REG_CHAN1_LOW)
        chan0 = self.i2c.readU16(self.COMMAND_BIT | self.WORD_BIT |
                                 self.REG_CHAN0_LOW)
        self.disable();
        return (chan0, chan1)

#############################################################################
#
def main():
    """
    Create a TSL2561 control object instance on the right address, set it up,
    and read lux values from it every couple of seconds.
    """

    tsl2561 = TSL2561(TSL2561.ADDR_FLOAT)
    # tsl2561.set_timing(TSL2561.INTEG_TIME_101MS)
    part_number, revision = tsl2561.get_id()
    print "TSL2561 part number: %d, revision: %d" % (part_number, revision)
    while True:
        full_spectrum, ir = tsl2561.get_luminosity()
        print "\n*******"
        print "Full spectrum: %d, ir: %d, visible: %d" % (full_spectrum, ir,
                                                          full_spectrum - ir)
        print "Lux: %d" % tsl2561.calculate_lux(full_spectrum, ir)
        time.sleep(5)
    return

############################################################################
############################################################################
#
# Here is where it all starts
#
if __name__ == "__main__":
    main()
#
#
############################################################################
############################################################################
