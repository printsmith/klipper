# TMC2130 configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import pins

IHOLDDELAY = 4
TPOWERDOWN = 8
BLANK_TIME_SELECT = 1
TOFF = 5
HSTRT = 3
HEND = 2

class tmc2130:
    def __init__(self, config):
        printer = config.get_printer()
        # Register DUMP_TMC2130
        gcode = printer.lookup_object("gcode")
        gcode.register_command("DUMP_TMC2130", self.cmd_DUMP_TMC2130)
        # pin setup
        ppins = printer.lookup_object("pins")
        enable_pin = config.get('enable_pin')
        enable_pin_params = ppins.lookup_pin('digital_out', enable_pin)
        if enable_pin_params['invert']:
            raise pins.error("tmc2130 can not invert pin")
        self.mcu = enable_pin_params['chip']
        self.pin = enable_pin_params['pin']
        if config.getboolean('full_reset', True):
            self.full_reset()
        run_current = config.getfloat('run_current', above=0.)
        hold_current = config.getfloat('hold_current', above=0.)
        sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
                 '8': 5, '4': 6, '2': 7, '1': 8}
        microsteps = config.getchoice('microsteps', steps, '16')
        # configure CHOPCONF
        vsense = False
        irun = self.current_bits(run_current, sense_resistor, vsense)
        ihold = self.current_bits(hold_current, sense_resistor, vsense)
        if irun < 16 and ihold < 16:
            vsense = True
            irun = self.current_bits(run_current, sense_resistor, vsense)
            ihold = self.current_bits(hold_current, sense_resistor, vsense)
        self.add_config_cmd(
            0x6c, TOFF | (HSTRT << 4) | (HEND << 7) | (BLANK_TIME_SELECT << 15)
            | (vsense << 17) | (microsteps << 24))
        # configure IHOLD_IRUN
        self.add_config_cmd(0x10, ihold | (irun << 8) | (IHOLDDELAY << 16))
        # configure TPOWERDOWN
        self.add_config_cmd(0x11, TPOWERDOWN)
    def add_config_cmd(self, addr, val):
        self.mcu.add_config_cmd("send_spi_message pin=%s msg=%02x%08x" % (
            self.pin, (addr | 0x80) & 0xff, val & 0xffffffff))
    def full_reset(self):
        regs = [0x00, 0x10, 0x11, 0x13, 0x14, 0x15, 0x2d, 0x33,
                0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
                0x68, 0x69, 0x6c, 0x6e, 0x70, 0x72]
        state = {0x60: 0xAAAAB554, 0x61: 0x4A9554AA, 0x62: 0x24492929,
                 0x63: 0x10104222, 0x64: 0xFBFFFFFF, 0x65: 0xB5BB777D,
                 0x66: 0x49295556, 0x67: 0x00404222,
                 0x68: 0xFFFF8056, 0x69: 0x00F70000,
                 0x72: 0x00050480}
        for reg in regs:
            self.add_config_cmd(reg, state.get(reg, 0x00000000))
    def current_bits(self, current, sense_resistor, vsense_on):
        sense_resistor += 0.020
        vsense = 0.32
        if vsense_on:
            vsense = 0.18
        cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense
                 - 1. + .5)
        return max(0, min(31, cs))
    def cmd_DUMP_TMC2130(self, params):
        serial = self.mcu._serial
        mcu_type = serial.msgparser.get_constant('MCU')
        pin_resolver = pins.PinResolver(mcu_type)

        eventtime = self.mcu.monotonic()
        regs = [0x00, 0x01, 0x04, 0x12, 0x2d, 0x6a, 0x6b, 0x6c, 0x6f,
                0x71, 0x73, 0x73]
        for addr in regs:
            eventtime = self.mcu.pause(eventtime + 0.005)
            msg = "send_spi_message pin=%s msg=%02x%08x" % (self.pin, addr, 0)
            self.mcu._serial.send(pin_resolver.update_command(msg))

def load_config_prefix(config):
    return tmc2130(config)
