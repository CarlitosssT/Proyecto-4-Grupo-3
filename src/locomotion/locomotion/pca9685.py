#!/usr/bin/env python3
"""Driver mínimo del PCA9685 (16 canales PWM de 12 bits por I2C)."""

import time

from smbus2 import SMBus

# Registros
MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD
PRESCALE = 0xFE

# Bits de MODE1 / MODE2
MODE1_RESTART = 0x80
MODE1_AI = 0x20      # auto-incremento de registro
MODE1_SLEEP = 0x10
MODE2_OUTDRV = 0x04  # salidas totem-pole (push-pull)

FULL_BIT = 0x10      # bit 4 de LEDn_ON_H / LEDn_OFF_H
OSC_HZ = 25_000_000


class PCA9685:
    def __init__(self, bus=1, address=0x40, frequency=1000):
        self.address = address
        self.bus = SMBus(bus)

        self.bus.write_byte_data(self.address, MODE2, MODE2_OUTDRV)
        self.bus.write_byte_data(self.address, MODE1, MODE1_AI)
        time.sleep(0.005)  # el oscilador tarda ~500 us en arrancar
        self.set_frequency(frequency)
        self.all_off()

    def set_frequency(self, frequency):
        # El prescaler solo se puede escribir con el chip dormido.
        prescale = round(OSC_HZ / (4096 * frequency)) - 1
        prescale = max(3, min(255, prescale))  # ~24 Hz .. ~1526 Hz

        old_mode = self.bus.read_byte_data(self.address, MODE1)
        self.bus.write_byte_data(self.address, MODE1, (old_mode & ~MODE1_RESTART) | MODE1_SLEEP)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE1, old_mode & ~MODE1_SLEEP)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, MODE1, (old_mode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI)

    def set_duty(self, channel, duty):
        """duty en [0, 1]. 0 y 1 usan los bits full-off/full-on (sin pulsos)."""
        if duty <= 0.0:
            data = [0, 0, 0, FULL_BIT]
        elif duty >= 1.0:
            data = [0, FULL_BIT, 0, 0]
        else:
            off = int(duty * 4095)
            data = [0, 0, off & 0xFF, off >> 8]
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel, data)

    def all_off(self):
        self.bus.write_byte_data(self.address, ALL_LED_OFF_H, FULL_BIT)

    def close(self):
        self.all_off()
        self.bus.close()
