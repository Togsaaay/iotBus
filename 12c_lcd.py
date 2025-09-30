from lcd_api import LcdApi
from machine import I2C
from time import sleep_ms

class I2cLcd(LcdApi):
    def __init__(self, i2c, i2c_addr, num_lines, num_columns):
        self.i2c = i2c
        self.i2c_addr = i2c_addr
        self.num_lines = num_lines
        self.num_columns = num_columns
        self.backlight = 0x08
        sleep_ms(20)
        self.hal_write_init_nibble(0x03)
        sleep_ms(5)
        self.hal_write_init_nibble(0x03)
        sleep_ms(1)
        self.hal_write_init_nibble(0x03)
        self.hal_write_init_nibble(0x02)
        self.hal_write_command(self.LCD_FUNCTION_SET | self.FUNCTION_2LINE)
        self.hal_write_command(self.LCD_DISPLAY_CTRL | self.DISPLAY_ON)
        self.hal_write_command(self.LCD_CLR)
        sleep_ms(2)
        self.hal_write_command(self.LCD_ENTRY_MODE | self.ENTRY_LEFT | self.ENTRY_SHIFT_DECREMENT)

    def hal_write_init_nibble(self, nibble):
        byte = (nibble << 4) | self.backlight
        self.i2c.writeto(self.i2c_addr, bytes([byte | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([byte]))

    def hal_backlight_on(self):
        self.backlight = 0x08
        self.i2c.writeto(self.i2c_addr, bytes([self.backlight]))

    def hal_backlight_off(self):
        self.backlight = 0x00
        self.i2c.writeto(self.i2c_addr, bytes([self.backlight]))

    def hal_write_command(self, cmd):
        self.hal_write_byte(cmd, 0)

    def hal_write_data(self, data):
        self.hal_write_byte(data, 1)

    def hal_write_byte(self, byte, mode):
        high = mode | (byte & 0xF0) | self.backlight
        low = mode | ((byte << 4) & 0xF0) | self.backlight
        self.i2c.writeto(self.i2c_addr, bytes([high | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([high]))
        self.i2c.writeto(self.i2c_addr, bytes([low | 0x04]))
        self.i2c.writeto(self.i2c_addr, bytes([low]))