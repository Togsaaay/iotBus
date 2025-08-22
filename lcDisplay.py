from machine import Pin, I2C
import time
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

I2C_ADDR = 0x27
i2c = I2C(0, scl=Pin(19), sda=Pin(18), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, 4, 20)

keys = [
    ['1', '2', '3', 'A'],
    ['4', '5', '6', 'B'],
    ['7', '8', '9', 'C'],
    ['*', '0', '#', 'D']
]

row_pins = [Pin(13, Pin.OUT), Pin(12, Pin.OUT), Pin(14, Pin.OUT), Pin(27, Pin.OUT)]
col_pins = [Pin(26, Pin.IN, Pin.PULL_DOWN), Pin(25, Pin.IN, Pin.PULL_DOWN),
            Pin(23, Pin.IN, Pin.PULL_DOWN), Pin(32, Pin.IN, Pin.PULL_DOWN)]

def show_message(line1="", line2="", line3="", line4=""):
    lcd.clear()
    lcd.move_to(0, 0)
    lcd.putstr(line1[:20])
    lcd.move_to(0, 1)
    lcd.putstr(line2[:20])
    lcd.move_to(0, 2)
    lcd.putstr(line3[:20])
    lcd.move_to(0, 3)
    lcd.putstr(line4[:20])

def scan_keypad():
    for row_num, row_pin in enumerate(row_pins):

        for r in row_pins:
            r.value(0)

        row_pin.value(1)
        for col_num, col_pin in enumerate(col_pins):
            if col_pin.value() == 1:
                time.sleep(0.02)
                if col_pin.value() == 1:
                    return keys[row_num][col_num]
    return None

show_message("1=FULL  A=STANDING", "2=AVAILABLE", "4=INACTIVE", "5=HELP ME!")

last_key = None
show_message("1=FULL  A=STANDING", "2=AVAILABLE", "4=INACTIVE", "5=HELP ME!")

while True:
    key = scan_keypad()
    if key and key != last_key:
        last_key = key
        if key == '1':
            print("The bus is full")
            show_message("STATUS:", "FULL")
        elif key == '2':
            print("The bus is available")
            show_message("STATUS:", "AVAILABLE")
        elif key == 'A':
            print("Passengers are standing")
            show_message("STATUS:", "STANDING")
        elif key == '4':
            print("The bus is inactive")
            show_message("The bus is", "INACTIVE")
        elif key == '5':
            print("You requested help")
            show_message("HELP", "REQUESTED")
        else:
            print("Invalid key")
            show_message("Invalid key!", "Use 1, 2, A, 4, 5")
        time.sleep(0.5)

    if key is None:
        last_key = None
