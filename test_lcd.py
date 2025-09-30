from machine import I2C, Pin
from i2c_lcd import I2cLcd
import time

# Scan for I2C devices
i2c = I2C(0, scl=Pin(5), sda=Pin(19), freq=400000)
devices = i2c.scan()

print("Scanning I2C bus...")
print(f"I2C devices found: {[hex(device) for device in devices]}")

if not devices:
    print("No I2C devices found! Check wiring:")
    print("  SCL -> GPIO 5")
    print("  SDA -> GPIO 19")
    print("  VCC -> 5V")
    print("  GND -> GND")
else:
    # Try each address found
    for addr in devices:
        print(f"\nTrying LCD at address {hex(addr)}...")
        try:
            lcd = I2cLcd(i2c, addr, 4, 20)
            time.sleep(0.5)
            
            # Clear and test
            lcd.clear()
            time.sleep(0.1)
            
            lcd.move_to(0, 0)
            lcd.putstr("LCD Test")
            time.sleep(0.1)
            
            lcd.move_to(0, 1)
            lcd.putstr(f"Address: {hex(addr)}")
            time.sleep(0.1)
            
            lcd.move_to(0, 2)
            lcd.putstr("Line 3: OK")
            time.sleep(0.1)
            
            lcd.move_to(0, 3)
            lcd.putstr("Line 4: OK")
            
            print(f"SUCCESS at {hex(addr)}!")
            print("If you see clear text, use this address in your main code.")
            print("If you see garbage, adjust the contrast potentiometer.")
            break
            
        except Exception as e:
            print(f"Failed at {hex(addr)}: {e}")

print("\nTest complete.")