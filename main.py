from machine import UART, Pin, I2C
import time, struct, utime
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

#LCD part ni
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(19), sda=Pin(18), freq=400000)
lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)

def show_message(line1="", line2="", line3="", line4=""):
    lcd.clear()
    lcd.move_to(0, 0); lcd.putstr(line1[:20])
    lcd.move_to(0, 1); lcd.putstr(line2[:20])
    lcd.move_to(0, 2); lcd.putstr(line3[:20])
    lcd.move_to(0, 3); lcd.putstr(line4[:20])

#GPS
try:
    gps = UART(1, baudrate=9600, tx=17, rx=16)
    gps_enabled = True
    print("GPS: Initialized")
except:
    gps = None
    gps_enabled = False
    print("GPS: Not detected")

gps_data = {
    'message_type': '',
    'utc_time': '',
    'date': '',
    'latitude': '',
    'longitude': '',
    'altitude': '',
    'speed': '',
    'course': '',
    'fix_status': '',
    'fix_quality': '',
    'satellites': '',
    'hdop': '',
    'vdop': '',
    'pdop': '',
    'mode': '',
    'satellite_info': {},
    'raw_sentences': []
}

def safe_convert(value, converter, default):
    try:
        return converter(value) if value else default
    except:
        return default

def convert_coord(coord, direction):
    try:
        val = float(coord)
        degrees = int(val / 100)
        minutes = val - degrees * 100
        decimal = degrees + minutes / 60
        if direction in ['S', 'W']:
            decimal = -decimal
        return f"{decimal:.6f}°"
    except:
        return "0.0°"

def parse_nmea(sentence):
    if not sentence.startswith('$'):
        return
    
    parts = sentence.split(',')
    sentence_type = parts[0][1:]
    
    if sentence_type == 'GPGGA':
        gps_data['message_type'] = 'GPGGA'
        gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(parts[1]) >= 6 else ''
        gps_data['latitude'] = convert_coord(parts[2], parts[3]) if parts[2] else '0.0'
        gps_data['longitude'] = convert_coord(parts[4], parts[5]) if parts[4] else '0.0'
        gps_data['fix_quality'] = parts[6]
        gps_data['satellites'] = parts[7]
        gps_data['hdop'] = parts[8]
        gps_data['altitude'] = f"{safe_convert(parts[9], float, 0.0):.1f} m"
        gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"
    
    elif sentence_type == 'GPRMC':
        gps_data['message_type'] = 'GPRMC'
        if len(parts[1]) >= 6:
            gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
        if len(parts[9]) >= 6:
            gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
        gps_data['speed'] = f"{safe_convert(parts[7], float, 0.0) * 1.852:.1f} km/h"
        gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"
        gps_data['mode'] = parts[12][0] if len(parts) > 12 else ''
    
    elif sentence_type == 'GPGSV':
        total_sats = safe_convert(parts[3], int, 0)
        gps_data['satellites'] = str(total_sats)
        for i in range(4):
            idx = 4 + i*4
            if len(parts) > idx + 3:
                svid = parts[idx]
                if svid:
                    gps_data['satellite_info'][svid] = {
                        'elevation': f"{safe_convert(parts[idx+1], int, 0)}°",
                        'azimuth': f"{safe_convert(parts[idx+2], int, 0)}°",
                        'snr': f"{safe_convert(parts[idx+3].split('*')[0], int, 0)} dBHz"
                    }
    
    elif sentence_type == 'GPGSA':
        gps_data['mode'] = 'Auto' if parts[1] == 'A' else 'Manual'
        gps_data['fix_type'] = 'No fix' if parts[2] == '1' else ('2D' if parts[2] == '2' else '3D')
        if len(parts) > 15: gps_data['pdop'] = parts[15]
        if len(parts) > 16: gps_data['hdop'] = parts[16]
        if len(parts) > 17: gps_data['vdop'] = parts[17].split('*')[0]
    
    elif sentence_type == 'GPVTG':
        gps_data['course'] = f"{safe_convert(parts[1], float, 0.0):.1f}°"
        gps_data['speed'] = f"{safe_convert(parts[7], float, 0.0):.1f} km/h"
    
    gps_data['raw_sentences'].append(sentence)

def display_gps_data():
    print("GPS Info")

    print(f"\n[Time/Date]")
    print(f"UTC Time: {gps_data['utc_time']}")
    print(f"Date: {gps_data['date']}")

    print(f"\n[Position]")
    print(f"Latitude: {gps_data['latitude']}")
    print(f"Longitude: {gps_data['longitude']}")
    print(f"Altitude: {gps_data['altitude']}")

    print(f"\n[Movement]")
    print(f"Speed: {gps_data['speed']}")
    print(f"Course: {gps_data['course']}")

    print(f"\n[Fix Information]")
    print(f"Status: {gps_data['fix_status']}")
    print(f"Quality: {gps_data['fix_quality']}")
    print(f"Mode: {gps_data['mode']}")
    print(f"Satellites: {gps_data['satellites']}")

    print(f"\n[Dilution of Precision]")
    print(f"HDOP: {gps_data['hdop']} (Horizontal)")
    print(f"VDOP: {gps_data['vdop']} (Vertical)")
    print(f"PDOP: {gps_data['pdop']} (Position)")

    if gps_data['satellite_info']:
        print(f"\n[Satellite Details] (SNR > 0)")
        print("PRN  Elevation Azimuth  SNR")
        for svid, info in sorted(gps_data['satellite_info'].items(), key=lambda x: int(x[0])):
            if int(info['snr'].split()[0]) > 0:
                print(f"{svid:>3}  {info['elevation']:>8}  {info['azimuth']:>7}  {info['snr']}")
    
    print(f"\n[Last NMEA Sentences]")
    for sentence in gps_data['raw_sentences'][-3:]:
        print(sentence)

#GSM
try:
    gsm = UART(2, baudrate=9600, tx=15, rx=2)
    gsm_enabled = True
    print("GSM: Initialized")
except:
    gsm = None
    gsm_enabled = False
    print("GSM: Not detected")

# === GSM Functions ===
def send_gsm_command(cmd, wait_ms=500):
    """Send AT command and read response."""
    if not gsm_enabled:
        return "GSM not initialized"
    gsm.write((cmd + "\r\n").encode())
    utime.sleep_ms(wait_ms)
    resp = gsm.read()
    if resp:
        return resp.decode(errors="ignore").strip()
    return ""

def gsm_status():
    if not gsm_enabled:
        print("GSM: Not detected")
        return

    for cmd in ["AT", "AT+CPIN?", "AT+CREG?", "AT+CSQ"]:
        gsm.write((cmd + "\r\n").encode())
        utime.sleep_ms(500)
        resp = gsm.read()
        print(f"Command: {cmd}")
        print("Response:", resp.decode(errors="ignore") if resp else "No response")

#MPU
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
devices = i2c_bus.scan()
mpu_enabled = MPU_ADDR in devices

def mpu_write(reg, data):
    i2c_bus.writeto_mem(MPU_ADDR, reg, bytes([data]))

def mpu_read(reg, n=1):
    return i2c_bus.readfrom_mem(MPU_ADDR, reg, n)

if mpu_enabled:
    mpu_write(0x6B, 0)
    print("MPU6050: Initialized")
else:
    print("MPU6050: Not detected")

def read_mpu():
    data = mpu_read(0x3B, 14)
    vals = struct.unpack('>hhhhhhh', data)
    accel_x, accel_y, accel_z, temp_raw, gyro_x, gyro_y, gyro_z = vals
    accel_x /= 16384.0; accel_y /= 16384.0; accel_z /= 16384.0
    temp_c = (temp_raw / 340.0) + 36.53
    gyro_x /= 131.0; gyro_y /= 131.0; gyro_z /= 131.0
    return {"accel": (accel_x, accel_y, accel_z), "gyro": (gyro_x, gyro_y, gyro_z), "temp": temp_c}

#Keypad
row_pins = [13, 12, 14, 27]
col_pins = [26, 25, 23, 32]
rows = [Pin(pin, Pin.OUT) for pin in row_pins]
cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]

key_map = [['1','2','3','A'],['4','5','6','B'],['7','8','9','C'],['*','0','#','D']]
last_key = None
last_time = 0

def scan_keypad():
    global last_key, last_time
    for i, row in enumerate(rows):
        row.value(1)
        for j, col in enumerate(cols):
            if col.value() == 1:
                key = key_map[i][j]
                if key != last_key or (time.ticks_ms() - last_time) > 200:
                    last_key = key
                    last_time = time.ticks_ms()
                    row.value(0)
                    return key
        row.value(0)
    return None

#Main Loop
print("\nSystem Running...")
current_status = "STANDBY"
show_message("Bus Online", "Monitoring...")

while True:
    key = scan_keypad()
    if key:
        if key == '1':
            current_status = "FULL"
            show_message("STATUS:", "FULL")
        elif key == '2':
            current_status = "AVAILABLE"
            show_message("STATUS:", "AVAILABLE")
        elif key == 'A':
            current_status = "STANDING"
            show_message("STATUS:", "STANDING")
        elif key == '4':
            current_status = "INACTIVE"
            show_message("STATUS:", "INACTIVE")
        elif key == '5':
            current_status = "HELP REQUESTED"
            show_message("HELP", "REQUESTED")
        else:
            show_message("Invalid key!", "Use 1,2,A,4,5")
        print("Key pressed:", key)
        print("Bus Status:", current_status)

    print("Bus Status:", current_status)

    #GPS
    if gps_enabled and gps.any():
        try:
            sentence = gps.readline().decode('ascii').strip()
            parse_nmea(sentence)
            if gps_data['fix_status'] == "Active":
                display_gps_data()
            else:
                print("Searching for satellites...", end='\r')
        except Exception as e:
            print("GPS error:", e)
    else:
        print("GPS: No data")

    #GSM
    gsm_status()

    #MPU
    if mpu_enabled:
        mpu_data = read_mpu()
        print("Accel (g):", mpu_data["accel"])
        print("Gyro (°/s):", mpu_data["gyro"])
        print("Temp (C):", round(mpu_data["temp"], 2))
    else:
        print("MPU: Not detected")

    time.sleep(2)
