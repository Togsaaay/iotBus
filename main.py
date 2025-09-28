from machine import UART, Pin, I2C
import time
import struct, _thread
import utime
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

import time
import struct

# Import for HTTP requests and WiFi
try:
    import urequests as requests
    import network
    HTTP_AVAILABLE = True
    WIFI_AVAILABLE = True
    print("HTTP and WiFi modules available")
except ImportError:
    try:
        import requests
        import network
        HTTP_AVAILABLE = True
        WIFI_AVAILABLE = True
        print("HTTP and WiFi modules available (standard)")
    except ImportError:
        HTTP_AVAILABLE = False
        WIFI_AVAILABLE = False
        print("HTTP/WiFi modules not available")

# Health check and prediction configuration
HEALTH_URL = "https://ridealert-backend.onrender.com/health"
PREDICT_URL = "https://ridealert-backend.onrender.com/predict"
REFRESH_INTERVAL = 7.5  # seconds


VEHICLE_ID = "Janith"  # Static vehicle identifier
DEVICE_ID = "Janith ni nag test"  # Static device identifier



# WiFi credentials 

WIFI_SSID = "JFADeco_AD5C"  
WIFI_PASSWORD = "1234567890"  
# Status tracking
wifi_connected = False
last_health_status = "Unknown"
last_prediction_status = "Unknown"


def connect_wifi():
    """Connect to WiFi network"""
    if not WIFI_AVAILABLE:
        print("WiFi not available")
        return False

    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

        if wlan.isconnected():
            print("Already connected to WiFi")
            print(f"IP: {wlan.ifconfig()[0]}")
            return True

        print(f"Connecting to WiFi: {WIFI_SSID}")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        # Wait for connection
        timeout = 10
        while timeout > 0:
            if wlan.isconnected():
                print(f"WiFi connected! IP: {wlan.ifconfig()[0]}")
                return True
            time.sleep(1)
            timeout -= 1
            print(".", end="")

        print("\nWiFi connection failed")
        return False

    except Exception as e:
        print(f"WiFi connection error: {e}")
        return False


def ping_health_endpoint():
    """Make a proper GET request to the health endpoint"""
    global last_health_status

    if not HTTP_AVAILABLE:
        print("HTTP not available - skipping health check")
        last_health_status = "No HTTP"
        return False

    # Check WiFi connection first
    if not connect_wifi():
        print("No WiFi connection - skipping health check")
        last_health_status = "No WiFi"
        return False

    try:
        print(f"Making GET request to: {HEALTH_URL}")

        # Make the GET request with proper headers
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Accept': 'application/json'
        }

        response = requests.get(HEALTH_URL, headers=headers, timeout=10)
        # post with signed JWT token device id

        print(f"Response status: {response.status_code}")

        # Try to read response content
        try:
            content = response.text
            print(f"Response content: {content[:100]}...")  # First 100 chars
        except:
            print("Could not read response content")

        if response.status_code == 200:
            print("✓ Health check successful")
            last_health_status = "OK"
            response.close()
            return True
        else:
            print(f"✗ Health check failed with status: {response.status_code}")
            last_health_status = f"HTTP {response.status_code}"
            response.close()
            return False

    except OSError as e:
        print(f"Network error: {e}")
        last_health_status = "Network Error"
        return False
    except Exception as e:
        print(f"Health check error: {e}")
        
        return False
    
def get_highest_snr_satellite():
    """Get the satellite with the highest SNR from GPS data"""
    if not gps_data['satellite_info']:
        print("No satellite data available")
        return None

    highest_snr = -1
    best_satellite = None

    print("Available satellites:")
    for svid, info in gps_data['satellite_info'].items():
        try:
            # Extract SNR value (remove ' dBHz' suffix)
            snr_value = float(info['snr'].split()[0])
            print(f"  Satellite {svid}: SNR {snr_value} dBHz")

            if snr_value > highest_snr:
                highest_snr = snr_value
                best_satellite = {
                    'svid': svid,
                    'snr': snr_value,
                    'elevation': float(info['elevation'].replace('°', '')),
                    'azimuth': float(info['azimuth'].replace('°', ''))
                }
        except (ValueError, KeyError, IndexError) as e:
            print(f"  Satellite {svid}: Invalid data ({e})")
            continue

    if best_satellite:
        print(
            f"Selected satellite {best_satellite['svid']} with highest SNR: {best_satellite['snr']} dBHz")

    return best_satellite


def display_system_status():
    """Display system status on LCD"""
    with lcd_lock:
        lcd.clear()

        # Line 1: WiFi status
        if wifi_connected:
            lcd.putstr("WiFi: Connected")
        else:
            lcd.putstr("WiFi: Disconnected")

        # Line 2: GPS status
        lcd.move_to(0, 1)
        if gps_data['latitude'] and gps_data['longitude']:
            lcd.putstr(f"GPS: {len(gps_data['satellite_info'])} sats")
        else:
            lcd.putstr("GPS: No fix")

        # Line 3: Backend status
        lcd.move_to(0, 2)
        lcd.putstr(f"Backend: {last_health_status}")

        # Line 4: Current time
        lcd.move_to(0, 3)
        current_time = utime.localtime()
        lcd.putstr(
            f"{current_time[3]:02d}:{current_time[4]:02d}:{current_time[5]:02d}")


def make_prediction_request(mpu_data):
    """Make a prediction request with GPS and IMU data"""
    global last_prediction_status

    if not HTTP_AVAILABLE:
        print("HTTP not available - skipping prediction")
        last_prediction_status = "No HTTP"
        return False

    # Check WiFi connection
    if not connect_wifi():
        print("No WiFi connection - skipping prediction")
        last_prediction_status = "No WiFi"
        return False

    # Get highest SNR satellite
    best_satellite = get_highest_snr_satellite()
    if not best_satellite:
        print("No satellite data available - skipping prediction")
        last_prediction_status = "No Satellite"
        return False

    # Check if we have GPS fix and location data
    if gps_data['fix_status'] != "Active":
        print("No GPS fix - skipping prediction")
        last_prediction_status = "No GPS Fix"
        return False

    # Prepare payload
    try:
        # Extract latitude/longitude values (remove degree symbol)
        raw_lat = float(gps_data['latitude'].replace('°', ''))
        raw_lon = float(gps_data['longitude'].replace('°', ''))
        raw_alt = float(gps_data['altitude'].replace(' m', ''))

        # Extract speed value (remove ' km/h' suffix)
        raw_speed = float(gps_data['speed'].replace(' km/h', '')) if gps_data['speed'] else 0.0
        
        payload = {
            "vehicle_id": VEHICLE_ID,
            "device_id": DEVICE_ID,
            "Cn0DbHz": best_satellite['snr'],
            "Svid": int(best_satellite['svid']),
            "SvElevationDegrees": best_satellite['elevation'],
            "SvAzimuthDegrees": best_satellite['azimuth'],
            "IMU_MessageType": "UncalAccel",
            "MeasurementX": mpu_data["accel"][0],
            "MeasurementY": mpu_data["accel"][1],
            "MeasurementZ": mpu_data["accel"][2],
            "BiasX": 0.0,
            "BiasY": 0.0,
            "BiasZ": 0.0,
            "raw_latitude": raw_lat,
            "raw_longitude": raw_lon,
            "raw_altitude": raw_alt,
            "raw_speed_kmh": raw_speed
        }

        print(
            f"Making prediction with satellite {best_satellite['svid']} (SNR: {best_satellite['snr']} dBHz)")

        # Make POST request
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Content-Type': 'application/json'
        }

        response = requests.post(
            PREDICT_URL, json=payload, headers=headers, timeout=10)

        print(f"Prediction response status: {response.status_code}")

        if response.status_code == 200:
            try:
                result = response.json()
                print(f"✓ Prediction successful: {result}")
                last_prediction_status = "OK"
                response.close()
                return True
            except:
                print("✓ Prediction request successful (could not parse JSON)")
                last_prediction_status = "OK"
                response.close()
                return True
        else:
            print(f"✗ Prediction failed with status: {response.status_code}")
            try:
                print(f"Error response: {response.text}")
            except:
                pass
            last_prediction_status = f"HTTP {response.status_code}"
            response.close()
            return False

    except Exception as e:
        print(f"Error preparing prediction payload: {e}")
        last_prediction_status = "Payload Error"
        return False
    except OSError as e:
        print(f"Network error during prediction: {e}")
        last_prediction_status = "Network Error"
        return False


# LCD part ni
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(5), sda=Pin(19), freq=400000)
lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)

# Thread-safe LCD lock
lcd_lock = _thread.allocate_lock()

def show_message(line1="", line2="", line3="", line4=""):
    with lcd_lock:
        lcd.clear()
        lcd.move_to(0,0); lcd.putstr(line1[:20])
        lcd.move_to(0,1); lcd.putstr(line2[:20])
        lcd.move_to(0,2); lcd.putstr(line3[:20])
        lcd.move_to(0,3); lcd.putstr(line4[:20])


# GPS
try:
    gps = UART(1, baudrate=9600, tx=18, rx=16)
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
        gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(
            parts[1]) >= 6 else ''
        gps_data['latitude'] = convert_coord(
            parts[2], parts[3]) if parts[2] else '0.0'
        gps_data['longitude'] = convert_coord(
            parts[4], parts[5]) if parts[4] else '0.0'
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
        gps_data['fix_type'] = 'No fix' if parts[2] == '1' else (
            '2D' if parts[2] == '2' else '3D')
        if len(parts) > 15:
            gps_data['pdop'] = parts[15]
        if len(parts) > 16:
            gps_data['hdop'] = parts[16]
        if len(parts) > 17:
            gps_data['vdop'] = parts[17].split('*')[0]

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
                print(
                    f"{svid:>3}  {info['elevation']:>8}  {info['azimuth']:>7}  {info['snr']}")

    print(f"\n[Last NMEA Sentences]")
    for sentence in gps_data['raw_sentences'][-3:]:
        print(sentence)

# MPU
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
    accel_x /= 16384.0
    accel_y /= 16384.0
    accel_z /= 16384.0
    temp_c = (temp_raw / 340.0) + 36.53
    gyro_x /= 131.0
    gyro_y /= 131.0
    gyro_z /= 131.0
    return {"accel": (accel_x, accel_y, accel_z), "gyro": (gyro_x, gyro_y, gyro_z), "temp": temp_c}


# Keypad
row_pins = [13, 12, 14, 27]
col_pins = [26, 25, 23, 32]
rows = [Pin(pin, Pin.OUT) for pin in row_pins]
cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]

key_map = [['1', '2', '3', 'A'], ['4', '5', '6', 'B'],
           ['7', '8', '9', 'C'], ['*', '0', '#', 'D']]
last_key = None
last_time = 0

# Global status variable that can be modified by keypad thread
current_status = "STANDBY"
status_lock = _thread.allocate_lock()

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


# Add a timestamp when keypad was last pressed
last_keypad_time = utime.ticks_ms()

def keypad_thread():
    global current_status, last_keypad_time
    print("Keypad thread started")

    while True:
        key = scan_keypad()
        if key:
            with status_lock:
                last_keypad_time = utime.ticks_ms()
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
        
        time.sleep_ms(50)



# Main Loop
print("\nSystem Running...")

# Initialize WiFi connection for health checks
if HTTP_AVAILABLE and WIFI_AVAILABLE:
    print("Initializing WiFi for health checks...")
    wifi_connected = connect_wifi()
    if wifi_connected:
        print("WiFi ready for health checks")
    else:
        print("WiFi connection failed - health checks will be skipped")
else:
    wifi_connected = False
    print("WiFi not available - health checks disabled")

show_message("Bus Online", "Monitoring...")

# Start keypad thread
print("Starting keypad thread...")
_thread.start_new_thread(keypad_thread, ())
time.sleep(1)  # Give thread time to start

while True:
    # Thread-safe status reading
    with status_lock:
        print("Bus Status:", current_status)

    # GPS
    # GPS speed only
    if gps_enabled and gps.any():
        try:
            sentence = gps.readline().decode('ascii').strip()
            parse_nmea(sentence)
            if gps_data['fix_status'] == "Active":
                current_speed = gps_data['speed']
                print(f"Current Speed (GPS): {current_speed}")
            else:
                print("Searching for satellites...", end='\r')
        except Exception as e:
            print("GPS error:", e)
    else:
        print("GPS: No data")

    # MPU
    mpu_data = None
    if mpu_enabled:
        mpu_data = read_mpu()
        if mpu_data:
            print("Accel (g):", mpu_data["accel"])
            print("Gyro (°/s):", mpu_data["gyro"])
            print("Temp (C):", round(mpu_data["temp"], 2))
        else:
            print("MPU: Failed to read data")
    else:
        print("MPU: Not detected")

    # Health check ping
    print("--- Health Check ---")
    health_status = ping_health_endpoint()
    if health_status:
        print("✓ Backend health: OK")
    else:
        print("✗ Backend health: FAILED")
    print("--- End Health Check ---")

    # Prediction request
    print("--- Prediction Request ---")
    if mpu_enabled and mpu_data:
        prediction_status = make_prediction_request(mpu_data)
        if prediction_status:
            print("✓ Prediction request: OK")
        else:
            print("✗ Prediction request: FAILED")
    else:
        print("⚠ Prediction skipped: No MPU data")
    print("--- End Prediction ---")

    # Sleep for 7.5 seconds
    print(f"Waiting {REFRESH_INTERVAL} seconds...")
    time.sleep(REFRESH_INTERVAL)
