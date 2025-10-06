from machine import UART, Pin, I2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
import struct
import utime
import _thread
import network
import time
import urequests
import ujson

# -------------------- WiFi Setup (FIRST!) --------------------


def connect_wifi(ssid, password, timeout=10):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(ssid, password)

        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                print("WiFi connection timeout")
                return False
            time.sleep(0.5)

    print('WiFi connected:', wlan.ifconfig()[0])
    return True


# Connect to WiFi BEFORE initializing anything else
WIFI_SSID = "RideAlert-WiFi"
WIFI_PASSWORD = "ride-alert05"
connect_wifi(WIFI_SSID, WIFI_PASSWORD)


# -------------------- Configuration --------------------
API_ENDPOINT = "https://ridealert-backend.onrender.com/predict"
FLEET_ID = "68bee7eb753d0934fd57bdea"
DEVICE_ID = "68e20f304073a18ae0f980b4"
POST_INTERVAL = 2  # Send data every 2 seconds

# -------------------- LCD Setup --------------------
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(5), sda=Pin(19), freq=100000)
lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)

# Thread-safe LCD lock
lcd_lock = _thread.allocate_lock()


def show_message(line1="", line2="", line3="", line4=""):
    with lcd_lock:
        lcd.clear()
        time.sleep_ms(5)  # let LCD settle
        lcd.move_to(0, 0)
        lcd.putstr(line1[:20])
        lcd.move_to(0, 1)
        lcd.putstr(line2[:20])
        lcd.move_to(0, 2)
        lcd.putstr(line3[:20])
        lcd.move_to(0, 3)
        lcd.putstr(line4[:20])


# -------------------- GPS Setup --------------------
try:
    gps = UART(1, baudrate=9600, tx=18, rx=16)
    gps_enabled = True
    print("GPS: Initialized")
except:
    gps = None
    gps_enabled = False
    print("GPS: Not detected")

gps_data = {'utc_time': '', 'date': '', 'latitude': '', 'longitude': '',
            'altitude': '', 'speed': '', 'course': '', 'fix_status': '', 'fix_quality': '', 'satellites': '',
            'raw_latitude': 0.0, 'raw_longitude': 0.0, 'raw_altitude': 0.0, 'raw_speed': 0.0,
            'satellites_detailed': {}}


def convert_coord(coord, direction):
    """Convert NMEA coordinate format to decimal degrees"""
    try:
        if not coord or not direction:
            return "0.0"

        val = float(coord)
        if val == 0:
            return "0.0"

        degrees = int(val // 100)
        minutes = val - (degrees * 100)

        if minutes >= 60:
            return "0.0"

        decimal = degrees + (minutes / 60)

        if direction.upper() in ['S', 'W']:
            decimal = -decimal

        return str(round(decimal, 6))

    except:
        return "0.0"


def convert_coord_raw(coord, direction):
    """Convert NMEA coordinate format to raw decimal degrees (float)"""
    try:
        if not coord or not direction:
            return 0.0

        val = float(coord)
        if val == 0:
            return 0.0

        degrees = int(val // 100)
        minutes = val - (degrees * 100)

        if minutes >= 60:
            return 0.0

        decimal = degrees + (minutes / 60)

        if direction.upper() in ['S', 'W']:
            decimal = -decimal

        return round(decimal, 6)

    except:
        return 0.0


def parse_nmea(sentence):
    """Parse NMEA sentence"""
    try:
        if not sentence or not sentence.startswith('$'):
            return

        sentence = sentence.strip()
        if '*' in sentence:
            sentence = sentence.split('*')[0]

        parts = sentence.split(',')
        if len(parts) < 3:
            return

        sentence_type = parts[0][1:]

        if sentence_type == 'GPGGA' and len(parts) >= 15:
            if parts[1] and len(parts[1]) >= 6:
                gps_data['utc_time'] = parts[1][:6]

            if parts[2] and parts[3]:
                gps_data['latitude'] = convert_coord(parts[2], parts[3])
                gps_data['raw_latitude'] = convert_coord_raw(
                    parts[2], parts[3])

            if parts[4] and parts[5]:
                gps_data['longitude'] = convert_coord(parts[4], parts[5])
                gps_data['raw_longitude'] = convert_coord_raw(
                    parts[4], parts[5])

            gps_data['fix_quality'] = parts[6] if parts[6] else '0'
            gps_data['satellites'] = parts[7] if parts[7] else '0'

            if parts[9]:
                try:
                    alt = float(parts[9])
                    gps_data['altitude'] = str(round(alt, 1)) + " m"
                    gps_data['raw_altitude'] = round(alt, 1)
                except:
                    gps_data['altitude'] = "0.0 m"
                    gps_data['raw_altitude'] = 0.0

            gps_data['fix_status'] = "Active" if parts[6] != '0' and parts[6] != '' else "No Fix"

        elif sentence_type == 'GPRMC' and len(parts) >= 12:
            if parts[2] == 'A':
                if parts[9] and len(parts[9]) == 6:
                    gps_data['date'] = parts[9]
                if parts[7]:
                    try:
                        # GPS speed is in knots
                        # OLD: speed_kmh = float(parts[7]) * 1.852
                        # NEW: Convert knots to m/s directly (1 knot = 0.514444 m/s)
                        speed_mps = float(parts[7]) * 0.514444
                        gps_data['speed'] = str(round(speed_mps, 2)) + " m/s"
                        gps_data['raw_speed'] = round(speed_mps, 2)
                    except:
                        gps_data['speed'] = "0.0 m/s"
                        gps_data['raw_speed'] = 0.0
                if parts[8]:
                    gps_data['course'] = parts[8]

        elif sentence_type == 'GPGSV' and len(parts) >= 8:
            # Parse real satellite data from NEO-6M: SNR, satellite ID, elevation, azimuth
            for i in range(4):  # Up to 4 satellites per GPGSV sentence
                sat_idx = 4 + i * 4
                if len(parts) > sat_idx + 3 and parts[sat_idx]:
                    sat_id = parts[sat_idx]  # Satellite ID (PRN)
                    try:
                        elevation = float(
                            parts[sat_idx + 1]) if parts[sat_idx + 1] else 0.0
                        azimuth = float(
                            parts[sat_idx + 2]) if parts[sat_idx + 2] else 0.0
                        # Parse SNR (Signal-to-Noise Ratio) - NEO-6M's equivalent to C/N0
                        snr_str = parts[sat_idx + 3]
                        if '*' in snr_str:
                            snr_str = snr_str.split('*')[0]
                        snr = float(snr_str) if snr_str else 0.0

                        gps_data['satellites_detailed'][sat_id] = {
                            'elevation': elevation,
                            'azimuth': azimuth,
                            'snr': snr  # NEO-6M SNR used as C/N0 approximation
                        }
                    except (ValueError, IndexError):
                        continue  # Skip invalid satellite data

    except Exception as e:
        print("NMEA parse error:", e)


def read_gps():
    """Read and parse GPS data (non-blocking)"""
    if not gps_enabled or not gps:
        return False

    try:
        if gps.any():
            data = gps.read()
            if data:
                try:
                    text = data.decode('ascii')
                except:
                    try:
                        text = data.decode('latin-1')
                    except:
                        return False

                lines = text.split('\n')
                for line in lines:
                    sentence = line.strip()
                    if sentence.startswith('$') and len(sentence) > 10:
                        parse_nmea(sentence)
                return True
    except Exception as e:
        print("GPS read error:", e)

    return False


def get_best_satellite():
    """Get satellite with highest SNR from real NEO-6M GPGSV data"""
    if not gps_data['satellites_detailed']:
        # Fallback when no satellite data available
        return {'id': 1, 'elevation': 35.2, 'azimuth': 120.8, 'snr': 30.0}

    best_sat = None
    best_snr = -1

    for sat_id, sat_data in gps_data['satellites_detailed'].items():
        if sat_data['snr'] > best_snr:
            best_snr = sat_data['snr']
            best_sat = {
                'id': int(sat_id) if sat_id.isdigit() else 1,
                'elevation': sat_data['elevation'],
                'azimuth': sat_data['azimuth'],
                'snr': sat_data['snr']
            }

    return best_sat if best_sat else {'id': 1, 'elevation': 35.2, 'azimuth': 120.8, 'snr': 30.0}


def display_gps_data():
    print("Time:", gps_data['utc_time'], "Date:", gps_data['date'])
    print("Lat:", gps_data['latitude'], "Lon:", gps_data['longitude'])
    print("Alt:", gps_data['altitude'], "Speed:", gps_data['speed'])
    print("Fix:", gps_data['fix_status'], "Sats:", gps_data['satellites'])
    if gps_data['satellites_detailed']:
        best_sat = get_best_satellite()
        print("Best satellite: ID={}, SNR={}, Elev={}, Azim={}".format(
            best_sat['id'], best_sat['snr'], best_sat['elevation'], best_sat['azimuth']))


# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
mpu_lock = _thread.allocate_lock()

# Store latest MPU data
mpu_latest = {
    "accel": (0.0, 0.0, 0.0),
    "gyro": (0.0, 0.0, 0.0),
    "temp": 0.0
}


def mpu_write(reg, data):
    with mpu_lock:
        i2c_bus.writeto_mem(MPU_ADDR, reg, bytes([data]))


def mpu_read(reg, n=1):
    with mpu_lock:
        return i2c_bus.readfrom_mem(MPU_ADDR, reg, n)


if mpu_enabled:
    mpu_write(0x6B, 0)
    print("MPU6050: Initialized")
else:
    print("MPU6050: Not detected")


def read_mpu():
    global mpu_latest
    data = mpu_read(0x3B, 14)
    vals = struct.unpack('>hhhhhhh', data)
    accel_x, accel_y, accel_z, temp_raw, gyro_x, gyro_y, gyro_z = vals
    mpu_latest = {
        "accel": (accel_x/16384.0, accel_y/16384.0, accel_z/16384.0),
        "gyro": (gyro_x/131.0, gyro_y/131.0, gyro_z/131.0),
        "temp": (temp_raw/340.0) + 36.53
    }
    return mpu_latest


# -------------------- Keypad Setup --------------------
row_pins = [13, 12, 14, 27]
col_pins = [26, 25, 23, 32]
rows = [Pin(pin, Pin.OUT) for pin in row_pins]
cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]
key_map = [['1', '2', '3', 'A'], ['4', '5', '6', 'B'],
           ['7', '8', '9', 'C'], ['*', '0', '#', 'D']]
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


# -------------------- Global Variables --------------------
current_status = "STANDBY"
status_lock = _thread.allocate_lock()


def set_status(new_status):
    global current_status
    with status_lock:
        current_status = new_status


def get_status():
    with status_lock:
        return current_status

# -------------------- API POST Function --------------------


def send_sensor_data():
    """Send sensor data to API endpoint"""
    try:
        # Get current sensor readings
        ax, ay, az = mpu_latest["accel"]
        gx, gy, gz = mpu_latest["gyro"]

        # Get real satellite data from NEO-6M GPGSV sentences
        best_sat = get_best_satellite()

        # Prepare JSON payload with real NEO-6M satellite data
        payload = {
            "fleet_id": FLEET_ID,
            "device_id": DEVICE_ID,
            # Real NEO-6M SNR (Signal-to-Noise Ratio) used as C/N0 approximation
            "Cn0DbHz": best_sat['snr'],
            "Svid": best_sat['id'],  # Real satellite ID (PRN number)
            # Real satellite geometry from NEO-6M GPGSV
            "SvElevationDegrees": best_sat['elevation'],
            "SvAzimuthDegrees": best_sat['azimuth'],
            "IMU_MessageType": "UncalAccel",
            "MeasurementX": round(ax, 7),
            "MeasurementY": round(ay, 7),
            "MeasurementZ": round(az, 7),
            "BiasX": 0.0,
            "BiasY": 0.0,
            "BiasZ": 0.0,
            "raw_latitude": gps_data['raw_latitude'],
            "raw_longitude": gps_data['raw_longitude'],
            "raw_altitude": gps_data['raw_altitude'],
            "speed": gps_data['raw_speed']
        }

        # Convert to JSON
        json_data = ujson.dumps(payload)

        print("Sending data to API...")
        print("Payload:", json_data)

        # Send POST request
        headers = {'Content-Type': 'application/json'}
        response = urequests.post(
            API_ENDPOINT, data=json_data, headers=headers)

        print("Response status:", response.status_code)
        print("Response:", response.text)

        response.close()
        return True

    except Exception as e:
        print("API POST error:", e)
        return False

# -------------------- Vehicle Status/Help API (Keypad-driven) --------------------


def post_vehicle_status(key):
    """POST /vehicles/status/device/{device_id} with body {"key": key}"""
    try:
        # Encode device_id for URL path (handle spaces)
        device_id_path = DEVICE_ID.replace(' ', '%20')
        url = "https://ridealert-backend.onrender.com/vehicles/status/device/" + device_id_path
        headers = {'Content-Type': 'application/json'}
        body = ujson.dumps({"key": key})
        resp = urequests.post(url, data=body, headers=headers)
        print("Status POST:", resp.status_code)
        try:
            print("Status Response:", resp.text)
        except Exception:
            pass
        resp.close()
        return True
    except Exception as e:
        print("Status POST error:", e)
        return False


def post_help_request(message=""):
    """POST /vehicles/help-request/device/{device_id} with body {"message": message}"""
    try:
        device_id_path = DEVICE_ID.replace(' ', '%20')
        url = "https://ridealert-backend.onrender.com/vehicles/help-request/device/" + device_id_path
        headers = {'Content-Type': 'application/json'}
        body = ujson.dumps({"message": message})
        resp = urequests.post(url, data=body, headers=headers)
        print("Help POST:", resp.status_code)
        try:
            print("Help Response:", resp.text)
        except Exception:
            pass
        resp.close()
        return True
    except Exception as e:
        print("Help POST error:", e)
        return False


def post_bound_for(key):
    """POST /vehicles/bound-for/device/{device_id} with body {"key": key} (6=IGPIT, 7=BUGO)"""
    try:
        device_id_path = DEVICE_ID.replace(' ', '%20')
        url = "https://ridealert-backend.onrender.com/vehicles/bound-for/device/" + device_id_path
        headers = {'Content-Type': 'application/json'}
        body = ujson.dumps({"key": key})
        resp = urequests.post(url, data=body, headers=headers)
        print("Bound-for POST:", resp.status_code)
        try:
            print("Bound-for Response:", resp.text)
        except Exception:
            pass
        resp.close()
        return True
    except Exception as e:
        print("Bound-for POST error:", e)
        return False


def post_iot_key(key, message=None):
    """POST unified keypad endpoint /vehicles/iot/device/{device_id} with body {"key": <key>[, "message": str]}"""
    try:
        device_id_path = DEVICE_ID.replace(' ', '%20')
        url = "https://ridealert-backend.onrender.com/vehicles/iot/device/" + device_id_path
        headers = {'Content-Type': 'application/json'}

        # Convert numeric keys to integers where appropriate per backend guidance
        key_value = key
        if key in ('1', '2', '4', '5', '6', '7'):
            try:
                key_value = int(key)
            except Exception:
                key_value = key  # fallback to string

        body_obj = {"key": key_value}
        if message is not None and message != "":
            body_obj["message"] = message

        body = ujson.dumps(body_obj)
        resp = urequests.post(url, data=body, headers=headers)
        print("IOT POST:", resp.status_code)
        try:
            print("IOT Response:", resp.text)
        except Exception:
            pass
        resp.close()
        return True
    except Exception as e:
        print("IOT POST error:", e)
        return False

# -------------------- MPU Thread Function --------------------


def mpu_thread():
    print("MPU thread started")
    while True:
        if mpu_enabled:
            try:
                mpu_data = read_mpu()
                ax, ay, az = mpu_data["accel"]
                gx, gy, gz = mpu_data["gyro"]
                temp = mpu_data["temp"]
                print("MPU | Accel: ({:.2f}, {:.2f}, {:.2f}) | Gyro: ({:.2f}, {:.2f}, {:.2f}) | Temp: {:.1f}C".format(
                    ax, ay, az, gx, gy, gz, temp))
            except Exception as e:
                print("MPU error:", e)
        time.sleep(2)


# -------------------- Initialization --------------------
print("System Running...")
show_message("Bus Online", "Initializing...")

print("Initialization complete!")
show_message("Bus Online", "Monitoring...")

# Start MPU background thread
try:
    _thread.start_new_thread(mpu_thread, ())
    print("MPU thread started")
except Exception as e:
    print("Failed to start MPU thread:", e)

# -------------------- Main Loop --------------------
loop_count = 0
last_gps_display = 0
last_api_post = 0

while True:
    loop_count += 1
    current_time = time.time()

    # Read GPS data in main loop
    if gps_enabled:
        read_gps()

        # Display GPS info every 5 seconds
        if current_time - last_gps_display >= 5:
            sats = gps_data.get('satellites', '0')
            if gps_data.get('fix_status') == "Active":
                display_gps_data()
            else:
                print("GPS: Searching... Sats:", sats)
            last_gps_display = current_time

    # Send data to API every POST_INTERVAL seconds
    if current_time - last_api_post >= POST_INTERVAL:
        if gps_data.get('fix_status') == "Active":  # Only send when GPS has fix
            send_sensor_data()
        else:
            print("Skipping API post - waiting for GPS fix")
        last_api_post = current_time

    # ---------------- Keypad input (Instant LCD + API, ignores interval) ----------------
    key = scan_keypad()
    if key:
        if key == '1':
            set_status("FULL")
            post_iot_key('1')
        elif key == '2':
            set_status("AVAILABLE")
            post_iot_key('2')
        elif key == 'A':
            set_status("STANDING")
            post_iot_key('A')
        elif key == '4':
            set_status("UNAVAILABLE")
            post_iot_key('4')
        elif key == '5':
            set_status("HELP REQUESTED")
            post_iot_key('5', "")
        elif key == 'B':
            set_status("IGPIT")
            post_iot_key('B')
        elif key == 'C':
            set_status("BUGO")
            post_iot_key('C')
        else:
            set_status("INVALID")

        # Instant LCD update
        current_status = get_status()
        show_message("STATUS:", current_status)
        print("Key:", key, "| Status:", current_status)

    # Update display periodically
    if loop_count % 10 == 0:
        current_status = get_status()
        gps_sats = gps_data.get('satellites', '0')
        gps_fix = gps_data.get('fix_status', 'No Fix')
        show_message("Bus Online",
                     "Status: " + current_status[:11],
                     "GPS: " + gps_fix[:8] + " S:" + gps_sats)

    time.sleep(0.1)