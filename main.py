from machine import UART, Pin, I2C
import time
import struct
import utime
import ujson
import _thread
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# -------------------- LCD Setup --------------------
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

# -------------------- HTTP and WiFi Setup --------------------
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

# Static configuration
VEHICLE_ID = "68b3ef0d3ed9beb95ace98fc"
FLEET_ID = "68b3e4d19f1c8d7ccdb6c991"
DEVICE_ID = "iot-device-001"

# WiFi credentials
WIFI_SSID = "JFADeco_AD5C"
WIFI_PASSWORD = "1234567890"

# Status tracking
wifi_connected = False
last_health_status = "Unknown"
last_prediction_status = "Unknown"
http_lock = _thread.allocate_lock()

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

        timeout = 10
        while timeout > 0:
            if wlan.isconnected():
                print(f"WiFi connected! IP: {wlan.ifconfig()[0]}")
                show_message("WiFi Connected", f"IP: {wlan.ifconfig()[0][:16]}")
                return True
            time.sleep(1)
            timeout -= 1
            print(".", end="")

        print("\nWiFi connection failed")
        show_message("WiFi Failed", "Check credentials")
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

    if not connect_wifi():
        print("No WiFi connection - skipping health check")
        last_health_status = "No WiFi"
        return False

    try:
        print(f"Making GET request to: {HEALTH_URL}")
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Accept': 'application/json'
        }

        response = requests.get(HEALTH_URL, headers=headers, timeout=10)
        print(f"Response status: {response.status_code}")

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

def send_prediction_data(prediction_payload):
    """Send prediction data to the predict endpoint"""
    global last_prediction_status
    
    if not HTTP_AVAILABLE:
        print("HTTP not available - skipping prediction")
        last_prediction_status = "No HTTP"
        return False

    if not connect_wifi():
        print("No WiFi connection - skipping prediction")
        last_prediction_status = "No WiFi"
        return False

    try:
        print(f"Sending prediction data to: {PREDICT_URL}")
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        
        json_data = ujson.dumps(prediction_payload)
        print(f"Payload: {json_data}")
        
        response = requests.post(PREDICT_URL, data=json_data, headers=headers, timeout=15)
        print(f"Prediction response status: {response.status_code}")
        
        try:
            content = response.text
            print(f"Prediction response: {content}")
        except:
            print("Could not read prediction response")
        
        if response.status_code == 200:
            print("✓ Prediction data sent successfully")
            last_prediction_status = "OK"
            response.close()
            return True
        else:
            print(f"✗ Prediction failed with status: {response.status_code}")
            last_prediction_status = f"HTTP {response.status_code}"
            response.close()
            return False
            
    except OSError as e:
        print(f"Network error during prediction: {e}")
        last_prediction_status = "Network Error"
        return False
    except Exception as e:
        print(f"Prediction error: {e}")
        last_prediction_status = f"Error: {str(e)}"
        return False

# -------------------- GPS Setup --------------------
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
    'raw_sentences': [],
    'raw_latitude': 0.0,
    'raw_longitude': 0.0,
    'raw_altitude': 0.0,
    'raw_speed_kmh': 0.0
}
gps_lock = _thread.allocate_lock()

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
        return decimal, f"{decimal:.6f}°"
    except:
        return 0.0, "0.0°"

def parse_nmea(sentence):
    if not sentence.startswith('$'):
        return

    parts = sentence.split(',')
    sentence_type = parts[0][1:]

    if sentence_type == 'GPGGA':
        with gps_lock:
            gps_data['message_type'] = 'GPGGA'
            gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(parts[1]) >= 6 else ''
            
            if parts[2]:
                raw_lat, formatted_lat = convert_coord(parts[2], parts[3])
                gps_data['latitude'] = formatted_lat
                gps_data['raw_latitude'] = raw_lat
            
            if parts[4]:
                raw_lon, formatted_lon = convert_coord(parts[4], parts[5])
                gps_data['longitude'] = formatted_lon
                gps_data['raw_longitude'] = raw_lon
            
            gps_data['fix_quality'] = parts[6]
            gps_data['satellites'] = parts[7]
            gps_data['hdop'] = parts[8]
            
            altitude_raw = safe_convert(parts[9], float, 0.0)
            gps_data['altitude'] = f"{altitude_raw:.1f} m"
            gps_data['raw_altitude'] = altitude_raw
            
            gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"

    elif sentence_type == 'GPRMC':
        with gps_lock:
            gps_data['message_type'] = 'GPRMC'
            if len(parts[1]) >= 6:
                gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
            if len(parts[9]) >= 6:
                gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
            
            speed_knots = safe_convert(parts[7], float, 0.0)
            speed_kmh = speed_knots * 1.852
            gps_data['speed'] = f"{speed_kmh:.1f} km/h"
            gps_data['raw_speed_kmh'] = speed_kmh
            
            gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"

    with gps_lock:
        gps_data['raw_sentences'].append(sentence)

def display_gps_data():
    with gps_lock:
        print("GPS Info")
        print(f"UTC Time: {gps_data['utc_time']}")
        print(f"Date: {gps_data['date']}")
        print(f"Latitude: {gps_data['latitude']}")
        print(f"Longitude: {gps_data['longitude']}")
        print(f"Altitude: {gps_data['altitude']}")
        print(f"Speed: {gps_data['speed']}")
        print(f"Satellites: {gps_data['satellites']}")

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
devices = i2c_bus.scan()
mpu_enabled = MPU_ADDR in devices
mpu_lock = _thread.allocate_lock()

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
    data = mpu_read(0x3B, 14)
    vals = struct.unpack('>hhhhhhh', data)
    accel_x, accel_y, accel_z, temp_raw, gyro_x, gyro_y, gyro_z = vals
    return {"accel": (accel_x/16384.0, accel_y/16384.0, accel_z/16384.0),
            "gyro": (gyro_x/131.0, gyro_y/131.0, gyro_z/131.0),
            "temp": (temp_raw/340.0)+36.53}

def create_prediction_payload(mpu_data=None):
    """Create prediction payload based on current sensor data"""
    
    if mpu_data is None and mpu_enabled:
        mpu_data = read_mpu()
    
    if not mpu_data:
        mpu_data = {"accel": (0.0, 0.0, 0.0), "gyro": (0.0, 0.0, 0.0), "temp": 0.0}
    
    with gps_lock:
        payload = {
            "device_id": DEVICE_ID,
            "fleet_id": FLEET_ID,
            "Cn0DbHz": 45.5,
            "Svid": int(safe_convert(gps_data.get('satellites', '0'), int, 12)),
            "SvElevationDegrees": 35.2,
            "SvAzimuthDegrees": 120.8,
            "IMU_MessageType": "UncalAccel",
            "MeasurementX": mpu_data["accel"][0],
            "MeasurementY": mpu_data["accel"][1],
            "MeasurementZ": mpu_data["accel"][2],
            "BiasX": 0.0,
            "BiasY": 0.0,
            "BiasZ": 0.0,
            "Speed": gps_data.get('raw_speed_kmh', 0.0)
        }
    
    return payload

# -------------------- Thread Functions --------------------
def http_thread():
    """Background thread for HTTP requests"""
    print("HTTP thread started")
    
    while True:
        time.sleep(REFRESH_INTERVAL)
        
        with http_lock:
            show_message("HTTP Request", "Health check...")
            
            # Health check
            health_status = ping_health_endpoint()
            if health_status:
                print("✓ Backend health: OK")
                show_message("Health Check", "Status: OK")
            else:
                print("✗ Backend health: FAILED")
                show_message("Health Check", f"Status: {last_health_status}")
            
            time.sleep(1)
            
            # Send prediction data
            show_message("HTTP Request", "Sending data...")
            try:
                mpu_data = read_mpu() if mpu_enabled else None
                prediction_payload = create_prediction_payload(mpu_data)
                prediction_success = send_prediction_data(prediction_payload)
                
                if prediction_success:
                    print("✓ Prediction data sent successfully")
                    show_message("Prediction Sent", "Status: OK")
                else:
                    print("✗ Failed to send prediction data")
                    show_message("Prediction Failed", f"Status: {last_prediction_status}")
            except Exception as e:
                print(f"✗ Prediction error: {e}")
                show_message("Prediction Error", str(e)[:20])

def gps_thread():
    """Background thread for GPS reading"""
    print("GPS thread started")
    
    while True:
        if gps_enabled and gps and gps.any():
            try:
                sentence = gps.readline().decode('ascii').strip()
                parse_nmea(sentence)
                
                with gps_lock:
                    if gps_data['fix_status'] == "Active":
                        display_gps_data()
                    else:
                        print("GPS: Searching for satellites...")
            except Exception as e:
                print("GPS error:", e)
        else:
            time.sleep(1)

def mpu_thread():
    """Background thread for MPU6050 reading"""
    print("MPU thread started")
    
    while True:
        if mpu_enabled:
            try:
                mpu_data = read_mpu()
                print("MPU Accel:", mpu_data["accel"], 
                      "Gyro:", mpu_data["gyro"], 
                      "Temp:", round(mpu_data["temp"], 2))
            except Exception as e:
                print("MPU thread error:", e)
        time.sleep(2)

# -------------------- Initialization --------------------
print("\nSystem Running...")
show_message("Bus Online", "Initializing...")

if HTTP_AVAILABLE and WIFI_AVAILABLE:
    print("Initializing WiFi...")
    wifi_connected = connect_wifi()
else:
    wifi_connected = False
    print("WiFi not available")
    show_message("WiFi Not Available", "Check modules")

print("Initialization complete!")
show_message("Bus Online", "Monitoring...")

# Start background threads
try:
    _thread.start_new_thread(http_thread, ())
    print("HTTP thread started successfully")
except Exception as e:
    print("Failed to start HTTP thread:", e)

try:
    _thread.start_new_thread(gps_thread, ())
    print("GPS thread started successfully")
except Exception as e:
    print("Failed to start GPS thread:", e)

try:
    _thread.start_new_thread(mpu_thread, ())
    print("MPU thread started successfully")
except Exception as e:
    print("Failed to start MPU thread:", e)

# -------------------- Main Loop (Display Updates) --------------------
loop_count = 0

while True:
    loop_count += 1
    
    # Update display with system status
    if loop_count % 5 == 0:  # Every 5 seconds
        with gps_lock:
            gps_status = gps_data.get('fix_status', 'No Fix')
            satellites = gps_data.get('satellites', '0')
        
        with http_lock:
            health = last_health_status[:10]
            pred = last_prediction_status[:10]
        
        show_message(
            "Bus Monitoring",
            f"GPS: {gps_status[:14]}",
            f"Sats: {satellites} H:{health}",
            f"P:{pred} L:{loop_count}"
        )
    
    print(f"Loop {loop_count} | GPS: {gps_data.get('fix_status', 'Unknown')} | "
          f"Health: {last_health_status} | Pred: {last_prediction_status}")
    
    time.sleep(1)