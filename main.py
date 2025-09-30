from machine import UART, Pin, I2C
import time, struct, utime, _thread, ujson
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

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

# -------------------- Configuration --------------------
HEALTH_URL = "https://ridealert-backend.onrender.com/health"
PREDICT_URL = "https://ridealert-backend.onrender.com/predict"
REFRESH_INTERVAL = 7.5  # seconds

VEHICLE_ID = "68b3ef0d3ed9beb95ace98fc"
FLEET_ID = "68b3e4d19f1c8d7ccdb6c991"
DEVICE_ID = "iot-device-001"

WIFI_SSID = "JFADeco_AD5C"
WIFI_PASSWORD = "1234567890"

# -------------------- LCD Setup --------------------
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(19), sda=Pin(18), freq=400000)
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

# -------------------- WiFi Functions --------------------
wifi_connected = False
last_health_status = "Unknown"
last_prediction_status = "Unknown"

def connect_wifi():
    """Connect to WiFi network"""
    global wifi_connected
    if not WIFI_AVAILABLE:
        print("WiFi not available")
        return False

    try:
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

        if wlan.isconnected():
            print("Already connected to WiFi")
            print(f"IP: {wlan.ifconfig()[0]}")
            wifi_connected = True
            return True

        print(f"Connecting to WiFi: {WIFI_SSID}")
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)

        timeout = 10
        while timeout > 0:
            if wlan.isconnected():
                print(f"WiFi connected! IP: {wlan.ifconfig()[0]}")
                wifi_connected = True
                return True
            time.sleep(1)
            timeout -= 1

        print("WiFi connection failed")
        wifi_connected = False
        return False

    except Exception as e:
        print(f"WiFi connection error: {e}")
        wifi_connected = False
        return False

def ping_health_endpoint():
    """Make GET request to health endpoint"""
    global last_health_status

    if not HTTP_AVAILABLE:
        last_health_status = "No HTTP"
        return False

    if not connect_wifi():
        last_health_status = "No WiFi"
        return False

    try:
        print(f"Health check: {HEALTH_URL}")
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Accept': 'application/json'
        }

        response = requests.get(HEALTH_URL, headers=headers, timeout=10)
        print(f"Response status: {response.status_code}")

        if response.status_code == 200:
            print("✓ Health check OK")
            last_health_status = "OK"
            response.close()
            return True
        else:
            print(f"✗ Health check failed: {response.status_code}")
            last_health_status = f"HTTP {response.status_code}"
            response.close()
            return False

    except Exception as e:
        print(f"Health check error: {e}")
        last_health_status = "Error"
        return False

def send_prediction_data(prediction_payload):
    """Send prediction data to predict endpoint"""
    global last_prediction_status
    
    if not HTTP_AVAILABLE:
        last_prediction_status = "No HTTP"
        return False

    if not connect_wifi():
        last_prediction_status = "No WiFi"
        return False

    try:
        print(f"Sending prediction: {PREDICT_URL}")
        headers = {
            'User-Agent': 'ESP32-IoT-Bus/1.0',
            'Content-Type': 'application/json',
            'Accept': 'application/json'
        }
        
        json_data = ujson.dumps(prediction_payload)
        print(f"Payload size: {len(json_data)} bytes")
        
        response = requests.post(PREDICT_URL, data=json_data, headers=headers, timeout=15)
        print(f"Prediction status: {response.status_code}")
        
        if response.status_code == 200:
            print("✓ Prediction sent")
            last_prediction_status = "OK"
            response.close()
            return True
        else:
            print(f"✗ Prediction failed: {response.status_code}")
            last_prediction_status = f"HTTP {response.status_code}"
            response.close()
            return False
            
    except Exception as e:
        print(f"Prediction error: {e}")
        last_prediction_status = f"Error"
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
    'utc_time': '', 'date': '', 'latitude': '', 'longitude': '',
    'altitude': '', 'speed': '', 'course': '', 'fix_status': '',
    'fix_quality': '', 'satellites': '', 'raw_latitude': 0.0,
    'raw_longitude': 0.0, 'raw_altitude': 0.0, 'raw_speed_kmh': 0.0
}
gps_lock = _thread.allocate_lock()

def safe_convert(value, converter, default):
    try:
        return converter(value) if value else default
    except:
        return default

def convert_coord(coord, direction):
    try:
        if not coord or not direction:
            return 0.0, "0.0°"
        val = float(coord)
        if val == 0:
            return 0.0, "0.0°"
        degrees = int(val // 100)
        minutes = val - (degrees * 100)
        if minutes >= 60:
            return 0.0, "0.0°"
        decimal = degrees + (minutes / 60)
        if direction.upper() in ['S', 'W']:
            decimal = -decimal
        return decimal, f"{decimal:.6f}°"
    except:
        return 0.0, "0.0°"

def parse_nmea(sentence):
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
            with gps_lock:
                if parts[1] and len(parts[1]) >= 6:
                    gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
                
                if parts[2] and parts[3]:
                    raw_lat, fmt_lat = convert_coord(parts[2], parts[3])
                    gps_data['latitude'] = fmt_lat
                    gps_data['raw_latitude'] = raw_lat
                
                if parts[4] and parts[5]:
                    raw_lon, fmt_lon = convert_coord(parts[4], parts[5])
                    gps_data['longitude'] = fmt_lon
                    gps_data['raw_longitude'] = raw_lon
                
                gps_data['fix_quality'] = parts[6] if parts[6] else '0'
                gps_data['satellites'] = parts[7] if parts[7] else '0'
                
                if parts[9]:
                    alt_val = safe_convert(parts[9], float, 0.0)
                    gps_data['altitude'] = f"{alt_val:.1f} m"
                    gps_data['raw_altitude'] = alt_val
                
                gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"
        
        elif sentence_type == 'GPRMC' and len(parts) >= 12:
            with gps_lock:
                if parts[2] == 'A':
                    if parts[9] and len(parts[9]) == 6:
                        gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
                    if parts[7]:
                        speed_knots = safe_convert(parts[7], float, 0.0)
                        speed_kmh = speed_knots * 1.852
                        gps_data['speed'] = f"{speed_kmh:.1f} km/h"
                        gps_data['raw_speed_kmh'] = speed_kmh
                    if parts[8]:
                        gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"
    
    except Exception as e:
        print(f"NMEA parse error: {e}")

def display_gps_data():
    with gps_lock:
        print(f"Time: {gps_data['utc_time']} {gps_data['date']}")
        print(f"Pos: {gps_data['latitude']} {gps_data['longitude']}")
        print(f"Alt: {gps_data['altitude']} Speed: {gps_data['speed']}")
        print(f"Fix: {gps_data['fix_status']} Sats: {gps_data['satellites']}")

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
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
    return {
        "accel": (accel_x/16384.0, accel_y/16384.0, accel_z/16384.0),
        "gyro": (gyro_x/131.0, gyro_y/131.0, gyro_z/131.0),
        "temp": (temp_raw/340.0) + 36.53
    }

def create_prediction_payload(mpu_data=None):
    """Create prediction payload from sensor data"""
    if mpu_data is None and mpu_enabled:
        try:
            mpu_data = read_mpu()
        except:
            mpu_data = None
    
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
def gps_thread():
    global gps
    print("GPS thread started")
    buffer = ""
    last_display_time = time.time()
    display_interval = 5

    while True:
        try:
            if gps_enabled and gps:
                try:
                    available = False
                    try:
                        available = gps.any() if callable(getattr(gps, "any", None)) else False
                    except:
                        available = False

                    if available:
                        data = gps.read()
                        if data:
                            try:
                                text = data.decode('ascii', errors='ignore')
                            except:
                                text = str(data)

                            buffer += text
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                sentence = line.strip()
                                if sentence.startswith('$') and len(sentence) > 10:
                                    try:
                                        parse_nmea(sentence)
                                    except Exception as e:
                                        print(f"GPS parse error: {e}")

                            if len(buffer) > 1000:
                                buffer = buffer[-500:]
                    else:
                        time.sleep(0.1)

                    current_time = time.time()
                    if current_time - last_display_time >= display_interval:
                        try:
                            with gps_lock:
                                if gps_data.get('fix_status') == "Active":
                                    display_gps_data()
                                else:
                                    print("GPS: Searching...")
                            last_display_time = current_time
                        except Exception as e:
                            print(f"GPS display error: {e}")

                except Exception as e:
                    print(f"GPS read error: {e}")
                    time.sleep(0.5)
            else:
                time.sleep(10)

        except Exception as e:
            print(f"GPS thread error: {e}")
            time.sleep(2)
            if gps_enabled and not gps:
                try:
                    gps = UART(1, baudrate=9600, tx=18, rx=16)
                    print("GPS: Reinitialized")
                except Exception as e:
                    print("GPS: Reinit failed:", e)
                    time.sleep(5)

def mpu_thread():
    print("MPU thread started")
    while True:
        if mpu_enabled:
            try:
                mpu_data = read_mpu()
                print(f"MPU - Accel: {mpu_data['accel']}, Temp: {round(mpu_data['temp'],2)}°C")
            except Exception as e:
                print(f"MPU error: {e}")
        time.sleep(2)

def http_thread():
    print("HTTP thread started")
    while True:
        time.sleep(REFRESH_INTERVAL)
        
        # Health check
        print("--- Health Check ---")
        ping_health_endpoint()
        
        # Send prediction
        print("--- Prediction ---")
        try:
            mpu_data = None
            if mpu_enabled:
                try:
                    mpu_data = read_mpu()
                except:
                    pass
            
            payload = create_prediction_payload(mpu_data)
            send_prediction_data(payload)
        except Exception as e:
            print(f"Prediction error: {e}")
        
        # Update LCD
        try:
            with gps_lock:
                fix = gps_data.get('fix_status', 'Unknown')[:8]
                sats = gps_data.get('satellites', '0')
            
            show_message(
                "Bus Online",
                f"Health: {last_health_status[:12]}",
                f"Predict: {last_prediction_status[:10]}",
                f"GPS:{fix} S:{sats}"
            )
        except Exception as e:
            print(f"LCD update error: {e}")

# -------------------- Initialization --------------------
print("\nSystem Running...")
show_message("Bus Online", "Initializing...")

if HTTP_AVAILABLE and WIFI_AVAILABLE:
    print("Initializing WiFi...")
    wifi_connected = connect_wifi()
else:
    wifi_connected = False
    print("WiFi not available")

show_message("Bus Online", "Starting threads...")

# Start threads
try:
    _thread.start_new_thread(gps_thread, ())
    print("GPS thread started")
except Exception as e:
    print(f"Failed to start GPS thread: {e}")

try:
    _thread.start_new_thread(mpu_thread, ())
    print("MPU thread started")
except Exception as e:
    print(f"Failed to start MPU thread: {e}")

try:
    _thread.start_new_thread(http_thread, ())
    print("HTTP thread started")
except Exception as e:
    print(f"Failed to start HTTP thread: {e}")

show_message("Bus Online", "Monitoring...", f"WiFi: {'Yes' if wifi_connected else 'No'}", "Ready")

# -------------------- Main Loop --------------------
loop_count = 0

while True:
    loop_count += 1
    
    if loop_count % 10 == 0:
        with gps_lock:
            fix = gps_data.get('fix_status', 'Unknown')
            sats = gps_data.get('satellites', '0')
        
        print(f"Loop {loop_count} | Health: {last_health_status} | Predict: {last_prediction_status}")
        print(f"GPS: {fix} ({sats} sats) | WiFi: {'Connected' if wifi_connected else 'Disconnected'}")
    
    time.sleep(1)