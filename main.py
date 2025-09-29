from machine import UART, Pin, I2C
import time
import struct
import utime
import ujson

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

# Static configuration - update these with your actual values
VEHICLE_ID = "68b3ef0d3ed9beb95ace98fc"  # Updated to match your example
FLEET_ID = "68b3e4d19f1c8d7ccdb6c991"   # From your example
DEVICE_ID = "iot-device-001"             # Updated to match your example

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

        try:
            content = response.text
            print(f"Response content: {content[:100]}...")
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
        
        # Convert payload to JSON string
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
    'raw_sentences': [],
    'raw_latitude': 0.0,
    'raw_longitude': 0.0,
    'raw_altitude': 0.0,
    'raw_speed_kmh': 0.0
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
        return decimal, f"{decimal:.6f}°"
    except:
        return 0.0, "0.0°"


def parse_nmea(sentence):
    if not sentence.startswith('$'):
        return

    parts = sentence.split(',')
    sentence_type = parts[0][1:]

    if sentence_type == 'GPGGA':
        gps_data['message_type'] = 'GPGGA'
        gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(parts[1]) >= 6 else ''
        
        # Store both raw and formatted coordinates
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
        
        # Store raw altitude
        altitude_raw = safe_convert(parts[9], float, 0.0)
        gps_data['altitude'] = f"{altitude_raw:.1f} m"
        gps_data['raw_altitude'] = altitude_raw
        
        gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"

    elif sentence_type == 'GPRMC':
        gps_data['message_type'] = 'GPRMC'
        if len(parts[1]) >= 6:
            gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
        if len(parts[9]) >= 6:
            gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
        
        # Store both formatted and raw speed
        speed_knots = safe_convert(parts[7], float, 0.0)
        speed_kmh = speed_knots * 1.852
        gps_data['speed'] = f"{speed_kmh:.1f} km/h"
        gps_data['raw_speed_kmh'] = speed_kmh
        
        gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"

    gps_data['raw_sentences'].append(sentence)


def display_gps_data():
    print("GPS Info")
    print(f"UTC Time: {gps_data['utc_time']}")
    print(f"Date: {gps_data['date']}")
    print(f"Latitude: {gps_data['latitude']}")
    print(f"Longitude: {gps_data['longitude']}")
    print(f"Altitude: {gps_data['altitude']}")
    print(f"Speed: {gps_data['speed']}")
    print(f"Course: {gps_data['course']}")
    print(f"Status: {gps_data['fix_status']}")
    print(f"Satellites: {gps_data['satellites']}")


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


def create_prediction_payload(mpu_data=None):
    """Create prediction payload based on current sensor data"""
    
    # Get current MPU data if not provided
    if mpu_data is None and mpu_enabled:
        mpu_data = read_mpu()
    
    # Use default values if sensors are not available
    if not mpu_data:
        mpu_data = {"accel": (0.0, 0.0, 0.0), "gyro": (0.0, 0.0, 0.0), "temp": 0.0}
    
    # Create payload matching your backend PredictionRequest model
    payload = {
        "device_id": DEVICE_ID,
        "fleet_id": FLEET_ID,
        "Cn0DbHz": 45.5,  # Static value - replace with actual GPS signal strength if available
        "Svid": int(safe_convert(gps_data.get('satellites', '0'), int, 12)),  # Use actual satellite count
        "SvElevationDegrees": 35.2,  # Static value - replace with actual satellite elevation
        "SvAzimuthDegrees": 120.8,   # Static value - replace with actual satellite azimuth
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


# Main Loop
print("\nSystem Running...")

if HTTP_AVAILABLE and WIFI_AVAILABLE:
    print("Initializing WiFi for health checks...")
    wifi_connected = connect_wifi()
else:
    wifi_connected = False
    print("WiFi not available - health checks disabled")

while True:
    # GPS
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

    # Health check
    print("--- Health Check ---")
    health_status = ping_health_endpoint()
    if health_status:
        print("✓ Backend health: OK")
    else:
        print("✗ Backend health: FAILED")
    print("--- End Health Check ---")
    
    # Send prediction data
    print("--- Sending Prediction Data ---")
    try:
        prediction_payload = create_prediction_payload(mpu_data)
        prediction_success = send_prediction_data(prediction_payload)
        if prediction_success:
            print("✓ Prediction data sent successfully")
        else:
            print("✗ Failed to send prediction data")
    except Exception as e:
        print(f"✗ Prediction error: {e}")
    print("--- End Prediction ---")
    
    # Status summary
    print(f"Status - Health: {last_health_status}, Prediction: {last_prediction_status}")

    # Sleep
    print(f"Waiting {REFRESH_INTERVAL} seconds...")
    time.sleep(REFRESH_INTERVAL)