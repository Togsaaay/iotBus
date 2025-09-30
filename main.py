# from machine import UART, Pin, I2C
# import time
# import struct
# import utime
# import ujson
# import _thread

# # Import LCD libraries - fix the import structure
# try:
#     from lcd_api import LcdApi
#     from i2c_lcd import I2cLcd
#     LCD_AVAILABLE = True
#     print("LCD modules loaded successfully")
# except ImportError:
#     LCD_AVAILABLE = False
#     print("LCD modules not available")

# # Import for HTTP requests and WiFi
# try:
#     import urequests as requests
#     import network
#     HTTP_AVAILABLE = True
#     WIFI_AVAILABLE = True
#     print("HTTP and WiFi modules available")
# except ImportError:
#     try:
#         import requests
#         import network
#         HTTP_AVAILABLE = True
#         WIFI_AVAILABLE = True
#         print("HTTP and WiFi modules available (standard)")
#     except ImportError:
#         HTTP_AVAILABLE = False
#         WIFI_AVAILABLE = False
#         print("HTTP/WiFi modules not available")

# # Health check and prediction configuration
# HEALTH_URL = "https://ridealert-backend.onrender.com/health"
# PREDICT_URL = "https://ridealert-backend.onrender.com/predict"
# REFRESH_INTERVAL = 7.5  # seconds

# # Static configuration
# VEHICLE_ID = "68b3ef0d3ed9beb95ace98fc"
# FLEET_ID = "68b3e4d19f1c8d7ccdb6c991"
# DEVICE_ID = "iot-device-001"

# # WiFi credentials
# WIFI_SSID = "JFADeco_AD5C"
# WIFI_PASSWORD = "1234567890"

# # Status tracking
# wifi_connected = False
# last_health_status = "Unknown"
# last_prediction_status = "Unknown"
# current_status = "AVAILABLE"

# # LCD Setup
# I2C_ADDR = 0x27
# lcd = None
# lcd_lock = _thread.allocate_lock()

# if LCD_AVAILABLE:
#     try:
#         i2c_lcd = I2C(0, scl=Pin(5), sda=Pin(19), freq=400000)
#         lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)
#         print("LCD initialized successfully")
#     except Exception as e:
#         print(f"LCD initialization failed: {e}")
#         LCD_AVAILABLE = False

# def show_message(line1="", line2="", line3="", line4=""):
#     if not LCD_AVAILABLE or lcd is None:
#         print(f"LCD (disabled): {line1} | {line2} | {line3} | {line4}")
#         return
    
#     try:
#         with lcd_lock:
#             lcd.clear()
#             if line1:
#                 lcd.move_to(0, 0)
#                 lcd.putstr(line1[:20])
#             if line2:
#                 lcd.move_to(0, 1)
#                 lcd.putstr(line2[:20])
#             if line3:
#                 lcd.move_to(0, 2)
#                 lcd.putstr(line3[:20])
#             if line4:
#                 lcd.move_to(0, 3)
#                 lcd.putstr(line4[:20])
#         print(f"LCD: {line1} | {line2} | {line3} | {line4}")
#     except Exception as e:
#         print(f"LCD Error: {e}")

# # Numpad Setup
# row_pins = [13, 12, 14, 27]
# col_pins = [26, 25, 23, 32]
# rows = [Pin(pin, Pin.OUT) for pin in row_pins]
# cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]

# key_map = [['1', '2', '3', 'A'], 
#            ['4', '5', '6', 'B'],
#            ['7', '8', '9', 'C'], 
#            ['*', '0', '#', 'D']]
# last_key = None
# last_time = 0

# def scan_keypad():
#     """Scan keypad for button presses"""
#     global last_key, last_time
    
#     for i, row in enumerate(rows):
#         row.value(1)
#         time.sleep_ms(1)
        
#         for j, col in enumerate(cols):
#             if col.value() == 1:
#                 key = key_map[i][j]
#                 current_time = utime.ticks_ms()
                
#                 # Debounce check
#                 if key != last_key or utime.ticks_diff(current_time, last_time) > 300:
#                     last_key = key
#                     last_time = current_time
#                     row.value(0)
#                     return key
        
#         row.value(0)
    
#     return None

# def update_lcd_default():
#     """Update LCD with current status"""
#     show_message(
#         "RideAlert System",
#         f"Status: {current_status}",
#         "",
#         "Press D for info"
#     )

# def keypad_thread():
#     """Thread for monitoring keypad input"""
#     global current_status, wifi_connected, last_health_status, last_prediction_status
    
#     print("Keypad thread started")
    
#     while True:
#         key = scan_keypad()
        
#         if key:
#             print(f"Key pressed: {key}")
            
#             if key == '1':
#                 current_status = "FULL"
#                 show_message("STATUS UPDATED:", "FULL", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
                
#             elif key == '2':
#                 current_status = "AVAILABLE"
#                 show_message("STATUS UPDATED:", "AVAILABLE", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
                
#             elif key == 'A':
#                 current_status = "STANDING"
#                 show_message("STATUS UPDATED:", "STANDING", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
                
#             elif key == '4':
#                 current_status = "INACTIVE"
#                 show_message("STATUS UPDATED:", "INACTIVE", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
                
#             elif key == '5':
#                 current_status = "HELP REQUESTED"
#                 show_message("STATUS UPDATED:", "HELP REQUESTED", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
                
#             elif key == 'D':
#                 # Display system status
#                 wifi_stat = "OK" if wifi_connected else "FAIL"
#                 health_stat = last_health_status[:8]
#                 predict_stat = last_prediction_status[:8]
                
#                 show_message(
#                     "SYSTEM STATUS:",
#                     f"WiFi: {wifi_stat}",
#                     f"Health: {health_stat}",
#                     f"Predict: {predict_stat}"
#                 )
#                 time.sleep(3)
#                 update_lcd_default()
                
#             else:
#                 show_message("Invalid key!", "Use: 1,2,A,4,5,D", "", "")
#                 time.sleep(2)
#                 update_lcd_default()
        
#         time.sleep_ms(50)  # Small delay to prevent CPU hogging


# def connect_wifi():
#     """Connect to WiFi network"""
#     global wifi_connected
    
#     if not WIFI_AVAILABLE:
#         print("WiFi not available")
#         wifi_connected = False
#         return False

#     try:
#         wlan = network.WLAN(network.STA_IF)
#         wlan.active(True)

#         if wlan.isconnected():
#             print("Already connected to WiFi")
#             print(f"IP: {wlan.ifconfig()[0]}")
#             wifi_connected = True
#             return True

#         print(f"Connecting to WiFi: {WIFI_SSID}")
#         wlan.connect(WIFI_SSID, WIFI_PASSWORD)

#         # Wait for connection
#         timeout = 10
#         while timeout > 0:
#             if wlan.isconnected():
#                 print(f"WiFi connected! IP: {wlan.ifconfig()[0]}")
#                 wifi_connected = True
#                 return True
#             time.sleep(1)
#             timeout -= 1
#             print(".", end="")

#         print("\nWiFi connection failed")
#         wifi_connected = False
#         return False

#     except Exception as e:
#         print(f"WiFi connection error: {e}")
#         wifi_connected = False
#         return False


# def ping_health_endpoint():
#     """Make a proper GET request to the health endpoint"""
#     global last_health_status

#     if not HTTP_AVAILABLE:
#         print("HTTP not available - skipping health check")
#         last_health_status = "No HTTP"
#         return False

#     if not connect_wifi():
#         print("No WiFi connection - skipping health check")
#         last_health_status = "No WiFi"
#         return False

#     try:
#         print(f"Making GET request to: {HEALTH_URL}")
#         headers = {
#             'User-Agent': 'ESP32-IoT-Bus/1.0',
#             'Accept': 'application/json'
#         }

#         response = requests.get(HEALTH_URL, headers=headers, timeout=10)
#         print(f"Response status: {response.status_code}")

#         try:
#             content = response.text
#             print(f"Response content: {content[:100]}...")
#         except:
#             print("Could not read response content")

#         if response.status_code == 200:
#             print("✓ Health check successful")
#             last_health_status = "OK"
#             response.close()
#             return True
#         else:
#             print(f"✗ Health check failed with status: {response.status_code}")
#             last_health_status = f"HTTP {response.status_code}"
#             response.close()
#             return False

#     except OSError as e:
#         print(f"Network error: {e}")
#         last_health_status = "Net Error"
#         return False
#     except Exception as e:
#         print(f"Health check error: {e}")
#         last_health_status = "Error"
#         return False


# def send_prediction_data(prediction_payload):
#     """Send prediction data to the predict endpoint"""
#     global last_prediction_status
    
#     if not HTTP_AVAILABLE:
#         print("HTTP not available - skipping prediction")
#         last_prediction_status = "No HTTP"
#         return False

#     if not connect_wifi():
#         print("No WiFi connection - skipping prediction")
#         last_prediction_status = "No WiFi"
#         return False

#     try:
#         print(f"Sending prediction data to: {PREDICT_URL}")
#         headers = {
#             'User-Agent': 'ESP32-IoT-Bus/1.0',
#             'Content-Type': 'application/json',
#             'Accept': 'application/json'
#         }
        
#         # Add current status to payload
#         prediction_payload['vehicle_status'] = current_status
        
#         # Convert payload to JSON string
#         json_data = ujson.dumps(prediction_payload)
#         print(f"Payload: {json_data}")
        
#         response = requests.post(PREDICT_URL, data=json_data, headers=headers, timeout=15)
#         print(f"Prediction response status: {response.status_code}")
        
#         try:
#             content = response.text
#             print(f"Prediction response: {content}")
#         except:
#             print("Could not read prediction response")
        
#         if response.status_code == 200:
#             print("✓ Prediction data sent successfully")
#             last_prediction_status = "OK"
#             response.close()
#             return True
#         else:
#             print(f"✗ Prediction failed with status: {response.status_code}")
#             last_prediction_status = f"HTTP {response.status_code}"
#             response.close()
#             return False
            
#     except OSError as e:
#         print(f"Network error during prediction: {e}")
#         last_prediction_status = "Net Error"
#         return False
#     except Exception as e:
#         print(f"Prediction error: {e}")
#         last_prediction_status = "Error"
#         return False


# # GPS
# try:
#     gps = UART(1, baudrate=9600, tx=18, rx=16)
#     gps_enabled = True
#     print("GPS: Initialized")
# except:
#     gps = None
#     gps_enabled = False
#     print("GPS: Not detected")

# gps_data = {
#     'message_type': '',
#     'utc_time': '',
#     'date': '',
#     'latitude': '',
#     'longitude': '',
#     'altitude': '',
#     'speed': '',
#     'course': '',
#     'fix_status': '',
#     'fix_quality': '',
#     'satellites': '',
#     'hdop': '',
#     'vdop': '',
#     'pdop': '',
#     'mode': '',
#     'satellite_info': {},
#     'raw_sentences': [],
#     'raw_latitude': 0.0,
#     'raw_longitude': 0.0,
#     'raw_altitude': 0.0,
#     'raw_speed_kmh': 0.0
# }


# def safe_convert(value, converter, default):
#     try:
#         return converter(value) if value else default
#     except:
#         return default


# def convert_coord(coord, direction):
#     try:
#         val = float(coord)
#         degrees = int(val / 100)
#         minutes = val - degrees * 100
#         decimal = degrees + minutes / 60
#         if direction in ['S', 'W']:
#             decimal = -decimal
#         return decimal, f"{decimal:.6f}°"
#     except:
#         return 0.0, "0.0°"


# def parse_nmea(sentence):
#     if not sentence.startswith('$'):
#         return

#     parts = sentence.split(',')
#     sentence_type = parts[0][1:]

#     if sentence_type == 'GPGGA':
#         gps_data['message_type'] = 'GPGGA'
#         gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(parts[1]) >= 6 else ''
        
#         if parts[2]:
#             raw_lat, formatted_lat = convert_coord(parts[2], parts[3])
#             gps_data['latitude'] = formatted_lat
#             gps_data['raw_latitude'] = raw_lat
        
#         if parts[4]:
#             raw_lon, formatted_lon = convert_coord(parts[4], parts[5])
#             gps_data['longitude'] = formatted_lon
#             gps_data['raw_longitude'] = raw_lon
        
#         gps_data['fix_quality'] = parts[6]
#         gps_data['satellites'] = parts[7]
#         gps_data['hdop'] = parts[8]
        
#         altitude_raw = safe_convert(parts[9], float, 0.0)
#         gps_data['altitude'] = f"{altitude_raw:.1f} m"
#         gps_data['raw_altitude'] = altitude_raw
        
#         gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"

#     elif sentence_type == 'GPRMC':
#         gps_data['message_type'] = 'GPRMC'
#         if len(parts[1]) >= 6:
#             gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
#         if len(parts[9]) >= 6:
#             gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
        
#         speed_knots = safe_convert(parts[7], float, 0.0)
#         speed_kmh = speed_knots * 1.852
#         gps_data['speed'] = f"{speed_kmh:.1f} km/h"
#         gps_data['raw_speed_kmh'] = speed_kmh
        
#         gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"

#     gps_data['raw_sentences'].append(sentence)


# def display_gps_data():
#     print("GPS Info")
#     print(f"UTC Time: {gps_data['utc_time']}")
#     print(f"Date: {gps_data['date']}")
#     print(f"Latitude: {gps_data['latitude']}")
#     print(f"Longitude: {gps_data['longitude']}")
#     print(f"Altitude: {gps_data['altitude']}")
#     print(f"Speed: {gps_data['speed']}")
#     print(f"Course: {gps_data['course']}")
#     print(f"Status: {gps_data['fix_status']}")
#     print(f"Satellites: {gps_data['satellites']}")


# # MPU
# MPU_ADDR = 0x68
# i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
# devices = i2c_bus.scan()
# mpu_enabled = MPU_ADDR in devices


# def mpu_write(reg, data):
#     i2c_bus.writeto_mem(MPU_ADDR, reg, bytes([data]))


# def mpu_read(reg, n=1):
#     return i2c_bus.readfrom_mem(MPU_ADDR, reg, n)


# if mpu_enabled:
#     mpu_write(0x6B, 0)
#     print("MPU6050: Initialized")
# else:
#     print("MPU6050: Not detected")


# def read_mpu():
#     data = mpu_read(0x3B, 14)
#     vals = struct.unpack('>hhhhhhh', data)
#     accel_x, accel_y, accel_z, temp_raw, gyro_x, gyro_y, gyro_z = vals
#     accel_x /= 16384.0
#     accel_y /= 16384.0
#     accel_z /= 16384.0
#     temp_c = (temp_raw / 340.0) + 36.53
#     gyro_x /= 131.0
#     gyro_y /= 131.0
#     gyro_z /= 131.0
#     return {"accel": (accel_x, accel_y, accel_z), "gyro": (gyro_x, gyro_y, gyro_z), "temp": temp_c}


# def create_prediction_payload(mpu_data=None):
#     """Create prediction payload based on current sensor data"""
    
#     if mpu_data is None and mpu_enabled:
#         mpu_data = read_mpu()
    
#     if not mpu_data:
#         mpu_data = {"accel": (0.0, 0.0, 0.0), "gyro": (0.0, 0.0, 0.0), "temp": 0.0}
    
#     payload = {
#         "device_id": DEVICE_ID,
#         "fleet_id": FLEET_ID,
#         "Cn0DbHz": 45.5,
#         "Svid": int(safe_convert(gps_data.get('satellites', '0'), int, 12)),
#         "SvElevationDegrees": 35.2,
#         "SvAzimuthDegrees": 120.8,
#         "IMU_MessageType": "UncalAccel",
#         "MeasurementX": mpu_data["accel"][0],
#         "MeasurementY": mpu_data["accel"][1],
#         "MeasurementZ": mpu_data["accel"][2],
#         "BiasX": 0.0,
#         "BiasY": 0.0,
#         "BiasZ": 0.0,
#         "Speed": gps_data.get('raw_speed_kmh', 0.0),
#         "vehicle_status": current_status
#     }
    
#     return payload


# # Main Loop
# print("\nSystem Running...")

# # Display startup message
# print("Displaying startup message on LCD...")
# show_message("RideAlert System", "Initializing...", "", "")
# time.sleep(3)

# # Start keypad thread
# print("Starting keypad thread...")
# _thread.start_new_thread(keypad_thread, ())
# time.sleep(1)

# if HTTP_AVAILABLE and WIFI_AVAILABLE:
#     print("Initializing WiFi for health checks...")
#     show_message("RideAlert System", "Connecting WiFi...", "", "")
#     wifi_connected = connect_wifi()
#     time.sleep(2)
# else:
#     wifi_connected = False
#     print("WiFi not available - health checks disabled")

# print("Updating LCD to default view...")
# update_lcd_default()
# time.sleep(1)

# while True:
#     # GPS
#     if gps_enabled and gps.any():
#         try:
#             sentence = gps.readline().decode('ascii').strip()
#             parse_nmea(sentence)
#             if gps_data['fix_status'] == "Active":
#                 display_gps_data()
#             else:
#                 print("Searching for satellites...", end='\r')
#         except Exception as e:
#             print("GPS error:", e)
#     else:
#         print("GPS: No data")

#     # MPU
#     mpu_data = None
#     if mpu_enabled:
#         mpu_data = read_mpu()
#         if mpu_data:
#             print("Accel (g):", mpu_data["accel"])
#             print("Gyro (°/s):", mpu_data["gyro"])
#             print("Temp (C):", round(mpu_data["temp"], 2))
#         else:
#             print("MPU: Failed to read data")
#     else:
#         print("MPU: Not detected")

#     # Health check
#     print("--- Health Check ---")
#     health_status = ping_health_endpoint()
#     if health_status:
#         print("✓ Backend health: OK")
#     else:
#         print("✗ Backend health: FAILED")
#     print("--- End Health Check ---")
    
#     # Send prediction data
#     print("--- Sending Prediction Data ---")
#     try:
#         prediction_payload = create_prediction_payload(mpu_data)
#         prediction_success = send_prediction_data(prediction_payload)
#         if prediction_success:
#             print("✓ Prediction data sent successfully")
#         else:
#             print("✗ Failed to send prediction data")
#     except Exception as e:
#         print(f"✗ Prediction error: {e}")
#     print("--- End Prediction ---")
    
#     # Status summary
#     print(f"Status - Health: {last_health_status}, Prediction: {last_prediction_status}")

#     # Sleep
#     print(f"Waiting {REFRESH_INTERVAL} seconds...")
#     time.sleep(REFRESH_INTERVAL)