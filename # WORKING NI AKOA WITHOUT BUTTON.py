# WORKING NI AKOA WITHOUT BUTTON

from machine import UART, Pin, I2C
import time, struct, utime, _thread
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
import json

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

# -------------------- GPS Setup --------------------
try:
    gps = UART(1, baudrate=9600, tx=17, rx=16)
    gps_enabled = True
    print("GPS: Initialized")
except:
    gps = None
    gps_enabled = False
    print("GPS: Not detected")

# Updated GPS data structure from working code
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

gps_lock = _thread.allocate_lock()

def safe_convert(value, converter, default):
    try:
        return converter(value) if value else default
    except:
        return default

def convert_coord(coord, direction):
    try:
        val = float(coord)
        degrees = int(val/100)
        minutes = val - degrees*100
        decimal = degrees + minutes/60
        if direction in ['S','W']:
            decimal = -decimal
        return f"{decimal:.6f}°"
    except:
        return "0.0°"

def get_raw_coordinates():
    """Extract raw numeric coordinates from GPS data for API"""
    try:
        if gps_data['latitude'] and gps_data['longitude'] and gps_data['altitude']:
            # Remove degree symbols and convert to float
            raw_lat = float(gps_data['latitude'].replace('°', ''))
            raw_lon = float(gps_data['longitude'].replace('°', ''))
            raw_alt = float(gps_data['altitude'].replace(' m', ''))
            return raw_lat, raw_lon, raw_alt
        else:
            # Return default coordinates if no GPS fix
            return 0.0, 0.0, 0.0
    except:
        return 0.0, 0.0, 0.0

def get_best_satellite():
    """Get the satellite with the highest SNR"""
    if not gps_data['satellite_info']:
        return None
    
    best_satellite = None
    highest_snr = -1
    
    for svid, info in gps_data['satellite_info'].items():
        try:
            snr_value = float(info['snr'].split()[0])
            if snr_value > highest_snr:
                highest_snr = snr_value
                best_satellite = {
                    'svid': int(svid),
                    'snr': snr_value,
                    'elevation': float(info['elevation'].replace('°', '')),
                    'azimuth': float(info['azimuth'].replace('°', ''))
                }
        except:
            continue
    
    return best_satellite

# Updated parse_nmea function from working code
def parse_nmea(sentence):
    if not sentence.startswith('$'): return
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
        gps_data['altitude'] = str(round(safe_convert(parts[9], float, 0.0),1)) + " m"
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
                if svid and svid.strip():  # Make sure svid is not empty
                    try:
                        # Clean up SNR field
                        snr_raw = parts[idx+3].split('*')[0] if '*' in parts[idx+3] else parts[idx+3]
                        snr_clean = ''.join(c for c in snr_raw if c.isdigit() or c == '.')
                        snr_value = safe_convert(snr_clean, int, 0)
                        
                        gps_data['satellite_info'][svid] = {
                            'elevation': f"{safe_convert(parts[idx+1], int, 0)}°",
                            'azimuth': f"{safe_convert(parts[idx+2], int, 0)}°",
                            'snr': f"{snr_value} dBHz"
                        }
                    except (IndexError, ValueError) as e:
                        print(f"Error parsing satellite {svid}: {e}")
                        continue

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
    with gps_lock:
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
            for svid, info in sorted(gps_data['satellite_info'].items(), key=lambda x: safe_convert(x[0], int, 0)):
                try:
                    snr_value = safe_convert(info['snr'].split()[0], int, 0)
                    if snr_value > 0:
                        print(f"{svid:>3}  {info['elevation']:>8}  {info['azimuth']:>7}  {info['snr']}")
                except (ValueError, IndexError, AttributeError) as e:
                    print(f"{svid:>3}  Invalid satellite data: {e}")
                    continue

        print(f"\n[Last NMEA Sentences]")
        for sentence in gps_data['raw_sentences'][-3:]:
            print(sentence)

# -------------------- GSM Setup --------------------
try:
    gsm = UART(2, baudrate=9600, tx=2, rx=0)
    gsm_enabled = True
    print("GSM: Initialized")
except:
    gsm = None
    gsm_enabled = False
    print("GSM: Not detected")

# Thread-safe GSM lock
gsm_lock = _thread.allocate_lock()

def send_gsm_command(cmd, wait_ms=500):
    if not gsm_enabled: return "GSM not initialized"
    with gsm_lock:
        gsm.write((cmd+"\r\n").encode())
        utime.sleep_ms(wait_ms)
        resp = gsm.read()
        if resp:
            try: return resp.decode('utf-8').strip()
            except:
                try: return resp.decode('ascii').strip()
                except: return str(resp)
        return ""

def gsm_status():
    if not gsm_enabled: return
    for cmd in ["AT","AT+CPIN?","AT+CREG?","AT+CSQ","AT+CGATT?"]:
        resp = send_gsm_command(cmd, wait_ms=500)
        print(cmd, "->", resp if resp else "No response")

# -------------------- GPRS and HTTP --------------------
def gsm_gprs_connect(apn, user="", pwd=""):
    if not gsm_enabled: return False
    print("Setting APN:", apn)
    send_gsm_command("AT+CGATT=1", wait_ms=1000)
    send_gsm_command('AT+CSTT="{}","{}","{}"'.format(apn,user,pwd), wait_ms=1000)
    send_gsm_command("AT+CIICR", wait_ms=4000)
    ip = send_gsm_command("AT+CIFSR", wait_ms=2000)
    print("GPRS IP ->", ip)
    
    # -------------------- SAPBR Setup --------------------
    send_gsm_command('AT+SAPBR=3,1,"Contype","GPRS"', wait_ms=1000)
    send_gsm_command('AT+SAPBR=3,1,"APN","{}"'.format(apn), wait_ms=1000)
    send_gsm_command('AT+SAPBR=1,1', wait_ms=3000)  # open bearer
    status = send_gsm_command('AT+SAPBR=2,1', wait_ms=1000)
    print("SAPBR Status:", status)
    
    return True if ip else False

def gsm_http_post(url, json_data, use_ssl=False):
    if not gsm_enabled: 
        return False
    print("=" * 50)
    print("HTTP POST REQUEST STARTED")
    print("URL:", url)
    print("SSL:", "Enabled" if use_ssl else "Disabled")
    print("Data Length:", len(json_data), "bytes")
    print("=" * 50)

    # Cleanup before new request
    cleanup_resp = send_gsm_command("AT+HTTPTERM", wait_ms=1000)
    print("Cleanup response:", cleanup_resp)

    # Init HTTP service
    init_resp = send_gsm_command("AT+HTTPINIT", wait_ms=2000)
    print("HTTP Init response:", init_resp)
    
    if "ERROR" in init_resp:
        print("HTTP initialization failed, trying again...")
        send_gsm_command("AT+HTTPTERM", wait_ms=1000)
        utime.sleep(2)
        init_resp = send_gsm_command("AT+HTTPINIT", wait_ms=3000)
        print("HTTP Init retry response:", init_resp)
    
    cid_resp = send_gsm_command('AT+HTTPPARA="CID",1', wait_ms=1000)
    print("CID setup response:", cid_resp)

    # Enable/Disable SSL
    if use_ssl:
        ssl_resp = send_gsm_command('AT+HTTPSSL=1', wait_ms=1000)
        print("SSL enable response:", ssl_resp)
    else:
        ssl_resp = send_gsm_command('AT+HTTPSSL=0', wait_ms=1000)
        print("SSL disable response:", ssl_resp)

    # Set URL
    url_resp = send_gsm_command(f'AT+HTTPPARA="URL","{url}"', wait_ms=1000)
    print("URL setup response:", url_resp)

    # Set content type for JSON
    content_resp = send_gsm_command('AT+HTTPPARA="CONTENT","application/json"', wait_ms=1000)
    print("Content-type response:", content_resp)

    # Set data to be posted
    print(f"\nSetting up POST data ({len(json_data)} bytes)...")
    data_cmd = f'AT+HTTPDATA={len(json_data)},10000'
    data_setup_resp = send_gsm_command(data_cmd, wait_ms=1000)
    print("Data setup response:", data_setup_resp)

    # Send the actual JSON data
    if "DOWNLOAD" in data_setup_resp:
        print("Sending JSON payload...")
        print("JSON Data preview:", json_data[:200] + "..." if len(json_data) > 200 else json_data)
        with gsm_lock:
            gsm.write(json_data.encode())
            utime.sleep_ms(3000)  # Wait for data to be sent
            data_resp = gsm.read()
            if data_resp:
                try:
                    data_response = data_resp.decode('utf-8').strip()
                    print("Data send response:", data_response)
                except:
                    print("Data send response (raw):", data_resp)

    # Perform POST (action=1 for POST)
    print("\nPerforming HTTP POST request...")
    resp = send_gsm_command("AT+HTTPACTION=1", wait_ms=20000)  # Longer timeout for POST
    print("HTTPACTION Raw Response:", resp)

    success = False
    if "+HTTPACTION:" in resp:
        try:
            parts = resp.split("+HTTPACTION: 1,")[1].split(",")
            status_code = parts[0]
            data_length = parts[1] if len(parts) > 1 else "0"
            print("\n" + "=" * 30)
            print("HTTP POST RESPONSE DETAILS:")
            print("Status Code:", status_code)
            print("Data Length:", data_length, "bytes")
            print("=" * 30)
            
            if status_code == "200":
                success = True
                print("\nReading response data...")
                read_resp = send_gsm_command("AT+HTTPREAD", wait_ms=5000)
                print("\nHTTP RESPONSE BODY:")
                print("-" * 40)
                
                # Extract actual response content
                if "+HTTPREAD:" in read_resp:
                    try:
                        response_parts = read_resp.split("+HTTPREAD:")
                        if len(response_parts) > 1:
                            actual_response = response_parts[1].split('\n', 1)
                            if len(actual_response) > 1:
                                clean_response = actual_response[1].strip()
                                print("ACTUAL RESPONSE DATA:")
                                print(clean_response)
                            else:
                                print("Full response after +HTTPREAD:")
                                print(response_parts[1])
                        else:
                            print("Raw HTTPREAD response:")
                            print(read_resp)
                    except Exception as parse_error:
                        print("Error parsing response data:", parse_error)
                        print("Raw HTTPREAD response:")
                        print(read_resp)
                else:
                    print("Raw HTTPREAD response (no +HTTPREAD marker found):")
                    print(read_resp)
                print("-" * 40)
            else:
                print("\nHTTP POST request failed!")
                print("Status code:", status_code)
                if status_code == "601": 
                    print("Error 601: Network error - check GPRS/SIM/APN settings")
                elif status_code == "603":
                    print("Error 603: SSL handshake failed - trying without SSL next time")
                elif status_code == "404":
                    print("Error 404: URL not found")
                elif status_code == "500":
                    print("Error 500: Internal server error")
                else:
                    print("Unknown error code. Check network connectivity.")
        except Exception as e:
            print("Error parsing HTTPACTION response:", e)
            print("Raw response was:", resp)
    else:
        print("No +HTTPACTION response received!")
        print("Raw response:", resp)

    # Terminate HTTP
    term_resp = send_gsm_command("AT+HTTPTERM", wait_ms=1000)
    print("HTTP terminate response:", term_resp)
    print("=" * 50)
    print("HTTP POST REQUEST COMPLETED")
    print("=" * 50)
    
    return success

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
mpu_lock = _thread.allocate_lock()

# Global variable to store latest MPU data
latest_mpu_data = None
mpu_data_lock = _thread.allocate_lock()

def mpu_write(reg,data): 
    with mpu_lock:
        i2c_bus.writeto_mem(MPU_ADDR, reg, bytes([data]))
        
def mpu_read(reg,n=1): 
    with mpu_lock:
        return i2c_bus.readfrom_mem(MPU_ADDR, reg, n)

if mpu_enabled:
    mpu_write(0x6B,0)
    print("MPU6050: Initialized")
else: print("MPU6050: Not detected")

def read_mpu():
    global latest_mpu_data
    try:
        data = mpu_read(0x3B,14)
        vals = struct.unpack('>hhhhhhh',data)
        accel_x,accel_y,accel_z,temp_raw,gyro_x,gyro_y,gyro_z = vals
        
        mpu_result = {
            "accel": (accel_x/16384.0, accel_y/16384.0, accel_z/16384.0),
            "gyro": (gyro_x/131.0, gyro_y/131.0, gyro_z/131.0),
            "temp": (temp_raw/340.0)+36.53
        }
        
        # Update global latest data
        with mpu_data_lock:
            latest_mpu_data = mpu_result
            
        return mpu_result
    except Exception as e:
        print(f"MPU read error: {e}")
        return None

def get_latest_mpu_data():
    """Get the most recent MPU data in a thread-safe way"""
    with mpu_data_lock:
        return latest_mpu_data

# -------------------- Keypad Setup --------------------
row_pins = [13,12,14,27]; col_pins = [26,25,23,32]
rows = [Pin(pin, Pin.OUT) for pin in row_pins]
cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]
key_map = [['1','2','3','A'],['4','5','6','B'],['7','8','9','C'],['*','0','#','D']]
last_key=None; last_time=0

def scan_keypad():
    global last_key, last_time
    current_time = time.ticks_ms()
    
    for i, row in enumerate(rows):
        row.value(1)
        time.sleep_ms(1)  # Small delay for signal to stabilize
        
        for j, col in enumerate(cols):
            if col.value() == 1:
                key = key_map[i][j]
                
                # Debouncing logic
                if key != last_key or (current_time - last_time) > 500:  # Increased debounce time
                    last_key = key
                    last_time = current_time
                    
                    # Wait for key release to prevent multiple triggers
                    while col.value() == 1:
                        time.sleep_ms(10)
                    
                    row.value(0)  # Reset row
                    return key
        
        row.value(0)  # Reset row after checking all columns
    
    # Reset last_key if no key is pressed for a while (key release detection)
    if (current_time - last_time) > 100:
        last_key = None
    
    return None

# -------------------- Global Variables --------------------
current_status = "STANDBY"
gsm_initialized = False
status_lock = _thread.allocate_lock()
http_request_active = False
http_lock = _thread.allocate_lock()

def set_status(new_status):
    global current_status
    with status_lock:
        current_status = new_status

def get_status():
    with status_lock:
        return current_status

def set_http_active(active):
    global http_request_active
    with http_lock:
        http_request_active = active

def is_http_active():
    with http_lock:
        return http_request_active

# -------------------- Data Collection Function --------------------
def create_dynamic_payload():
    """Create payload using real GPS and IMU data"""
    
    # Get current GPS coordinates
    raw_lat, raw_lon, raw_alt = get_raw_coordinates()
    
    # Get best satellite data
    best_satellite = get_best_satellite()
    if not best_satellite:
        # Use default values if no satellite data
        best_satellite = {
            'svid': 1,
            'snr': 0.0,
            'elevation': 0.0,
            'azimuth': 0.0
        }
    
    # Get current IMU data
    current_mpu_data = get_latest_mpu_data()
    if not current_mpu_data:
        # Use default values if no IMU data
        measurement_x, measurement_y, measurement_z = 0.0, 0.0, 0.0
    else:
        measurement_x, measurement_y, measurement_z = current_mpu_data["accel"]
    
    # Create payload with real data
    payload = {
        "fleet_id": "68b3e4d19f1c8d7ccdb6c991",
        "device_id": "ESP32_DYNAMIC_001",
        "Cn0DbHz": best_satellite['snr'],
        "Svid": best_satellite['svid'],
        "SvElevationDegrees": best_satellite['elevation'],
        "SvAzimuthDegrees": best_satellite['azimuth'],
        "IMU_MessageType": "UncalAccel", 
        "MeasurementX": measurement_x,
        "MeasurementY": measurement_y,
        "MeasurementZ": measurement_z,
        "BiasX": 0.0,  # Assuming no bias correction for now
        "BiasY": 0.0,
        "BiasZ": 0.0,
        "raw_latitude": raw_lat,
        "raw_longitude": raw_lon,
        "raw_altitude": raw_alt,
        "gps_fix_status": gps_data.get('fix_status', 'No Fix'),
        "satellites_count": int(gps_data.get('satellites', '0')) if gps_data.get('satellites', '0').isdigit() else 0,
        "timestamp": utime.time()
    }
    
    return payload

# -------------------- HTTP Thread Function --------------------
def http_thread():
    print("HTTP thread started")
    
    while True:
        time.sleep(30)  # Wait 30 seconds between requests
        if gsm_initialized:
            print("Starting HTTP POST request with dynamic data...")
            set_http_active(True)
            try:
                # Create payload with current sensor data
                payload = create_dynamic_payload()
                json_data = json.dumps(payload)
                
                print("Dynamic payload created:")
                print(f"- GPS: {payload['raw_latitude']:.6f}, {payload['raw_longitude']:.6f}, {payload['raw_altitude']:.1f}m")
                print(f"- Satellite: {payload['Svid']} (SNR: {payload['Cn0DbHz']:.1f} dBHz)")
                print(f"- IMU: X={payload['MeasurementX']:.3f}, Y={payload['MeasurementY']:.3f}, Z={payload['MeasurementZ']:.3f}")
                print(f"- GPS Status: {payload['gps_fix_status']}, Satellites: {payload['satellites_count']}")
                
                # Send POST request
                success = gsm_http_post("http://60ee3ba2f17650722b29736f77460b16.serveo.net/predict", 
                                      json_data, use_ssl=False)
                
                if success:
                    print("✓ Dynamic POST request successful!")
                else:
                    print("✗ Dynamic POST request failed!")
                
            except Exception as e:
                print("HTTP POST thread error:", e)
            finally:
                set_http_active(False)
            print("HTTP request completed")
        else:
            print("GSM not initialized, skipping HTTP request")

# -------------------- GPS Thread Function --------------------
def gps_thread():
    global gps
    print("GPS thread started")
    buffer = ""
    last_display_time = time.time()
    display_interval = 5
    read_count = 0

    while True:
        try:
            if gps_enabled and gps:
                try:
                    data = gps.read()
                    read_count += 1
                    
                    if data:
                        print(f"GPS: Received {len(data)} bytes (read #{read_count})")
                        try:
                            text = str(data, 'ascii')
                            buffer += text
                            
                            # Process complete sentences
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                sentence = line.strip()
                                
                                if sentence.startswith('$') and len(sentence) > 10:
                                    print(f"GPS: Parsing: {sentence[:50]}...")
                                    try:
                                        with gps_lock:
                                            parse_nmea(sentence)
                                    except Exception as parse_error:
                                        print(f"GPS parse error: {parse_error}")
                        except:
                            buffer += str(data)
                    
                    # Display GPS data periodically
                    current_time = time.time()
                    if current_time - last_display_time >= display_interval:
                        try:
                            with gps_lock:
                                if gps_data.get('fix_status') == "Active":
                                    display_gps_data()
                                    # Show current coordinates for debugging
                                    raw_lat, raw_lon, raw_alt = get_raw_coordinates()
                                    print(f"Raw coordinates: {raw_lat:.6f}, {raw_lon:.6f}, {raw_alt:.1f}m")
                                else:
                                    print("GPS: Searching for satellites...")
                            last_display_time = current_time
                        except Exception as display_error:
                            print(f"GPS display error: {display_error}")
                            
                    time.sleep(0.1)
                        
                except Exception as read_error:
                    print(f"GPS read error: {read_error}")
                    time.sleep(0.5)

            else:
                print("GPS not enabled or not available, thread sleeping...")
                time.sleep(10)

        except Exception as e:
            print(f"GPS thread critical error: {e}")
            time.sleep(2)
            
            # Try to reinitialize GPS if needed
            if gps_enabled and not gps:
                try:
                    gps = UART(1, baudrate=9600, tx=17, rx=16)
                    print("GPS: Reinitialized after error")
                except Exception as reinit_err:
                    print("GPS: Reinitialize failed:", reinit_err)
                    time.sleep(5)

# -------------------- MPU Thread Function --------------------
def mpu_thread():
    print("MPU thread started")
    while True:
        if mpu_enabled:
            try:
                mpu_data = read_mpu()
                if mpu_data:
                    print(f"MPU - Accel: [{mpu_data['accel'][0]:.3f}, {mpu_data['accel'][1]:.3f}, {mpu_data['accel'][2]:.3f}], " +
                          f"Gyro: [{mpu_data['gyro'][0]:.2f}, {mpu_data['gyro'][1]:.2f}, {mpu_data['gyro'][2]:.2f}], " +
                          f"Temp: {mpu_data['temp']:.2f}C")
                else:
                    print("MPU: Failed to read data")
            except Exception as e:
                print("MPU thread error:", e)
        else:
            print("MPU: Not detected, thread sleeping...")
            time.sleep(10)
        time.sleep(2)  # Read MPU data every 2 seconds

# -------------------- LCD Display Thread --------------------
def lcd_display_thread():
    """Thread dedicated to updating the LCD display"""
    print("LCD display thread started")
    display_mode = "status"  # Default display mode
    last_update = 0
    update_interval = 2  # Update every 2 seconds
    
    while True:
        current_time = time.time()
        
        # Only update at the specified interval
        if current_time - last_update >= update_interval:
            if display_mode == "status":
                # Show status information
                current_status = get_status()
                http_indicator = " [HTTP]" if is_http_active() else ""
                gsm_status_text = "Connected" if gsm_initialized else "Not Connected"
                
                # Get current sensor status for display
                with gps_lock:
                    gps_status_short = "GPS:" + ("OK" if gps_data.get('fix_status') == "Active" else "NO")
                    sat_count = gps_data.get('satellites', '0')
                
                current_mpu = get_latest_mpu_data()
                imu_status_short = "IMU:" + ("OK" if current_mpu else "NO")
                
                show_message("Bus Online", 
                            f"St: {current_status[:10]}{http_indicator}", 
                            f"{gps_status_short} {imu_status_short}",
                            f"Sats:{sat_count}")
                
            elif display_mode == "sensors":
                # Show detailed sensor information
                with gps_lock:
                    raw_lat, raw_lon, raw_alt = get_raw_coordinates()
                    best_sat = get_best_satellite()
                    sat_info = f"Sat:{best_sat['svid'] if best_sat else 'None'}"
                
                current_mpu = get_latest_mpu_data()
                mpu_info = f"IMU:{'OK' if current_mpu else 'NO'}"
                
                show_message("SENSOR STATUS", 
                            f"GPS: {raw_lat:.4f},{raw_lon:.4f}",
                            sat_info + " " + mpu_info,
                            f"Alt: {raw_alt:.1f}m")
            
            last_update = current_time
        
        # Check for keypress to change display mode
        key = scan_keypad()
        if key == '#':
            display_mode = "sensors" if display_mode == "status" else "status"
            print(f"Display mode changed to: {display_mode}")
            last_update = 0  # Force immediate update
        
        time.sleep(0.1)  # Short sleep to prevent CPU hogging

# -------------------- Initialization --------------------
print("System Running...")
show_message("Bus Online","Initializing...")

if gsm_enabled:
    gsm_status()
    if gsm_gprs_connect("internet","",""):
        gsm_initialized=True
        print("GPRS Connected.")
    else:
        print("GPRS connection failed")

print("Initialization complete!")
show_message("Bus Online","Monitoring...")

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

try:
    _thread.start_new_thread(lcd_display_thread, ())
    print("LCD display thread started successfully")
except Exception as e:
    print("Failed to start LCD display thread:", e)

# -------------------- Main Loop (UI Thread) --------------------
loop_count = 0

while True:
    loop_count += 1

    # Keypad input (runs in main thread for immediate response)
    key = scan_keypad()
    if key:
        if key == '1': set_status("FULL")
        elif key == '2': set_status("AVAILABLE")
        elif key == 'A': set_status("STANDING")
        elif key == '4': set_status("INACTIVE")
        elif key == '5': set_status("HELP REQUESTED")
        elif key == '*':
            # Manual trigger HTTP POST with current data
            if gsm_initialized and not is_http_active():
                print("Manual HTTP POST triggered...")
                set_http_active(True)
                try:
                    payload = create_dynamic_payload()
                    json_data = json.dumps(payload)
                    success = gsm_http_post("http://60ee3ba2f17650722b29736f77460b16.serveo.net/predict", 
                                          json_data, use_ssl=False)
                    show_message("MANUAL POST", 
                                "Success" if success else "Failed",
                                f"GPS: {payload['gps_fix_status']}",
                                f"Sats: {payload['satellites_count']}")
                    # Keep the message displayed for 3 seconds
                    time.sleep(3)
                except Exception as e:
                    print(f"Manual POST error: {e}")
                    show_message("MANUAL POST", "ERROR", str(e)[:20], "")
                    time.sleep(3)
                finally:
                    set_http_active(False)
            else:
                show_message("MANUAL POST", "BUSY or NO GSM", "", "")
                time.sleep(2)
        else: 
            set_status("INVALID")
        
        current_status = get_status()
        
        # Show status change confirmation
        if key not in ['#', '*']:  # Don't override special displays
            show_message("STATUS:", current_status)
            print("Key pressed:", key, "| Bus Status:", current_status)
            time.sleep(1)  # Show the status for 1 second
    
    # Enhanced status logging
    current_status = get_status()
    with gps_lock:
        gps_fix = gps_data.get('fix_status', 'Unknown')
        sat_count = gps_data.get('satellites', '0')
    
    current_mpu = get_latest_mpu_data()
    imu_status = "OK" if current_mpu else "NO"
    
    print(f"Loop {loop_count} | Status: {current_status} | GPS: {gps_fix} | Sats: {sat_count} | IMU: {imu_status} | GSM: {'OK' if gsm_initialized else 'NO'} | HTTP: {'Active' if is_http_active() else 'Idle'}")
    
    time.sleep(0.5)  # Shorter sleep for more responsive keypad