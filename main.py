from machine import UART, Pin, I2C
import time, struct, utime, _thread
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

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

gps_data = {'utc_time':'','date':'','latitude':'','longitude':'',
            'altitude':'','speed':'','course':'','fix_status':'','fix_quality':'','satellites':''}
gps_lock = _thread.allocate_lock()

def safe_convert(value, converter, default):
    try:
        return converter(value) if value else default
    except:
        return default

def convert_coord(coord, direction):
    """Convert NMEA coordinate format to decimal degrees with better error handling"""
    try:
        if not coord or not direction:
            return "0.0°"
            
        val = float(coord)
        if val == 0:
            return "0.0°"
            
        degrees = int(val // 100)
        minutes = val - (degrees * 100)
        
        if minutes >= 60:  # Invalid minutes value
            return "0.0°"
            
        decimal = degrees + (minutes / 60)
        
        if direction.upper() in ['S', 'W']:
            decimal = -decimal
            
        return f"{decimal:.6f}°"
        
    except (ValueError, TypeError) as e:
        print(f"Coordinate conversion error: {e}")
        return "0.0°"
    
def parse_nmea(sentence):
    """Parse NMEA sentence with better error handling"""
    try:
        if not sentence or not sentence.startswith('$'):
            return
            
        # Remove any extra whitespace and validate checksum if present
        sentence = sentence.strip()
        if '*' in sentence:
            data_part, checksum_part = sentence.split('*')
            sentence = data_part  # Use only the data part for parsing
        
        parts = sentence.split(',')
        if len(parts) < 3:  # Minimum viable sentence length
            return
            
        sentence_type = parts[0][1:]  # Remove the '$'
        
        if sentence_type == 'GPGGA' and len(parts) >= 15:
            with gps_lock:
                # Parse time
                if parts[1] and len(parts[1]) >= 6:
                    gps_data['utc_time'] = parts[1][:6]
                else:
                    gps_data['utc_time'] = ''
                
                # Parse coordinates
                if parts[2] and parts[3]:
                    gps_data['latitude'] = convert_coord(parts[2], parts[3])
                else:
                    gps_data['latitude'] = '0.0°'
                    
                if parts[4] and parts[5]:
                    gps_data['longitude'] = convert_coord(parts[4], parts[5])
                else:
                    gps_data['longitude'] = '0.0°'
                
                # Parse fix quality and satellites
                gps_data['fix_quality'] = parts[6] if parts[6] else '0'
                gps_data['satellites'] = parts[7] if parts[7] else '0'
                
                # Parse altitude
                if parts[9]:
                    try:
                        alt_val = float(parts[9])
                        gps_data['altitude'] = f"{alt_val:.1f} m"
                    except ValueError:
                        gps_data['altitude'] = "0.0 m"
                else:
                    gps_data['altitude'] = "0.0 m"
                
                # Set fix status
                gps_data['fix_status'] = "Active" if parts[6] != '0' and parts[6] != '' else "No Fix"
                
        elif sentence_type == 'GPRMC' and len(parts) >= 12:
            # Also parse RMC sentences for date and speed
            with gps_lock:
                if parts[2] == 'A':  # Active fix
                    if parts[9] and len(parts[9]) == 6:  # DDMMYY format
                        gps_data['date'] = parts[9]
                    if parts[7]:  # Speed in knots
                        try:
                            speed_knots = float(parts[7])
                            speed_kmh = speed_knots * 1.852
                            gps_data['speed'] = f"{speed_kmh:.1f} km/h"
                        except ValueError:
                            gps_data['speed'] = "0.0 km/h"
                    if parts[8]:  # Course
                        gps_data['course'] = parts[8] + "°"
                        
    except Exception as e:
        print(f"NMEA parse error: {e}")

def display_gps_data():
    with gps_lock:
        print("Time/Date:", gps_data['utc_time'], gps_data['date'])
        print("Position:", gps_data['latitude'], gps_data['longitude'], "Alt:", gps_data['altitude'])
        print("Fix:", gps_data['fix_status'], "Quality:", gps_data['fix_quality'], "Satellites:", gps_data['satellites'])

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

def gsm_http_get(url, use_ssl=False):
    if not gsm_enabled: 
        return
    print("=" * 50)
    print("HTTP GET REQUEST STARTED")
    print("URL:", url)
    print("SSL:", "Enabled" if use_ssl else "Disabled")
    print("=" * 50)

    # Cleanup before new request
    cleanup_resp = send_gsm_command("AT+HTTPTERM", wait_ms=1000)
    print("Cleanup response:", cleanup_resp)

    # Init HTTP service
    init_resp = send_gsm_command("AT+HTTPINIT", wait_ms=2000)
    print("HTTP Init response:", init_resp)
    
    cid_resp = send_gsm_command('AT+HTTPPARA="CID",1', wait_ms=1000)
    print("CID setup response:", cid_resp)

    # Enable SSL if needed
    if use_ssl:
        ssl_resp = send_gsm_command('AT+HTTPSSL=1', wait_ms=1000)  # enable SSL
        print("SSL enable response:", ssl_resp)
    else:
        ssl_resp = send_gsm_command('AT+HTTPSSL=1', wait_ms=1000)  # disable SSL
        print("SSL disable response:", ssl_resp)

    # Set URL
    url_resp = send_gsm_command(f'AT+HTTPPARA="URL","{url}"', wait_ms=1000)
    print("URL setup response:", url_resp)

    # Optional: set content type (for GET it usually doesn't matter)
    content_resp = send_gsm_command('AT+HTTPPARA="CONTENT","application/json"', wait_ms=1000)
    print("Content-type response:", content_resp)

    # Perform GET
    print("\nPerforming HTTP GET request...")
    resp = send_gsm_command("AT+HTTPACTION=0", wait_ms=15000)
    print("HTTPACTION Raw Response:", resp)

    if "+HTTPACTION:" in resp:
        try:
            parts = resp.split("+HTTPACTION: 0,")[1].split(",")
            status_code = parts[0]
            data_length = parts[1] if len(parts) > 1 else "0"
            print("\n" + "=" * 30)
            print("HTTP RESPONSE DETAILS:")
            print("Status Code:", status_code)
            print("Data Length:", data_length, "bytes")
            print("=" * 30)
            
            if status_code == "200":
                print("\nReading response data...")
                read_resp = send_gsm_command("AT+HTTPREAD", wait_ms=5000)
                print("\nHTTP RESPONSE BODY:")
                print("-" * 40)
                
                # Extract actual response content from AT command response
                if "+HTTPREAD:" in read_resp:
                    try:
                        # Split by +HTTPREAD: to get the actual response
                        response_parts = read_resp.split("+HTTPREAD:")
                        if len(response_parts) > 1:
                            # Get everything after the length indicator
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
                print("\nHTTP request failed!")
                print("Status code:", status_code)
                if status_code == "601": 
                    print("Error 601: Network error - check GPRS/SIM/APN settings")
                elif status_code == "603":
                    print("Error 603: SSL handshake failed (certificate or TLS version issue)")
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
    print("HTTP GET REQUEST COMPLETED")
    print("=" * 50)

def gsm_http_post(url, json_data, use_ssl=False):
    if not gsm_enabled: 
        return
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
    
    cid_resp = send_gsm_command('AT+HTTPPARA="CID",1', wait_ms=1000)
    print("CID setup response:", cid_resp)

    # Enable/Disable SSL
    if use_ssl:
        ssl_resp = send_gsm_command('AT+HTTPSSL=1', wait_ms=1000)
        print("SSL enable response:", ssl_resp)
    else:
        ssl_resp = send_gsm_command('AT+HTTPSSL=1', wait_ms=1000)
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
        print("JSON Data:", json_data)
        with gsm_lock:
            gsm.write(json_data.encode())
            utime.sleep_ms(2000)  # Wait for data to be sent
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

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
mpu_lock = _thread.allocate_lock()

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
    data = mpu_read(0x3B,14)
    vals = struct.unpack('>hhhhhhh',data)
    accel_x,accel_y,accel_z,temp_raw,gyro_x,gyro_y,gyro_z = vals
    return {"accel": (accel_x/16384.0, accel_y/16384.0, accel_z/16384.0),
            "gyro": (gyro_x/131.0, gyro_y/131.0, gyro_z/131.0),
            "temp": (temp_raw/340.0)+36.53}

# -------------------- Keypad Setup --------------------
row_pins = [13,12,14,27]; col_pins = [26,25,23,32]
rows = [Pin(pin, Pin.OUT) for pin in row_pins]
cols = [Pin(pin, Pin.IN, Pin.PULL_DOWN) for pin in col_pins]
key_map = [['1','2','3','A'],['4','5','6','B'],['7','8','9','C'],['*','0','#','D']]
last_key=None; last_time=0

def scan_keypad():
    global last_key,last_time
    for i,row in enumerate(rows):
        row.value(1)
        for j,col in enumerate(cols):
            if col.value()==1:
                key=key_map[i][j]
                if key!=last_key or (time.ticks_ms()-last_time)>200:
                    last_key=key; last_time=time.ticks_ms()
                    row.value(0)
                    return key
        row.value(0)
    return None

# -------------------- Global Variables --------------------
current_status = "STANDBY"
gsm_initialized = False
status_lock = _thread.allocate_lock()
http_request_active = False
http_response = ""
http_status_code = ""
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

# -------------------- HTTP Thread Function --------------------
def http_thread():
    print("HTTP thread started")
    
    # Hardcoded test data
    test_json_data = '''{
  "fleet_id": "68b3e4d19f1c8d7ccdb6c991",
  "device_id": "USTP",
  "Cn0DbHz": 45.5,
  "Svid": 12,
  "SvElevationDegrees": 35.2,
  "SvAzimuthDegrees": 120.8,
  "IMU_MessageType": "UncalAccel", 
  "MeasurementX": 0.7854004,
  "MeasurementY": -0.6618652,
  "MeasurementZ": -0.06811523,
  "BiasX": 0.0,
  "BiasY": 0.0,
  "BiasZ": 0.0,
  "raw_latitude": 8.585581,
  "raw_longitude": 124.769386,
  "raw_altitude": 3.0
}'''
    
#     test_json_data = '''{
#     "title": "foo",
#     "body": "bar",
#     "userId": 1
# }'''
    while True:
        time.sleep(30)  # Wait 30 seconds between requests
        if gsm_initialized:
            print("Starting HTTP POST request in background...")
            set_http_active(True)
            try:
                # First try with SSL
                print("Attempting POST with SSL...")
                gsm_http_post("http://69260b4c26b9.ngrok-free.app/predict", test_json_data, use_ssl=False)
                
                # If SSL fails (603), try without SSL (but this won't work with HTTPS)
                # You might want to add a fallback HTTP URL for testing
                
            except Exception as e:
                print("HTTP POST thread error:", e)
                # Fallback: try the old health check
                try:
                    print("Fallback to health check...")
                    gsm_http_get("http://0902ca5f11ff.ngrok-free.app/predict")
                except Exception as e2:
                    print("Fallback also failed:", e2)
            finally:
                set_http_active(False)
            print("HTTP request completed")
        else:
            print("GSM not initialized, skipping HTTP request")

# -------------------- GPS Thread Function --------------------
def gps_thread():
    global gps  # <-- important: tell Python you mean the global gps variable
    print("GPS thread started")
    buffer = ""  # Buffer to accumulate partial sentences
    last_display_time = time.time()
    display_interval = 5  # Display GPS data every 5 seconds

    while True:
        try:
            if gps_enabled and gps:
                try:
                    # Check if data is available (some ports don't implement .any())
                    available = False
                    try:
                        available = gps.any() if callable(getattr(gps, "any", None)) else False
                    except Exception:
                        # Some UART implementations may throw; fallback to read with timeout
                        available = False

                    if available:
                        data = gps.read()
                        if data:
                            # Decode bytes to string, handling encoding errors
                            try:
                                text = data.decode('ascii', errors='ignore')
                            except Exception:
                                text = str(data)

                            # Add to buffer
                            buffer += text

                            # Process complete sentences
                            while '\n' in buffer:
                                line, buffer = buffer.split('\n', 1)
                                sentence = line.strip()

                                # Only process valid NMEA sentences
                                if sentence.startswith('$') and len(sentence) > 10:
                                    try:
                                        parse_nmea(sentence)
                                    except Exception as parse_error:
                                        print(f"GPS parse error: {parse_error}")
                                        # keep going to next sentence

                            # Prevent buffer growing indefinitely
                            if len(buffer) > 1000:
                                buffer = buffer[-500:]  # keep last 500 chars

                    else:
                        # No data available, short sleep
                        time.sleep(0.1)

                    # Display GPS data periodically (not on every sentence)
                    current_time = time.time()
                    if current_time - last_display_time >= display_interval:
                        try:
                            with gps_lock:
                                if gps_data.get('fix_status') == "Active":
                                    display_gps_data()
                                else:
                                    print("GPS: Searching for satellites...")
                            last_display_time = current_time
                        except Exception as display_error:
                            print(f"GPS display error: {display_error}")

                except Exception as read_error:
                    print(f"GPS read error: {read_error}")
                    time.sleep(0.5)  # Wait before retrying

            else:
                # GPS not enabled or gps object is None
                print("GPS not enabled or not available, thread sleeping...")
                time.sleep(10)

        except Exception as e:
            # Top-level safeguard so thread doesn't die
            print(f"GPS thread critical error: {e}")
            time.sleep(2)  # Wait before continuing

            # Try to reinitialize GPS if flag says it's enabled but object is None
            if gps_enabled and not gps:
                try:
                    # reassign to global gps
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
                print("MPU Accel:", mpu_data["accel"], "Gyro:", mpu_data["gyro"], "Temp:", round(mpu_data["temp"],2))
            except Exception as e:
                print("MPU thread error:", e)
        time.sleep(2)  # Read MPU data every 2 seconds

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
        else: set_status("INVALID")
        
        current_status = get_status()
        
        # Show HTTP status indicator
        http_indicator = " [HTTP]" if is_http_active() else ""
        show_message("STATUS:", current_status + http_indicator)
        print("Key pressed:", key, "| Bus Status:", current_status)
    
    # Update display periodically with current status
    if loop_count % 10 == 0:  # Every 10 iterations (roughly 10 seconds)
        current_status = get_status()
        http_indicator = " [HTTP]" if is_http_active() else ""
        gsm_status_text = "Connected" if gsm_initialized else "Not Connected"
        show_message("Bus Online", 
                    f"Status: {current_status[:12]}", 
                    f"GSM: {gsm_status_text[:12]}", 
                    f"Loop: {loop_count}{http_indicator}")
    
    print(f"Loop {loop_count} | Status: {get_status()} | GSM: {'Connected' if gsm_initialized else 'Not Connected'} | HTTP Active: {is_http_active()}")
    time.sleep(1)