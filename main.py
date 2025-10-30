import network
import time
import urequests
import ujson
import ubinascii
from ucryptolib import aes

# -------------------- Encryption Setup --------------------
# IMPORTANT: Keep this key secure
ENCRYPTION_KEY = b'MySecureKey12345MySecureKey12345'  # Must be exactly 32 bytes for AES-256

def pad_data(data):
    """PKCS7 padding"""
    padding_length = 16 - (len(data) % 16)
    return data + bytes([padding_length] * padding_length)

def encrypt_data(plaintext):
    """Encrypt data using AES-256-CBC"""
    try:
        # Generate a simple IV based on time
        import utime
        ms = utime.ticks_ms()
        iv_bytes = []
        for i in range(16):
            iv_bytes.append((ms >> (i * 2)) & 0xFF)
        iv = bytes(iv_bytes)
        
        # Convert string to bytes if needed
        if isinstance(plaintext, str):
            plaintext = plaintext.encode('utf-8')
        
        # Pad the plaintext
        padded = pad_data(plaintext)
        
        # Encrypt using AES CBC mode
        cipher = aes(ENCRYPTION_KEY, 2, iv)  # mode 2 = CBC
        encrypted = cipher.encrypt(padded)
        
        # Return IV + encrypted data, base64 encoded
        combined = iv + encrypted
        return ubinascii.b2a_base64(combined).decode('utf-8').strip()
        
    except Exception as e:
        print("Encryption error:", e)
        return None

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

from machine import UART, Pin, I2C
import time, struct, utime, _thread
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# -------------------- Configuration --------------------
API_ENDPOINT = "https://ridealert-backend.onrender.com/predict"
STATUS_ENDPOINT = "https://ridealert-backend.onrender.com/vehicles/iot/device/"
FLEET_ID = "68bee7eb753d0934fd57bdea"
DEVICE_ID = "68e20f304073a18ae0f980b4"
POST_INTERVAL = 5  # Send data every 5 seconds

# -------------------- LCD Setup --------------------
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(5), sda=Pin(19), freq=100000)
lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)

# Thread-safe LCD lock
lcd_lock = _thread.allocate_lock()

def show_message(line1="", line2="", line3="", line4=""):
    with lcd_lock:
        lcd.clear()
        time.sleep_ms(5)
        lcd.move_to(0,0); lcd.putstr(line1[:20])
        lcd.move_to(0,1); lcd.putstr(line2[:20])
        lcd.move_to(0,2); lcd.putstr(line3[:20])
        lcd.move_to(0,3); lcd.putstr(line4[:20])

# -------------------- GPS Setup --------------------
try:
    gps = UART(1, baudrate=9600, tx=18, rx=16)
    gps_enabled = True
    print("GPS: Initialized")
except:
    gps = None
    gps_enabled = False
    print("GPS: Not detected")

gps_data = {'utc_time':'','date':'','latitude':'','longitude':'',
            'altitude':'','speed':'','course':'','fix_status':'','fix_quality':'','satellites':'',
            'raw_latitude': 0.0, 'raw_longitude': 0.0, 'raw_altitude': 0.0, 'raw_speed': 0.0}

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
                gps_data['raw_latitude'] = convert_coord_raw(parts[2], parts[3])
            
            if parts[4] and parts[5]:
                gps_data['longitude'] = convert_coord(parts[4], parts[5])
                gps_data['raw_longitude'] = convert_coord_raw(parts[4], parts[5])
            
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
                        speed_kmh = float(parts[7]) * 1.852
                        gps_data['speed'] = str(round(speed_kmh, 1)) + " km/h"
                        gps_data['raw_speed'] = round(speed_kmh, 1)
                    except:
                        gps_data['speed'] = "0.0 km/h"
                        gps_data['raw_speed'] = 0.0
                if parts[8]:
                    gps_data['course'] = parts[8]
                        
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

def display_gps_data():
    print("Time:", gps_data['utc_time'], "Date:", gps_data['date'])
    print("Lat:", gps_data['latitude'], "Lon:", gps_data['longitude'])
    print("Alt:", gps_data['altitude'], "Speed:", gps_data['speed'])
    print("Fix:", gps_data['fix_status'], "Sats:", gps_data['satellites'])

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
mpu_lock = _thread.allocate_lock()

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
key_map = [['1','2','3','A'], ['4','5','6','B'], ['7','8','9','C'], ['*','0','#','D']]
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

keypad_queue = []
keypad_queue_lock = _thread.allocate_lock()

def set_status(new_status):
    global current_status
    with status_lock:
        current_status = new_status

def get_status():
    with status_lock:
        return current_status

def add_keypad_to_queue(key):
    with keypad_queue_lock:
        keypad_queue.append(key)

def get_keypad_from_queue():
    with keypad_queue_lock:
        if len(keypad_queue) > 0:
            return keypad_queue.pop(0)
    return None

# -------------------- API POST Functions --------------------
def send_keypad_status(key):
    """Send ENCRYPTED keypad press to status endpoint"""
    try:
        url = STATUS_ENDPOINT + DEVICE_ID
        
        # Prepare payload
        payload = {"key": key}
        json_data = ujson.dumps(payload)
        
        # Encrypt the entire payload
        encrypted_data = encrypt_data(json_data)
        
        if not encrypted_data:
            print("Encryption failed")
            return False
        
        # Send as encrypted string
        encrypted_payload = {"encrypted_data": encrypted_data}
        
        print("Sending encrypted keypad status...")
        print("Encrypted:", encrypted_data[:50] + "...")  # Show first 50 chars
        
        headers = {'Content-Type': 'application/json'}
        response = urequests.post(url, data=ujson.dumps(encrypted_payload), headers=headers)
        
        print("Status Response:", response.status_code)
        
        response.close()
        return True
        
    except Exception as e:
        print("Status API error:", e)
        return False

def send_sensor_data():
    """Send ENCRYPTED sensor data to API endpoint"""
    try:
        ax, ay, az = mpu_latest["accel"]
        gx, gy, gz = mpu_latest["gyro"]
        
        # Prepare sensor payload
        payload = {
            "fleet_id": FLEET_ID,
            "device_id": DEVICE_ID,
            "Cn0DbHz": float(gps_data.get('satellites', '0')) * 4.0 + 30.0,
            "Svid": int(gps_data.get('satellites', '0')) if gps_data.get('satellites', '0').isdigit() else 0,
            "SvElevationDegrees": 35.2,
            "SvAzimuthDegrees": 120.8,
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
            "Speed": gps_data['raw_speed']
        }
        
        json_data = ujson.dumps(payload)
        
        # Encrypt the entire payload
        encrypted_data = encrypt_data(json_data)
        
        if not encrypted_data:
            print("Encryption failed")
            return False
        
        # Send as encrypted string
        encrypted_payload = {"encrypted_data": encrypted_data}
        
        print("Sending encrypted sensor data...")
        print("Encrypted:", encrypted_data[:50] + "...")  # Show first 50 chars
        
        headers = {'Content-Type': 'application/json'}
        response = urequests.post(API_ENDPOINT, data=ujson.dumps(encrypted_payload), headers=headers)
        
        print("Sensor Response:", response.status_code)
        
        response.close()
        return True
        
    except Exception as e:
        print("API POST error:", e)
        return False

# -------------------- Thread Functions --------------------
def mpu_thread():
    """Background thread for reading MPU data"""
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

def sensor_post_thread():
    """Background thread for posting sensor data"""
    print("Sensor POST thread started")
    last_post = 0
    
    while True:
        try:
            current_time = time.time()
            
            if current_time - last_post >= POST_INTERVAL:
                if gps_data.get('fix_status') == "Active":
                    send_sensor_data()
                else:
                    print("Skipping sensor post - waiting for GPS fix")
                last_post = current_time
            
            time.sleep(1)
            
        except Exception as e:
            print("Sensor POST thread error:", e)
            time.sleep(5)

def keypad_post_thread():
    """Background thread for posting keypad status"""
    print("Keypad POST thread started")
    
    while True:
        try:
            key = get_keypad_from_queue()
            
            if key:
                print("Processing keypad press from queue:", key)
                send_keypad_status(key)
            
            time.sleep(0.1)
            
        except Exception as e:
            print("Keypad POST thread error:", e)
            time.sleep(1)

# -------------------- Initialization --------------------
print("System Running...")
show_message("Bus Online", "Initializing...")

print("Initialization complete!")
show_message("Bus Online", "Monitoring...")

# Start all background threads
try:
    _thread.start_new_thread(mpu_thread, ())
    print("MPU thread started")
except Exception as e:
    print("Failed to start MPU thread:", e)

try:
    _thread.start_new_thread(sensor_post_thread, ())
    print("Sensor POST thread started")
except Exception as e:
    print("Failed to start sensor POST thread:", e)

try:
    _thread.start_new_thread(keypad_post_thread, ())
    print("Keypad POST thread started")
except Exception as e:
    print("Failed to start keypad POST thread:", e)

# -------------------- Main Loop --------------------
loop_count = 0
last_gps_display = 0

while True:
    loop_count += 1
    current_time = time.time()
    
    if gps_enabled:
        read_gps()
        
        if current_time - last_gps_display >= 5:
            sats = gps_data.get('satellites', '0')
            if gps_data.get('fix_status') == "Active":
                display_gps_data()
            else:
                print("GPS: Searching... Sats:", sats)
            last_gps_display = current_time
    
    key = scan_keypad()
    if key:
        add_keypad_to_queue(key)
        
        if key == '1': 
            set_status("FULL")
        elif key == '2': 
            set_status("AVAILABLE")
        elif key == 'A': 
            set_status("STANDING")
        elif key == '4': 
            set_status("INACTIVE")
        elif key == '5': 
            set_status("HELP REQUESTED")
        elif key == 'B': 
            set_status("BUGO")
        elif key == 'C': 
            set_status("IGPIT")
        else: 
            set_status("INVALID")
        
        current_status = get_status()
        show_message("STATUS:", current_status)
        print("Key:", key, "| Status:", current_status)
    
    if loop_count % 10 == 0:
        current_status = get_status()
        gps_sats = gps_data.get('satellites', '0')
        gps_fix = gps_data.get('fix_status', 'No Fix')
        show_message("Bus Online", 
                    "Status: " + current_status[:11], 
                    "GPS: " + gps_fix[:8] + " S:" + gps_sats)
    
    time.sleep(0.1)