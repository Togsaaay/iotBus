from machine import UART, Pin, I2C
import time, struct, utime
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

# -------------------- LCD Setup --------------------
I2C_ADDR = 0x27
i2c_lcd = I2C(0, scl=Pin(19), sda=Pin(18), freq=400000)
lcd = I2cLcd(i2c_lcd, I2C_ADDR, 4, 20)

def show_message(line1="", line2="", line3="", line4=""):
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
        return str(round(decimal,6))+"°"
    except:
        return "0.0°"

def parse_nmea(sentence):
    if not sentence.startswith('$'): return
    parts = sentence.split(',')
    if parts[0][1:] == 'GPGGA':
        gps_data['utc_time'] = parts[1][:6] if len(parts[1])>=6 else ''
        gps_data['latitude'] = convert_coord(parts[2], parts[3]) if parts[2] else '0.0'
        gps_data['longitude'] = convert_coord(parts[4], parts[5]) if parts[4] else '0.0'
        gps_data['fix_quality'] = parts[6]
        gps_data['satellites'] = parts[7]
        gps_data['altitude'] = str(round(safe_convert(parts[9], float, 0.0),1)) + " m"
        gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"

def display_gps_data():
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

def send_gsm_command(cmd, wait_ms=500):
    if not gsm_enabled: return "GSM not initialized"
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

def gsm_http_get(url):
    if not gsm_enabled: return
    print("HTTP GET:", url)
    send_gsm_command("AT+HTTPTERM", wait_ms=1000)
    send_gsm_command("AT+HTTPINIT", wait_ms=2000)
    send_gsm_command('AT+HTTPPARA="CID",1', wait_ms=1000)
    send_gsm_command('AT+HTTPPARA="URL","{}"'.format(url), wait_ms=1000)
    send_gsm_command('AT+HTTPPARA="USERDATA","User-Agent: ESP32-GSM"', wait_ms=1000)
    
    resp = send_gsm_command("AT+HTTPACTION=0", wait_ms=15000)
    print("HTTPACTION Response:", resp)
    
    if "+HTTPACTION:" in resp:
        try:
            parts = resp.split("+HTTPACTION: 0,")[1].split(",")
            status_code = parts[0]
            print("HTTP Status:", status_code)
            if status_code == "200":
                read_resp = send_gsm_command("AT+HTTPREAD", wait_ms=5000)
                print("HTTPREAD Response:", read_resp)
            else:
                print("HTTP request failed with status code:", status_code)
                if status_code == "601": print("Error 601: Network error - check GPRS/SIM/APN")
        except:
            print("Error parsing HTTPACTION response")
    send_gsm_command("AT+HTTPTERM", wait_ms=1000)

# -------------------- MPU6050 Setup --------------------
MPU_ADDR = 0x68
i2c_bus = I2C(1, scl=Pin(22), sda=Pin(21))
mpu_enabled = MPU_ADDR in i2c_bus.scan()
def mpu_write(reg,data): i2c_bus.writeto_mem(MPU_ADDR, reg, bytes([data]))
def mpu_read(reg,n=1): return i2c_bus.readfrom_mem(MPU_ADDR, reg, n)
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

# -------------------- Initialization --------------------
print("System Running...")
current_status="STANDBY"
show_message("Bus Online","Initializing...")
gsm_initialized=False

if gsm_enabled:
    gsm_status()
    if gsm_gprs_connect("internet","",""):
        gsm_initialized=True
        print("GPRS Connected.")
    else:
        print("GPRS connection failed")

print("Initialization complete!")
show_message("Bus Online","Monitoring...")

# -------------------- Main Loop --------------------
loop_count=0
http_timer=0
HTTP_INTERVAL=30  # seconds

while True:
    loop_count += 1
    print("\n--- Loop", loop_count, "---")
    
    # Keypad input
    key = scan_keypad()
    if key:
        if key == '1': current_status = "FULL"
        elif key == '2': current_status = "AVAILABLE"
        elif key == 'A': current_status = "STANDING"
        elif key == '4': current_status = "INACTIVE"
        elif key == '5': current_status = "HELP REQUESTED"
        else: current_status = "INVALID"
        show_message("STATUS:", current_status)
        print("Key pressed:", key, "| Bus Status:", current_status)
    
    # GPS data
    if gps_enabled and gps.any():
        try:
            sentence = gps.readline().decode('ascii').strip()
            parse_nmea(sentence)
            if gps_data['fix_status'] == "Active":
                display_gps_data()
            else:
                print("GPS: Searching for satellites...")
        except Exception as e:
            print("GPS Error:", e)
    
    # MPU6050
    if mpu_enabled:
        try:
            mpu_data = read_mpu()
            print("MPU Accel:", mpu_data["accel"], "Gyro:", mpu_data["gyro"], "Temp:", round(mpu_data["temp"],2))
        except Exception as e:
            print("MPU Error:", e)
    
    # HTTP check every 30 sec
    http_timer += 1
    if gsm_initialized and http_timer >= HTTP_INTERVAL:
        print("Checking JSONPlaceholder...")
        gsm_http_get("http://jsonplaceholder.typicode.com/posts/1")
        http_timer = 0
    
    print("Bus Status:", current_status, "| GSM:", "Connected" if gsm_initialized else "Not Connected")
    time.sleep(1)
