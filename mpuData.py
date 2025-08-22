# import machine
# import time
# import struct
# import math

# class MPU6050:
#     """Complete MPU6050/MPU9250 interface for ESP32 with MicroPython"""
    
#     # MPU6050 Register Addresses
#     PWR_MGMT_1 = 0x6B
#     PWR_MGMT_2 = 0x6C
#     SMPLRT_DIV = 0x19
#     CONFIG = 0x1A
#     GYRO_CONFIG = 0x1B
#     ACCEL_CONFIG = 0x1C
    
#     # Data Registers
#     ACCEL_XOUT_H = 0x3B
#     ACCEL_XOUT_L = 0x3C
#     ACCEL_YOUT_H = 0x3D
#     ACCEL_YOUT_L = 0x3E
#     ACCEL_ZOUT_H = 0x3F
#     ACCEL_ZOUT_L = 0x40
    
#     TEMP_OUT_H = 0x41
#     TEMP_OUT_L = 0x42
    
#     GYRO_XOUT_H = 0x43
#     GYRO_XOUT_L = 0x44
#     GYRO_YOUT_H = 0x45
#     GYRO_YOUT_L = 0x46
#     GYRO_ZOUT_H = 0x47
#     GYRO_ZOUT_L = 0x48
    
#     # MPU9250 Magnetometer (AK8963) Registers
#     MAG_ADDRESS = 0x0C
#     MAG_XOUT_L = 0x03
#     MAG_XOUT_H = 0x04
#     MAG_YOUT_L = 0x05
#     MAG_YOUT_H = 0x06
#     MAG_ZOUT_L = 0x07
#     MAG_ZOUT_H = 0x08
#     MAG_CNTL = 0x0A
#     MAG_ST2 = 0x09
    
#     # WHO_AM_I Register
#     WHO_AM_I = 0x75
    
#     def __init__(self, i2c, address=0x68, mag_address=0x0C):
#         """
#         Initialize MPU sensor
#         Args:
#             i2c: I2C object
#             address: MPU6050/9250 I2C address (default 0x68)
#             mag_address: Magnetometer address for MPU9250 (default 0x0C)
#         """
#         self.i2c = i2c
#         self.address = address
#         self.mag_address = mag_address
        
#         # Check if device is connected
#         self.device_id = self._read_byte(self.WHO_AM_I)
#         print(f"Device ID: 0x{self.device_id:02X}")
        
#         # Determine device type
#         if self.device_id == 0x68:
#             self.device_type = "MPU6050"
#             self.has_magnetometer = False
#         elif self.device_id == 0x71:
#             self.device_type = "MPU9250"
#             self.has_magnetometer = True
#         else:
#             self.device_type = "Unknown"
#             self.has_magnetometer = False
#             print(f"Warning: Unknown device ID 0x{self.device_id:02X}")
        
#         print(f"Detected: {self.device_type}")
        
#         # Initialize the sensor
#         self._initialize()
        
#         # Calibration offsets (bias values)
#         self.accel_offset = [0, 0, 0]  # X, Y, Z bias
#         self.gyro_offset = [0, 0, 0]   # X, Y, Z bias
#         self.mag_offset = [0, 0, 0]    # X, Y, Z bias
        
#         # Scale factors for bias correction
#         self.accel_scale = [1.0, 1.0, 1.0]  # X, Y, Z scale factors
#         self.gyro_scale = [1.0, 1.0, 1.0]   # X, Y, Z scale factors
#         self.mag_scale = [1.0, 1.0, 1.0]    # X, Y, Z scale factors
        
#         # Axis orientation (can be remapped)
#         self.axis_map = {'x': 0, 'y': 1, 'z': 2}  # Default mapping
#         self.axis_sign = {'x': 1, 'y': 1, 'z': 1}  # Default signs
        
#     def _read_byte(self, register):
#         """Read a single byte from register"""
#         return self.i2c.readfrom_mem(self.address, register, 1)[0]
    
#     def _write_byte(self, register, value):
#         """Write a single byte to register"""
#         self.i2c.writeto_mem(self.address, register, bytes([value]))
    
#     def _read_word(self, register):
#         """Read a 16-bit word from register (big-endian)"""
#         data = self.i2c.readfrom_mem(self.address, register, 2)
#         return struct.unpack('>h', data)[0]  # signed 16-bit big-endian
    
#     def _read_word_unsigned(self, register):
#         """Read a 16-bit unsigned word from register (big-endian)"""
#         data = self.i2c.readfrom_mem(self.address, register, 2)
#         return struct.unpack('>H', data)[0]  # unsigned 16-bit big-endian
    
#     def _initialize(self):
#         """Initialize the MPU sensor"""
#         # Wake up the device
#         self._write_byte(self.PWR_MGMT_1, 0x01)  # Use PLL with X axis gyroscope reference
#         time.sleep_ms(100)
        
#         # Configure sample rate (1kHz / (1 + SMPLRT_DIV))
#         self._write_byte(self.SMPLRT_DIV, 0x07)  # Sample rate = 125Hz
        
#         # Configure low pass filter
#         self._write_byte(self.CONFIG, 0x06)  # DLPF = 5Hz
        
#         # Configure gyroscope (±500 deg/s)
#         self._write_byte(self.GYRO_CONFIG, 0x08)  # FS_SEL = 1
        
#         # Configure accelerometer (±4g)
#         self._write_byte(self.ACCEL_CONFIG, 0x08)  # AFS_SEL = 1
        
#         # Initialize magnetometer if available (MPU9250)
#         if self.has_magnetometer:
#             self._init_magnetometer()
    
#     def _init_magnetometer(self):
#         """Initialize magnetometer for MPU9250"""
#         try:
#             # Enable I2C bypass to access magnetometer
#             self._write_byte(0x37, 0x02)  # INT_PIN_CFG
#             self._write_byte(0x6A, 0x00)  # USER_CTRL
#             time.sleep_ms(10)
            
#             # Configure magnetometer for continuous measurement mode
#             self.i2c.writeto_mem(self.mag_address, self.MAG_CNTL, bytes([0x16]))  # 16-bit, 100Hz
#             time.sleep_ms(10)
#             print("Magnetometer initialized")
#         except Exception as e:
#             print(f"Magnetometer initialization failed: {e}")
#             self.has_magnetometer = False
    
#     def read_accel_raw(self):
#         """Read raw accelerometer data"""
#         x = self._read_word(self.ACCEL_XOUT_H)
#         y = self._read_word(self.ACCEL_YOUT_H)
#         z = self._read_word(self.ACCEL_ZOUT_H)
#         return x, y, z
    
#     def read_gyro_raw(self):
#         """Read raw gyroscope data"""
#         x = self._read_word(self.GYRO_XOUT_H)
#         y = self._read_word(self.GYRO_YOUT_H)
#         z = self._read_word(self.GYRO_ZOUT_H)
#         return x, y, z
    
#     def read_temp_raw(self):
#         """Read raw temperature data"""
#         return self._read_word(self.TEMP_OUT_H)
    
#     def read_mag_raw(self):
#         """Read raw magnetometer data (MPU9250 only)"""
#         if not self.has_magnetometer:
#             return None, None, None
        
#         try:
#             # Check if data is ready
#             st2 = self.i2c.readfrom_mem(self.mag_address, self.MAG_ST2, 1)[0]
#             if st2 & 0x01:  # Data ready
#                 # Read magnetometer data (little-endian for magnetometer)
#                 data = self.i2c.readfrom_mem(self.mag_address, self.MAG_XOUT_L, 6)
#                 x = struct.unpack('<h', data[0:2])[0]
#                 y = struct.unpack('<h', data[2:4])[0]
#                 z = struct.unpack('<h', data[4:6])[0]
#                 return x, y, z
#             else:
#                 return None, None, None
#         except Exception as e:
#             print(f"Magnetometer read error: {e}")
#             return None, None, None
    
#     def read_accel(self):
#         """Read accelerometer data in g (gravity units)"""
#         x_raw, y_raw, z_raw = self.read_accel_raw()
#         # Scale factor for ±4g range: 8192 LSB/g
#         scale = 8192.0
#         x = ((x_raw - self.accel_offset[0]) * self.accel_scale[0]) / scale
#         y = ((y_raw - self.accel_offset[1]) * self.accel_scale[1]) / scale
#         z = ((z_raw - self.accel_offset[2]) * self.accel_scale[2]) / scale
#         return x, y, z
    
#     def read_gyro(self):
#         """Read gyroscope data in degrees per second"""
#         x_raw, y_raw, z_raw = self.read_gyro_raw()
#         # Scale factor for ±500°/s range: 65.5 LSB/(°/s)
#         scale = 65.5
#         x = ((x_raw - self.gyro_offset[0]) * self.gyro_scale[0]) / scale
#         y = ((y_raw - self.gyro_offset[1]) * self.gyro_scale[1]) / scale
#         z = ((z_raw - self.gyro_offset[2]) * self.gyro_scale[2]) / scale
#         return x, y, z
    
#     def read_temp(self):
#         """Read temperature in Celsius"""
#         temp_raw = self.read_temp_raw()
#         # Temperature formula: (temp_raw / 340.0) + 36.53
#         return temp_raw / 340.0 + 36.53
    
#     def read_mag(self):
#         """Read magnetometer data in µT (microtesla) - MPU9250 only"""
#         if not self.has_magnetometer:
#             return None, None, None
        
#         x_raw, y_raw, z_raw = self.read_mag_raw()
#         if x_raw is None:
#             return None, None, None
        
#         # Scale factor for magnetometer: 0.6 µT/LSB (approximate)
#         scale = 0.6
#         x = ((x_raw - self.mag_offset[0]) * self.mag_scale[0]) * scale
#         y = ((y_raw - self.mag_offset[1]) * self.mag_scale[1]) * scale
#         z = ((z_raw - self.mag_offset[2]) * self.mag_scale[2]) * scale
#         return x, y, z
    
#     def get_axis_info(self):
#         """Get detailed axis information and bias data"""
#         return {
#             'axis_mapping': self.axis_map,
#             'axis_signs': self.axis_sign,
#             'accel_bias': {
#                 'x': self.accel_offset[0],
#                 'y': self.accel_offset[1], 
#                 'z': self.accel_offset[2]
#             },
#             'gyro_bias': {
#                 'x': self.gyro_offset[0],
#                 'y': self.gyro_offset[1],
#                 'z': self.gyro_offset[2]
#             },
#             'mag_bias': {
#                 'x': self.mag_offset[0],
#                 'y': self.mag_offset[1],
#                 'z': self.mag_offset[2]
#             },
#             'accel_scale': {
#                 'x': self.accel_scale[0],
#                 'y': self.accel_scale[1],
#                 'z': self.accel_scale[2]
#             },
#             'gyro_scale': {
#                 'x': self.gyro_scale[0],
#                 'y': self.gyro_scale[1],
#                 'z': self.gyro_scale[2]
#             },
#             'mag_scale': {
#                 'x': self.mag_scale[0],
#                 'y': self.mag_scale[1],
#                 'z': self.mag_scale[2]
#             }
#         }
    
#     def set_axis_mapping(self, x_axis=0, y_axis=1, z_axis=2, x_sign=1, y_sign=1, z_sign=1):
#         """
#         Set custom axis mapping and signs
#         Args:
#             x_axis, y_axis, z_axis: Which physical axis (0,1,2) maps to logical X,Y,Z
#             x_sign, y_sign, z_sign: Sign multiplier (+1 or -1) for each axis
#         """
#         self.axis_map = {'x': x_axis, 'y': y_axis, 'z': z_axis}
#         self.axis_sign = {'x': x_sign, 'y': y_sign, 'z': z_sign}
#         print(f"Axis mapping updated: X->axis{x_axis}({'+' if x_sign>0 else '-'}), Y->axis{y_axis}({'+' if y_sign>0 else '-'}), Z->axis{z_axis}({'+' if z_sign>0 else '-'})")
    
#     def apply_axis_mapping(self, x, y, z):
#         """Apply axis mapping and sign corrections"""
#         data = [x, y, z]
#         mapped_x = data[self.axis_map['x']] * self.axis_sign['x']
#         mapped_y = data[self.axis_map['y']] * self.axis_sign['y'] 
#         mapped_z = data[self.axis_map['z']] * self.axis_sign['z']
#         return mapped_x, mapped_y, mapped_z
    
#     def read_accel_mapped(self):
#         """Read accelerometer with axis mapping applied"""
#         x, y, z = self.read_accel()
#         return self.apply_axis_mapping(x, y, z)
    
#     def read_gyro_mapped(self):
#         """Read gyroscope with axis mapping applied"""
#         x, y, z = self.read_gyro()
#         return self.apply_axis_mapping(x, y, z)
    
#     def read_mag_mapped(self):
#         """Read magnetometer with axis mapping applied"""
#         if not self.has_magnetometer:
#             return None, None, None
#         x, y, z = self.read_mag()
#         if x is None:
#             return None, None, None
#         return self.apply_axis_mapping(x, y, z)
    
#     def read_all_raw(self):
#         """Read all sensor data (raw values)"""
#         data = {}
#         data['accel'] = self.read_accel_raw()
#         data['gyro'] = self.read_gyro_raw()
#         data['temp'] = self.read_temp_raw()
        
#         if self.has_magnetometer:
#             data['mag'] = self.read_mag_raw()
        
#         return data
    
#     def read_all(self):
#         """Read all sensor data (converted values)"""
#         data = {}
#         data['accel'] = self.read_accel()  # g
#         data['gyro'] = self.read_gyro()    # °/s
#         data['temp'] = self.read_temp()    # °C
        
#         if self.has_magnetometer:
#             data['mag'] = self.read_mag()  # µT
        
#         return data
    
#     def calculate_angles(self):
#         """Calculate roll and pitch angles from accelerometer"""
#         ax, ay, az = self.read_accel()
        
#         roll = math.atan2(ay, az) * 180 / math.pi
#         pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
        
#         return roll, pitch
    
#     def calculate_magnitude(self, x, y, z):
#         """Calculate magnitude/length of 3D vector"""
#         return math.sqrt(x*x + y*y + z*z)
    
#     def calculate_all_angles(self):
#         """Calculate all possible angles and orientations"""
#         ax, ay, az = self.read_accel()
#         gx, gy, gz = self.read_gyro()
        
#         # Accelerometer angles
#         roll = math.atan2(ay, az) * 180 / math.pi
#         pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180 / math.pi
        
#         # Total acceleration magnitude
#         accel_magnitude = self.calculate_magnitude(ax, ay, az)
#         gyro_magnitude = self.calculate_magnitude(gx, gy, gz)
        
#         # Tilt angle from vertical
#         tilt_angle = math.acos(abs(az) / accel_magnitude) * 180 / math.pi
        
#         return {
#             'roll': roll,
#             'pitch': pitch,
#             'tilt_angle': tilt_angle,
#             'accel_magnitude': accel_magnitude,
#             'gyro_magnitude': gyro_magnitude
#         }
    
#     def calculate_heading(self):
#         """Calculate magnetic heading (compass) - MPU9250 only"""
#         if not self.has_magnetometer:
#             return None
        
#         mx, my, mz = self.read_mag()
#         if mx is None:
#             return None
        
#         # Simple 2D compass heading (assuming level orientation)
#         heading = math.atan2(my, mx) * 180 / math.pi
        
#         # Normalize to 0-360 degrees
#         if heading < 0:
#             heading += 360
        
#         return heading
    
#     def calculate_tilt_compensated_heading(self):
#         """Calculate tilt-compensated compass heading - MPU9250 only"""
#         if not self.has_magnetometer:
#             return None
        
#         ax, ay, az = self.read_accel()
#         mx, my, mz = self.read_mag()
        
#         if mx is None:
#             return None
        
#         # Calculate roll and pitch in radians
#         roll = math.atan2(ay, az)
#         pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        
#         # Tilt compensation
#         mx_comp = mx * math.cos(pitch) + mz * math.sin(pitch)
#         my_comp = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)
        
#         # Calculate heading
#         heading = math.atan2(my_comp, mx_comp) * 180 / math.pi
        
#         # Normalize to 0-360 degrees
#         if heading < 0:
#             heading += 360
        
#         return heading
    
#     def detect_motion(self, threshold=0.1):
#         """Detect if device is in motion based on gyroscope"""
#         gx, gy, gz = self.read_gyro()
#         gyro_magnitude = self.calculate_magnitude(gx, gy, gz)
#         return gyro_magnitude > threshold
    
#     def detect_freefall(self, threshold=0.3):
#         """Detect freefall condition"""
#         ax, ay, az = self.read_accel()
#         accel_magnitude = self.calculate_magnitude(ax, ay, az)
#         return accel_magnitude < threshold
    
#     def detect_tap(self, threshold=2.0):
#         """Simple tap detection"""
#         ax, ay, az = self.read_accel()
#         accel_magnitude = self.calculate_magnitude(ax, ay, az)
#         return accel_magnitude > threshold
    
#     def get_orientation_string(self):
#         """Get human-readable orientation"""
#         ax, ay, az = self.read_accel()
        
#         # Determine dominant axis
#         if abs(az) > abs(ax) and abs(az) > abs(ay):
#             if az > 0.7:
#                 return "Face Up"
#             elif az < -0.7:
#                 return "Face Down"
#         elif abs(ax) > abs(ay):
#             if ax > 0.7:
#                 return "Tilted Right"
#             elif ax < -0.7:
#                 return "Tilted Left"
#         else:
#             if ay > 0.7:
#                 return "Tilted Forward"
#             elif ay < -0.7:
#                 return "Tilted Backward"
        
#         return "Unknown Position"
    
#     def calculate_rotation_matrix(self):
#         """Calculate rotation matrix from accelerometer and magnetometer"""
#         if not self.has_magnetometer:
#             return None
        
#         ax, ay, az = self.read_accel()
#         mx, my, mz = self.read_mag()
        
#         if mx is None:
#             return None
        
#         # Normalize accelerometer
#         accel_mag = self.calculate_magnitude(ax, ay, az)
#         ax, ay, az = ax/accel_mag, ay/accel_mag, az/accel_mag
        
#         # Normalize magnetometer
#         mag_mag = self.calculate_magnitude(mx, my, mz)
#         mx, my, mz = mx/mag_mag, my/mag_mag, mz/mag_mag
        
#         # Calculate rotation matrix (simplified)
#         # This is a basic implementation - full implementation would be more complex
#         return {
#             'accel_normalized': (ax, ay, az),
#             'mag_normalized': (mx, my, mz)
#         }
    
#     def get_comprehensive_data(self):
#         """Get all possible data from the MPU sensor"""
#         data = {}
        
#         # Basic sensor readings
#         data['accel'] = self.read_accel()
#         data['gyro'] = self.read_gyro()
#         data['temp'] = self.read_temp()
        
#         if self.has_magnetometer:
#             data['mag'] = self.read_mag()
        
#         # Raw readings
#         data['accel_raw'] = self.read_accel_raw()
#         data['gyro_raw'] = self.read_gyro_raw()
#         data['temp_raw'] = self.read_temp_raw()
        
#         if self.has_magnetometer:
#             data['mag_raw'] = self.read_mag_raw()
        
#         # Calculated values
#         angles = self.calculate_all_angles()
#         data.update(angles)
        
#         # Additional calculations
#         data['orientation'] = self.get_orientation_string()
#         data['motion_detected'] = self.detect_motion()
#         data['freefall_detected'] = self.detect_freefall()
#         data['tap_detected'] = self.detect_tap()
        
#         # Compass data (if available)
#         if self.has_magnetometer:
#             data['heading'] = self.calculate_heading()
#             data['tilt_compensated_heading'] = self.calculate_tilt_compensated_heading()
        
#         return data
    
#     def calibrate_gyro(self, samples=1000):
#         """Calibrate gyroscope (calculate offsets when stationary)"""
#         print("🔧 Calibrating gyroscope... Keep sensor stationary!")
#         sum_x, sum_y, sum_z = 0, 0, 0
        
#         for i in range(samples):
#             x, y, z = self.read_gyro_raw()
#             sum_x += x
#             sum_y += y
#             sum_z += z
#             time.sleep_ms(2)
            
#             if i % 100 == 0:
#                 print(f"Progress: {i}/{samples}")
        
#         self.gyro_offset[0] = sum_x // samples
#         self.gyro_offset[1] = sum_y // samples
#         self.gyro_offset[2] = sum_z // samples
        
#         print(f"✅ Gyro calibration complete!")
#         print(f"   X-axis bias: {self.gyro_offset[0]}")
#         print(f"   Y-axis bias: {self.gyro_offset[1]}")
#         print(f"   Z-axis bias: {self.gyro_offset[2]}")
    
#     def calibrate_accel(self, samples=1000):
#         """Calibrate accelerometer (basic offset calibration)"""
#         print("🔧 Calibrating accelerometer... Place sensor on level surface!")
#         sum_x, sum_y, sum_z = 0, 0, 0
        
#         for i in range(samples):
#             x, y, z = self.read_accel_raw()
#             sum_x += x
#             sum_y += y
#             sum_z += z
#             time.sleep_ms(2)
            
#             if i % 100 == 0:
#                 print(f"Progress: {i}/{samples}")
        
#         self.accel_offset[0] = sum_x // samples
#         self.accel_offset[1] = sum_y // samples
#         self.accel_offset[2] = (sum_z // samples) - 8192  # Subtract 1g for Z-axis
        
#         print(f"✅ Accel calibration complete!")
#         print(f"   X-axis bias: {self.accel_offset[0]}")
#         print(f"   Y-axis bias: {self.accel_offset[1]}")
#         print(f"   Z-axis bias: {self.accel_offset[2]}")
    
#     def calibrate_mag(self, samples=500):
#         """Calibrate magnetometer - rotate sensor in all directions"""
#         if not self.has_magnetometer:
#             print("❌ Magnetometer not available for calibration")
#             return
        
#         print("🔧 Calibrating magnetometer... Rotate sensor in figure-8 pattern!")
#         print("   Move the sensor slowly in all directions for best results")
        
#         # Collect min/max values for each axis
#         min_vals = [float('inf')] * 3
#         max_vals = [float('-inf')] * 3
        
#         for i in range(samples):
#             x, y, z = self.read_mag_raw()
#             if x is not None:
#                 # Update min/max for each axis
#                 min_vals[0] = min(min_vals[0], x)
#                 max_vals[0] = max(max_vals[0], x)
#                 min_vals[1] = min(min_vals[1], y)
#                 max_vals[1] = max(max_vals[1], y)
#                 min_vals[2] = min(min_vals[2], z)
#                 max_vals[2] = max(max_vals[2], z)
            
#             time.sleep_ms(10)
            
#             if i % 50 == 0:
#                 print(f"Progress: {i}/{samples} - Range X:[{min_vals[0]:.0f},{max_vals[0]:.0f}] Y:[{min_vals[1]:.0f},{max_vals[1]:.0f}] Z:[{min_vals[2]:.0f},{max_vals[2]:.0f}]")
        
#         # Calculate offsets (hard iron correction)
#         for i in range(3):
#             if min_vals[i] != float('inf') and max_vals[i] != float('-inf'):
#                 self.mag_offset[i] = (max_vals[i] + min_vals[i]) / 2
#                 self.mag_scale[i] = 2.0 / (max_vals[i] - min_vals[i])  # Soft iron correction
        
#         print(f"✅ Magnetometer calibration complete!")
#         print(f"   X-axis bias: {self.mag_offset[0]:.1f}, scale: {self.mag_scale[0]:.3f}")
#         print(f"   Y-axis bias: {self.mag_offset[1]:.1f}, scale: {self.mag_scale[1]:.3f}")
#         print(f"   Z-axis bias: {self.mag_offset[2]:.1f}, scale: {self.mag_scale[2]:.3f}")
    
#     def measure_axis_sensitivity(self):
#         """Measure sensitivity of each axis"""
#         print("📏 Measuring axis sensitivity...")
#         print("   Place sensor in different orientations for 5 seconds each")
        
#         orientations = [
#             ("X-up (rotate around Y-Z)", "measuring X-axis sensitivity"),
#             ("Y-up (rotate around X-Z)", "measuring Y-axis sensitivity"),
#             ("Z-up (rotate around X-Y)", "measuring Z-axis sensitivity")
#         ]
        
#         sensitivities = {'accel': [0, 0, 0], 'gyro': [0, 0, 0]}
        
#         for i, (orientation, desc) in enumerate(orientations):
#             print(f"\n{i+1}. {orientation}")
#             print(f"   {desc} - rotate gently...")
            
#             # Measure for 5 seconds
#             accel_max = [0, 0, 0]
#             gyro_max = [0, 0, 0]
            
#             for _ in range(50):  # 5 seconds at 10Hz
#                 ax, ay, az = self.read_accel()
#                 gx, gy, gz = self.read_gyro()
                
#                 accel_max[0] = max(accel_max[0], abs(ax))
#                 accel_max[1] = max(accel_max[1], abs(ay))
#                 accel_max[2] = max(accel_max[2], abs(az))
                
#                 gyro_max[0] = max(gyro_max[0], abs(gx))
#                 gyro_max[1] = max(gyro_max[1], abs(gy))
#                 gyro_max[2] = max(gyro_max[2], abs(gz))
                
#                 time.sleep_ms(100)
            
#             sensitivities['accel'][i] = max(accel_max)
#             sensitivities['gyro'][i] = max(gyro_max)
            
#             print(f"   Max accel: {max(accel_max):.3f}g, Max gyro: {max(gyro_max):.1f}°/s")
        
#         print(f"\n📊 Sensitivity Analysis:")
#         print(f"   Accel sensitivity - X:{sensitivities['accel'][0]:.3f}g Y:{sensitivities['accel'][1]:.3f}g Z:{sensitivities['accel'][2]:.3f}g")
#         print(f"   Gyro sensitivity  - X:{sensitivities['gyro'][0]:.1f}°/s Y:{sensitivities['gyro'][1]:.1f}°/s Z:{sensitivities['gyro'][2]:.1f}°/s")
        
#         return sensitivities

# # Example usage and test functions
# def setup_mpu():
#     """Setup I2C and MPU sensor"""
#     # Initialize I2C (adjust pins according to your wiring)
#     i2c = machine.I2C(0, scl=machine.Pin(22), sda=machine.Pin(21), freq=400000)
    
#     # Scan for devices
#     devices = i2c.scan()
#     print("I2C devices found:", [hex(device) for device in devices])
    
#     if 0x68 not in devices:
#         print("MPU sensor not found!")
#         return None
    
#     # Initialize MPU
#     mpu = MPU6050(i2c)
#     return mpu

# def print_sensor_data(mpu, duration=10, show_all=False):
#     """Print sensor data for specified duration"""
#     start_time = time.time()
    
#     print("\n" + "="*80)
#     print(f"Reading {mpu.device_type} sensor data for {duration} seconds...")
#     if show_all:
#         print("COMPREHENSIVE MODE - All available data")
#     print("="*80)
    
#     while time.time() - start_time < duration:
#         try:
#             if show_all:
#                 # Get comprehensive data
#                 data = mpu.get_comprehensive_data()
                
#                 print(f"\n⏰ Time: {time.time() - start_time:.1f}s")
#                 print("=" * 80)
                
#                 # Basic sensor data
#                 ax, ay, az = data['accel']
#                 print(f"📊 Accelerometer (g):     X={ax:7.3f}  Y={ay:7.3f}  Z={az:7.3f}  |Mag|={data['accel_magnitude']:.3f}")
                
#                 gx, gy, gz = data['gyro']
#                 print(f"🌀 Gyroscope (°/s):       X={gx:7.2f}  Y={gy:7.2f}  Z={gz:7.2f}  |Mag|={data['gyro_magnitude']:.2f}")
                
#                 print(f"🌡️  Temperature (°C):      {data['temp']:.2f}")
                
#                 # Magnetometer (if available)
#                 if mpu.has_magnetometer and data['mag'][0] is not None:
#                     mx, my, mz = data['mag']
#                     mag_magnitude = mpu.calculate_magnitude(mx, my, mz)
#                     print(f"🧭 Magnetometer (µT):     X={mx:7.2f}  Y={my:7.2f}  Z={mz:7.2f}  |Mag|={mag_magnitude:.2f}")
#                     print(f"🧭 Compass Heading (°):   Simple={data['heading']:.1f}°  Tilt-Comp={data['tilt_compensated_heading']:.1f}°")
                
#                 # Angles and orientation
#                 print(f"📐 Angles (°):            Roll={data['roll']:6.2f}  Pitch={data['pitch']:6.2f}  Tilt={data['tilt_angle']:6.2f}")
#                 print(f"📱 Orientation:           {data['orientation']}")
                
#                 # Motion detection
#                 status_indicators = []
#                 if data['motion_detected']:
#                     status_indicators.append("🏃 MOTION")
#                 if data['freefall_detected']:
#                     status_indicators.append("🪂 FREEFALL")
#                 if data['tap_detected']:
#                     status_indicators.append("👆 TAP")
                
#                 if status_indicators:
#                     print(f"🚨 Status:                {' | '.join(status_indicators)}")
#                 else:
#                     print(f"😴 Status:                STATIONARY")
                
#                 # Raw values (compact format)
#                 ax_raw, ay_raw, az_raw = data['accel_raw']
#                 gx_raw, gy_raw, gz_raw = data['gyro_raw']
#                 print(f"📋 Raw Data:              A({ax_raw:6d},{ay_raw:6d},{az_raw:6d}) G({gx_raw:6d},{gy_raw:6d},{gz_raw:6d}) T({data['temp_raw']:5d})")
                
#                 # Bias information
#                 axis_info = data['axis_info']
#                 print(f"⚖️  Accelerometer Bias:    X={axis_info['accel_bias']['x']:6d}  Y={axis_info['accel_bias']['y']:6d}  Z={axis_info['accel_bias']['z']:6d}")
#                 print(f"⚖️  Gyroscope Bias:        X={axis_info['gyro_bias']['x']:6d}  Y={axis_info['gyro_bias']['y']:6d}  Z={axis_info['gyro_bias']['z']:6d}")
                
#                 if mpu.has_magnetometer:
#                     print(f"⚖️  Magnetometer Bias:     X={axis_info['mag_bias']['x']:6.1f}  Y={axis_info['mag_bias']['y']:6.1f}  Z={axis_info['mag_bias']['z']:6.1f}")
                
#                 # Scale factors
#                 print(f"📏 Accelerometer Scale:   X={axis_info['accel_scale']['x']:.3f}  Y={axis_info['accel_scale']['y']:.3f}  Z={axis_info['accel_scale']['z']:.3f}")
#                 print(f"📏 Gyroscope Scale:       X={axis_info['gyro_scale']['x']:.3f}  Y={axis_info['gyro_scale']['y']:.3f}  Z={axis_info['gyro_scale']['z']:.3f}")
                
#                 # Mapped values (with axis corrections)
#                 ax_mapped, ay_mapped, az_mapped = data['accel_mapped']
#                 gx_mapped, gy_mapped, gz_mapped = data['gyro_mapped']
#                 print(f"🗺️  Mapped Accel (g):      X={ax_mapped:7.3f}  Y={ay_mapped:7.3f}  Z={az_mapped:7.3f}")
#                 print(f"🗺️  Mapped Gyro (°/s):     X={gx_mapped:7.2f}  Y={gy_mapped:7.2f}  Z={gz_mapped:7.2f}")
                
#                 # Axis mapping info
#                 print(f"🧭 Axis Mapping:          X->axis{axis_info['axis_mapping']['x']} Y->axis{axis_info['axis_mapping']['y']} Z->axis{axis_info['axis_mapping']['z']}")
#                 print(f"🧭 Axis Signs:            X{'+' if axis_info['axis_signs']['x']>0 else '-'} Y{'+' if axis_info['axis_signs']['y']>0 else '-'} Z{'+' if axis_info['axis_signs']['z']>0 else '-'}")
                
#             else:
#                 # Simple mode (original format)
#                 data = mpu.read_all()
                
#                 print(f"\nTime: {time.time() - start_time:.1f}s")
#                 print("-" * 40)
                
#                 # Accelerometer
#                 ax, ay, az = data['accel']
#                 print(f"Accel (g):  X={ax:7.3f}  Y={ay:7.3f}  Z={az:7.3f}")
                
#                 # Gyroscope
#                 gx, gy, gz = data['gyro']
#                 print(f"Gyro (°/s): X={gx:7.2f}  Y={gy:7.2f}  Z={gz:7.2f}")
                
#                 # Temperature
#                 temp = data['temp']
#                 print(f"Temp (°C):  {temp:.2f}")
                
#                 # Magnetometer (if available)
#                 if mpu.has_magnetometer and data['mag'][0] is not None:
#                     mx, my, mz = data['mag']
#                     print(f"Mag (µT):   X={mx:7.2f}  Y={my:7.2f}  Z={mz:7.2f}")
                
#                 # Calculate angles
#                 roll, pitch = mpu.calculate_angles()
#                 print(f"Angles (°): Roll={roll:6.2f}  Pitch={pitch:6.2f}")
            
#             time.sleep(0.1)  # 10Hz update rate
            
#         except KeyboardInterrupt:
#             break
#         except Exception as e:
#             print(f"Error reading sensor: {e}")
#             time.sleep(0.1)

# # def print_all_capabilities():
# #     """Print all data types the MPU can provide"""
# #     print("\n" + "="*80)
# #     print("🚀 COMPLETE MPU SENSOR CAPABILITIES")
# #     print("="*80)
    
# #     print("\n📊 BASIC SENSOR DATA:")
# #     print("  • Accelerometer (X, Y, Z) - Linear acceleration in g units")
# #     print("  • Gyroscope (X, Y, Z) - Angular velocity in degrees/second") 
# #     print("  • Temperature - Internal sensor temperature in Celsius")
# #     print("  • Magnetometer (X, Y, Z) - Magnetic field in microtesla (MPU9250 only)")
    
# #     print("\n📐 CALCULATED ORIENTATIONS:")
# #     print("  • Roll angle - Rotation around X-axis")
# #     print("  • Pitch angle - Rotation around Y-axis") 
# #     print("  • Tilt angle - Total tilt from vertical")
# #     print("  • Compass heading - Direction in degrees (0-360°)")
# #     print("  • Tilt-compensated heading - Accurate compass even when tilted")
    
# #     print("\n📏 MAGNITUDE CALCULATIONS:")
# #     print("  • Acceleration magnitude - Total acceleration vector length")
# #     print("  • Gyroscope magnitude - Total rotation speed")
# #     print("  • Magnetic field magnitude - Total magnetic field strength")
    
# #     print("\n🔍 MOTION DETECTION:")
# #     print("  • Motion detection - Device is moving/rotating")
# #     print("  • Freefall detection - Device is falling freely")
# #     print("  • Tap detection - Sudden acceleration spike")
# #     print("  • Orientation detection - Face up/down, tilted left/right, etc.")
    
# #     print("\n🔢 RAW DATA:")
# #     print("  • Raw accelerometer values - Unprocessed ADC readings")
# #     print("  • Raw gyroscope values - Unprocessed ADC readings")  
# #     print("  • Raw magnetometer values - Unprocessed ADC readings")
# #     print("  • Raw temperature value - Unprocessed temperature ADC")
    
# #     print("\n⚖️  BIAS & CALIBRATION DATA:")
# #     print("  • X, Y, Z axis bias values - Offset corrections for each axis")
# #     print("  • X, Y, Z scale factors - Sensitivity corrections for each axis")
# #     print("  • Hard iron bias - Magnetometer offset corrections")
# #     print("  • Soft iron bias - Magnetometer scaling corrections")
# #     print("  • Temperature compensation - Bias drift correction")
    
# #     print("\n🧭 AXIS MAPPING & ORIENTATION:")
# #     print("  • Axis remapping - Custom X/Y/Z axis assignments")
# #     print("  • Sign corrections - Flip axis directions (+/-)")
# #     print("  • Coordinate system transformation - Convert between reference frames")
# #     print("  • Sensor mounting orientation - Account for physical installation")
    
# #     print("\n🔧 ADVANCED BIAS FEATURES:")
# #     print("  • Individual axis sensitivity measurement")
# #     print("  • Cross-axis sensitivity corrections")
# #     print("  • Temperature-dependent bias tracking")
# #     print("  • Automatic bias updating during operation")
# #     print("  • Multi-point calibration (6-position for accelerometer)")
    
# #     print("\n🧮 ADVANCED CALCULATIONS:")
# #     print("  • Rotation matrices - 3D orientation representation")
# #     print("  • Quaternions - Alternative orientation representation (can be added)")
# #     print("  • Kalman filtering - Sensor fusion for better accuracy (can be added)")
# #     print("  • Dead reckoning - Position tracking over time (can be added)")
# #     print("  • Gesture recognition - Pattern detection in motion (can be added)")
    
# #     print("\n⚙️ CONFIGURATION DATA:")
# #     print("  • Device ID - Sensor identification")
# #     print("  • Calibration offsets - Bias correction values")
# #     print("  • Sample rates - Data acquisition frequency")
# #     print("  • Scale factors - Conversion from raw to physical units")
    
# #     print("\n🎯 PRACTICAL APPLICATIONS:")
# #     print("  • Drone/quadcopter stabilization")
# #     print("  • Smartphone screen rotation")
# #     print("  • Gaming controllers")
# #     print("  • Activity trackers")
# #     print("  • Robotics navigation")
# #     print("  • Virtual/Augmented Reality")
# #     print("  • Industrial vibration monitoring")
# #     print("  • Earthquake detection")
    
# #     print("="*80)

# # Test bias and axis features
# def test_bias_and_axis_features(mpu):
#     """Test bias, calibration and axis mapping features"""
#     print("\n🔬 TESTING BIAS AND AXIS FEATURES:")
#     print("="*60)
    
#     # Show current bias values
#     axis_info = mpu.get_axis_info()
    
#     print("📊 CURRENT BIAS VALUES:")
#     print(f"   Accelerometer: X={axis_info['accel_bias']['x']:6d}, Y={axis_info['accel_bias']['y']:6d}, Z={axis_info['accel_bias']['z']:6d}")
#     print(f"   Gyroscope:     X={axis_info['gyro_bias']['x']:6d}, Y={axis_info['gyro_bias']['y']:6d}, Z={axis_info['gyro_bias']['z']:6d}")
#     if mpu.has_magnetometer:
#         print(f"   Magnetometer:  X={axis_info['mag_bias']['x']:6.1f}, Y={axis_info['mag_bias']['y']:6.1f}, Z={axis_info['mag_bias']['z']:6.1f}")
    
#     print("\n📏 CURRENT SCALE FACTORS:")
#     print(f"   Accelerometer: X={axis_info['accel_scale']['x']:.3f}, Y={axis_info['accel_scale']['y']:.3f}, Z={axis_info['accel_scale']['z']:.3f}")
#     print(f"   Gyroscope:     X={axis_info['gyro_scale']['x']:.3f}, Y={axis_info['gyro_scale']['y']:.3f}, Z={axis_info['gyro_scale']['z']:.3f}")
#     if mpu.has_magnetometer:
#         print(f"   Magnetometer:  X={axis_info['mag_scale']['x']:.3f}, Y={axis_info['mag_scale']['y']:.3f}, Z={axis_info['mag_scale']['z']:.3f}")
    
#     print("\n🧭 CURRENT AXIS MAPPING:")
#     print(f"   Logical X -> Physical axis {axis_info['axis_mapping']['x']} (sign: {'+' if axis_info['axis_signs']['x']>0 else '-'})")
#     print(f"   Logical Y -> Physical axis {axis_info['axis_mapping']['y']} (sign: {'+' if axis_info['axis_signs']['y']>0 else '-'})")
#     print(f"   Logical Z -> Physical axis {axis_info['axis_mapping']['z']} (sign: {'+' if axis_info['axis_signs']['z']>0 else '-'})")
    
#     # Compare raw vs corrected vs mapped readings
#     print("\n📈 DATA COMPARISON (Raw vs Corrected vs Mapped):")
#     for i in range(5):
#         # Raw data
#         ax_raw, ay_raw, az_raw = mpu.read_accel_raw()
#         gx_raw, gy_raw, gz_raw = mpu.read_gyro_raw()
        
#         # Bias-corrected data
#         ax_corr, ay_corr, az_corr = mpu.read_accel()
#         gx_corr, gy_corr, gz_corr = mpu.read_gyro()
        
#         # Axis-mapped data
#         ax_mapped, ay_mapped, az_mapped = mpu.read_accel_mapped()
#         gx_mapped, gy_mapped, gz_mapped = mpu.read_gyro_mapped()
        
#         print(f"\n   Sample {i+1}:")
#         print(f"      Raw Accel:    ({ax_raw:6d}, {ay_raw:6d}, {az_raw:6d})")
#         print(f"      Corrected:    ({ax_corr:6.3f}, {ay_corr:6.3f}, {az_corr:6.3f}) g")
#         print(f"      Mapped:       ({ax_mapped:6.3f}, {ay_mapped:6.3f}, {az_mapped:6.3f}) g")
#         print(f"      Raw Gyro:     ({gx_raw:6d}, {gy_raw:6d}, {gz_raw:6d})")
#         print(f"      Corrected:    ({gx_corr:6.2f}, {gy_corr:6.2f}, {gz_corr:6.2f}) °/s")
#         print(f"      Mapped:       ({gx_mapped:6.2f}, {gy_mapped:6.2f}, {gz_mapped:6.2f}) °/s")
        
#         time.sleep(0.5)

# def demo_axis_remapping(mpu):
#     """Demonstrate axis remapping functionality"""
#     print("\n🔄 AXIS REMAPPING DEMONSTRATION:")
#     print("="*50)
    
#     print("📍 Original axis mapping (default):")
#     ax, ay, az = mpu.read_accel()
#     print(f"   Accel: X={ax:6.3f}, Y={ay:6.3f}, Z={az:6.3f}")
    
#     # Remap axes (swap X and Y, invert Z)
#     print("\n🔄 Remapping: X->Y, Y->X, Z->-Z")
#     mpu.set_axis_mapping(x_axis=1, y_axis=0, z_axis=2, x_sign=1, y_sign=1, z_sign=-1)
    
#     ax_new, ay_new, az_new = mpu.read_accel_mapped()
#     print(f"   Mapped Accel: X={ax_new:6.3f}, Y={ay_new:6.3f}, Z={az_new:6.3f}")
    
#     # Reset to default
#     print("\n↩️  Resetting to default mapping")
#     mpu.set_axis_mapping()  # Reset to default
    
# def full_calibration_suite(mpu):
#     """Complete calibration suite for all sensors"""
#     print("\n🎯 FULL CALIBRATION SUITE:")
#     print("="*50)
    
#     print("This will calibrate all sensors with bias correction...")
    
#     # Calibrate gyroscope
#     print("\n1️⃣ GYROSCOPE CALIBRATION:")
#     mpu.calibrate_gyro(500)
    
#     # Calibrate accelerometer  
#     print("\n2️⃣ ACCELEROMETER CALIBRATION:")
#     mpu.calibrate_accel(500)
    
#     # Calibrate magnetometer if available
#     if mpu.has_magnetometer:
#         print("\n3️⃣ MAGNETOMETER CALIBRATION:")
#         mpu.calibrate_mag(300)
    
#     # Measure axis sensitivity
#     print("\n4️⃣ AXIS SENSITIVITY MEASUREMENT:")
#     sensitivities = mpu.measure_axis_sensitivity()
    
#     # Show final calibration results
#     print("\n✅ CALIBRATION COMPLETE!")
#     axis_info = mpu.get_axis_info()
    
#     print("\n📊 FINAL BIAS VALUES:")
#     print(f"   Accelerometer: X={axis_info['accel_bias']['x']:6d}, Y={axis_info['accel_bias']['y']:6d}, Z={axis_info['accel_bias']['z']:6d}")
#     print(f"   Gyroscope:     X={axis_info['gyro_bias']['x']:6d}, Y={axis_info['gyro_bias']['y']:6d}, Z={axis_info['gyro_bias']['z']:6d}")
#     if mpu.has_magnetometer:
#         print(f"   Magnetometer:  X={axis_info['mag_bias']['x']:6.1f}, Y={axis_info['mag_bias']['y']:6.1f}, Z={axis_info['mag_bias']['z']:6.1f}")
    
#     return axis_info

# def main():
#     """Main function to demonstrate ALL MPU capabilities including bias and axis features"""
#     print("🚀 MPU6050/9250 COMPLETE INTERFACE with BIAS & AXIS CONTROL")
#     print("=" * 70)
    
#     # Show all capabilities first
#     # print_all_capabilities()
    
#     # Setup sensor
#     mpu = setup_mpu()
#     if mpu is None:
#         return
    
#     print(f"\n✅ Device detected: {mpu.device_type}")
#     print(f"🧭 Magnetometer available: {mpu.has_magnetometer}")
    
#     # Test bias and axis features
#     test_bias_and_axis_features(mpu)
    
#     # Demonstrate axis remapping
#     demo_axis_remapping(mpu)
    
#     print("\n" + "="*70)
#     print("📊 DATA DEMONSTRATION MODES:")
#     print("1. Simple mode (basic data like your original example)")
#     print("2. Comprehensive mode (ALL available data + bias + axis info)")
#     print("3. Calibration suite (full sensor calibration)")
#     print("="*70)
    
#     # Demo simple mode first (like user's example)
#     print("\n🔹 SIMPLE MODE (5 seconds):")
#     print_sensor_data(mpu, duration=5, show_all=False)
    
#     print("\n" + "="*70)
#     print("🔸 COMPREHENSIVE MODE (15 seconds):")
#     print("This shows ALL data including bias, axis mapping, and raw values!")
#     print("="*70)
    
#     # Demo comprehensive mode with all bias and axis data
#     print_sensor_data(mpu, duration=15, show_all=True)
    
#     # Optional full calibration (uncomment to run)
#     # print("\n" + "="*70)
#     # print("🎯 FULL CALIBRATION SUITE:")
#     # print("Uncomment this section to run complete calibration")
#     # print("="*70)
#     # full_calibration_suite(mpu)
    
#     print("\n🎉 COMPLETE DEMONSTRATION FINISHED!")
#     print("\n📋 SUMMARY - The MPU provides:")
#     print("   ✅ Basic sensor data (accel, gyro, temp, mag)")
#     print("   ✅ Raw ADC values for all sensors")
#     print("   ✅ Individual X, Y, Z bias corrections")
#     print("   ✅ Scale factor corrections per axis")
#     print("   ✅ Axis remapping and sign corrections")
#     print("   ✅ Advanced calibration routines")
#     print("   ✅ Motion detection and orientation")
#     print("   ✅ Compass heading with tilt compensation")
#     print("   ✅ Complete axis sensitivity analysis")
#     print("\n🔥 This is EVERYTHING the MPU can provide!")

# # Test specific features
# def test_advanced_features(mpu):
#     """Test advanced MPU features including bias data"""
#     print("\n🧪 TESTING ADVANCED FEATURES + BIAS DATA:")
#     print("="*50)
    
#     data = mpu.get_comprehensive_data()
    
#     print("🔍 Motion Detection Test:")
#     print(f"  - Motion detected: {data['motion_detected']}")
#     print(f"  - Freefall detected: {data['freefall_detected']}")  
#     print(f"  - Tap detected: {data['tap_detected']}")
    
#     print(f"\n📱 Orientation: {data['orientation']}")
    
#     if mpu.has_magnetometer:
#         print(f"\n🧭 Compass Data:")
#         print(f"  - Simple heading: {data['heading']:.1f}°")
#         print(f"  - Tilt-compensated: {data['tilt_compensated_heading']:.1f}°")
    
#     print(f"\n📐 All Angles:")
#     print(f"  - Roll: {data['roll']:.2f}°")
#     print(f"  - Pitch: {data['pitch']:.2f}°") 
#     print(f"  - Tilt from vertical: {data['tilt_angle']:.2f}°")
    
#     # Show bias information
#     axis_info = data['axis_info']
#     print(f"\n⚖️  Current Bias Values:")
#     print(f"  - Accel bias: X={axis_info['accel_bias']['x']:6d}, Y={axis_info['accel_bias']['y']:6d}, Z={axis_info['accel_bias']['z']:6d}")
#     print(f"  - Gyro bias:  X={axis_info['gyro_bias']['x']:6d}, Y={axis_info['gyro_bias']['y']:6d}, Z={axis_info['gyro_bias']['z']:6d}")
    
#     if mpu.has_magnetometer:
#         print(f"  - Mag bias:   X={axis_info['mag_bias']['x']:6.1f}, Y={axis_info['mag_bias']['y']:6.1f}, Z={axis_info['mag_bias']['z']:6.1f}")
    
#     print(f"\n📏 Scale Factors:")
#     print(f"  - Accel scale: X={axis_info['accel_scale']['x']:.3f}, Y={axis_info['accel_scale']['y']:.3f}, Z={axis_info['accel_scale']['z']:.3f}")
#     print(f"  - Gyro scale:  X={axis_info['gyro_scale']['x']:.3f}, Y={axis_info['gyro_scale']['y']:.3f}, Z={axis_info['gyro_scale']['z']:.3f}")

# # Quick demonstration function
# def quick_demo():
#     """Quick demo showing basic vs comprehensive data"""
#     mpu = setup_mpu()
#     if mpu is None:
#         return
    
#     print("\n" + "="*60)
#     print("⚡ QUICK COMPARISON: Your Basic Data vs Full Capabilities")
#     print("="*60)
    
#     # Your basic data (what you originally saw)
#     print("\n📱 YOUR ORIGINAL DATA:")
#     print("-" * 30)
#     ax, ay, az = mpu.read_accel()
#     gx, gy, gz = mpu.read_gyro()
#     temp = mpu.read_temp()
#     roll, pitch = mpu.calculate_angles()
    
#     print(f"Accel (g):  X={ax:7.3f}  Y={ay:7.3f}  Z={az:7.3f}")
#     print(f"Gyro (°/s): X={gx:7.2f}  Y={gy:7.2f}  Z={gz:7.2f}")
#     print(f"Temp (°C):  {temp:.2f}")
#     print(f"Angles (°): Roll={roll:6.2f}  Pitch={pitch:6.2f}")
    
#     # Full comprehensive data
#     print("\n🔥 COMPLETE DATA AVAILABLE:")
#     print("-" * 30)
#     data = mpu.get_comprehensive_data()
    
#     # Raw values
#     ax_raw, ay_raw, az_raw = data['accel_raw']
#     gx_raw, gy_raw, gz_raw = data['gyro_raw']
#     print(f"Raw Accel:     ({ax_raw:6d}, {ay_raw:6d}, {az_raw:6d})")
#     print(f"Raw Gyro:      ({gx_raw:6d}, {gy_raw:6d}, {gz_raw:6d})")
#     print(f"Raw Temp:      {data['temp_raw']:5d}")
    
#     # Bias values
#     axis_info = data['axis_info']
#     print(f"Accel Bias:    X={axis_info['accel_bias']['x']:6d}, Y={axis_info['accel_bias']['y']:6d}, Z={axis_info['accel_bias']['z']:6d}")
#     print(f"Gyro Bias:     X={axis_info['gyro_bias']['x']:6d}, Y={axis_info['gyro_bias']['y']:6d}, Z={axis_info['gyro_bias']['z']:6d}")
    
#     # Magnitudes
#     print(f"Accel Mag:     {data['accel_magnitude']:.3f} g")
#     print(f"Gyro Mag:      {data['gyro_magnitude']:.2f} °/s")
#     print(f"Tilt Angle:    {data['tilt_angle']:.2f}°")
#     print(f"Orientation:   {data['orientation']}")
    
#     # Motion status
#     status = []
#     if data['motion_detected']: status.append("MOTION")
#     if data['freefall_detected']: status.append("FREEFALL") 
#     if data['tap_detected']: status.append("TAP")
#     print(f"Status:        {' | '.join(status) if status else 'STATIONARY'}")
    
#     if mpu.has_magnetometer:
#         mx_raw, my_raw, mz_raw = data['mag_raw']
#         if mx_raw is not None:
#             print(f"Raw Mag:       ({mx_raw:6d}, {my_raw:6d}, {mz_raw:6d})")
#             print(f"Mag Bias:      X={axis_info['mag_bias']['x']:6.1f}, Y={axis_info['mag_bias']['y']:6.1f}, Z={axis_info['mag_bias']['z']:6.1f}")
#             print(f"Heading:       {data['heading']:.1f}°")
     
#     print(f"\n💡 TOTAL: {len([k for k in data.keys() if not k.startswith('axis')])} different data points available!")
#     print("   vs your original 7 data points (3 accel + 3 gyro + 1 temp)")

# # Run the main function if this file is executed directly
# if __name__ == "__main__":
#     # Uncomment the line you want to run:
#     main()           # Full demonstration
#     # quick_demo()   # Quick comparison