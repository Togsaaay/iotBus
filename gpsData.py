# from machine import UART
# import utime, time

# # Initialize UART for GPS
# gpsModule = UART(2, baudrate=9600)
# print("GPS Module Initialized:", gpsModule)

# # Data storage
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
#     'raw_sentences': []
# }

# def safe_convert(value, converter, default):
#     try:
#         return converter(value) if value else default
#     except:
#         return default

# def parse_nmea(sentence):
#     if not sentence.startswith('$'):
#         return
    
#     parts = sentence.split(',')
#     sentence_type = parts[0][1:]
    
#     # GGA - Global Positioning System Fix Data
#     if sentence_type == 'GPGGA':
#         gps_data['message_type'] = 'GPGGA'
#         gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}" if len(parts[1]) >= 6 else ''
#         gps_data['latitude'] = convert_coord(parts[2], parts[3]) if parts[2] else '0.0'
#         gps_data['longitude'] = convert_coord(parts[4], parts[5]) if parts[4] else '0.0'
#         gps_data['fix_quality'] = parts[6]
#         gps_data['satellites'] = parts[7]
#         gps_data['hdop'] = parts[8]
#         gps_data['altitude'] = f"{safe_convert(parts[9], float, 0.0):.1f} m"
#         gps_data['fix_status'] = "Active" if parts[6] != '0' else "No Fix"
    
#     # RMC - Recommended Minimum Navigation Information
#     elif sentence_type == 'GPRMC':
#         gps_data['message_type'] = 'GPRMC'
#         if len(parts[1]) >= 6:
#             gps_data['utc_time'] = f"{parts[1][0:2]}:{parts[1][2:4]}:{parts[1][4:6]}"
#         if len(parts[9]) >= 6:
#             gps_data['date'] = f"20{parts[9][4:6]}-{parts[9][2:4]}-{parts[9][0:2]}"
#         gps_data['speed'] = f"{safe_convert(parts[7], float, 0.0) * 1.852:.1f} km/h"  # Convert knots to km/h
#         gps_data['course'] = f"{safe_convert(parts[8], float, 0.0):.1f}°"
#         gps_data['mode'] = parts[12][0] if len(parts) > 12 else ''
    
#     # GSV - Satellites in View
#     elif sentence_type == 'GPGSV':
#         total_sats = safe_convert(parts[3], int, 0)
#         gps_data['satellites'] = str(total_sats)
        
#         # Each GSV message contains up to 4 satellites
#         for i in range(4):
#             idx = 4 + i*4
#             if len(parts) > idx + 3:
#                 svid = parts[idx]
#                 if svid:
#                     gps_data['satellite_info'][svid] = {
#                         'elevation': f"{safe_convert(parts[idx+1], int, 0)}°",
#                         'azimuth': f"{safe_convert(parts[idx+2], int, 0)}°",
#                         'snr': f"{safe_convert(parts[idx+3].split('*')[0], int, 0)} dBHz"
#                     }
    
#     # GSA - GPS DOP and Active Satellites
#     elif sentence_type == 'GPGSA':
#         gps_data['mode'] = 'Auto' if parts[1] == 'A' else 'Manual'
#         gps_data['fix_type'] = 'No fix' if parts[2] == '1' else ('2D' if parts[2] == '2' else '3D')
#         if len(parts) > 15: gps_data['pdop'] = parts[15]
#         if len(parts) > 16: gps_data['hdop'] = parts[16]
#         if len(parts) > 17: gps_data['vdop'] = parts[17].split('*')[0]
    
#     # VTG - Course Over Ground and Ground Speed
#     elif sentence_type == 'GPVTG':
#         gps_data['course'] = f"{safe_convert(parts[1], float, 0.0):.1f}°"
#         gps_data['speed'] = f"{safe_convert(parts[7], float, 0.0):.1f} km/h"
    
#     # Store raw sentence
#     gps_data['raw_sentences'].append(sentence)

# def convert_coord(coord, direction):
#     try:
#         val = float(coord)
#         degrees = int(val / 100)
#         minutes = val - degrees * 100
#         decimal = degrees + minutes / 60
#         if direction in ['S', 'W']:
#             decimal = -decimal
#         return f"{decimal:.6f}°"
#     except:
#         return "0.0°"

# def display_gps_data():
#     print("\n" + "="*40)
#     print("=== COMPLETE GPS INFORMATION ===")
#     print("="*40)
    
#     # Basic Info
#     print(f"\n[Time/Date]")
#     print(f"UTC Time: {gps_data['utc_time']}")
#     print(f"Date: {gps_data['date']}")
    
#     # Position Info
#     print(f"\n[Position]")
#     print(f"Latitude: {gps_data['latitude']}")
#     print(f"Longitude: {gps_data['longitude']}")
#     print(f"Altitude: {gps_data['altitude']}")
    
#     # Movement Info
#     print(f"\n[Movement]")
#     print(f"Speed: {gps_data['speed']}")
#     print(f"Course: {gps_data['course']}")
    
#     # Fix Info
#     print(f"\n[Fix Information]")
#     print(f"Status: {gps_data['fix_status']}")
#     print(f"Quality: {gps_data['fix_quality']}")
#     print(f"Mode: {gps_data['mode']}")
#     print(f"Satellites: {gps_data['satellites']}")
    
#     # DOP Values
#     print(f"\n[Dilution of Precision]")
#     print(f"HDOP: {gps_data['hdop']} (Horizontal)")
#     print(f"VDOP: {gps_data['vdop']} (Vertical)")
#     print(f"PDOP: {gps_data['pdop']} (Position)")
    
#     # Satellite Details
#     if gps_data['satellite_info']:
#         print(f"\n[Satellite Details] (SNR > 0)")
#         print("PRN  Elevation Azimuth  SNR")
#         for svid, info in sorted(gps_data['satellite_info'].items(), key=lambda x: int(x[0])):
#             if int(info['snr'].split()[0]) > 0:
#                 print(f"{svid:>3}  {info['elevation']:>8}  {info['azimuth']:>7}  {info['snr']}")
    
#     # Raw Data
#     print(f"\n[Last NMEA Sentences]")
#     for sentence in gps_data['raw_sentences'][-3:]:  # Show last 3 sentences
#         print(sentence)
    
#     print("="*40 + "\n")

# # Main loop
# while True:
#     line = gpsModule.readline()
#     if line:
#         try:
#             sentence = line.decode('ascii').strip()
#             parse_nmea(sentence)
#             if gps_data['fix_status'] == "Active":
#                 display_gps_data()
#                 utime.sleep(20)  # Update every second when fix is active
#             else:
#                 print("Searching for satellites...", end='\r')
#         except UnicodeError:
#             continue
#         except Exception as e:
#             print(f"Error: {e}")
    
#     utime.sleep_ms(100)