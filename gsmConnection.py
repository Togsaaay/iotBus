# from machine import UART, Pin
# import time
# import re

# class SIM900A:
#     def __init__(self, uart_id=2, tx_pin=0, rx_pin=2, baud=9600):
#         """
#         Initialize SIM900A module
#         Default pins: TX=17, RX=16 (you can change these)
#         """
#         self.uart = UART(uart_id, baudrate=baud, tx=tx_pin, rx=rx_pin)
#         self.uart.init(baud, bits=8, parity=None, stop=1)
#         time.sleep(2)
        
#     def send_at_command(self, command, expected_response="OK", timeout=10):
#         """Send AT command and wait for response"""
#         print("Sending: " + command)
#         self.uart.write((command + '\r\n').encode())
        
#         start_time = time.time()
#         response = ""
        
#         while (time.time() - start_time) < timeout:
#             if self.uart.any():
#                 try:
#                     data = self.uart.read()
#                     if data:
#                         response += data.decode('utf-8')
#                 except:
#                     # Skip any decode issues (including invalid UTF-8)
#                     pass
                    
#                 if expected_response in response:
#                     print("Response: " + response.strip())
#                     return True, response
#             time.sleep(0.1)
        
#         print("Timeout or unexpected response: " + response.strip())
#         return False, response
    
#     def check_module(self):
#         """Check if SIM900A is responding"""
#         print("Checking SIM900A module...")
        
#         # First try basic AT command
#         success, response = self.send_at_command("AT")
#         if not success:
#             return False
            
#         # Check module information
#         print("Getting module info...")
#         self.send_at_command("ATI", "OK", 5)
        
#         # Set text mode for better compatibility
#         self.send_at_command("AT+CMGF=1", "OK", 5)
        
#         # Disable echo if needed
#         self.send_at_command("ATE0", "OK", 5)
        
#         return True
    
#     def check_sim_card(self):
#         """Check SIM card status"""
#         print("Checking SIM card...")
#         success, response = self.send_at_command("AT+CPIN?", "READY", 15)
#         if not success:
#             print("SIM card not ready or not inserted")
#         return success
    
#     def check_network(self):
#         """Check network registration"""
#         print("Checking network registration...")
#         success, response = self.send_at_command("AT+CREG?", "OK", 15)
        
#         if success:
#             # Look for network registration status
#             # Status codes: 0=not searching, 1=registered home, 2=searching, 3=denied, 5=registered roaming
#             # Format can be: +CREG: n,stat or +CREG: n,stat,"lac","ci"
#             if ",1" in response or ",5" in response:  # Registered (home or roaming)
#                 if ",1" in response:
#                     print("Connected to home network")
#                 else:
#                     print("Connected to roaming network")
#                 return True
#             elif ",2" in response:
#                 print("Searching for network...")
#                 return False
#             elif ",3" in response:
#                 print("Network registration denied")
#                 return False
#             else:
#                 print("Network status unknown")
#                 return False
#         else:
#             print("Failed to check network status")
#             return False
    
#     def check_signal_strength(self):
#         """Check signal strength"""
#         print("Checking signal strength...")
#         success, response = self.send_at_command("AT+CSQ", "OK", 10)
#         if success:
#             # Extract signal strength from response
#             match = re.search(r'\+CSQ: (\d+),', response)
#             if match:
#                 rssi = int(match.group(1))
#                 if rssi == 99:
#                     print("No signal")
#                 else:
#                     print("Signal strength: " + str(rssi))
#         return success
    
#     def setup_gprs(self, apn="internet"):
#         """
#         Setup GPRS connection
#         Smart SIM Philippines typically uses "internet" as APN
#         You can also try "smartlte" or "smart" if "internet" doesn't work
#         """
#         print("Setting up GPRS connection...")
        
#         # Close any existing connection first (ERROR response is normal if no connection exists)
#         print("Closing any existing GPRS connection...")
#         self.send_at_command("AT+SAPBR=0,1", "OK", 5)
#         time.sleep(2)
        
#         # Set connection type to GPRS
#         print("Setting connection type to GPRS...")
#         success, _ = self.send_at_command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"")
#         if not success:
#             print("Failed to set connection type")
#             return False
        
#         # Set APN
#         print("Setting APN to: " + apn)
#         success, _ = self.send_at_command("AT+SAPBR=3,1,\"APN\",\"" + apn + "\"")
#         if not success:
#             print("Failed to set APN")
#             return False
        
#         # Smart SIM usually doesn't require username/password, but set empty ones just in case
#         self.send_at_command("AT+SAPBR=3,1,\"USER\",\"\"", "OK", 5)
#         self.send_at_command("AT+SAPBR=3,1,\"PWD\",\"\"", "OK", 5)
        
#         # Open GPRS context
#         print("Opening GPRS context... (this may take 10-30 seconds)")
#         success, response = self.send_at_command("AT+SAPBR=1,1", "OK", 45)
#         if not success:
#             print("Failed to open GPRS context")
#             print("Response was: " + response.strip())
            
#             # Try checking current status
#             print("Checking current GPRS status...")
#             status_success, status_response = self.send_at_command("AT+SAPBR=2,1", "OK", 10)
#             if status_success:
#                 print("Current status: " + status_response.strip())
            
#             return False
        
#         # Check if we got an IP address
#         print("Checking IP address...")
#         time.sleep(3)
#         success, response = self.send_at_command("AT+SAPBR=2,1", "OK", 10)
#         if success and "." in response:  # Look for IP address format
#             print("GPRS connection established!")
#             # Extract IP address
#             match = re.search(r'(\d+\.\d+\.\d+\.\d+)', response)
#             if match:
#                 ip = match.group(1)
#                 print("IP Address: " + ip)
#                 return True
#             else:
#                 print("Got response but no valid IP found: " + response.strip())
        
#         print("Failed to establish GPRS connection or get IP address")
#         return False
    
#     def test_internet_connection(self):
#         """Test internet connection by pinging Google"""
#         print("Testing internet connection...")
        
#         # Terminate any existing HTTP service first
#         print("Terminating any existing HTTP service...")
#         self.send_at_command("AT+HTTPTERM", "OK", 5)
#         time.sleep(1)
        
#         # Initialize HTTP service
#         print("Initializing HTTP service...")
#         success, _ = self.send_at_command("AT+HTTPINIT", "OK", 10)
#         if not success:
#             print("Failed to initialize HTTP service")
#             return False
        
#         # Set HTTP parameters
#         print("Setting HTTP parameters...")
#         success, _ = self.send_at_command("AT+HTTPPARA=\"CID\",1", "OK", 5)
#         if not success:
#             print("Failed to set CID parameter")
#             self.send_at_command("AT+HTTPTERM")
#             return False
        
#         success, _ = self.send_at_command("AT+HTTPPARA=\"URL\",\"http://httpbin.org/get\"", "OK", 10)
#         if not success:
#             print("Failed to set URL parameter")
#             self.send_at_command("AT+HTTPTERM")
#             return False
        
#         # Perform HTTP GET request
#         print("Performing HTTP GET request...")
#         success, response = self.send_at_command("AT+HTTPACTION=0", "OK", 5)
#         if not success:
#             print("Failed to start HTTP action")
#             self.send_at_command("AT+HTTPTERM")
#             return False
        
#         # Wait for HTTP response
#         print("Waiting for HTTP response...")
#         time.sleep(10)
        
#         # Read HTTP response
#         success, response = self.send_at_command("AT+HTTPREAD", "OK", 15)
#         if success:
#             print("HTTP Response received!")
#             print("Response preview: " + response[:200] + "..." if len(response) > 200 else response)
            
#             # Check if we got a valid HTTP response
#             if "200" in response or "HTTP" in response or "{" in response:
#                 print("Internet connection test SUCCESSFUL!")
#                 self.send_at_command("AT+HTTPTERM")
#                 return True
        
#         print("Internet connection test failed - no valid response received")
#         self.send_at_command("AT+HTTPTERM")
#         return False
    
#     def simple_connectivity_test(self):
#         """Simple test to verify GPRS connectivity"""
#         print("Performing simple connectivity test...")
        
#         # Check if we have an IP address
#         success, response = self.send_at_command("AT+SAPBR=2,1", "OK", 5)
#         if success and "." in response:
#             print("GPRS connection active with IP address")
            
#             # Try a simple ping-like test using AT+CIICR (if supported)
#             print("Testing basic connectivity...")
#             test_success, _ = self.send_at_command("AT+CGATT?", "OK", 10)
#             if test_success:
#                 print("Packet service attached - connectivity looks good!")
#                 return True
        
#         return False
    
#     def disconnect_gprs(self):
#         """Disconnect GPRS connection"""
#         print("Disconnecting GPRS...")
#         self.send_at_command("AT+SAPBR=0,1")
    
#     def connect_to_internet(self):
#         """Main function to connect to internet"""
#         print("Starting internet connection process...")
        
#         # Step 1: Check module
#         if not self.check_module():
#             print("ERROR: SIM900A module not responding")
#             return False
        
#         # Step 2: Check SIM card
#         if not self.check_sim_card():
#             print("ERROR: SIM card issue")
#             return False
        
#         # Step 3: Check signal strength
#         self.check_signal_strength()
        
#         # Step 4: Wait for network registration
#         print("Waiting for network registration...")
#         network_registered = False
#         for i in range(60):  # Wait up to 60 seconds
#             if self.check_network():
#                 network_registered = True
#                 break
#             print("Waiting for network... (" + str(i+1) + "/60)")
#             time.sleep(2)
        
#         if not network_registered:
#             print("ERROR: Failed to register to network after 60 seconds")
#             return False
        
#         # Step 5: Setup GPRS
#         if not self.setup_gprs():
#             print("ERROR: Failed to setup GPRS")
#             return False
        
#         # Step 6: Test internet connection
#         print("Testing connectivity...")
        
#         # First try simple connectivity test
#         if self.simple_connectivity_test():
#             print("Basic connectivity confirmed!")
            
#             # Try full HTTP test
#             if self.test_internet_connection():
#                 print("SUCCESS: Full internet connectivity confirmed!")
#                 return True
#             else:
#                 print("WARNING: GPRS connected but HTTP test failed")
#                 print("This might be due to firewall or HTTP service issues")
#                 print("Your GPRS connection is working - you can proceed with your application")
#                 return True  # Consider this a success since GPRS is working
#         else:
#             print("ERROR: Basic connectivity test failed")
#             return False

# # Main program
# def main():
#     # Initialize SIM900A (adjust pins according to your wiring)
#     sim = SIM900A(uart_id=2, tx_pin=0, rx_pin=2, baud=9600)
    
#     # Try to connect to internet
#     if sim.connect_to_internet():
#         print("\n=== Internet Connection Established ===")
#         print("You can now use the SIM900A for internet applications")
        
#         # Keep connection alive and monitor
#         try:
#             while True:
#                 print("Connection active... (Press Ctrl+C to disconnect)")
#                 time.sleep(30)
                
#                 # Optionally check connection status periodically
#                 sim.check_signal_strength()
                
#         except KeyboardInterrupt:
#             print("\nDisconnecting...")
#             sim.disconnect_gprs()
#             print("Disconnected.")
#     else:
#         print("\n=== Failed to Connect ===")
#         print("Please check:")
#         print("1. SIM card is inserted and active")
#         print("2. SIM card has data balance")
#         print("3. Antenna is connected")
#         print("4. Wiring is correct")
#         print("5. Power supply is adequate (SIM900A needs 2A)")

# # Run the program
# if __name__ == "__main__":
#     main()