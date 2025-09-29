import cv2
import mediapipe as mp
import numpy as np
import math
from collections import Counter
import time
import requests
import socket
import threading
import queue
import json
import yaml
import os

class HandGestureRecognizer:
    def __init__(self, config_path="config.yaml"):
        # Load configuration
        self.config = self.load_config(config_path)
        
        # Initialize MediaPipe with config settings
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=self.config['gesture_recognition']['max_num_hands'],
            min_detection_confidence=self.config['gesture_recognition']['min_detection_confidence'],
            min_tracking_confidence=self.config['gesture_recognition']['min_tracking_confidence']
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Gesture recognition parameters from config
        self.gesture_buffer = []
        self.buffer_size = self.config['gesture_recognition']['buffer_size']
        self.confidence_threshold = self.config['gesture_recognition']['confidence_threshold']
        
        # ESP32 wireless communication from config
        self.esp32_ip = self.config['esp32']['ip_address']
        self.esp32_port = self.config['esp32']['http_port']
        self.udp_port = self.config['esp32']['udp_port']
        self.local_udp_port = self.config['esp32']['local_udp_port']
        self.session = requests.Session()
        self.session.timeout = self.config['esp32']['connection_timeout']
        
        # UDP socket for faster gesture commands
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.settimeout(self.config['esp32']['udp_timeout'])
        
        # Setup UDP listener for heartbeats
        try:
            self.udp_socket.bind(('', self.local_udp_port))
            self.heartbeat_thread = threading.Thread(target=self.listen_for_heartbeats, daemon=True)
            self.heartbeat_thread.start()
        except Exception as e:
            print(f"⚠️ Could not bind UDP listener: {e}")
        
        self.command_queue = queue.Queue()
        self.last_command_time = 0
        self.command_cooldown = self.config['gesture_recognition']['cooldown_interval']
        self.gesture_recognition_cooldown = self.config['gesture_recognition']['cooldown_interval']
        self.last_gesture_recognition_time = 0
        self.connected = False
        self.last_heartbeat = 0
        
        # Initialize camera with config settings
        self.cap = cv2.VideoCapture(self.config['camera']['index'])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['camera']['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['camera']['height'])
        
        # Gesture to fan speed mapping from config
        self.gesture_commands = self.config['fan_commands']
        
        self.last_sent_gesture = None
        self.last_gesture_time = 0
        
        # Try to discover and connect to ESP32
        self.discover_esp32()
        
        print("🎮 Hand Gesture Fan Controller Initialized (Wireless)")
        print("📡 ESP32 Connection:", "✅ Connected" if self.connected else "❌ Not Connected")
        if self.esp32_ip:
            print(f"🌐 ESP32 IP: {self.esp32_ip}")
        print(f"\n⏱️ Gesture Recognition Interval: {self.gesture_recognition_cooldown} seconds")
        print("\n🎯 Gesture Commands:")
        for gesture, cmd in self.gesture_commands.items():
            print(f"   {cmd['emoji']} {gesture} - {cmd['description']} ({cmd['speed']}%)")
        print("\nPress 'q' to quit, 'r' to reconnect ESP32")

    def load_config(self, config_path):
        """Load configuration from YAML file"""
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as file:
                    config = yaml.safe_load(file)
                print(f"📄 Configuration loaded from {config_path}")
                return config
            else:
                print(f"⚠️ Config file {config_path} not found, using defaults")
                return self.get_default_config()
        except Exception as e:
            print(f"❌ Error loading config: {e}")
            print("🔄 Using default configuration")
            return self.get_default_config()

    def get_default_config(self):
        """Return default configuration if config file is not available"""
        return {
            'esp32': {
                'ip_address': "192.168.1.164",
                'http_port': 80,
                'udp_port': 4210,
                'local_udp_port': 4211,
                'connection_timeout': 2,
                'udp_timeout': 1,
                'discovery_timeout': 1
            },
            'camera': {
                'index': 0,
                'width': 1280,
                'height': 720,
                'flip_horizontal': True
            },
            'gesture_recognition': {
                'cooldown_interval': 2.0,
                'buffer_size': 8,
                'confidence_threshold': 0.7,
                'min_detection_confidence': 0.7,
                'min_tracking_confidence': 0.5,
                'max_num_hands': 1
            },
            'fan_commands': {
                "Rock": {"speed": 0, "description": "Turn OFF", "emoji": "✊"},
                "Index Up": {"speed": 100, "description": "Max Speed", "emoji": "👆"},
                "L-shape": {"speed": 50, "description": "Turn ON (50%)", "emoji": "🤟"},
                "Paper": {"speed": 75, "description": "High Speed", "emoji": "✋"},
                "Scissors": {"speed": 25, "description": "Low Speed", "emoji": "✌️"}
            },
            'display': {
                'window_title': "Wireless Gesture-Controlled Fan System",
                'main_panel_height': 180,
                'main_panel_width': 550,
                'commands_panel_height': 220,
                'commands_panel_width': 650,
                'colors': {
                    'background': [0, 0, 0],
                    'border': [255, 255, 255],
                    'text_primary': [255, 255, 255],
                    'text_secondary': [255, 255, 0],
                    'connected': [0, 255, 0],
                    'disconnected': [0, 0, 255],
                    'cooldown': [255, 165, 0],
                    'ready': [0, 255, 0]
                }
            },
            'monitoring': {
                'status_check_interval': 10,
                'heartbeat_threshold': 15,
                'verbose_logging': True,
                'show_heartbeats': True
            },
            'network': {
                'auto_discovery': True,
                'scan_start': 1,
                'scan_end': 254,
                'scan_threads': 10
            },
            'advanced': {
                'http_session_timeout': 2,
                'max_retries': 3,
                'clear_buffer_on_cooldown': True,
                'auto_reconnect': True
            }
        }

    def discover_esp32(self):
        """Auto-discover ESP32 on the local network"""
        print("🔍 Connecting to ESP32...")
        
        # Try known ESP32 IP first if configured
        if self.esp32_ip and self.esp32_ip != "null":
            print(f"📡 Testing configured ESP32 IP: {self.esp32_ip}")
            if self.test_esp32_connection(self.esp32_ip):
                self.connected = True
                print(f"✅ Connected to ESP32 at {self.esp32_ip}")
                return True
        
        # Auto-discovery if enabled and known IP failed
        if self.config['network']['auto_discovery']:
            print("⚠️ Known IP not responding, scanning network...")
            
            # Get local network base for scanning
            try:
                # Get local IP to determine network range
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
                s.close()
                
                network_base = '.'.join(local_ip.split('.')[:-1]) + '.'
                scan_start = self.config['network']['scan_start']
                scan_end = self.config['network']['scan_end']
                print(f"📡 Scanning network: {network_base}{scan_start}-{scan_end}")
                
                # Scan IP range
                for i in range(scan_start, scan_end + 1):
                    test_ip = f"{network_base}{i}"
                    if self.test_esp32_connection(test_ip):
                        self.esp32_ip = test_ip
                        self.connected = True
                        print(f"✅ Found ESP32 at {test_ip}")
                        return True
                
                print("❌ ESP32 not found on network")
                return False
                
            except Exception as e:
                print(f"❌ Network scan failed: {e}")
                return False
        else:
            print("❌ Auto-discovery disabled in config")
            return False

    def test_esp32_connection(self, ip, timeout=None):
        """Test if ESP32 is accessible at given IP"""
        if timeout is None:
            timeout = self.config['esp32']['discovery_timeout']
        
        try:
            response = requests.get(f"http://{ip}/status", timeout=timeout)
            if response.status_code == 200:
                data = response.json()
                if 'fanSpeed' in data:  # Verify it's our ESP32
                    return True
        except:
            pass
        return False

    def listen_for_heartbeats(self):
        """Listen for UDP heartbeats from ESP32"""
        while True:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                message = data.decode()
                if message.startswith("HEARTBEAT:"):
                    self.last_heartbeat = time.time()
                    parts = message.split(':')
                    if len(parts) >= 3:
                        fan_speed = parts[1]
                        status = parts[2]
                        if self.config['monitoring']['show_heartbeats']:
                            print(f"💓 ESP32 Heartbeat: Speed {fan_speed}%, Status: {status}")
            except socket.timeout:
                continue
            except Exception as e:
                if self.config['monitoring']['verbose_logging']:
                    print(f"⚠️ Heartbeat error: {e}")
                time.sleep(1)

    def connect_to_esp32(self):
        """Try to connect to ESP32 via WiFi"""
        return self.discover_esp32()

    def send_gesture_command(self, gesture):
        """Send gesture command to ESP32 via WiFi"""
        if not self.connected or not self.esp32_ip:
            return False
            
        current_time = time.time()
        
        # Prevent spam commands
        if (current_time - self.last_command_time < self.command_cooldown and 
            gesture == self.last_sent_gesture):
            return False
            
        try:
            if gesture in self.gesture_commands:
                speed = self.gesture_commands[gesture]["speed"]
                
                # Try UDP first (faster)
                try:
                    udp_message = f"GESTURE:{speed}"
                    self.udp_socket.sendto(udp_message.encode(), (self.esp32_ip, self.udp_port))
                    
                    # Wait for UDP acknowledgment
                    self.udp_socket.settimeout(1)
                    ack_data, addr = self.udp_socket.recvfrom(1024)
                    ack_message = ack_data.decode()
                    
                    if ack_message.startswith("ACK:"):
                        print(f"📤 UDP Sent: {gesture} → {speed}% (Acknowledged)")
                        self.last_sent_gesture = gesture
                        self.last_command_time = current_time
                        return True
                        
                except socket.timeout:
                    print("⚠️ UDP timeout, trying HTTP...")
                
                # Fallback to HTTP
                payload = {
                    "gesture": gesture,
                    "speed": speed,
                    "timestamp": current_time
                }
                
                response = self.session.post(
                    f"http://{self.esp32_ip}/gesture",
                    json=payload,
                    headers={'Content-Type': 'application/json'}
                )
                
                if response.status_code == 200:
                    result = response.json()
                    print(f"📤 HTTP Sent: {gesture} → {speed}% (Status: {result.get('status', 'unknown')})")
                    self.last_sent_gesture = gesture
                    self.last_command_time = current_time
                    return True
                else:
                    print(f"❌ HTTP Error: {response.status_code}")
                    
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            self.connected = False
            
        return False

    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def is_finger_extended(self, landmarks, finger_tip, finger_pip, finger_mcp=None):
        """Check if a finger is extended based on landmark positions"""
        if finger_mcp is None:
            return landmarks[finger_tip].y < landmarks[finger_pip].y
        else:
            return (landmarks[finger_tip].y < landmarks[finger_pip].y and 
                   landmarks[finger_pip].y < landmarks[finger_mcp].y)

    def classify_gesture(self, landmarks):
        """Classify hand gesture based on landmark positions"""
        # Landmark indices for fingers
        thumb_tip, thumb_ip, thumb_mcp = 4, 3, 2
        index_tip, index_pip, index_mcp = 8, 6, 5
        middle_tip, middle_pip, middle_mcp = 12, 10, 9
        ring_tip, ring_pip, ring_mcp = 16, 14, 13
        pinky_tip, pinky_pip, pinky_mcp = 20, 18, 17
        
        # Check which fingers are extended
        fingers_up = []
        
        # Thumb (special case - check x coordinate)
        if landmarks[thumb_tip].x > landmarks[thumb_ip].x:
            fingers_up.append(1)
        else:
            fingers_up.append(0)
            
        # Other four fingers
        finger_landmarks = [
            (index_tip, index_pip, index_mcp),
            (middle_tip, middle_pip, middle_mcp),
            (ring_tip, ring_pip, ring_mcp),
            (pinky_tip, pinky_pip, pinky_mcp)
        ]
        
        for tip, pip, mcp in finger_landmarks:
            if self.is_finger_extended(landmarks, tip, pip, mcp):
                fingers_up.append(1)
            else:
                fingers_up.append(0)
        
        # Gesture classification based on finger positions
        total_fingers = sum(fingers_up)
        
        # Rock - all fingers closed
        if total_fingers == 0:
            return "Rock", 0.9
        
        # Paper - all fingers extended
        elif total_fingers == 5:
            return "Paper", 0.9
        
        # Scissors - index and middle finger extended
        elif (fingers_up == [0, 1, 1, 0, 0]):
            return "Scissors", 0.9
        
        # Index Up - only index finger extended
        elif fingers_up == [0, 1, 0, 0, 0]:
            return "Index Up", 0.9
        
        # L-shape - thumb and index finger extended
        elif fingers_up == [1, 1, 0, 0, 0]:
            return "L-shape", 0.9
        
        # Additional checks with slight variations
        elif (total_fingers == 1 and fingers_up[1] == 1):
            return "Index Up", 0.8
        
        elif (total_fingers == 2 and fingers_up[0] == 1 and fingers_up[1] == 1):
            return "L-shape", 0.8
        
        # Additional check for Scissors variations
        elif (total_fingers == 2 and fingers_up[1] == 1 and fingers_up[2] == 1):
            return "Scissors", 0.8
        
        # Unknown gesture
        else:
            return "Unknown", 0.3

    def smooth_predictions(self, gesture, confidence):
        """Smooth predictions using a buffer to reduce noise"""
        current_time = time.time()
        
        # Check global gesture recognition cooldown first - regardless of gesture detection
        time_since_last_gesture = current_time - self.last_gesture_recognition_time
        if time_since_last_gesture < self.gesture_recognition_cooldown:
            # Still in cooldown period - don't process any gestures
            time_remaining = self.gesture_recognition_cooldown - time_since_last_gesture
            return gesture, confidence  # Return gesture for display but don't process
        
        # Only process gestures if cooldown period has passed
        self.gesture_buffer.append((gesture, confidence))
        
        if len(self.gesture_buffer) > self.buffer_size:
            self.gesture_buffer.pop(0)
        
        # Get most common gesture in buffer
        gestures_only = [g for g, c in self.gesture_buffer if c > self.confidence_threshold]
        
        if len(gestures_only) > self.buffer_size * 0.5:  # At least 50% confidence
            most_common = Counter(gestures_only).most_common(1)
            if most_common:
                final_gesture = most_common[0][0]
                
                # Send command to ESP32 if gesture is recognized and cooldown has passed
                if final_gesture in self.gesture_commands:
                    success = self.send_gesture_command(final_gesture)
                    if success:
                        self.last_gesture_time = current_time
                        self.last_gesture_recognition_time = current_time  # Start new cooldown period
                        print(f"🕐 Next gesture can be recognized in {self.gesture_recognition_cooldown} seconds")
                        # Clear buffer after successful command to avoid repeat processing
                        self.gesture_buffer.clear()
                
                return final_gesture, confidence
        
        return gesture, confidence

    def draw_landmarks_and_info(self, image, landmarks, gesture, confidence):
        """Draw hand landmarks and gesture information on image"""
        # Draw hand landmarks
        self.mp_draw.draw_landmarks(image, landmarks, self.mp_hands.HAND_CONNECTIONS)
        
        # Draw gesture information
        height, width = image.shape[:2]
        
        # Main info panel
        panel_height = self.config['display']['panel_height']
        cv2.rectangle(image, (10, 10), (550, panel_height), (0, 0, 0), -1)
        cv2.rectangle(image, (10, 10), (550, panel_height), (255, 255, 255), 2)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = self.config['display']['font_size']
        font_thickness = self.config['display']['font_thickness']
        
        # Current gesture
        color = (0, 255, 0) if gesture in self.gesture_commands else (0, 0, 255)
        cv2.putText(image, f"Gesture: {gesture}", (20, 40), font, font_size * 1.4, color, font_thickness)
        cv2.putText(image, f"Confidence: {confidence:.2f}", (20, 70), font, font_size, (255, 255, 255), font_thickness)
        
        # ESP32 connection status
        status_color = (0, 255, 0) if self.connected else (0, 0, 255)
        status_text = f"ESP32: Connected ({self.esp32_ip})" if self.connected else "ESP32: Disconnected"
        cv2.putText(image, status_text, (20, 100), font, font_size * 0.8, status_color, font_thickness)
        
        # Connection type and heartbeat
        if self.connected:
            connection_info = "WiFi/UDP+HTTP"
            heartbeat_age = time.time() - self.last_heartbeat
            heartbeat_threshold = self.config['monitoring']['heartbeat_timeout']
            if heartbeat_age < heartbeat_threshold:  # Recent heartbeat
                heartbeat_color = (0, 255, 0)
                heartbeat_text = f"Heartbeat: {heartbeat_age:.1f}s ago"
            else:
                heartbeat_color = (0, 255, 255)
                heartbeat_text = "Heartbeat: Old/None"
            
            cv2.putText(image, f"Mode: {connection_info}", (20, 130), font, font_size * 0.7, (255, 255, 0), 1)
            cv2.putText(image, heartbeat_text, (20, 150), font, font_size * 0.7, heartbeat_color, 1)
        
        # Fan command info
        if gesture in self.gesture_commands:
            cmd_info = self.gesture_commands[gesture]
            cv2.putText(image, f"Command: {cmd_info['description']}", (20, 170), 
                       font, font_size * 0.8, (255, 255, 0), font_thickness)
        
        # Gesture recognition cooldown indicator
        current_time = time.time()
        time_since_last_gesture = current_time - self.last_gesture_recognition_time
        cooldown_seconds = self.config['gesture_recognition']['cooldown_seconds']
        if time_since_last_gesture < cooldown_seconds:
            time_remaining = cooldown_seconds - time_since_last_gesture
            cooldown_color = (0, 165, 255)  # Orange color
            cv2.putText(image, f"Cooldown: {time_remaining:.1f}s", (20, 195), 
                       font, font_size * 0.7, cooldown_color, font_thickness)
        else:
            cv2.putText(image, "Ready for gesture", (20, 195), 
                       font, font_size * 0.7, (0, 255, 0), 1)
        
        # Gesture commands panel
        commands_y = height - self.config['display']['commands_panel_height']
        cv2.rectangle(image, (10, commands_y), (650, height-10), (0, 0, 0), -1)
        cv2.rectangle(image, (10, commands_y), (650, height-10), (255, 255, 255), 2)
        
        cv2.putText(image, f"Fan Control Gestures (Wireless) - {cooldown_seconds}s Interval:", (20, commands_y + 25), 
                   font, font_size, (255, 255, 255), font_thickness)
        
        y_offset = 50
        for gesture_name, cmd in self.gesture_commands.items():
            cmd_text = f"{cmd['emoji']} {gesture_name}: {cmd['description']}"
            cv2.putText(image, cmd_text, (20, commands_y + y_offset), 
                       font, self.config['display']['instruction_font_size'], (255, 255, 255), 1)
            y_offset += 25
        
        # Controls
        cv2.putText(image, "Press 'q' to quit, 'r' to reconnect ESP32", 
                   (20, commands_y + y_offset + 10), font, self.config['display']['instruction_font_size'], (200, 200, 200), 1)
        
        return image

    def get_esp32_status(self):
        """Get current status from ESP32"""
        if not self.connected or not self.esp32_ip:
            return None
        
        try:
            response = self.session.get(f"http://{self.esp32_ip}/status")
            if response.status_code == 200:
                return response.json()
        except Exception as e:
            print(f"⚠️ Status request failed: {e}")
            
        return None

    def run(self):
        """Main loop for gesture recognition"""
        print("\n🎥 Starting camera feed...")
        print("👋 Show your hand gestures to control the fan wirelessly!")
        
        # Periodic status check
        last_status_check = 0
        
        while True:
            success, image = self.cap.read()
            if not success:
                print("Failed to read from camera")
                break
            
            # Flip image horizontally for selfie-view
            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Process the image
            results = self.hands.process(image_rgb)
            
            gesture = "No Hand Detected"
            confidence = 0.0
            
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Classify gesture
                    gesture, confidence = self.classify_gesture(hand_landmarks.landmark)
                    
                    # Check if we're in cooldown period
                    current_time = time.time()
                    time_since_last_gesture = current_time - self.last_gesture_recognition_time
                    
                    if time_since_last_gesture < self.gesture_recognition_cooldown:
                        # In cooldown period - clear buffer and don't process gestures
                        self.gesture_buffer.clear()
                        # Still show the detected gesture but don't process it
                    else:
                        # Cooldown period has passed - process gestures normally
                        gesture, confidence = self.smooth_predictions(gesture, confidence)
                    
                    # Draw landmarks and information (always show current detection)
                    image = self.draw_landmarks_and_info(image, hand_landmarks, gesture, confidence)
            else:
                # Clear buffer when no hand is detected
                self.gesture_buffer.clear()
                
                # Draw "No Hand" message
                height, width = image.shape[:2]
                panel_width = self.config['display']['no_hand_panel_width']
                panel_height = self.config['display']['no_hand_panel_height']
                cv2.rectangle(image, (10, 10), (panel_width, panel_height), (0, 0, 0), -1)
                cv2.rectangle(image, (10, 10), (panel_width, panel_height), (255, 255, 255), 2)
                cv2.putText(image, "No Hand Detected", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                           self.config['display']['font_size'] * 1.4, (0, 0, 255), 
                           self.config['display']['font_thickness'])
                
                # Show ESP32 status
                status_color = (0, 255, 0) if self.connected else (0, 0, 255)
                status_text = f"ESP32: Connected ({self.esp32_ip})" if self.connected else "ESP32: Disconnected"
                cv2.putText(image, status_text, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 
                           self.config['display']['font_size'] * 0.8, status_color, 
                           self.config['display']['font_thickness'])
                
                if self.connected:
                    cv2.putText(image, "Mode: WiFi/UDP+HTTP", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                               self.config['display']['font_size'] * 0.7, (255, 255, 0), 1)
                
                # Show cooldown status even when no hand is detected
                current_time = time.time()
                time_since_last_gesture = current_time - self.last_gesture_recognition_time
                cooldown_seconds = self.config['gesture_recognition']['cooldown_seconds']
                if time_since_last_gesture < cooldown_seconds:
                    time_remaining = cooldown_seconds - time_since_last_gesture
                    cooldown_color = (0, 165, 255)  # Orange color
                    cv2.putText(image, f"Cooldown: {time_remaining:.1f}s", (20, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, self.config['display']['font_size'] * 0.7, 
                               cooldown_color, self.config['display']['font_thickness'])
                else:
                    cv2.putText(image, "Ready for gesture", (20, 120), 
                               cv2.FONT_HERSHEY_SIMPLEX, self.config['display']['font_size'] * 0.7, 
                               (0, 255, 0), 1)
            
            # Periodic status check
            current_time = time.time()
            status_check_interval = self.config['monitoring']['status_check_interval']
            if current_time - last_status_check > status_check_interval:
                last_status_check = current_time
                status = self.get_esp32_status()
                if status:
                    if self.config['monitoring']['show_status_updates']:
                        print(f"📊 ESP32 Status: Fan {status.get('fanSpeed', 'Unknown')}%, "
                              f"Blynk: {'✅' if status.get('blynkConnected') else '❌'}")
                else:
                    # Try to reconnect
                    self.connected = False
                    print("🔄 Lost connection, attempting to reconnect...")
                    self.discover_esp32()
            
            # Display the image
            cv2.imshow(self.config['display']['window_title'], image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                print("🔄 Attempting to reconnect to ESP32...")
                self.connected = False
                self.esp32_ip = None
                self.discover_esp32()
        
        # Cleanup
        if hasattr(self, 'udp_socket'):
            self.udp_socket.close()
        self.cap.release()
        cv2.destroyAllWindows()
        print("🎮 Wireless Gesture-Controlled Fan System Closed")

# Additional utility class for batch processing (kept from original)
class GestureDatasetCreator:
    """Utility class to create training dataset from recorded gestures"""
    
    def __init__(self):
        self.recognizer = HandGestureRecognizer()
        self.dataset = []
    
    def record_gestures(self, gesture_name, num_samples=100):
        """Record gesture samples for training dataset"""
        print(f"Recording {num_samples} samples for '{gesture_name}'")
        print("Press 's' to save sample, 'q' to quit, any other key to skip")
        
        samples_recorded = 0
        
        while samples_recorded < num_samples:
            success, image = self.recognizer.cap.read()
            if not success:
                continue
                
            image = cv2.flip(image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.recognizer.hands.process(image_rgb)
            
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                
                # Extract landmark coordinates
                landmarks = []
                for landmark in hand_landmarks.landmark:
                    landmarks.extend([landmark.x, landmark.y, landmark.z])
                
                # Draw landmarks
                self.recognizer.mp_draw.draw_landmarks(
                    image, hand_landmarks, self.recognizer.mp_hands.HAND_CONNECTIONS)
                
                # Show recording info
                cv2.putText(image, f"Recording: {gesture_name}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(image, f"Samples: {samples_recorded}/{num_samples}", (10, 70), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(image, "Press 's' to save, 'q' to quit", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow('Gesture Recording', image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s') and results.multi_hand_landmarks:
                # Save sample
                self.dataset.append({
                    'gesture': gesture_name,
                    'landmarks': landmarks
                })
                samples_recorded += 1
                print(f"Sample {samples_recorded} saved")
                
            elif key == ord('q'):
                break
        
        print(f"Recorded {samples_recorded} samples for '{gesture_name}'")
    
    def save_dataset(self, filename="gesture_dataset.npy"):
        """Save recorded dataset to file"""
        np.save(filename, self.dataset)
        print(f"Dataset saved to {filename}")

# Main execution
if __name__ == "__main__":
    try:
        print("🎮 Starting Wireless Gesture-Controlled Fan System...")
        print("📋 Requirements:")
        print("   - ESP32 connected to WiFi network")
        print("   - Camera available")
        print("   - Python packages: opencv-python, mediapipe, requests")
        print("\n🔧 Make sure ESP32 is running the updated wireless firmware!")
        print("📡 System will auto-discover ESP32 on your WiFi network")
        
        # Create and run gesture recognizer with config
        recognizer = HandGestureRecognizer(config_path="config.yaml")
        recognizer.run()
        
    except KeyboardInterrupt:
        print("\n⚠️ Program interrupted by user")
    except Exception as e:
        print(f"❌ An error occurred: {e}")
        print("\n🔧 Troubleshooting:")
        print("1. Check ESP32 WiFi connection (same network as computer)")
        print("2. Install required packages: pip install opencv-python mediapipe requests")
        print("3. Make sure camera is not being used by other applications")
        print("4. Check ESP32 firmware is updated with wireless support")
        print("5. Ensure computer and ESP32 are on the same WiFi network")