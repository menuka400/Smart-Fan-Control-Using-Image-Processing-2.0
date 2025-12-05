# 🌪️ Smart Fan Control Using Image Processing - V2.0 🤖

[![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://python.org)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.0+-green.svg)](https://opencv.org)
[![ESP32](https://img.shields.io/badge/ESP32-Wireless-orange.svg)](https://espressif.com)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-Hand%20Detection-red.svg)](https://mediapipe.dev)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

> 🚀 **Updated Version** of the original "Smart-Fan-Control-Using-Image-Processing" project with enhanced wireless capabilities, multi-control interfaces, and advanced ESP32 integration!

## 🎬 Demo Video

🎥 **Watch the project in action!** See all three control methods working together:

[![Smart Fan Control V2.0 Demo](https://img.youtube.com/vi/kb7266dbLCk/maxresdefault.jpg)](https://youtu.be/kb7266dbLCk?si=MQXQ2FEmzqJiqpps)

**[🎬 Click here to watch the full demo on YouTube](https://youtu.be/VqwC6AHt0g4)**

In this video you'll see:
- 👋 **Hand gesture recognition** in real-time
- 📱 **Blynk mobile app control** with live synchronization
- 🎛️ **Rotary encoder physical control** with audio feedback
- 💡 **RGB LED status indicators** showing system state
- 📺 **LCD display** with real-time information
- 🔊 **Audio feedback** for all interactions
- 🌡️ **Temperature monitoring** integration

## 📋 Table of Contents
- [� Demo Video](#-demo-video)
- [�🎯 Project Overview](#-project-overview)
- [✨ New Features in V2.0](#-new-features-in-v20)
- [🛠️ Hardware Components](#️-hardware-components)
- [🎮 Control Methods](#-control-methods)
- [👋 Gesture Commands](#-gesture-commands)
- [🔧 Installation & Setup](#-installation--setup)
- [⚙️ Configuration](#️-configuration)
- [🚀 Usage](#-usage)
- [📡 Network Architecture](#-network-architecture)
- [🔌 Hardware Connections](#-hardware-connections)
- [🐛 Troubleshooting](#-troubleshooting)
- [🤝 Contributing](#-contributing)
- [📄 License](#-license)

## 🎯 Project Overview

The **Smart Fan Control Using Image Processing V2.0** is an advanced IoT system that controls a fan using hand gestures, rotary encoder, and mobile app through wireless communication. This project combines computer vision, embedded systems, and IoT technologies to create a contactless fan control system.

### 🔄 What's New in V2.0?
This is a major update from the original "Smart-Fan-Control-Using-Image-Processing" project with significant enhancements:

- 🌐 **Wireless Communication**: ESP32 WiFi integration for remote control
- 📱 **Multi-Interface Control**: Blynk app, rotary encoder, and gesture control
- 🔊 **Audio Feedback**: Buzzer notifications for all interactions
- 💡 **RGB LED Indicators**: Visual status feedback with dual RGB LEDs
- 📺 **LCD Display**: Real-time system status and control information
- 🌡️ **Temperature Monitoring**: DS18B20 sensor integration
- ⚙️ **YAML Configuration**: Easy customization without code changes
- 🔄 **Auto-Discovery**: Automatic ESP32 network detection
- 💾 **Persistent Settings**: Configuration saved across restarts

## ✨ New Features in V2.0

| Feature | Description | Status |
|---------|-------------|---------|
| 🌐 **Wireless Control** | ESP32 WiFi communication with Python app | ✅ |
| 📱 **Blynk Integration** | Remote control via mobile app | ✅ |
| 🎛️ **Rotary Encoder** | Physical knob control with button | ✅ |
| 🔊 **Audio Feedback** | Buzzer sounds for all interactions | ✅ |
| 💡 **RGB Status LEDs** | Dual RGB LEDs for system status | ✅ |
| 📺 **LCD Display** | 16x2 I2C LCD for real-time info | ✅ |
| 🌡️ **Temperature Sensor** | DS18B20 digital temperature monitoring | ✅ |
| ⚙️ **YAML Config** | Easy configuration management | ✅ |
| 🔍 **Auto-Discovery** | Automatic ESP32 network scanning | ✅ |
| 🛡️ **Error Handling** | Robust error handling and recovery | ✅ |

## 🛠️ Hardware Components

### 🖥️ Computer Vision System
- 💻 **Computer/Laptop** with camera
- 🎥 **USB Camera** (or built-in webcam)
- 🐍 **Python Environment** with required libraries

### 🔧 ESP32 Microcontroller System
- 🎯 **ESP32 Development Board** (NodeMCU-32S or similar)
- 🌀 **12V DC Fan** with PWM control
- 🔌 **Relay Module** (5V) for fan power control
- 📺 **16x2 I2C LCD Display** (0x27 address)
- 🎛️ **KY-040 Rotary Encoder** with button
- 💡 **2x RGB LEDs** (common cathode)
- 🔊 **Active Buzzer** (5V)
- 🌡️ **DS18B20 Temperature Sensor** with module
- 🔗 **Jumper Wires & Breadboard**
- 🔋 **12V Power Supply** for fan
- ⚡ **5V Power Supply** for ESP32

## 🎮 Control Methods

This system offers **three independent control methods**:

### 1. 👋 Gesture Control (Computer Vision)
- **Contactless operation** using hand gestures
- **Real-time recognition** with MediaPipe
- **Wireless communication** to ESP32
- **Visual feedback** on computer screen

### 2. 📱 Mobile App Control (Blynk)
- **Remote control** from anywhere with internet
- **Real-time synchronization** with hardware
- **Temperature monitoring** display
- **Manual fan speed adjustment**

### 3. 🎛️ Physical Control (Rotary Encoder)
- **Offline operation** when WiFi is down
- **Tactile feedback** with encoder rotation
- **Push button** for ON/OFF toggle
- **Audio feedback** with buzzer

## 👋 Gesture Commands

| Gesture | Emoji | Description | Fan Speed | Pattern |
|---------|-------|-------------|-----------|---------|
| **Rock** | ✊ | All fingers closed (fist) | 0% (OFF) | `[0,0,0,0,0]` |
| **Index Up** | 👆 | Only index finger extended | 100% (MAX) | `[0,1,0,0,0]` |
| **L-Shape** | 🤟 | Thumb + index finger | 50% (ON) | `[1,1,0,0,0]` |
| **Paper** | ✋ | All 5 fingers extended | 75% (HIGH) | `[1,1,1,1,1]` |
| **Scissors** | ✌️ | Index + middle fingers | 25% (LOW) | `[0,1,1,0,0]` |

## 🔧 Installation & Setup

### 📋 Prerequisites
- Python 3.7 or higher
- Arduino IDE with ESP32 board support
- Blynk mobile app
- Basic electronics knowledge

### 🐍 Python Dependencies

```bash
# Clone the repository
git clone https://github.com/yourusername/Smart-Fan-Control-V2
cd Smart-Fan-Control-V2

# Install required packages
pip install opencv-python
pip install mediapipe
pip install requests
pip install PyYAML
pip install numpy
```

### 🔧 ESP32 Libraries (Arduino IDE)

Install these libraries through Arduino Library Manager:

```cpp
// Core libraries
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
```

### 📱 Blynk Setup

1. **Download Blynk App** from App Store/Google Play
2. **Create Account** and new project
3. **Add Widget**: Slider widget for V7 (Fan Speed)
4. **Add Widget**: Button widget for V1 (Temperature Request)
5. **Get Auth Token** from project settings
6. **Update** `main.cpp` with your credentials:

```cpp
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";
```

## ⚙️ Configuration

### 📄 config.yaml

The system uses a YAML configuration file for easy customization:

```yaml
# ESP32 Network Configuration
esp32:
  ip_address: "192.168.1.164"  # Set to null for auto-discovery
  http_port: 80
  udp_port: 4210

# Camera Settings
camera:
  index: 0
  width: 1280
  height: 720
  flip_horizontal: true

# Gesture Recognition
gesture_recognition:
  cooldown_interval: 4.0
  confidence_threshold: 0.7
  buffer_size: 8

# Fan Speed Commands
fan_commands:
  Rock: {speed: 0, description: "Turn OFF", emoji: "✊"}
  L-shape: {speed: 50, description: "Turn ON (50%)", emoji: "🤟"}
  Scissors: {speed: 25, description: "Low Speed", emoji: "✌️"}
  Paper: {speed: 75, description: "High Speed", emoji: "✋"}
  Index Up: {speed: 100, description: "Max Speed", emoji: "👆"}
```

## 🚀 Usage

### 1. 🔌 Hardware Setup
1. **Connect components** according to pin diagram
2. **Upload ESP32 code** using Arduino IDE
3. **Power on** the system
4. **Check LCD** for WiFi connection status

### 2. 💻 Software Setup
1. **Update configuration** in `config.yaml`
2. **Run Python script**:
   ```bash
   python main.py
   ```
3. **System will auto-discover** ESP32 on network
4. **Start making gestures** in front of camera

### 3. 📱 Mobile Control
1. **Open Blynk app**
2. **Start project**
3. **Use slider** to control fan speed
4. **Press button** to read temperature

### 4. 🎛️ Physical Control
1. **Rotate encoder** to adjust speed (5% per click)
2. **Press encoder** to toggle ON/OFF
3. **Listen for audio feedback**

## 📡 Network Architecture

```
┌─────────────────┐    WiFi     ┌─────────────────┐    I2C/GPIO    ┌─────────────────┐
│   Computer      │◄────────────►│     ESP32       │◄──────────────►│   Peripherals   │
│                 │              │                 │                │                 │
│ • Camera        │    HTTP/UDP  │ • WiFi Module   │   • LCD Display │
│ • Python App    │              │ • Web Server    │   • RGB LEDs    │
│ • MediaPipe     │              │ • Blynk Client  │   • Buzzer      │
│ • OpenCV        │              │ • GPIO Control  │   • Rotary Enc  │
│                 │              │                 │   • Temp Sensor │
└─────────────────┘              └─────────────────┘   • Fan + Relay │
                                                       └─────────────────┘
        ▲                                 ▲
        │                                 │
        │ Internet                        │ Internet
        │                                 │
        ▼                                 ▼
┌─────────────────┐              ┌─────────────────┐
│   Blynk App     │◄─────────────►│  Blynk Cloud    │
│   (Mobile)      │   Internet    │    Server       │
└─────────────────┘              └─────────────────┘
```

## 🔌 Hardware Connections

### ESP32 Pin Configuration

| Component | ESP32 Pin | Description |
|-----------|-----------|-------------|
| **Fan PWM** | GPIO 25 | PWM signal to fan |
| **Fan Relay** | GPIO 26 | Relay control for fan power |
| **LCD SDA** | GPIO 21 | I2C data line |
| **LCD SCL** | GPIO 22 | I2C clock line |
| **Buzzer** | GPIO 13 | Active buzzer positive |
| **RGB LED 1 R** | GPIO 32 | System status LED (Red) |
| **RGB LED 1 G** | GPIO 33 | System status LED (Green) |
| **RGB LED 1 B** | GPIO 4 | System status LED (Blue) |
| **RGB LED 2 R** | GPIO 16 | Fan activity LED (Red) |
| **RGB LED 2 G** | GPIO 17 | Fan activity LED (Green) |
| **RGB LED 2 B** | GPIO 18 | Fan activity LED (Blue) |
| **Encoder CLK** | GPIO 19 | Rotary encoder clock |
| **Encoder DT** | GPIO 23 | Rotary encoder data |
| **Encoder SW** | GPIO 5 | Rotary encoder button |
| **Temp Sensor** | GPIO 15 | DS18B20 data line |

### 🔋 Power Requirements

- **ESP32**: 5V via USB or Vin pin
- **Fan**: 12V DC power supply
- **Relay**: 5V (powered from ESP32)
- **LCD**: 5V (I2C module with level shifting)
- **LEDs**: 3.3V (current limited via resistors)

## 🐛 Troubleshooting

### 🔧 Common Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| **ESP32 not connecting** | Wrong WiFi credentials | Check SSID/password in code |
| **Camera not detected** | Wrong camera index | Try different indices (0, 1, 2) |
| **Gestures not recognized** | Poor lighting | Ensure good lighting conditions |
| **Fan not responding** | Relay/power issue | Check 12V supply and relay wiring |
| **LCD blank** | I2C connection | Check SDA/SCL wiring and address |
| **No audio feedback** | Buzzer wiring | Verify buzzer polarity |
| **Blynk not connecting** | Auth token issue | Verify Blynk credentials |

### 📊 Status LED Indicators

| LED 1 (System) | LED 2 (Fan) | Meaning |
|----------------|-------------|----------|
| 🔴 Red | ⚫ Off | WiFi disconnected |
| 🟡 Yellow | ⚫ Off | WiFi connected, Blynk disconnected |
| 🟢 Green | ⚫ Off | Fully connected, fan off |
| 🟢 Green | 🔵 Blue | Connected, gesture system active |
| 🟢 Green | 🟢 Green | Connected, fan running |

### 🔊 Audio Feedback Patterns

| Sound Pattern | Meaning |
|---------------|----------|
| Single beep | Encoder rotation |
| Double beep | Button press (toggle) |
| Triple beep | Special function |
| Rising tone | Success/Connection |
| Falling tone | Error/Disconnection |
| Quick triple | Gesture recognized |

## 📈 Performance Features

- **⚡ Fast Response**: UDP communication for <100ms gesture response
- **🔄 Auto-Recovery**: Automatic reconnection on WiFi loss
- **🛡️ Error Resilience**: Robust error handling and fallback modes
- **📊 Real-time Monitoring**: Live status updates on all interfaces
- **⚙️ Configurable**: All parameters adjustable via YAML config
- **🔍 Auto-Discovery**: No need to manually find ESP32 IP address

## 🎯 Project Applications

- 🏠 **Smart Home Automation**
- 🦽 **Accessibility Solutions**
- 🏥 **Healthcare Environments** (contactless control)
- 🎓 **Educational Projects** (IoT + Computer Vision)
- 🔬 **Research Applications** (HCI studies)

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/AmazingFeature`)
3. **Commit** your changes (`git commit -m 'Add AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### 🎯 Areas for Contribution

- 🎨 UI/UX improvements for Python app
- 📱 Enhanced Blynk widgets and layouts
- 🤖 Additional gesture recognition
- 🌐 Web interface for ESP32
- 📊 Data logging and analytics
- 🔒 Security enhancements
- 📝 Documentation improvements

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- 🤖 **MediaPipe Team** for excellent hand tracking
- 🎯 **ESP32 Community** for hardware support
- 📱 **Blynk Platform** for IoT connectivity
- 👥 **Original Project Contributors** from Smart-Fan-Control-Using-Image-Processing
- 🌟 **Open Source Community** for inspiration and support

## 📞 Support

If you encounter any issues or have questions:

1. 📖 Check the [Troubleshooting](#-troubleshooting) section
2. 🔍 Search existing [Issues](https://github.com/yourusername/Smart-Fan-Control-V2/issues)
3. 🆕 Create a new issue with detailed description
4. 📧 Contact the maintainers

## 🌟 Star History

[![Star History Chart](https://api.star-history.com/svg?repos=yourusername/Smart-Fan-Control-V2&type=Date)](https://star-history.com/#yourusername/Smart-Fan-Control-V2&Date)

---

<div align="center">

### 🚀 Upgrade Your Fan Control Experience! 

**From Simple Gestures to Smart Home Integration**

[⭐ Star this repo](https://github.com/yourusername/Smart-Fan-Control-V2) • [🐛 Report Bug](https://github.com/yourusername/Smart-Fan-Control-V2/issues) • [💡 Request Feature](https://github.com/yourusername/Smart-Fan-Control-V2/issues)

Made with ❤️ and lots of ☕

</div>
