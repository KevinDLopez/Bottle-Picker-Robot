# Bottle Picker Robot

An autonomous 6-axis robotic arm system designed to detect and pick up bottles using computer vision and precision control.


## Project Overview

The Bottle Picker Robot is an autonomous robotic system featuring a six-axis rotation arm powered by servo motors and DC motors. The robot uses camera and ultrasonic sensors for bottle detection and environmental awareness. This project demonstrates advanced integration of mechanical design, embedded systems, and computer vision techniques.

### Key Features

- **Six-axis rotation arm** with precise movement control
- **Computer vision system** for bottle detection
- **Ultrasonic sensors** for proximity and obstacle detection
- **Custom PCB** with ARM Cortex-M4 processor and SMD components
- **Bluetooth connectivity** via HM10 module
- **iOS companion app** for remote control and configuration

## Demo Videos

Watch the robot in action:

[![Bottle Picker Robot Demo](https://img.youtube.com/vi/Pb_iPCBAZcU/0.jpg)](https://www.youtube.com/watch?v=Pb_iPCBAZcU)

[Watch on YouTube: Bottle Picker Robot Demo](https://www.youtube.com/watch?v=Pb_iPCBAZcU)

## Hardware Components

- ARM Cortex-M4 microcontroller (TM4C123GH6PM)
- Servo motors for precise arm movement
- DC motors for base movement
- Camera module for computer vision
- Ultrasonic sensors for distance detection
- HM10 Bluetooth module for wireless connectivity
- Custom PCB designed and fabricated for this project

## Software Architecture

The project consists of several software components:

1. **Motor Control Firmware** - Controls the 6-axis arm movement
2. **Sensor Integration** - Processes data from ultrasonic and camera sensors
3. **Bluetooth Communication Protocol** - Enables remote control via the iOS app
4. **iOS Application** - Provides user interface for remote operation and configuration

## Project Structure

```
.
├── Bluetooth IOS Application   # iOS app for remote control
│   ├── HM10 Serial             # Core application code
│   └── HM10 SerialTests        # Test cases for the app
├── MotorDrivers                # Embedded firmware for motor control
│   ├── ArmControl.c            # Arm movement control logic
│   ├── Encoders.c              # Motor encoder interface
│   ├── SixMottorArmControl.c   # Six-motor coordination system
│   ├── ULTRASONIC.c            # Ultrasonic sensor interface
│   └── inc                     # Header files
└── Robotic Arm- 2 power supplies - 10mm.Zip  # CAD design files
```

## Getting Started

### Prerequisites

- Keil MDK or similar ARM development environment
- Xcode for iOS application development
- PCB fabrication equipment or service
- Required electronic components (listed in BOM)

### Firmware Setup

1. Clone the repository
2. Open the InputOutput.uvproj file in your ARM development environment
3. Compile and flash the firmware to your microcontroller

### iOS Application Setup

1. Navigate to the Bluetooth IOS Application directory
2. Open HM10 Serial.xcodeproj in Xcode
3. Build and deploy the application to your iOS device

## Usage

1. Power up the robotic arm system
2. Connect to the robot via the iOS application using Bluetooth
3. Use the app interface to:
   - Manually control the arm movement
   - Set autonomous mode parameters
   - Monitor sensor data in real-time

## Technical Achievements

- Designed and fabricated a custom PCB using SMD components for improved reliability
- Implemented complex control algorithms for coordinated 6-axis movement
- Created a reliable wireless communication system using the HM10 Bluetooth module
- Developed computer vision algorithms for bottle detection and tracking

## Contributors

- Kevin (Lead Developer)
- Team of 4 additional contributors

## License

This project is licensed under the MIT License - see the LICENSE file for details.
