# Line Following Robot By Group AAI 14

## Overview of Project

This project is about developing an autonomous robot capable of following a line, interpreting barcode commands, avoiding obstacles, and transmitting telemetry under real-time constraints.

This is a multicore version of the line following robot that uses **both cores** of the RP2040:
- **Core 0**: Main control loop (line following, navigation, barcode detection)
- **Core 1**: Ultrasonic obstacle detection (runs independently in parallel)

## Folder Structure & Summary

```
project_root/
├── src/
│   ├── main.c                  # Core 0: Main program and state machine
│   ├── imu.c                   # IMU sensor implementation
│   ├── motors.c                # Motor control
│   ├── encoders.c              # Encoder interrupts and odometry
│   ├── line_sensor.c           # IR line sensor
│   ├── barcode.c               # Barcode scanning
│   ├── ultrasonic.c            # Ultrasonic + servo (used by Core 1)
│   ├── navigation.c            # State machine and PID control
│   ├── multicore_obstacle.c    # Core 1: Obstacle detection task
│   └── utils.c                 # Utility functions
├── include/
│   ├── config.h                # Configuration constants
│   ├── imu.h                   # IMU interface
│   ├── motors.h                # Motor interface
│   ├── encoders.h              # Encoder interface
│   ├── line_sensor.h           # Line sensor interface
│   ├── barcode.h               # Barcode interface
│   ├── ultrasonic.h            # Ultrasonic interface
│   ├── navigation.h            # Navigation interface
│   ├── multicore_obstacle.h    # Multicore obstacle detection
│   └── utils.h                 # Utility functions
├── test/
│   ├── B1.c                    # Barcode decoding at speed test
│   ├── I1.c                    # IMU stability test
│   ├── I2.c                    # IMU turn accuracy test
│   ├── L1.c                    # Line sensor calibration test
│   ├── L2.c                    # Line following test
│   ├── M1.c                    # Motor control with PID test
│   ├── M2-L.c                  # Left turn test
│   ├── M2-R.c                  # Right turn test
│   ├── M2-U.c                  # U-turn test
│   ├── O1.c                    # Ultrasonic distance test
│   ├── O2.c                    # Obstacle width estimation test
│   ├── Q1.c                    # MQTT telemetry test
│   ├── UC01.c                  # Safe boot & initialization test
│   ├── UC02.c                  # Start line following test
│   ├── UC03.c                  # Line loss & recovery test
│   ├── UC04.c                  # Detect & decode barcode test
│   ├── UC05.c                  # Execute navigation command test
│   ├── UC06.c                  # Obstacle detection & safe stop test
│   ├── UC07.c                  # Obstacle edge scan & width estimation test
│   ├── UC08.c                  # Bypass obstacle & rejoin line test
│   ├── UC09.c                  # Telemetry & event publishing test
│   ├── UC10.c                  # Wi-Fi drop & auto-reconnect test
│   └── UC11.c                  # Emergency stop test
├── CMakeLists.txt              # Build configuration (with multicore)
└── README.md                   # This file
```

## Build Instructions

1. Download this repo to your computer
2. Unzip the folder and move it to your Pico SDK Environment
3. Add `Embedded-Sys-main` to your root CMakeLists.txt
4. Create the Build files by building `final_demo (Executable) ` in your CMake
5. Flash `final_demo.uf2` to your Robo Pico
6. Test the robot on a line following path



## Key Differences from Single-Core Implementation

### 1. **Multicore Architecture**
- Core 0 handles line following, barcode detection, and navigation
- Core 1 continuously monitors for obstacles independently
- Synchronization via mutex-protected shared state

## How Core 1 Works

### Core 1 Task Flow:
1. **Monitor**: Continuously measure center distance
2. **Detect**: If obstacle < 20cm threshold, set `obstacle_detected = true`
3. **Wait**: Wait for Core 0 to stop robot (`core0_stopped = true`)
4. **Scan**: Perform detailed left/right scan
5. **Update**: Store scan results in shared state, set `scan_complete = true`
6. **Block**: Wait for Core 0 to process (`obstacle_processed = true`)
7. **Reset**: Clear flags and resume monitoring

## Benefits of Multicore Architecture

1. **Non-blocking obstacle detection**: Core 0 never blocks waiting for ultrasonic readings
2. **Faster response time**: Core 1 continuously monitors while Core 0 follows line
3. **Better real-time performance**: Obstacle detection runs at full speed independently
4. **Separation of concerns**: Each core has a clear responsibility