# Arduino Ball Counter and Speed Measurement System

## Overview

This project is an Arduino-based ball counting and speed measurement system designed for use with a ball machine. The system uses laser beam sensors to automatically count balls and calculate launch speed in real time. Results are displayed on an LCD screen, and manual count adjustment is available through pushbuttons.

The project was developed to provide a low-cost method of tracking ball usage while also evaluating machine performance through speed measurements.

---

## Features

- Automatic ball counting
- Real-time speed measurement
- 16x2 LCD display
- Manual count adjustment buttons
- Laser/phototransistor sensing
- LM393 comparator signal conditioning
- Interrupt-based timing for improved accuracy
- Diagnostic and debug modes

---

## Hardware

### Main Components

- Arduino Uno R3
- LCD1602 Display
- 2 Laser Modules
- 2 Phototransistors (PT334-6C)
- 2 LM393 Comparator Modules
- Pushbuttons
- Assorted resistors
- Custom 3D printed housing

---

## System Operation

The system uses two laser beams spaced a fixed distance apart.

1. A ball interrupts the first beam.
2. The Arduino records a timestamp.
3. The ball interrupts the second beam.
4. A second timestamp is recorded.
5. Speed is calculated using the measured travel time.
6. Count and speed are displayed on the LCD.

The comparator modules convert phototransistor outputs into clean digital signals, improving timing accuracy and reducing susceptibility to noise.

---

## Repository Structure

```text
Arduino_Code/     Arduino sketches
CAD/              CAD models and drawings
Documentation/    User manual and project reports
Electrical/       Wiring diagrams and schematics
Validation/       Calibration and test data
Photos/           Project and assembly photos
```

---

## Calibration

Speed measurements were calibrated using a radar gun as a reference.

Calibration involved:

- Collecting radar gun and sensor data
- Comparing measured speeds
- Developing correction functions
- Verifying performance through repeated testing

Current calibration functions are implemented directly within the Arduino code.

---

## Setup

1. Assemble hardware according to the wiring diagram.
2. Verify laser alignment.
3. Upload the Arduino sketch.
4. Confirm both sensors transition correctly between blocked and clear states.
5. Test counting functionality.
6. Verify speed measurements using known test conditions.

---

## Troubleshooting

### Count Not Increasing

Possible causes:

- Laser misalignment
- Sensor wiring issue
- Comparator threshold incorrectly adjusted

### Incorrect Speed Readings

Possible causes:

- Sensor spacing entered incorrectly
- Sensor alignment issues
- Comparator triggering problems
- Calibration function requires updating

### No LCD Display

Possible causes:

- Wiring error
- Incorrect LCD pin assignments
- Power connection issue

---

## Current Version

Current hardware configuration includes:

- Dual laser speed measurement
- LM393 comparator modules
- Interrupt-based event detection
- LCD display interface
- Manual count adjustment buttons

---

## Future Improvements

Potential future work:

- Data logging to SD card
- Wireless communication
- Automatic calibration tools
- Battery-powered operation
- Additional diagnostic features

---

## Author

Developed by Nigel S

2026
