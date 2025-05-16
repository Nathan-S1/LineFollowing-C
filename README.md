# LineFollowing-C
Robot Line Following

# Line Following Robot

## Author
Nathan Shorez

---

## Project Overview
This project implements a line-following robot using C++ and Arduino. The robot uses IR sensors to detect line positioning and a PID controller to adjust motor speeds for precise path tracking. The objective is to maintain alignment with the line while navigating a predefined course.

---

## Key Components
- **PID Controller:** Adjusts motor speeds based on line position feedback to minimize error.
- **IR Sensors:** Detect line position relative to the robot.

---

## Hardware/Software Requirements
- Arduino-compatible microcontroller
- IR sensors for line tracking
- C++ compiler (e.g., g++)
- Arduino IDE for uploading and monitoring

---

## Compilation and Execution
1. **Navigate to the src directory:**
   ```bash
   cd src/
   ```
2. **Compile the source files:**
   ```bash
   g++ main.cpp PIDcontroller.cpp sensor.cpp -o line_follower
   ```
3. **Upload to Arduino:**
   - Open the Arduino IDE
   - Load `main.cpp`
   - Select the appropriate COM port and board
   - Upload the sketch

---

## Sample Output
Upon running, the system will output sensor readings and PID adjustments. Example:
```
Line position: 180
PID Output: -15
Left Motor Speed: 85
Right Motor Speed: 115
```

---

## Dependencies

- Arduino IDE (for uploading and monitoring)
- C++ Compiler (e.g., g++)
- IR Sensors for line detection
- Motor Driver (L298N or similar, if applicable)
- `Arduino.h` (Core library for Arduino functions)

---
