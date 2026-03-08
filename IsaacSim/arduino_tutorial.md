
# Developer's Guide: RoArm-M2-S Firmware Architecture 

## Part 1: Codebase Architecture

The Waveshare `RoArm-M2_example` (https://github.com/waveshareteam/roarm_m2/tree/main) is built on a modular header-file architecture. Instead of writing 2,000 lines of code in one `.ino` file, the logic is split into specific `.h` files.

Here is the anatomy of the firmware:

* **`RoArm-M2_example.ino`**: The main entry point. It runs `setup()` to initialize all modules and `loop()` to continuously listen for WiFi, Serial, and ESP-NOW data.
* **`base_config.h`**: The "Control Center". This contains all pin definitions, WiFi credentials, global variables (like `current_x`, `target_y`), and physical robotic parameters (link lengths).
* **`servo_ctrl.h`**: The low-level motor driver. It wraps the `SCServo` library to send hexadecimal commands to the ST3215 serial bus servos (setting IDs, reading angles, turning torque on/off).
* **`kinematics.h`**: The brain of the robot. It contains the mathematical formulas for Forward Kinematics (calculating XYZ from angles) and Inverse Kinematics (calculating angles from XYZ coordinates).
* **`json_cmd.h`**: The API Router. It uses the `ArduinoJson` library to parse incoming strings like `{"T":1041, "x":300}` and triggers the corresponding functions in `kinematics.h` or `servo_ctrl.h`.
* **`oled_ctrl.h` / `wifi_ctrl.h` / `esp_now_ctrl.h**`: Peripheral drivers for the screen and wireless communications.

---

## Part 2: How the Main Loop Works (The Execution Flow)

To write good secondary code, you must understand how the robot "thinks" every millisecond. The ESP32 does not use `delay()` in its main loop (using `delay()` would freeze the robot and crash the serial buffer). Instead, it uses a **Non-Blocking Super Loop**.

1. **Listen**: Check if new Serial or WiFi data has arrived.
2. **Parse**: If a complete JSON string is received, pass it to `jsonCmdReceive()` in `json_cmd.h`.
3. **Calculate**: If the command was `T:1041` (Cartesian movement), `kinematics.h` calculates the required joint angles.
4. **Execute**: `servo_ctrl.h` sends the new target angles to the servos.
5. **Feedback**: Every few milliseconds, query the servos for their actual position and update the OLED screen.

---

## Part 3: Step-by-Step Tutorial: Adding a Custom Module

Let's walk through a practical example of **Secondary Development**.
*Scenario: You want to attach a Laser Pointer to the end-effector of the RoArm and control it via a custom JSON command: `{"T": 999, "state": 1}` (Turn On) and `{"T": 999, "state": 0}` (Turn Off).*

### Step 1: Define the Hardware Pin

First, we need to assign an ESP32 pin for the laser. The RoArm driver board exposes several unused GPIO pins (like IO23 or IO19).

Open **`base_config.h`** and add your pin definition at the top:

```cpp
// --- Custom Module Pins ---
#define LASER_PIN 23  // Connect your laser module to GPIO 23

```

### Step 2: Create the Module Logic

Create a new file in your Arduino IDE (click the downward arrow on the top right of the editor -> "New Tab") and name it **`laser_ctrl.h`**.

Write the initialization and control logic:

```cpp
#ifndef LASER_CTRL_H
#define LASER_CTRL_H

#include "base_config.h"

// Initialize the pin
void initLaser() {
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW); // Default to off
    Serial.println("Laser Module Initialized.");
}

// Function to control the laser state
void setLaserState(int state) {
    if (state == 1) {
        digitalWrite(LASER_PIN, HIGH);
    } else {
        digitalWrite(LASER_PIN, LOW);
    }
}

#endif

```

### Step 3: Integrate the Module into the Main Program

Now, open the main **`RoArm-M2_example.ino`** file. You need to include your new header and initialize it in the `setup()` function.

```cpp
#include "base_config.h"
#include "servo_ctrl.h"
// ... other includes ...
#include "laser_ctrl.h"  // <-- 1. Include your new module here

void setup() {
    Serial.begin(115200);
    // ... existing initialization code ...
    
    initLaser(); // <-- 2. Initialize it in setup()
}

void loop() {
    // ... existing loop code ...
}

```

### Step 4: Map a Custom JSON Command to Your Module

To control your laser from your Mac/Python script, you need to teach the ESP32 a new "T" command.

Open **`json_cmd.h`**. Scroll down to the `jsonCmdReceive()` function. You will see a large `switch(cmdType)` block or an `if/else if` chain handling different `T` values. Add your custom command `T: 999`:

```cpp
void jsonCmdReceive(StaticJsonDocument<256>* doc) {
    int cmdType = (*doc)["T"];
    
    switch (cmdType) {
        case 100:
            // Homing logic...
            break;
            
        case 1041:
            // Kinematics logic...
            break;

        // 👇 --- Add your custom command here --- 👇
        case 999: 
            if ((*doc).containsKey("state")) {
                int laser_state = (*doc)["state"];
                setLaserState(laser_state);
                
                // Optional: Send a success response back to the Mac
                Serial.print("{\"Feedback\":\"Laser Set to ");
                Serial.print(laser_state);
                Serial.println("\"}");
            }
            break;
        // 👆 --------------------------------------- 👆
    }
}

```

### Step 5: Test it with Python!

Upload the modified firmware to the RoArm. Once it reboots, you can open your Mac's terminal and send:
`{"T": 999, "state": 1}`
The ESP32 will parse the JSON, route it to `case 999`, and call your `setLaserState(1)` function, turning the laser on instantly without interrupting the arm's movement!

---

## Part 4: Golden Rules for RoArm Secondary Development

1. **Never use `delay()**`: If you need to blink an LED or wait for a sensor, use `millis()` timers. A `delay(500)` will cause the arm to ignore 500 milliseconds of joystick commands, making it feel laggy and dangerous.
2. **Respect the Serial Buffer**: The `ArduinoJson` document is allocated a specific memory size (usually `StaticJsonDocument<256>`). If you create a custom JSON command that is extremely long, increase this size (e.g., to `512`), or the ESP32 will fail to parse it.
3. **Avoid Floating Point Math in the Main Loop**: The ESP32 is fast, but doing heavy trigonometry (like custom IK) inside the raw `loop()` can slow down the servo update rate. Only calculate IK when a new coordinate is received.
