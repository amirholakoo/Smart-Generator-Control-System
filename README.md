# Generator Control System with OLED Display 🌟

## Overview 🚀

This project implements a **smart generator control system** using an **ESP32**, a **ZMPT101B voltage sensor**, a **servo motor (MG995)**, and an **SSD1306 OLED display**. The system monitors the generator's output voltage and adjusts the gas input using a PID control loop to maintain a **stable 220V**. The OLED display shows real-time voltage, servo angle, and system notifications, giving you full control and visual feedback! 💡

---

## Features ✨

- **PID Voltage Control**: Automatically adjusts the servo to maintain a **stable 220V output**.
- **OLED Display**: Shows real-time **voltage**, **servo angle**, and system status.
- **Servo Control**: Dynamically adjust the **servo angle** (0-180°) to regulate generator output.
- **Safety Features**: Stops the system when voltage exceeds safety thresholds.
- **Progress Bars**: Visual representation of the **servo angle** and **voltage** relative to the setpoint.
- **Web Interface**: Update system parameters like PID values, safety limits, and more through a browser interface.
- **Watchdog System**: Monitors voltage fluctuations and takes corrective actions.

---

## Hardware Components 🔧

- **ESP32**: The brain of the operation 🧠
- **ZMPT101B**: AC Voltage Sensor 📡
- **MG995 Servo**: Controls gas input to regulate voltage ⚙️
- **SSD1306 OLED Display**: Displays voltage, servo angle, and errors 📺
- **Wiring**:
  - **VCC** (OLED) → **3.3V** (ESP32)
  - **GND** (OLED) → **GND** (ESP32)
  - **SCL** (OLED) → **GPIO 22** (ESP32)
  - **SDA** (OLED) → **GPIO 21** (ESP32)
  - **Servo** (Signal) → **GPIO 13** (ESP32)
  - **ZMPT101B** (Analog) → **GPIO 34** (ESP32)

---

## Software 📜

This project is powered by:

- **Arduino IDE** or **PlatformIO** for coding.
- **Adafruit SSD1306** and **Adafruit GFX** libraries for the OLED display.
- **ESPAsyncWebServer** for the web interface.
- **ZMPT101B library** for voltage readings.

---

## Installation and Setup ⚙️

### 1. Clone the repository:
```bash
git clone https://github.com/your-repo/generator-control-system.git
cd generator-control-system
```

### 2\. Install the required libraries:

-   **Adafruit SSD1306**: For OLED display support.
-   **Adafruit GFX**: Required for graphics.
-   **ESPAsyncWebServer**: For the web interface.
-   **ZMPT101B**: For accurate voltage sensing.

### 3\. Upload the Code:

-   Connect your ESP32 to your computer.
-   Open the project in **Arduino IDE** or **PlatformIO**.
-   Adjust the Wi-Fi credentials in the code (`ssid` and `password`).
-   Upload the code to the ESP32.

* * * * *

How It Works 🛠️
----------------

### 1\. Voltage and Servo Control:

-   The **ESP32** reads the generator's output voltage using the **ZMPT101B** sensor.
-   The system uses a **PID control loop** to calculate the error between the **setpoint** (220V) and the current voltage.
-   Based on the error, the **servo motor** adjusts the gas input to regulate the RPM and output voltage.

### 2\. OLED Display:

-   **Top Bar**: Displays the current **voltage** relative to the setpoint (220V).
-   **Middle**: Displays the actual **voltage** and **servo angle** (in degrees).
-   **Bottom Bar**: Displays the **servo angle** visually as a progress bar.

### 3\. Safety System:

-   If the voltage exceeds safety thresholds, the system stops and displays **System Stopped!** on the OLED.
-   The **watchdog system** monitors voltage fluctuations and adjusts the servo when needed.

### 4\. Web Interface:

-   The **ESP32** hosts a web interface where you can update the **PID values**, **setpoints**, **safety limits**, and other parameters in real-time.
-   Connect to the ESP32's Wi-Fi and access the web interface via its IP address.

* * * * *

Display Layout 🖥️
------------------

+-------------------------------+ | Voltage Bar (relative to 220V) | +-------------------------------+ 

|Voltage: xxx V                                                                                       |

| Servo Angle: xxx deg                                                                                | 

+-------------------------------+ | Servo Angle Bar (0-180 deg) | +-------------------------------+

* * * * *

Web Interface 🌐
----------------

-   The **web interface** allows you to adjust parameters dynamically:
    -   **KP, KI, KD**: PID control constants.
    -   **Voltage Setpoint**: Target voltage (default: 220V).
    -   **Safety Limits**: Configure upper and lower voltage thresholds.
    -   **Enable/Disable Safety, Watchdog, VSG**: Control system behaviors.

* * * * *

### Web Interface Options -- Detailed Descriptions and Instructions

* * * * *

### **1\. PID Control Parameters** 🔧

The PID (Proportional-Integral-Derivative) controller adjusts the servo angle to regulate the generator's output voltage. These settings allow you to fine-tune the control loop for optimal performance.

-   **KP (Proportional Gain)**:
    -   **Description**: Determines how aggressively the system reacts to voltage errors. The larger the error, the greater the output adjustment.
    -   **Example**: If KP is set to 2.0, the servo will react twice as fast to a voltage error compared to KP = 1.0.
    -   **Usage**: Increase KP if the system is too slow to adjust, but be careful not to set it too high, as it may cause instability (over-corrections).
-   **KI (Integral Gain)**:
    -   **Description**: Accounts for past errors to eliminate steady-state error. Helps fine-tune long-term accuracy.
    -   **Example**: If the output voltage is consistently below 220V, KI helps by slowly adjusting the servo until the error is eliminated.
    -   **Usage**: Increase KI if the system fails to reach the setpoint over time. Too high of a value might cause slow oscillations.
-   **KD (Derivative Gain)**:
    -   **Description**: Predicts future errors by responding to the rate of change of the error. Helps stabilize the system and prevent overshooting.
    -   **Example**: If the voltage fluctuates too quickly, KD will dampen the response to prevent over-corrections.
    -   **Usage**: Increase KD if the system is overshooting or reacting too aggressively to small changes.

#### **How to Set PID Values:**

-   **Start with KP**: Increase it gradually until the system responds well without oscillations.
-   **Adjust KI**: Once KP is tuned, increase KI to eliminate long-term errors (if the voltage stays slightly off from the setpoint).
-   **Tweak KD**: Finally, add KD to reduce overshooting and stabilize the system.

* * * * *

### **2\. Voltage Setpoint** 🎯

-   **Target Voltage (V)**:
    -   **Description**: The desired output voltage you want the generator to maintain. By default, this is set to **220V**.
    -   **Example**: If you set the target voltage to 230V, the system will adjust the gas input to stabilize the generator's output at 230V.
    -   **Usage**: Adjust this value depending on the generator's desired output voltage. Typically, you'll want this around **220V-230V** for most applications.

* * * * *

### **3\. Safety Limits** ⚠️

Safety limits help protect your generator from dangerous voltage fluctuations. These settings stop the system if the voltage goes beyond a specified range.

-   **Low Voltage Shutdown Threshold (V)**:
    -   **Description**: If the voltage drops below this threshold, the system stops the servo to prevent damage or further voltage drops.
    -   **Example**: If this is set to 200V, the system will shut down if the voltage drops below 200V.
    -   **Usage**: Set this just below your setpoint (e.g., around **200V** for a 220V system).
-   **High Voltage Shutdown Threshold (V)**:
    -   **Description**: If the voltage exceeds this threshold, the system stops to prevent the generator from producing dangerously high voltage.
    -   **Example**: If this is set to 240V, the system will shut down if the voltage exceeds 240V.
    -   **Usage**: Set this a little above your setpoint (e.g., **240V** for a 220V system).

* * * * *

### **4\. Watchdog System** 🕵️‍♂️

The watchdog monitors for significant and sudden voltage changes. If the voltage deviates too much, the system makes quick adjustments.

-   **Enable/Disable Watchdog**:

    -   **Description**: Toggle the watchdog system on or off. When enabled, the watchdog continuously monitors for dangerous fluctuations.
    -   **Usage**: Keep this enabled unless you are troubleshooting or testing the system.
-   **Watchdog Time Limit (ms)**:

    -   **Description**: How long the system waits (in milliseconds) before adjusting the servo after detecting a large voltage change.
    -   **Example**: If set to 3000ms (3 seconds), the system waits 3 seconds before making servo adjustments.
    -   **Usage**: Lower the time limit for faster adjustments if the load changes rapidly. A higher time limit prevents over-corrections.
-   **Max Voltage Change (V)**:

    -   **Description**: The maximum allowed voltage deviation before the watchdog triggers corrective actions.
    -   **Example**: If set to 10V, the watchdog will adjust the servo if the voltage changes by more than 10V.
    -   **Usage**: Keep this setting low (e.g., 5-10V) to ensure quick responses to significant voltage changes.

* * * * *

### **5\. Virtual Synchronous Generator (VSG)** ⚡

The VSG feature helps stabilize the system during sudden load changes by simulating the behavior of a traditional synchronous generator.

-   **Enable/Disable VSG**:

    -   **Description**: Toggle the VSG system on or off. When enabled, VSG provides smoother voltage control by emulating inertia and damping.
    -   **Usage**: Enable this for smoother generator performance during fluctuating loads.
-   **Inertia Emulation Level**:

    -   **Description**: Controls how much the system mimics the inertia of a traditional generator. Higher values result in slower, smoother responses.
    -   **Example**: Setting this to **0.5** slows the system's response slightly, preventing quick overshooting.
    -   **Usage**: Adjust this if the system is reacting too quickly to voltage changes. Higher values smooth out fluctuations.
-   **Damping Factor**:

    -   **Description**: Adds damping to prevent oscillations and overshooting. Higher values dampen the system more.
    -   **Example**: Setting this to **0.1** reduces oscillations by slowing down large adjustments.
    -   **Usage**: Increase this if the system tends to overshoot the setpoint or oscillate around it.

* * * * *

### **6\. Servo Angle Display and Control** 📏

This feature visually represents the current servo angle (0-180 degrees) as a **progress bar** on the OLED screen.

-   **Servo Angle Bar**:
    -   **Description**: A progress bar at the bottom of the OLED display shows how far the servo is open. A full bar indicates the servo is at 180 degrees, fully open.
    -   **Usage**: Monitor the bar to see how the servo is adjusting the gas input in real-time.

* * * * *

### Examples of Using the Interface ⚙️

1.  **Fine-Tuning the PID Values**:

    -   Start with **KP = 1.0**, **KI = 0.1**, and **KD = 0.05**. If the voltage is fluctuating too much, try increasing **KD** to **0.1** for better stability.
2.  **Adjusting the Setpoint**:

    -   If you want the generator to output **230V** instead of 220V, set the **Target Voltage** to **230**. The system will adjust accordingly.
3.  **Configuring Safety Limits**:

    -   For a **220V setpoint**, you can set the **Low Voltage Threshold** to **200V** and the **High Voltage Threshold** to **240V**. This prevents the generator from operating in dangerous voltage ranges.

* * * * *

Web Interface FAQ 📋
--------------------

### **Q: What happens if I disable the Watchdog system?**

-   **A**: If the watchdog is disabled, the system will no longer make quick adjustments in response to large, sudden voltage changes. Use this only when troubleshooting or testing the system.

### **Q: How do I know if the safety system is triggered?**

-   **A**: If the voltage exceeds your safety limits, the OLED will display "System Stopped!" and the system will stop adjusting the servo.

### **Q: Can I change the voltage setpoint while the system is running?**

-   **A**: Yes! You can adjust the **setpoint** in real-time through the web interface. The system will automatically respond and adjust to the new setpoint.


* * * * *

Future Improvements 🔮
----------------------

-   Add real-time data logging and graphing of voltage and servo angle 📊.
-   Implement additional safety features for overload protection ⚡.

* * * * *

Acknowledgements 🙏
-------------------

Special thanks to **ChatGPT 4o** for working on this project! And helping to make this all happen! 😊

* * * * *

License 📜
----------

This project is licensed under the MIT License - see the LICENSE file for details.
