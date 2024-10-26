#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ZMPT101B.h>
#include <ESP32Servo.h>


#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing Arduino reset pin)

// OLED I2C address
#define OLED_ADDRESS 0x3C

// Create an OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFi credentials
const char *ssid = "Your_SSID"; 
const char *password = "Your_PASSWORD";

// ESPAsyncWebServer object
AsyncWebServer server(80);

// Parameters and flags
float kp = 1.0, ki = 0.2, kd = 0.15;
float setpoint = 220.0;   // Target voltage
unsigned long safetyTimeLimit = 10000;  // 10 seconds
unsigned long watchdogTimeLimit = 3000; // 3 seconds
unsigned long angleTimeLimit = 60000;  // 1 minutes
unsigned long ChokeTimeLimit = 10000; // 10 seconds

int maxVoltageChange = 10;              // Voltage difference to trigger watchdog
int minPidOutput = 0, maxPidOutput = 180; 
int integralLimit = 1000;
float voltageTolerance = 5.0;

int lowVoltageThreshold = 200, highVoltageThreshold = 240;
int lowAngleLimit = 0, highAngleLimit = 180;
float inertiaEmulation = 0.5, dampingFactor = 0.1;

bool enableVSG = false, enableWatchdog = false, enableSafety = true;

// Control Variables
float previous_error = 0, integral = 0;
bool systemStopped = false;
unsigned long angleTimeStart = 0;
unsigned long watchdogStartTime = 0;

// Safety and monitoring variables
unsigned long lowVoltageStartTime = 0;
unsigned long highVoltageStartTime = 0;
unsigned long lastVoltageAdjustmentTime = 0;


// Create ZMPT101B object and Servo object
#define SENSITIVITY 633.5f
ZMPT101B voltageSensor(34, 50.0);  // ZMPT101B connected to ADC pin 34
Servo myServo;

const int LED_PIN = 2;  // Onboard LED (assumed to be on GPIO 2)

void setup() {
  Serial.begin(115200);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);  // Don't proceed, loop forever
  }

  // Clear the buffer and display initial text
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("Initializing..."));
  display.display(); 
  
  // Set sensitivity for ZMPT101B
  voltageSensor.setSensitivity(SENSITIVITY);
  
  // Setup WiFi
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  delay(5000);  // Wait for connection

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected, starting Access Point mode...");
    WiFi.softAP("Iranian_Generator", "12345678");
    display.clearDisplay();
    display.setTextSize(1);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 16);     // Start at top-left corner
    display.println(F("Wi-Fi not connected, starting Access Point mode..."));
    display.setCursor(0, 40);     // Start at top-left corner
    display.println(F("SSID:Iranian_Gen     PASS: 12345678"));
    display.display(); 
  } else {
    Serial.println("Connected to WiFi");
  }
  
  // Attach the servo
  myServo.attach(13);  // MG995 Servo connected to GPIO 13
  
  
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);

  // Initialize the web server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body><h2>Generator Control System Settings</h2>";
    html += "<form action=\"/update\" method=\"POST\">";

    // PID Parameters
    html += "<h3>PID Control Parameters:</h3>";
    html += "KP (0.2  - 0.5  - 0.8  - 1.0  - 1.5): <input type=\"number\" name=\"kp\" step=\"0.1\" value=\"" + String(kp) + "\"><br>";
    html += "KI (0.05 - 0.1  - 0.15 - 0.2  - 0.3): <input type=\"number\" name=\"ki\" step=\"0.1\" value=\"" + String(ki) + "\"><br>";
    html += "KD (0.01 - 0.05 - 0.1  - 0.15 - 0.2): <input type=\"number\" name=\"kd\" step=\"0.1\" value=\"" + String(kd) + "\"><br>";
    html += "Min PID Output: <input type=\"number\" name=\"minPidOutput\" value=\"" + String(minPidOutput) + "\"><br>";
    html += "Max PID Output: <input type=\"number\" name=\"maxPidOutput\" value=\"" + String(maxPidOutput) + "\"><br>";
    html += "Integral Windup Limit: <input type=\"number\" name=\"integralLimit\" value=\"" + String(integralLimit) + "\"><br><br>";

    // Setpoint
    html += "<h3>Setpoint:</h3>";
    html += "Target Voltage (V): <input type=\"number\" name=\"setpoint\" step=\"1\" value=\"" + String(setpoint) + "\"><br>";
    html += "Voltage Tolerance (±V): <input type=\"number\" name=\"voltageTolerance\" step=\"0.1\" value=\"" + String(voltageTolerance) + "\"><br><br>";

    // Safety System
    html += "<h3>Safety System(L/H Voltage Safe Time):</h3>";
    html += "Enable Safety Systems: <input type=\"checkbox\" name=\"safety\" " + String(enableSafety ? "checked" : "") + "><br>";
    html += "Safety Time Limit (ms): <input type=\"number\" name=\"safetyTimeLimit\" value=\"" + String(safetyTimeLimit) + "\"><br>";
    html += "Low Voltage Shutdown Threshold (V): <input type=\"number\" name=\"lowVoltageThreshold\" value=\"" + String(lowVoltageThreshold) + "\"><br>";
    html += "High Voltage Shutdown Threshold (V): <input type=\"number\" name=\"highVoltageThreshold\" value=\"" + String(highVoltageThreshold) + "\"><br>";
    html += "Servo Angle Limits: Low <input type=\"number\" name=\"lowAngleLimit\" value=\"" + String(lowAngleLimit) + "\"> High <input type=\"number\" name=\"highAngleLimit\" value=\"" + String(highAngleLimit) + "\"><br><br>";

    // Watchdog System
    html += "<h3>Watchdog System(Jump by +/- 15 Deg):</h3>";
    html += "Enable Watchdog: <input type=\"checkbox\" name=\"watchdog\" " + String(enableWatchdog ? "unchecked" : "") + "><br>";
    html += "Watchdog Time Limit (ms): <input type=\"number\" name=\"watchdogTimeLimit\" value=\"" + String(watchdogTimeLimit) + "\"><br>";
    html += "Max Voltage Change (V): <input type=\"number\" name=\"maxVoltageChange\" value=\"" + String(maxVoltageChange) + "\"><br><br>";

    // VSG System
    html += "<h3>Virtual Synchronous Generator (VSG):</h3>";
    html += "Enable VSG: <input type=\"checkbox\" name=\"vsg\" " + String(enableVSG ? "checked" : "") + "><br>";
    html += "Inertia Emulation Level: <input type=\"number\" name=\"inertiaEmulation\" step=\"0.1\" value=\"" + String(inertiaEmulation) + "\"><br>";
    html += "Damping Factor: <input type=\"number\" name=\"dampingFactor\" step=\"0.1\" value=\"" + String(dampingFactor) + "\"><br><br>";

    // Angle Safety Feature
    html += "<h3>Angle Safety Feature(Safe Time for 0 OR 180):</h3>";
    html += "Angle Time Limit (ms): <input type=\"number\" name=\"angleTimeLimit\" value=\"" + String(angleTimeLimit) + "\"><br><br>";

    // Choke Time to Shutdown
    html += "<h3>Choke Time OR 0 for Shutdown:</h3>";
    html += "Time Limit (ms): <input type=\"number\" name=\"ChokeTimeLimit\" value=\"" + String(ChokeTimeLimit) + "\"><br><br>";
    

    html += "<input type=\"submit\" value=\"Update\">";
    html += "</form></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("kp", true)) kp = request->getParam("kp", true)->value().toFloat();
    if (request->hasParam("ki", true)) ki = request->getParam("ki", true)->value().toFloat();
    if (request->hasParam("kd", true)) kd = request->getParam("kd", true)->value().toFloat();
    if (request->hasParam("setpoint", true)) setpoint = request->getParam("setpoint", true)->value().toFloat();
    if (request->hasParam("safetyTimeLimit", true)) safetyTimeLimit = request->getParam("safetyTimeLimit", true)->value().toInt();
    if (request->hasParam("watchdogTimeLimit", true)) watchdogTimeLimit = request->getParam("watchdogTimeLimit", true)->value().toInt();
    if (request->hasParam("maxVoltageChange", true)) maxVoltageChange = request->getParam("maxVoltageChange", true)->value().toInt();
    if (request->hasParam("angleTimeLimit", true)) angleTimeLimit = request->getParam("angleTimeLimit", true)->value().toInt();
    if (request->hasParam("ChokeTimeLimit", true)) ChokeTimeLimit = request->getParam("ChokeTimeLimit", true)->value().toInt();
    if (request->hasParam("vsg", true)) enableVSG = true; else enableVSG = false;
    if (request->hasParam("watchdog", true)) enableWatchdog = true; else enableWatchdog = false;
    if (request->hasParam("safety", true)) enableSafety = true; else enableSafety = false;
    request->send(200, "text/html", "<html><body><h2>Settings Updated!</h2><a href=\"/\">Go Back</a></body></html>");
  });

  // Start server
  server.begin();

  myServo.write(90);
  delay(5000);
}

void blinkRedLED() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = LOW;
  unsigned long currentMillis = millis();
  
  // Blink the LED at 1 second interval
  if (currentMillis - lastBlinkTime >= 1000) {
    lastBlinkTime = currentMillis;
    ledState = !ledState;  // Toggle the LED state
    digitalWrite(LED_PIN, ledState);
  }
}

void loop() {
  
  if (systemStopped) {
    
    blinkRedLED();  // Blink the LED when the system is stopped
    myServo.write(90);  // Move servo to 0 to shut down the engine

    return;  // Exit the loop when system is stopped
  }
  

  // 1. Read the current voltage from the sensor
  int currentVoltage = voltageSensor.getRmsVoltage();
  Serial.print("Current Voltage: ");
  Serial.println(currentVoltage);

  // 2. Calculate the error
  int error = setpoint - currentVoltage;
  //Serial.print("220 - CURRENT V: ");
  //Serial.println(error);

  // 3. Calculate PID terms
  integral += error;  // Accumulate the integral
  if (integral > integralLimit) integral = integralLimit;  // Prevent integral windup
  if (integral < -integralLimit) integral = -integralLimit;

  float derivative = error - previous_error;  // Calculate the derivative
  int pid_output = (kp * error) + (ki * integral) + (kd * derivative);

  // Constrain the PID output to servo angle limits
  pid_output = constrain(pid_output, minPidOutput, maxPidOutput);

  // Move the servo to the PID-calculated position
  myServo.write(pid_output);
  Serial.print("PID-calculated position: ");
  Serial.println(pid_output);

  // Display the current voltage and servo angle on the OLED
  display.clearDisplay();

  // Draw the degree bar at the top of the display
  drawDegreeBar(pid_output);

  // Draw the voltage bar at the top of the display
  drawVoltageBar(currentVoltage);
  
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.println("Vol: " + String(currentVoltage));
  display.println("Deg: " + String(pid_output));

  // 4. Safety features based on voltage and servo angle
  if (enableSafety && (currentVoltage < lowVoltageThreshold || currentVoltage > highVoltageThreshold) && (pid_output == lowAngleLimit || pid_output == highAngleLimit)) {
    if (lowVoltageStartTime == 0) {
      lowVoltageStartTime = millis();  // Start counting time
    } else if (millis() - lowVoltageStartTime >= safetyTimeLimit) {
      // Shut down system if condition persists for 10 seconds
      systemStopped = true;
      //myServo.write(0);  // Move servo to 0 to shut down the engine
      Serial.println("Safety triggered. System stopped.");
      display.setTextSize(1); 
      display.setCursor(0, 48);
      display.println("Safety triggered!");
    }
  } else {
    lowVoltageStartTime = 0;  // Reset timer if condition is not met
  }

  // 5. Watchdog system
  if (enableWatchdog && abs(error) > maxVoltageChange) {
    if (millis() - watchdogStartTime > watchdogTimeLimit) {
      // Adjust the servo for correction
      int adjustment = pid_output + (error > 0 ? 15 : -15);  // Add or subtract 15 degrees
      myServo.write(constrain(adjustment, minPidOutput, maxPidOutput));
      Serial.println("Watchdog adjustment applied.");
      display.setTextSize(1); 
      display.setCursor(0, 48);
      display.println("Watchdog adjustment");
      watchdogStartTime = 0;  // Reset the watchdog timer
    }
  }

  // 6. VSG simulation
  if (enableVSG) {
    // Apply Virtual Synchronous Generator logic
    // Inertia: smooth out the changes by delaying the response
    float vsg_adjustment = inertiaEmulation * derivative;
    
    // Damping: add a damping factor to reduce oscillations
    vsg_adjustment -= dampingFactor * error;
    
    // Apply the adjustment to the PID output
    pid_output += vsg_adjustment;
    pid_output = constrain(pid_output, minPidOutput, maxPidOutput);
    myServo.write(pid_output);
    Serial.print("VSG adjustment applied: ");
    display.setTextSize(1); 
    display.setCursor(0, 48);
    display.println("VSG adjustment");
    Serial.println(vsg_adjustment);
  }

  // 7. Angle safety feature
  if (pid_output == lowAngleLimit || pid_output == highAngleLimit) {
    if (millis() - angleTimeStart > angleTimeLimit) {
      systemStopped = true;
      display.setTextSize(1); 
      display.setCursor(0, 48);
      display.println("Servo angle safety!");
      myServo.write(0);  // Move servo to 0 to shut down the engine
      delay(ChokeTimeLimit); //Choke time
      
      Serial.println("Servo angle safetY. System stopped.");
    }
  } else {
    angleTimeStart = 0;  // Reset angle time tracking
  }

  // Update previous error for the next cycle
  previous_error = error;

  display.display();  // Update the display with the new values

  // Small delay for stability
  delay(500);
}

// Draw a bar at the top of the OLED to show the voltage visually (relative to setpoint)
void drawVoltageBar(int voltage) {
  int barWidth = map(voltage, 0, highVoltageThreshold, 0, SCREEN_WIDTH);  // Map the voltage to bar width
  
  // Draw the empty bar (background)
  display.drawRect(0, 0, SCREEN_WIDTH, 8, SSD1306_WHITE);  // Draw a rectangle at the top
  // Fill the bar according to the current voltage
  display.fillRect(0, 0, barWidth, 8, SSD1306_WHITE);      // Fill the bar proportionally
}

// Draw a bar at the bottom of the OLED to show the servo angle visually
void drawDegreeBar(int angle) {
  int barWidth = map(angle, 0, 180, 0, SCREEN_WIDTH);  // Map the angle to bar width (0-128 pixels)
  
  // Draw the empty bar (background)
  display.drawRect(0, 56, SCREEN_WIDTH, 8, SSD1306_WHITE);  // Draw a rectangle at the bottom
  // Fill the bar according to the servo angle
  display.fillRect(0, 56, barWidth, 8, SSD1306_WHITE);      // Fill the bar proportionally
}
