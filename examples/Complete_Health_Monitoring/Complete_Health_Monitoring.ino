/*
 * ESP32 Complete Health Monitor
 * Features: Heart Rate, SpO2, Temperature, Battery monitoring with BLE communication
 * Includes power management with deep sleep and LED status indicators
 */

// Required libraries for sensors and BLE communication
#include <MAX3010x.h>                    // MAX30101 heart rate and SpO2 sensor
#include <Wire.h>                        // I2C communication
#include "filters2.h"                    // Signal processing filters
#include "Protocentral_MAX30205.h"       // MAX30205 temperature sensor
#include <BLEDevice.h>                   // ESP32 BLE functionality
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =============================================================================
// BLE CONFIGURATION
// =============================================================================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// BLE connection management variables
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;           // True when client device is connected
bool oldDeviceConnected = false;        // Previous connection state for comparison

// Individual sensor enable/disable flags
bool temperatureEnabled = false; // Default to OFF
bool heartRateEnabled = true;
bool spo2Enabled = true;

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================
MAX30101 sensor;                         // Heart rate and SpO2 sensor object
MAX30205 tempSensor;                     // Temperature sensor object

// Sensor sampling settings
const auto kSamplingRate = sensor.SAMPLING_RATE_100SPS;    // 100 samples per second
const float kSamplingFrequency = 100.0;                   // Frequency for filter calculations

// Finger detection parameters
const unsigned long kFingerThreshold = 10000;             // Raw signal threshold for finger presence
const unsigned int kFingerCooldownMs = 500;               // Cooldown period after finger removal

// Heart rate detection parameters
const float kEdgeThreshold = -1500.0;                     // Threshold for heartbeat edge detection

// Signal processing filter parameters
const float kLowPassCutoff = 5.0;                         // Low-pass filter cutoff frequency
const float kHighPassCutoff = 0.5;                        // High-pass filter cutoff frequency

// Data validation parameters
const bool kEnableAveraging = false;                      // Enable/disable averaging (currently disabled)
const int kAveragingSamples = 5;                          // Number of samples to average
const int kSampleThreshold = 5;                           // Minimum samples before showing results

// SpO2 calculation coefficients (calibrated for this sensor)
float kSpO2_A = -17;
float kSpO2_B = 104;
// float kSpO2_A = 1.5958422;                                // Quadratic coefficient
// float kSpO2_B = -34.6596622;                              // Linear coefficient  
// float kSpO2_C = 112.6898759;                              // Constant coefficient

// =============================================================================
// SIGNAL PROCESSING FILTERS
// =============================================================================
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);    // Red LED signal filter
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);     // IR LED signal filter
LowPassFilter low_pass_filter_green(kLowPassCutoff, kSamplingFrequency);  // Green LED filter (unused)
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);     // Remove DC component
Differentiator differentiator(kSamplingFrequency);                       // Find signal derivative
MovingAverageFilter<kAveragingSamples> averager_bpm;                      // Heart rate averaging
MovingAverageFilter<kAveragingSamples> averager_r;                        // R-value averaging
MovingAverageFilter<kAveragingSamples> averager_spo2;                     // SpO2 averaging
MovingAverageFilter<kAveragingSamples> averager_temp;                     // Temperature averaging

// Statistical analysis for SpO2 calculation
MinMaxAvgStatistic stat_red;                         // Red LED statistics (min, max, avg)
MinMaxAvgStatistic stat_ir;                          // IR LED statistics (min, max, avg)

// =============================================================================
// GLOBAL STATE VARIABLES
// =============================================================================
// Timing and detection state
long last_heartbeat = 0;                             // Timestamp of last detected heartbeat
long finger_timestamp = 0;                           // Timestamp when finger was removed
bool finger_detected = false;                        // Flag indicating finger presence
float last_diff = NAN;                               // Previous derivative value
bool crossed = false;                                // Flag for zero-crossing detection
long crossed_time = 0;                               // Timestamp of zero-crossing

// Stabilization tracking
const unsigned long HR_SPO2_STABILIZATION_TIME = 15000;  // 15 seconds for heart rate and SpO2
const unsigned long TEMP_STABILIZATION_TIME = 30000;     // 30 seconds for temperature
bool is_stabilized = false;                              // Flag for HR and SpO2
bool temp_stabilized = false;                            // Flag for temperature
unsigned long stabilization_start = 0;                   // When HR/SpO2 stabilizing started
unsigned long temp_stabilization_start = 0;              // When temperature stabilizing started
bool measurement_in_progress = false;                    // Flag to track active measurement session

// Data sending interval
const unsigned long DATA_SEND_INTERVAL = 5000;           // Send data every 5 seconds
unsigned long last_data_send_time = 0;                   // Last time data was sent

// Temperature monitoring
unsigned long last_temp_reading = 0;                     // Last temperature reading timestamp
const unsigned long TEMP_READING_INTERVAL = 15000;       // Read temperature every 15 seconds
float current_temperature = 0.0;                         // Current temperature value

// Current sensor readings (updated in real-time)
int current_bpm = 0;                                     // Current heart rate (beats per minute)
float current_spo2 = 0;                                  // Current SpO2 percentage

// =============================================================================
// BATTERY MONITORING CONFIGURATION
// =============================================================================
const int BATTERY_PIN = 3;                             // GPIO3 connected to voltage divider
const float VOLTAGE_DIVIDER_RATIO = 3.2;               // Voltage divider: (220k + 100k) / 100k = 3.2
const float ADC_REFERENCE_VOLTAGE = 3.3;               // ESP32 ADC reference voltage
const int ADC_RESOLUTION = 4095;                       // 12-bit ADC resolution (2^12 - 1)
const float BATTERY_MAX_VOLTAGE = 4.2;                 // Lithium battery full voltage
const float BATTERY_MIN_VOLTAGE = 3.0;                 // Lithium battery empty voltage
unsigned long last_battery_reading = 0;                // Last battery check timestamp
const unsigned long BATTERY_READING_INTERVAL = 10000;   // Check battery every 10 seconds
float current_battery_percentage = 0;                  // Current battery percentage (0-100%)
float current_battery_voltage = 0;                     // Current battery voltage

// Battery voltage calibration factor
// Adjust this value based on real multimeter measurements
// Example: If multimeter shows 4.00V but code shows 4.33V, set factor = 4.00/4.33 = 0.923
float voltage_calibration_factor = 0.923;

// =============================================================================
// LED STATUS INDICATORS
// =============================================================================
const int LED_BATTERY_PIN = 19;                        // Battery status LED (Green) - GPIO19
const int LED_BLUETOOTH_PIN = 18;                      // Bluetooth connection LED (Blue) - GPIO18
const int LED_WIFI_PIN = 10;                           // WiFi status LED (Red) - GPIO10 (future use)

// =============================================================================
// POWER MANAGEMENT THRESHOLDS
// =============================================================================
const float BATTERY_CRITICAL_VOLTAGE = 3.1;            // Enter deep sleep below this voltage
const float BATTERY_WAKE_VOLTAGE = 3.15;               // Wake up when voltage exceeds this level
const float BATTERY_SAFE_PERCENTAGE = 15;              // Turn off battery LED below this percentage

// =============================================================================
// LED STATUS MANAGEMENT FUNCTIONS
// =============================================================================

/**
 * Update all status LEDs based on current system state
 * - Battery LED: Shows battery health (green when safe)
 * - Bluetooth LED: Shows BLE connection status (blue when connected)
 * - WiFi LED: Reserved for future WiFi functionality
 */
void updateStatusLEDs() {
  // Battery LED (Green): ON when battery percentage is above safe threshold
  if (current_battery_percentage > BATTERY_SAFE_PERCENTAGE) {
    digitalWrite(LED_BATTERY_PIN, HIGH);        // Battery safe - LED ON
  } else {
    digitalWrite(LED_BATTERY_PIN, LOW);         // Battery low - LED OFF
  }
  
  // Bluetooth LED (Blue): ON when device is connected via BLE
  if (deviceConnected) {
    digitalWrite(LED_BLUETOOTH_PIN, HIGH);      // Connected - LED ON
  } else {
    digitalWrite(LED_BLUETOOTH_PIN, LOW);       // Disconnected - LED OFF
  }
  
  // WiFi LED (Red): Reserved for future use, currently OFF
  digitalWrite(LED_WIFI_PIN, LOW);
}

// =============================================================================
// POWER MANAGEMENT FUNCTIONS
// =============================================================================

/**
 * Enter deep sleep mode to preserve battery when critically low
 * System will wake up every 30 seconds to check if battery has recovered
 * All LEDs are turned off and BLE is disconnected before sleeping
 */
void enterDeepSleep() {
  Serial.println("🔋 CRITICAL BATTERY LEVEL!");
  Serial.println("💤 Entering deep sleep to preserve battery...");
  
  // Turn off all status LEDs to save power
  digitalWrite(LED_BATTERY_PIN, LOW);
  digitalWrite(LED_BLUETOOTH_PIN, LOW);
  digitalWrite(LED_WIFI_PIN, LOW);
  
  // Properly disconnect BLE to avoid connection issues on wake
  if (deviceConnected) {
    pServer->getAdvertising()->stop();
    BLEDevice::deinit();
  }
  
  // Configure timer wake-up every 30 seconds to check battery recovery
  esp_sleep_enable_timer_wakeup(30 * 1000000);   // 30 seconds in microseconds
  
  Serial.println("💤 Going to sleep... Will check battery every 30 seconds");
  delay(1000);                                   // Allow time for serial output
  
  esp_deep_sleep_start();                        // Enter deep sleep (execution stops here)
}

/**
 * Check if battery voltage is critically low and enter deep sleep if necessary
 * @return true if entering deep sleep, false if battery is OK
 */
bool checkBatteryAndSleep() {
  if (current_battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
    enterDeepSleep();                             // This function never returns
    return true;                                  // Never reached, but for completeness
  }
  return false;                                   // Battery is OK, continue normal operation
}

// =============================================================================
// BLE CALLBACK CLASSES
// =============================================================================

/**
 * BLE Server Callbacks - Handle connection/disconnection events
 */
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;                      // Set connection flag
    Serial.println("📱 Device connected");
    updateStatusLEDs();                          // Update LED status
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;                     // Clear connection flag
    Serial.println("📱 Device disconnected");
    updateStatusLEDs();                          // Update LED status
  }
};

/**
 * Read battery voltage using ADC with averaging for better accuracy
 * @return Calibrated battery voltage in volts
 */
float readBatteryVoltage() {
  // Take multiple readings for better accuracy
  int total = 0;
  const int samples = 10;                        // Number of ADC samples to average
  
  for(int i = 0; i < samples; i++) {
    total += analogRead(BATTERY_PIN);
    delay(1);                                    // Small delay between readings
  }
  
  float adcValue = total / samples;              // Calculate average ADC value
  float voltage = (adcValue * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_RATIO;
  
  // Apply calibration factor
  voltage = voltage * voltage_calibration_factor;
  
  return voltage;
}

/**
 * Convert battery voltage to percentage (0-100%)
 * @param voltage Battery voltage in volts
 * @return Battery percentage (0-100%)
 */
float calculateBatteryPercentage(float voltage) {
  if (voltage >= BATTERY_MAX_VOLTAGE) {
    return 100.0;                                // Battery full
  } else if (voltage <= BATTERY_MIN_VOLTAGE) {
    return 0.0;                                  // Battery empty
  } else {
    // Linear interpolation between min and max voltage
    return ((voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0;
  }
}

/**
 * Send current sensor data via BLE to connected device
 * Data format varies based on current measurement mode
 */
void sendSensorData() {
  if (!deviceConnected) {
    return;                                      // Exit if no device connected
  }

  // Create JSON string with current sensor values based on enabled sensors
  String sensorData = "{";
  bool hasData = false;
  
  // Add heart rate if enabled, stabilized and valid (40-180 BPM)
  if (heartRateEnabled && is_stabilized && current_bpm >= 40 && current_bpm <= 180) {
    if (hasData) sensorData += ", ";
    sensorData += "\"heartRate\": " + String(current_bpm);
    hasData = true;
  }
  
  // Add SpO2 if enabled, stabilized and valid (80-100%)
  if (spo2Enabled && is_stabilized && current_spo2 >= 80 && current_spo2 <= 100) {
    if (hasData) sensorData += ", ";
    sensorData += "\"spo2\": " + String(current_spo2);
    hasData = true;
  }
  
  // Add temperature if enabled, stabilized and valid (32-42°C)
  if (temperatureEnabled && temp_stabilized && current_temperature >= 32 && current_temperature <= 42) {
    if (hasData) sensorData += ", ";
    sensorData += "\"temperature\": " + String(current_temperature, 1);
    hasData = true;
  }
  
  // Always include battery and enabled sensors status
  if (hasData) sensorData += ", ";
  sensorData += "\"batteryPercentage\": " + String(current_battery_percentage, 1);
  sensorData += ", \"enabledSensors\": {";
  sensorData += "\"heartRate\": " + String(heartRateEnabled ? "true" : "false");
  sensorData += ", \"spo2\": " + String(spo2Enabled ? "true" : "false");
  sensorData += ", \"temperature\": " + String(temperatureEnabled ? "true" : "false");
  sensorData += "}";
  
  sensorData += "}";
  
  try {
    pCharacteristic->setValue(sensorData.c_str());  // Set characteristic value
    pCharacteristic->notify();                       // Notify connected device
    Serial.println("Data sent successfully: " + sensorData);
  } catch (const std::exception& e) {
    Serial.println("Error sending data: " + String(e.what()));
  }
}

/**
 * Update individual sensor enable/disable status
 * @param tempEnabled Enable/disable temperature sensor
 * @param hrEnabled Enable/disable heart rate sensor  
 * @param spo2Enabled Enable/disable SpO2 sensor
 */
void updateSensorStatus(bool tempEnabled, bool hrEnabled, bool spo2En) {
  Serial.println("=== Updating Sensor Status ===");
  Serial.println("Previous state - Temp: " + String(temperatureEnabled ? "ON" : "OFF") + 
                ", HR: " + String(heartRateEnabled ? "ON" : "OFF") + 
                ", SpO2: " + String(spo2Enabled ? "ON" : "OFF"));
  
  // Record time when temperature is enabled
  if (!temperatureEnabled && tempEnabled) {
    temp_stabilization_start = millis();
    temp_stabilized = false;
    Serial.println("Temperature sensor enabled - will start displaying after 30 seconds");
  } else if (temperatureEnabled && !tempEnabled) {
    // Reset temperature stabilization when disabled
    temp_stabilized = false;
    Serial.println("Temperature sensor disabled - stabilization reset");
  }
  
  // Record time when HR/SpO2 is enabled
  if ((!heartRateEnabled && hrEnabled) || (!spo2Enabled && spo2En)) {
    stabilization_start = millis();
    is_stabilized = false;
    Serial.println("Heart rate/SpO2 sensor enabled - will start displaying after 15 seconds");
  }
  
  temperatureEnabled = tempEnabled;
  heartRateEnabled = hrEnabled;
  spo2Enabled = spo2En;
  
  Serial.println("New state - Temp: " + String(temperatureEnabled ? "ON" : "OFF") + 
                ", HR: " + String(heartRateEnabled ? "ON" : "OFF") + 
                ", SpO2: " + String(spo2Enabled ? "ON" : "OFF"));
  
  // Reset all filters and statistics when changing sensor status
  if (!heartRateEnabled && !spo2Enabled) {
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    // Reset detection variables
    finger_detected = false;
    finger_timestamp = millis();
    last_heartbeat = 0;
    crossed = false;
    last_diff = NAN;
    current_bpm = 0;
    current_spo2 = 0;
  }
  
  // Send status update via BLE
  String statusResponse = "{\"status\": \"sensors_updated\", \"enabledSensors\": {";
  statusResponse += "\"heartRate\": " + String(heartRateEnabled ? "true" : "false");
  statusResponse += ", \"spo2\": " + String(spo2Enabled ? "true" : "false");
  statusResponse += ", \"temperature\": " + String(temperatureEnabled ? "true" : "false");
  statusResponse += "}}";
  
  Serial.println("Sending status response: " + statusResponse);
  
  if (deviceConnected) {
    try {
      pCharacteristic->setValue(statusResponse.c_str());
      pCharacteristic->notify();
      Serial.println("Status response sent successfully");
    } catch (const std::exception& e) {
      Serial.println("Error sending sensor status: " + String(e.what()));
    }
  } else {
    Serial.println("Device not connected - cannot send status");
  }
  
  Serial.println("=== Sensor Status Update Complete ===");
}

/**
 * BLE Characteristic Callbacks - Handle read/write operations from client
 */
class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received Value: ");
      Serial.println(value.c_str());
    }

    // Handle different BLE commands from client
    // Simple individual sensor control commands
    if (value == "TEMP_ON") {
      temperatureEnabled = true;
      Serial.println("Temperature sensor ENABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else if (value == "TEMP_OFF") {
      temperatureEnabled = false;
      Serial.println("Temperature sensor DISABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else if (value == "HR_ON") {
      heartRateEnabled = true;
      Serial.println("Heart Rate sensor ENABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else if (value == "HR_OFF") {
      heartRateEnabled = false;
      Serial.println("Heart Rate sensor DISABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else if (value == "SPO2_ON") {
      spo2Enabled = true;
      Serial.println("SpO2 sensor ENABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else if (value == "SPO2_OFF") {
      spo2Enabled = false;
      Serial.println("SpO2 sensor DISABLED");
      updateSensorStatus(temperatureEnabled, heartRateEnabled, spo2Enabled);
    }
    else {
      Serial.println("Unknown command: " + String(value.c_str()));
    }
  }
};

// =============================================================================
// MAIN SETUP FUNCTION
// =============================================================================

void setup() {
  Serial.begin(115200);                         // Initialize serial communication
  Wire.begin(1,0);                              // Initialize I2C (SDA=1, SCL=0)

  // Initialize status LEDs
  pinMode(LED_BATTERY_PIN, OUTPUT);
  pinMode(LED_BLUETOOTH_PIN, OUTPUT);
  pinMode(LED_WIFI_PIN, OUTPUT);
  
  // Turn off all LEDs initially
  digitalWrite(LED_BATTERY_PIN, LOW);
  digitalWrite(LED_BLUETOOTH_PIN, LOW);
  digitalWrite(LED_WIFI_PIN, LOW);
  
  Serial.println("🚀 ESP32 Health Monitor Starting...");
  
  // Check if waking up from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    Serial.println("⏰ Woke up from deep sleep - checking battery...");
  }
  
  // Initialize battery monitoring
  pinMode(BATTERY_PIN, INPUT);
  analogReadResolution(12);                      // Set ADC resolution to 12 bits
  
  // Read initial battery level
  current_battery_voltage = readBatteryVoltage();
  current_battery_percentage = calculateBatteryPercentage(current_battery_voltage);
  Serial.print("🔋 Initial Battery Voltage: ");
  Serial.print(current_battery_voltage, 2);
  Serial.print("V, Percentage: ");
  Serial.print(current_battery_percentage, 1);
  Serial.println("%");
  
  // Check if battery is too low for operation
  if (current_battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
    Serial.println("⚠️  Battery too low for operation!");
    enterDeepSleep();
  }
  
  // If battery is okay but we woke from sleep, check if it's recovered enough
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    if (current_battery_voltage < BATTERY_WAKE_VOLTAGE) {
      Serial.println("🔋 Battery not recovered enough, going back to sleep...");
      enterDeepSleep();
    } else {
      Serial.println("✅ Battery recovered! Continuing normal operation...");
    }
  }
  
  // Update status LEDs based on current state
  updateStatusLEDs();

  // Initialize BLE device
  BLEDevice::init("Health Monitor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristic with read/write/notify properties
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );

  // Set callback for characteristic operations
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // Add BLE Descriptor for notifications
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the BLE service
  pService->start();

  // Start BLE advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE device started, waiting for connections...");
  
  // Initialize MAX30101 heart rate and SpO2 sensor
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    // Set LED current levels for optimal performance
    sensor.setLedCurrent(MAX30101::LED_RED, 200);
    sensor.setLedCurrent(MAX30101::LED_IR, 180);
    
    // Set SpO2 mode
    sensor.setMode(MAX30101::MODE_SPO2);
    sensor.setSamplingRate(MAX30101::SAMPLING_RATE_400SPS);
    sensor.setSampleAveraging(MAX30101::SMP_AVE_NONE);
    
    Serial.println("MAX30101 sensor initialized in SpO2 mode");
  } else {
    Serial.println("MAX30101 sensor not found");
    while(1);                                    // Stop execution if sensor not found
  }

  // Initialize MAX30205 temperature sensor
  if(tempSensor.scanAvailableSensors()) {
    tempSensor.begin();
    Serial.println("MAX30205 temperature sensor initialized");
  } else {
    Serial.println("MAX30205 temperature sensor not found");
    while(1);                                    // Stop execution if sensor not found
  }
  
  // Print initial sensor status
  Serial.println("✅ Continuous monitoring mode - Data will be sent automatically");
  Serial.println("✅ Temperature sensor: " + String(temperatureEnabled ? "ENABLED" : "DISABLED"));
  Serial.println("✅ Heart Rate sensor: " + String(heartRateEnabled ? "ENABLED" : "DISABLED"));
  Serial.println("✅ SpO2 sensor: " + String(spo2Enabled ? "ENABLED" : "DISABLED"));
  
  if (temperatureEnabled) {
    temp_stabilization_start = millis();
    temp_stabilized = false;
    Serial.println("Temperature sensor will start displaying after 30 seconds");
  }
  
  if (heartRateEnabled || spo2Enabled) {
    stabilization_start = millis();
    is_stabilized = false;
    Serial.println("Heart rate/SpO2 sensors will start displaying after 15 seconds");
  }
}

// =============================================================================
// MAIN LOOP FUNCTION
// =============================================================================

void loop() {
  // Handle BLE connection state changes
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                                  // Give bluetooth stack time to get ready
    pServer->startAdvertising();                 // Restart advertising
    Serial.println("Start advertising after disconnect");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("📱 New device connected, starting data transmission");
  }

  // Read MAX30101 sensor data based on current mode
  auto sample = sensor.readSample(1000);         // Read with 1 second timeout
  float current_value_red = sample.red;          // Red LED value
  float current_value_ir = sample.ir;            // IR LED value
  
  // Detect finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      if (!finger_detected) {
        // First time finger is detected in this session
        finger_detected = true;
        measurement_in_progress = true;
        is_stabilized = false;
        stabilization_start = millis();
        Serial.println("👆 Finger detected - Starting stabilization period...");
      }
    }
  } else {
    // Reset all processing when finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    is_stabilized = false;
    measurement_in_progress = false;
    finger_timestamp = millis();
    Serial.println("👆 Finger removed - Measurement stopped");
  }

  // Read temperature periodically (every 15 seconds)
  if (temperatureEnabled && (millis() - last_temp_reading >= TEMP_READING_INTERVAL)) {
    float temp = tempSensor.oneShotTemperature(); // Get temperature reading
    current_temperature = averager_temp.process(temp); // Apply averaging
    
    // Check if temperature stabilization time has passed
    if (!temp_stabilized && millis() - temp_stabilization_start >= TEMP_STABILIZATION_TIME) {
      temp_stabilized = true;
      Serial.println("✅ Temperature readings stabilized after 30 seconds");
    }
    
    if (averager_temp.count() >= kSampleThreshold) {
      Serial.print("Temperature: ");
      Serial.print(current_temperature, 2);
      Serial.println("°C");
      
      if (!temp_stabilized) {
        // Show stabilization progress
        int progress = ((millis() - temp_stabilization_start) * 100) / TEMP_STABILIZATION_TIME;
        Serial.print("⏳ Temperature stabilizing: ");
        Serial.print(progress);
        Serial.println("%");
      }
    }
    
    last_temp_reading = millis();
  }

  // Read battery level periodically (every 5 seconds)
  if (millis() - last_battery_reading >= BATTERY_READING_INTERVAL) {
    current_battery_voltage = readBatteryVoltage();
    current_battery_percentage = calculateBatteryPercentage(current_battery_voltage);
    
    Serial.print("🔋 Battery Voltage: ");
    Serial.print(current_battery_voltage, 2);
    Serial.print("V, Percentage: ");
    Serial.print(current_battery_percentage, 1);
    Serial.println("%");
    
    // Update status LEDs based on battery and connection state
    updateStatusLEDs();
    
    // Check for critical battery level and enter deep sleep if needed
    if (checkBatteryAndSleep()) {
      return;                                    // This will never be reached, but for safety
    }
    
    // Warning for low battery (below 20%)
    if (current_battery_percentage < 20) {
      Serial.println("⚠️  LOW BATTERY WARNING!");
    }
    
    last_battery_reading = millis();
  }

  // Process heart rate and SpO2 only when finger is detected
  if(finger_detected) {
    // Check if we're still in stabilization period for HR/SpO2
    if (!is_stabilized && millis() - stabilization_start >= HR_SPO2_STABILIZATION_TIME) {
      is_stabilized = true;
      Serial.println("✅ Heart rate/SpO2 readings stabilized after 15 seconds");
    } else if (!is_stabilized) {
      // Still stabilizing - calculate and show progress
      int progress = ((millis() - stabilization_start) * 100) / HR_SPO2_STABILIZATION_TIME;
      Serial.print("⏳ HR/SpO2 Stabilizing: ");
      Serial.print(progress);
      Serial.println("%");
    }

    // Apply low-pass filter to red LED signal
    current_value_red = low_pass_filter_red.process(current_value_red);
    
    // Only process IR if SpO2 is enabled (SpO2 mode required)
    if (spo2Enabled) {
      current_value_ir = low_pass_filter_ir.process(current_value_ir);
      // Collect statistics for pulse oximetry calculation
      stat_red.process(current_value_red);
      stat_ir.process(current_value_ir);
    }

    // Heart beat detection - Run when EITHER heart rate OR SpO2 is enabled
    // (because SpO2 calculation requires heartbeat detection)
    bool shouldDetectHeartbeat = (heartRateEnabled || spo2Enabled);
    
    if (shouldDetectHeartbeat) {
      float current_value = high_pass_filter.process(current_value_red); // Remove DC component
      float current_diff = differentiator.process(current_value);        // Get derivative

      // Valid values check
      if(!isnan(current_diff) && !isnan(last_diff)) {
        
        // Detect heartbeat - zero-crossing detection
        if(last_diff > 0 && current_diff < 0) {
          crossed = true;                          // Signal crossed zero downward
          crossed_time = millis();
        }
        
        if(current_diff > 0) {
          crossed = false;                         // Reset crossing flag when going positive
        }
    
        // Detect heartbeat - falling edge threshold
        if(crossed && current_diff < kEdgeThreshold) {
          if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
            // Calculate heart rate from time between beats
            int bpm = 60000/(crossed_time - last_heartbeat);
            
            // Validate heart rate range (50-250 BPM)
            if(bpm > 50 && bpm < 250) {
              // Always calculate BPM (needed for SpO2 even if HR not reported)
              if(kEnableAveraging) {
                current_bpm = averager_bpm.process(bpm);
              } else {
                current_bpm = bpm;
              }
              
              // Calculate SpO2 if enabled and in SpO2 mode
              if (spo2Enabled) {
                // Calculate R-ratio for SpO2
                float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
                float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
                float r = rred/rir;
                // Apply calibrated SpO2 formula
                float spo2 = kSpO2_A * r + kSpO2_B;
                current_spo2 = spo2;
                
                // Reset statistics for next measurement
                stat_red.reset();
                stat_ir.reset();
              }
              
              // Print results to serial monitor based on enabled sensors
              Serial.print("Time (ms): ");
              Serial.println(millis()); 
              
              // Always print detected BPM for debugging (internal calculation)
              Serial.print("Detected Heart Rate (internal): ");
              Serial.println(bpm);
              
              // Only print user-facing data if enabled
              if (heartRateEnabled) {
                Serial.print("Heart Rate (reported): ");
                Serial.println(current_bpm);
              }
              
              if (spo2Enabled) {
                Serial.print("SpO2 (reported): ");
                Serial.println(current_spo2);
              }
              
              Serial.print("Battery: ");
              Serial.print(current_battery_percentage, 1);
              Serial.println("%");
            }
          }
    
          crossed = false;                         // Reset crossing flag
          last_heartbeat = crossed_time;           // Update last heartbeat time
        }
      }

      last_diff = current_diff;                    // Store current derivative for next iteration
    }

    // Send data via BLE periodically (every 5 seconds) if connected
    if (deviceConnected && (millis() - last_data_send_time >= DATA_SEND_INTERVAL)) {
      sendSensorData();
      last_data_send_time = millis();
    }
  }
  
  // delay(20);                                     // Small delay to prevent overwhelming the serial output
}