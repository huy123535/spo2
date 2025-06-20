#include <MAX3010x.h>

// Sensor (adjust to your sensor type)
MAX30101 sensor;

// Constants
const float rThreshold = 0.7;
const float decayRate = 0.02;
const float thrRate = 0.05;
const int minDiff = 50;

// Current values
float maxValue = 0;
float minValue = 0;
float threshold = 0;

// Timestamp of the last heartbeat
long lastHeartbeat = 0;

// Last value to detect crossing the threshold
float lastValue = 0;

void scanI2C() {
  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  Serial.println("Scan complete");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);  // Initialize I2C with SDA=1, SCL=0 for ESP32
  
  // Scan for I2C devices first
  scanI2C();
  
  if(sensor.begin()) { 
    Serial.println("Basic initialization successful");
    
    // Reset sensor
    if(sensor.reset()) {
      Serial.println("Reset successful");
    } else {
      Serial.println("Reset failed");
    }
    delay(100);
    
    // Configure Multi-LED mode
    MAX30101::MultiLedConfiguration cfg;
    cfg.slot[0] = MAX30101::SLOT_RED;      // Slot 0: Red LED
    cfg.slot[1] = MAX30101::SLOT_IR;       // Slot 1: IR LED
    cfg.slot[2] = MAX30101::SLOT_GREEN;    // Slot 2: Green LED
    cfg.slot[3] = MAX30101::SLOT_OFF;      // Slot 3: Off
    
    if(sensor.setMultiLedConfiguration(cfg)) {
      Serial.println("Multi-LED configuration successful");
    } else {
      Serial.println("Multi-LED configuration failed");
    }
    delay(100);
    
    // Set LED currents
    if(sensor.setLedCurrent(MAX30101::LED_RED, 150)) {
      Serial.println("Set RED LED current successful");
    }
    if(sensor.setLedCurrent(MAX30101::LED_IR, 100)) {
      Serial.println("Set IR LED current successful");
    }
    if(sensor.setLedCurrent(MAX30101::LED_GREEN_CH1, 200)) {
      Serial.println("Set GREEN LED current successful");
    }
    delay(100);
    
    // Set to MULTI_LED mode
    if(sensor.setMode(MAX30101::MODE_MULTI_LED)) {
      Serial.println("Set to MULTI_LED mode successful");
    } else {
      Serial.println("Set to MULTI_LED mode failed");
    }
    delay(100);
    
    // Configure sensor settings
    if(sensor.setResolution(MAX30101::RESOLUTION_18BIT_4110US)) {
      Serial.println("Set resolution successful");
    }
    if(sensor.setSamplingRate(MAX30101::SAMPLING_RATE_400SPS)) {
      Serial.println("Set sampling rate successful");
    }
    if(sensor.setADCRange(MAX30101::ADC_RANGE_16384NA)) {
      Serial.println("Set ADC range successful");
    }
    
    // Enable FIFO rollover
    if(sensor.enableFIFORollover()) {
      Serial.println("Enable FIFO rollover successful");
    }
    
    // Clear FIFO
    if(sensor.clearFIFO()) {
      Serial.println("Clear FIFO successful");
    }
    
    Serial.println("Sensor fully initialized");
    Serial.println("Reading values...");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}
  
void loop() {
  auto sample = sensor.readSample(1000);
  
  // Print raw values from all slots for debugging
  Serial.print("Red: "); Serial.print(sample.red);
  Serial.print(" IR: "); Serial.print(sample.ir);
  Serial.print(" Green: "); Serial.print(sample.slot[2]); // Green LED is in slot 2
  Serial.println();
  
  float currentValue = sample.slot[2]; // Use green LED data from slot 2

  maxValue = max(maxValue, currentValue);
  minValue = min(minValue, currentValue);

  // Detect Heartbeat
  float nthreshold = (maxValue - minValue) * rThreshold + minValue;
  threshold = threshold * (1-thrRate) + nthreshold * thrRate;
  threshold = min(maxValue, max(minValue, threshold));
  
  if(currentValue >= threshold 
      && lastValue < threshold 
      && (maxValue-minValue) > minDiff 
      && millis() - lastHeartbeat > 300) {
        
    if(lastHeartbeat != 0) {
      // Show Results
      int bpm = 60000/(millis() - lastHeartbeat);
      if(bpm > 50 && bpm < 250) {
        Serial.print("Heart Rate (bpm): ");
        Serial.println(bpm);
      }
    }
    lastHeartbeat = millis();
  }

  // Decay for max/min
  maxValue -= (maxValue-currentValue)*decayRate;
  minValue += (currentValue-minValue)*decayRate;

  lastValue = currentValue;
  
  delay(100); // Add small delay to make serial output readable
}