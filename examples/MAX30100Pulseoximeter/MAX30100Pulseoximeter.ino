#include <MAX3010x.h>

// Sensor (adjust to your sensor type)
MAX30102 sensor;

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

void setup() {
  Serial.begin(115200);

  if(sensor.begin()) { 
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}
  
void loop() {
  auto sample = sensor.readSample(1000);
  float currentValue = sample.red;

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
}