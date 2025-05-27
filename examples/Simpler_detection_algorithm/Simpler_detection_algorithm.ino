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
float lastValue = 0;

// Timestamp of the last heartbeat
long lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
  sensor.setLedCurrent(MAX30102::LED_RED, 20);  // 12mA
  sensor.setLedCurrent(MAX30102::LED_IR, 10);   // 10mA
  sensor.setSamplingRate(MAX30102::SAMPLING_RATE_400SPS);
  sensor.setSampleAveraging(MAX30102::SMP_AVE_32);  // Lấy trung bình 32 mẫu
  if(sensor.begin()) { 
    Serial.println("Sensor initialized");  // Labels for Serial Plotter
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}

void loop() {
  auto sample = sensor.readSample(1000);
  float currentValue = sample.red;

  if(currentValue > 10000) {  // Có tiếp xúc
    // Khởi tạo giá trị nếu chưa có
    if(maxValue == 0) {
      maxValue = currentValue;
      minValue = currentValue;
      threshold = currentValue;
      lastValue = currentValue;
    }

    // Cập nhật max/min
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
    Serial.println(lastValue);
  }
  else {  // Không có tiếp xúc
    // Reset tất cả giá trị
    maxValue = 0;
    minValue = 0;
    threshold = 0;
    lastValue = 0;
    lastHeartbeat = 0;
    
    Serial.println("No finger detected");
  }
}