#include <MAX3010x.h>
#include "filters1.h"
#include "esp_sleep.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 50;
const int kSampleThreshold = 5;

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}

// Filter Instances
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  auto sample = sensor.readSample(1000);
  float current_value = sample.ir; // Sử dụng IR thay cho Red

  // Detect Finger using raw sensor value
  if(sample.ir > kFingerThreshold) { // Đổi sang IR
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager.reset();
    low_pass_filter.reset();
    high_pass_filter.reset();
    
    finger_detected = false;
    finger_timestamp = millis();

    // Giải pháp phần mềm: sleep ngắn rồi kiểm tra lại
    Serial.println("No finger detected. Going to deep sleep for 3s...");
    delay(100); // Đợi Serial in xong
    esp_sleep_enable_timer_wakeup(3000000); // Sleep 1 giây (1.000.000 us)
    esp_deep_sleep_start();
  }

  if(finger_detected) {
    current_value = low_pass_filter.process(current_value);
    current_value = high_pass_filter.process(current_value);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          if(bpm > 50 && bpm < 250) {
            // Average?
            if(kEnableAveraging) {
              int average_bpm = averager.process(bpm);
  
              // Show if enough samples have been collected
              if(averager.count() > kSampleThreshold) {
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
              }
            }
            else {
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);  
            }
          }
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    last_diff = current_diff;
  }

  // --- Xuất tín hiệu raw, tín hiệu đã lọc và BPM cho Serial Plotter ---
  //Serial.println(sample.red);      // Tín hiệu raw
  //Serial.print(",");
  Serial.println(current_value);   // Tín hiệu đã lọc
  //Serial.print(",");
  //Serial.println(last_diff);

  // if (finger_detected && !isnan(last_diff)) {
  //   int bpm = 0;
  //   if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
  //     bpm = 60000 / (crossed_time - last_heartbeat);
  //   }
  //   Serial.println(bpm); // BPM (nếu có, không thì là 0)
  // } else {
  //   Serial.println(0);   // Nếu chưa phát hiện nhịp, xuất 0
  // }

  delay(20); // Thêm dòng này để giảm tốc độ vẽ đồ thị (~50Hz)
}