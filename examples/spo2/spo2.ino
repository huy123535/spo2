#include <MAX3010x.h>
#include "filters.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 5000;
const unsigned int kFingerCooldownMs = 100;

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
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// SpO2 Lookup table for R values from 0.0 to 12.0 (121 entries)
const uint8_t spO2LUT[121] = {
    99, 99, 98, 98, 98, 97, 97, 97, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93,
    92, 92, 92, 91, 91, 91, 91, 90, 90, 89, 89, 89, 88, 88, 88, 88, 87, 87, 87, 86,
    86, 86, 85, 85, 85, 84, 84, 84, 83, 83, 83, 82, 82, 82, 81, 81, 81, 81, 80, 80,
    80, 79, 79, 79, 78, 78, 78, 77, 77, 77, 76, 76, 76, 76, 75, 75, 75, 74, 74, 74,
    73, 73, 73, 72, 72, 72, 72, 71, 71, 71, 70, 70, 70, 69, 69, 69, 69, 68, 68, 68,
    67, 67, 67, 66, 66, 66, 65, 65, 65, 65, 64
};

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
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
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
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
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
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          
          // Replace polynomial calculation with lookup table
          int r_index = constrain((int)(r * 10.0), 0, 120);  // Scale and constrain R value
          float spo2 = spO2LUT[r_index];
          
          // Average?
          if(kEnableAveraging) {
            int average_bpm = averager_bpm.process(bpm);
            int average_r = averager_r.process(r);
            int average_spo2 = averager_spo2.process(spo2);
  
            // Show if enough samples have been collected
            if(averager_bpm.count() >= kSampleThreshold) {
              Serial.print("Time (ms): ");
              Serial.println(millis()); 
              Serial.print("Heart Rate (avg, bpm): ");
              Serial.println(average_bpm);
              Serial.print("R-Value (avg): ");
              Serial.println(average_r);  
              Serial.print("SpO2 (avg, %): ");
              Serial.println(average_spo2);  
            }
          }
          else {
            Serial.print("Time (ms): ");
            Serial.println(millis()); 
            Serial.print("Heart Rate (current, bpm): ");
            Serial.println(bpm);  
            Serial.print("R-Value (current): ");
            Serial.println(r);
            Serial.print("SpO2 (current, %): ");
            Serial.println(spo2);   
          }
          

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}