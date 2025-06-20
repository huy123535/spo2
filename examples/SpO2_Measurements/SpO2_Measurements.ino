#include <MAX3010x.h>
#include "filters2.h"

// Sensor (adjust to your sensor type)
MAX30101 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 1000;
const unsigned int kFingerCooldownMs = 250;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -200.0;

// Filters
const float kLowPassCutoff = 3.0;
const float kHighPassCutoff = 0.3;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    // Configure Multi-LED mode
    MAX30101::MultiLedConfiguration cfg;
    cfg.slot[0] = MAX30101::SLOT_RED;      // Slot 0: Red LED
    cfg.slot[1] = MAX30101::SLOT_IR;       // Slot 1: IR LED
    cfg.slot[2] = MAX30101::SLOT_GREEN;    // Slot 2: Green LED
    cfg.slot[3] = MAX30101::SLOT_OFF;      // Slot 3: Off
    
    sensor.setMultiLedConfiguration(cfg);
    
    // Set LED currents
    sensor.setLedCurrent(MAX30101::LED_RED, 150);    // Tăng cường độ LED đỏ
    sensor.setLedCurrent(MAX30101::LED_IR, 130);     // Tăng cường độ LED IR
    sensor.setLedCurrent(MAX30101::LED_GREEN_CH1, 150);
    
    // Set to MULTI_LED mode
    sensor.setMode(MAX30101::MODE_MULTI_LED);
    
    // Configure other settings
    sensor.setSamplingRate(MAX30101::SAMPLING_RATE_400SPS);
    sensor.setSampleAveraging(MAX30101::SMP_AVE_16);

    Serial.println("Setup completed");
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }
}

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_green(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
// See https://www.maximintegrated.com/en/design/technical-documents/app-notes/6/6845.html
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

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
  float current_value_green = sample.green;
  
  // Detect Finger using raw sensor value
  if(sample.green > kFingerThreshold) {
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
    low_pass_filter_green.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);
    current_value_green = low_pass_filter_green.process(current_value_green);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_green);
    float current_diff = differentiator.process(current_value);

    // // Debug print để theo dõi giá trị diff và ngưỡng
    // Serial.print("Current diff: ");
    // Serial.print(current_diff);
    // Serial.print(" | Threshold: ");
    // Serial.println(kEdgeThreshold);

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
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
          
          if(bpm > 50 && bpm < 250) {
            // Debug print để theo dõi các giá trị
            Serial.print("R-Red: ");
            Serial.print(rred);
            Serial.print(" | R-IR: ");
            Serial.print(rir);
            Serial.print(" | R-Value: ");
            Serial.println(r);
            
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