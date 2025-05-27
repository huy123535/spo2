#include <MAX3010x.h>
#include "filters2.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
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
HighPassFilter high_pass_filter_red(kHighPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter_ir(kHighPassCutoff, kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// R value to SpO2 calibration factors
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

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
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter_red.reset();
    high_pass_filter_ir.reset();
    stat_red.reset();
    stat_ir.reset();
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    // Filter signals
    float filtered_red = low_pass_filter_red.process(current_value_red);
    filtered_red = high_pass_filter_red.process(filtered_red);
    float filtered_ir = low_pass_filter_ir.process(current_value_ir);
    filtered_ir = high_pass_filter_ir.process(filtered_ir);

    // Statistics for pulse oximetry
    stat_red.process(filtered_red);
    stat_ir.process(filtered_ir);

    // Calculate SpO2
    float rred = NAN, rir = NAN, r = NAN, spo2 = NAN;
    if(stat_red.average() != 0 && stat_ir.average() != 0) {
      rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
      rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
      if(rir != 0) {
        r = rred/rir;
        spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
      }
    }

    if(!isnan(r) && !isnan(spo2)) {
      if(kEnableAveraging) {
        float average_r = averager_r.process(r);
        float average_spo2 = averager_spo2.process(spo2);
        if(averager_r.count() >= kSampleThreshold) {
          Serial.print("Time (ms): ");
          Serial.println(millis()); 
          Serial.print("R-Value (avg): ");
          Serial.println(average_r);  
          Serial.print("SpO2 (avg, %): ");
          Serial.println(average_spo2);  
        }
      }
      else {
        Serial.print("Time (ms): ");
        Serial.println(millis()); 
        Serial.print("R-Value (current): ");
        Serial.println(r);
        Serial.print("SpO2 (current, %): ");
        Serial.println(spo2);   
      }
    }
    
    // Reset statistics periodically
    if(millis() % 1000 == 0) {
      stat_red.reset();
      stat_ir.reset();
    }
  }
}