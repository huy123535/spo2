#include <MAX3010x.h>
#include "filters1.h"
#include "esp_sleep.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// === WRIST-OPTIMIZED PARAMETERS ===

// Finger Detection - More sensitive for wrist
const unsigned long kFingerThreshold = 5000;  // Reduced for wrist (was 10000)
const unsigned int kFingerCooldownMs = 1000;  // Longer cooldown for stability
const int kFingerDetectionSamples = 10;       // Multiple samples for stable detection

// Signal Quality Assessment
const float kSignalQualityThreshold = 0.3;    // Minimum signal quality
const int kQualityWindowSize = 20;             // Window for quality assessment

// Edge Detection - More sensitive for wrist
const float kEdgeThreshold = -800.0;          // Reduced threshold (was -2000.0)
const float kAdaptiveThresholdFactor = 0.7;   // For adaptive threshold

// Enhanced Filters for wrist measurement
const float kLowPassCutoff = 8.0;             // Slightly higher for better response
const float kHighPassCutoff = 0.4;            // Lower to preserve weak signals
const float kNotchFrequency = 50.0;           // Power line interference
const float kMotionFilterCutoff = 15.0;       // Motion artifact removal

// Averaging and Validation
const bool kEnableAveraging = true;
const int kAveragingSamples = 8;               // Smaller window for faster response
const int kSampleThreshold = 3;                // Fewer samples needed
const int kValidationWindow = 5;               // For BPM validation

// Motion Detection
const float kMotionThreshold = 2000.0;        // Motion detection threshold
const int kMotionStabilityTime = 2000;        // Time to wait after motion (ms)

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);
  
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Wrist Heart Rate Monitor Initialized");
    
    // Optimize sensor settings for wrist measurement
    sensor.setLEDCurrent(sensor.LED_CURRENT_27MA);  // Higher current for wrist
    sensor.setPulseWidth(sensor.PULSE_WIDTH_411US); // Longer pulse width
    sensor.setADCRange(sensor.ADC_RANGE_16384);     // Higher resolution
    
  } else {
    Serial.println("Sensor not found");  
    while(1);
  }
}

// === ENHANCED FILTER INSTANCES ===
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
LowPassFilter motion_filter(kMotionFilterCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;
MovingAverageFilter<kValidationWindow> bpm_validator;

// === STATE VARIABLES ===
long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
int finger_detection_count = 0;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

// Signal quality and motion detection
float signal_baseline = 0;
float signal_amplitude = 0;
long last_motion_time = 0;
bool motion_detected = false;
float adaptive_threshold = kEdgeThreshold;

// Signal quality buffer
float quality_buffer[kQualityWindowSize];
int quality_index = 0;
bool quality_buffer_filled = false;

// BPM validation buffer
int recent_bpm[kValidationWindow] = {0};
int bpm_index = 0;

void updateSignalQuality(float raw_value, float filtered_value) {
  // Store values for quality assessment
  quality_buffer[quality_index] = raw_value;
  quality_index = (quality_index + 1) % kQualityWindowSize;
  
  if (quality_index == 0) quality_buffer_filled = true;
  
  if (quality_buffer_filled) {
    // Calculate signal statistics
    float sum = 0, min_val = quality_buffer[0], max_val = quality_buffer[0];
    
    for (int i = 0; i < kQualityWindowSize; i++) {
      sum += quality_buffer[i];
      if (quality_buffer[i] < min_val) min_val = quality_buffer[i];
      if (quality_buffer[i] > max_val) max_val = quality_buffer[i];
    }
    
    signal_baseline = sum / kQualityWindowSize;
    signal_amplitude = max_val - min_val;
    
    // Update adaptive threshold based on signal amplitude
    if (signal_amplitude > 0) {
      adaptive_threshold = kEdgeThreshold * kAdaptiveThresholdFactor * 
                          (signal_amplitude / 10000.0);
      adaptive_threshold = max(-3000.0f, min(-500.0f, adaptive_threshold));
    }
  }
}

bool isSignalQualityGood() {
  if (!quality_buffer_filled) return false;
  
  // Check if amplitude is sufficient
  if (signal_amplitude < 1000) return false;
  
  // Check signal-to-noise ratio
  float snr = signal_amplitude / (signal_baseline + 1);
  return snr > kSignalQualityThreshold;
}

bool detectMotion(float current_diff) {
  // Simple motion detection based on derivative magnitude
  if (abs(current_diff) > kMotionThreshold) {
    last_motion_time = millis();
    motion_detected = true;
    return true;
  }
  
  // Clear motion flag after stability time
  if (millis() - last_motion_time > kMotionStabilityTime) {
    motion_detected = false;
  }
  
  return motion_detected;
}

bool validateBPM(int bpm) {
  // Store recent BPM
  recent_bpm[bpm_index] = bpm;
  bpm_index = (bpm_index + 1) % kValidationWindow;
  
  // Check consistency of recent BPM values
  int valid_count = 0;
  for (int i = 0; i < kValidationWindow; i++) {
    if (recent_bpm[i] > 0 && 
        abs(recent_bpm[i] - bpm) < 15) { // Within 15 BPM tolerance
      valid_count++;
    }
  }
  
  return valid_count >= (kValidationWindow / 2);
}

void loop() {
  auto sample = sensor.readSample(1000);
  float current_value = sample.ir; // Use IR for better penetration
  
  // === ENHANCED FINGER DETECTION ===
  if(current_value > kFingerThreshold) {
    finger_detection_count++;
    if(finger_detection_count >= kFingerDetectionSamples && 
       millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    finger_detection_count = 0;
    if (finger_detected) {
      // Reset all filters and state
      differentiator.reset();
      averager.reset();
      low_pass_filter.reset();
      high_pass_filter.reset();
      motion_filter.reset();
      
      finger_detected = false;
      finger_timestamp = millis();
      quality_buffer_filled = false;
      quality_index = 0;
      
      Serial.println("No finger detected. Going to deep sleep for 3s...");
      delay(100);
      esp_sleep_enable_timer_wakeup(3000000);
      esp_deep_sleep_start();
    }
  }

  if(finger_detected) {
    // === ENHANCED SIGNAL PROCESSING ===
    
    // Apply motion filter first
    current_value = motion_filter.process(current_value);
    
    // Apply main filters
    current_value = low_pass_filter.process(current_value);
    current_value = high_pass_filter.process(current_value);
    
    // Get derivative
    float current_diff = differentiator.process(current_value);
    
    // Update signal quality metrics
    updateSignalQuality(sample.ir, current_value);
    
    // Check for motion artifacts
    bool has_motion = detectMotion(current_diff);
    
    // Only process heartbeat detection if signal quality is good and no motion
    if(!isnan(current_diff) && !isnan(last_diff) && 
       isSignalQualityGood() && !has_motion) {
      
      // === ENHANCED HEARTBEAT DETECTION ===
      
      // Detect Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat with adaptive threshold
      if(crossed && current_diff < adaptive_threshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          
          int interval = crossed_time - last_heartbeat;
          int bpm = 60000 / interval;
          
          // Enhanced BPM validation
          if(bpm > 45 && bpm < 200 && validateBPM(bpm)) {
            
            if(kEnableAveraging) {
              int average_bpm = averager.process(bpm);
              
              if(averager.count() > kSampleThreshold) {
                Serial.print("Wrist HR (avg): ");
                Serial.print(average_bpm);
                Serial.print(" bpm, Quality: ");
                Serial.print(signal_amplitude);
                Serial.print(", Threshold: ");
                Serial.println(adaptive_threshold);
              }
            } else {
              Serial.print("Wrist HR (current): ");
              Serial.print(bpm);
              Serial.println(" bpm");
            }
          }
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    
    last_diff = current_diff;
    
    // Debug output for signal quality issues
    if (has_motion) {
      Serial.println("Motion detected - pausing detection");
    } else if (!isSignalQualityGood()) {
      Serial.print("Poor signal quality - Amplitude: ");
      Serial.println(signal_amplitude);
    }
  }

  // === OUTPUT FOR SERIAL PLOTTER ===
  Serial.print(current_value);
  Serial.print(",");
  Serial.print(signal_amplitude);
  Serial.print(",");
  Serial.println(adaptive_threshold);

  delay(20); // ~50Hz update rate
} 