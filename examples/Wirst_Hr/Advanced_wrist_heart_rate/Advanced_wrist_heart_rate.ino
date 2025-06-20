#include <MAX3010x.h>
#include "filters1.h"
#include "wrist_filters.h"
#include "esp_sleep.h"

// Sensor (adjust to your sensor type)
MAX30102 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// === ADVANCED WRIST-OPTIMIZED PARAMETERS ===

// Contact Detection - Ultra sensitive for wrist
const unsigned long kFingerThreshold = 3000;  // Very low for wrist
const unsigned int kFingerCooldownMs = 1500;  // Longer stabilization
const int kFingerDetectionSamples = 15;       // More samples for reliability
const float kContactQualityThreshold = 0.6;   // Minimum contact quality

// Signal Quality Requirements
const float kMinSNR = 3.0;                    // Minimum SNR in dB
const float kMinStability = 0.4;              // Minimum signal stability
const float kMinAmplitude = 800.0;            // Minimum signal amplitude

// Advanced Edge Detection
const float kBaseEdgeThreshold = -600.0;      // Base threshold for wrist
const float kAdaptiveRange = 0.5;             // Adaptation range [0.5-1.5]
const float kAdaptationRate = 0.1;            // How fast threshold adapts

// Enhanced Filtering Parameters
const float kLowPassCutoff = 10.0;            // Higher cutoff for responsiveness
const float kHighPassCutoff = 0.3;            // Lower to preserve weak signals
const float kNotchFrequency = 50.0;           // Power line frequency
const float kNotchBandwidth = 2.0;            // Notch filter bandwidth
const float kMotionFilterCutoff = 20.0;       // Motion artifact cutoff

// Advanced Validation
const int kAveragingSamples = 6;               // Fast response averaging
const int kValidationWindow = 4;               // BPM consistency check
const int kSampleThreshold = 2;                // Quick response
const float kBPMTolerance = 20.0;             // BPM validation tolerance
const int kMaxBPMHistory = 10;                // Historical BPM tracking

// Motion and Quality Control
const float kMotionThreshold = 1500.0;        // Motion sensitivity
const int kQualityStabilizationTime = 1000;   // ms to wait for quality
const int kMotionRecoveryTime = 2000;         // ms to recover from motion

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);
  
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("=== Advanced Wrist HR Monitor Initialized ===");
    
    // Optimize sensor for wrist measurement
    sensor.setLedCurrent(MAX30102::LED_RED, 255);  // Maximum current
    
    Serial.println("Sensor optimized for wrist measurement");
    
  } else {
    Serial.println("Sensor initialization failed!");
    while(1);
  }
}

// === ADVANCED FILTER INSTANCES ===
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
LowPassFilter motion_filter(kMotionFilterCutoff, kSamplingFrequency);
NotchFilter notch_filter(kNotchFrequency, kNotchBandwidth, kSamplingFrequency);
KalmanFilter kalman_filter(0.005, 0.1);  // Low process noise, moderate measurement noise
AdaptiveFilter adaptive_filter(0.02);     // Moderate learning rate
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;
SignalQualityEstimator quality_estimator;
MotionDetector motion_detector(kMotionThreshold, 0.6);

// === ADVANCED STATE VARIABLES ===
long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;
int finger_detection_count = 0;

float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

// Advanced signal analysis
float signal_baseline = 0;
float signal_amplitude = 0;
float current_snr = 0;
float signal_stability = 0;
float adaptive_threshold = kBaseEdgeThreshold;

// Motion and quality tracking
long last_motion_time = 0;
long quality_stable_time = 0;
bool motion_detected = false;
bool signal_quality_good = false;

// Historical BPM tracking
int bpm_history[kMaxBPMHistory] = {0};
int bpm_history_index = 0;
int valid_bpm_count = 0;

// Performance metrics
unsigned long total_samples = 0;
unsigned long valid_detections = 0;
unsigned long last_stats_time = 0;

// === ADVANCED FUNCTIONS ===

void updateAdvancedSignalQuality(float raw_ir, float raw_red, float filtered_signal) {
  total_samples++;
  
  // Update quality estimator
  quality_estimator.updateSignal(filtered_signal);
  quality_estimator.updateNoise(raw_ir - filtered_signal);
  
  // Get quality metrics
  current_snr = quality_estimator.getSNR();
  signal_stability = quality_estimator.getSignalStability();
  
  // Calculate amplitude from recent samples
  static float min_recent = filtered_signal, max_recent = filtered_signal;
  static int amplitude_counter = 0;
  
  if(filtered_signal < min_recent) min_recent = filtered_signal;
  if(filtered_signal > max_recent) max_recent = filtered_signal;
  
  amplitude_counter++;
  if(amplitude_counter >= 20) { // Update every 20 samples
    signal_amplitude = max_recent - min_recent;
    min_recent = max_recent = filtered_signal;
    amplitude_counter = 0;
    
    // Adapt threshold based on signal characteristics
    float amplitude_factor = constrain(signal_amplitude / 5000.0, 0.3, 2.0);
    float snr_factor = constrain(current_snr / 10.0, 0.5, 1.5);
    float stability_factor = constrain(signal_stability * 2.0, 0.7, 1.3);
    
    float target_threshold = kBaseEdgeThreshold * amplitude_factor * snr_factor * stability_factor;
    
    // Smooth adaptation
    adaptive_threshold += kAdaptationRate * (target_threshold - adaptive_threshold);
    adaptive_threshold = constrain(adaptive_threshold, -3000.0, -200.0);
  }
}

bool isAdvancedSignalQualityGood() {
  // Multiple quality criteria
  bool snr_good = current_snr >= kMinSNR;
  bool stability_good = signal_stability >= kMinStability;
  bool amplitude_good = signal_amplitude >= kMinAmplitude;
  
  signal_quality_good = snr_good && stability_good && amplitude_good;
  
  if(signal_quality_good) {
    if(quality_stable_time == 0) {
      quality_stable_time = millis();
    }
    return (millis() - quality_stable_time) >= kQualityStabilizationTime;
  } else {
    quality_stable_time = 0;
    return false;
  }
}

bool detectAdvancedMotion(float current_diff, float raw_signal) {
  // Update motion detector
  motion_detector.updateAcceleration(current_diff); // Using derivative as motion proxy
  motion_detector.updatePPG(raw_signal);
  
  if(motion_detector.detectMotion() || abs(current_diff) > kMotionThreshold) {
    last_motion_time = millis();
    motion_detected = true;
    return true;
  }
  
  if(millis() - last_motion_time > kMotionRecoveryTime) {
    motion_detected = false;
  }
  
  return motion_detected;
}

bool validateAdvancedBPM(int bpm) {
  // Store in history
  bpm_history[bpm_history_index] = bpm;
  bpm_history_index = (bpm_history_index + 1) % kMaxBPMHistory;
  if(valid_bpm_count < kMaxBPMHistory) valid_bpm_count++;
  
  if(valid_bpm_count < 3) return true; // Accept first few readings
  
  // Statistical validation
  float mean_bpm = 0;
  for(int i = 0; i < valid_bpm_count; i++) {
    mean_bpm += bpm_history[i];
  }
  mean_bpm /= valid_bpm_count;
  
  // Check if current BPM is within reasonable range of historical mean
  bool statistical_valid = abs(bpm - mean_bpm) <= kBPMTolerance;
  
  // Check recent consistency
  int consistent_count = 0;
  int check_samples = min(valid_bpm_count, kValidationWindow);
  
  for(int i = 0; i < check_samples; i++) {
    int idx = (bpm_history_index - 1 - i + kMaxBPMHistory) % kMaxBPMHistory;
    if(abs(bpm_history[idx] - bpm) <= kBPMTolerance * 0.7) {
      consistent_count++;
    }
  }
  
  bool consistency_valid = consistent_count >= (check_samples / 2);
  
  return statistical_valid && consistency_valid;
}

void printAdvancedDiagnostics() {
  if(millis() - last_stats_time > 5000) { // Every 5 seconds
    last_stats_time = millis();
    
    float detection_rate = (float)valid_detections / total_samples * 100.0;
    
    Serial.println("\n=== ADVANCED DIAGNOSTICS ===");
    Serial.print("Signal Amplitude: "); Serial.println(signal_amplitude);
    Serial.print("SNR: "); Serial.print(current_snr); Serial.println(" dB");
    Serial.print("Stability: "); Serial.println(signal_stability);
    Serial.print("Adaptive Threshold: "); Serial.println(adaptive_threshold);
    Serial.print("Detection Rate: "); Serial.print(detection_rate); Serial.println("%");
    Serial.print("Motion Status: "); Serial.println(motion_detected ? "DETECTED" : "STABLE");
    Serial.print("Quality Status: "); Serial.println(signal_quality_good ? "GOOD" : "POOR");
    Serial.println("============================\n");
  }
}

void loop() {
  auto sample = sensor.readSample(1000);
  float current_value_ir = sample.ir;
  float current_value_red = sample.red;
  
  float processed_signal = current_value_ir;
  // === ADVANCED CONTACT DETECTION ===
  float contact_signal = (current_value_ir + current_value_red) / 2.0;
  
  if(contact_signal > kFingerThreshold) {
    finger_detection_count++;
    if(finger_detection_count >= kFingerDetectionSamples && 
       millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    finger_detection_count = 0;
    if(finger_detected) {
      // Reset all systems
      high_pass_filter.reset();
      low_pass_filter.reset();
      motion_filter.reset();
      notch_filter.reset();
      kalman_filter.reset();
      adaptive_filter.reset();
      differentiator.reset();
      averager.reset();
      quality_estimator.reset();
      motion_detector.reset();
      
      finger_detected = false;
      finger_timestamp = millis();
      signal_quality_good = false;
      quality_stable_time = 0;
      valid_bpm_count = 0;
      
      Serial.println("Contact lost. Entering deep sleep...");
      delay(100);
      esp_sleep_enable_timer_wakeup(3000000);
      esp_deep_sleep_start();
    }
  }

  if(finger_detected) {
    // === ADVANCED SIGNAL PROCESSING PIPELINE ===
    
    // Use IR signal as primary (better penetration for wrist)
    processed_signal = current_value_ir;
    
    // Step 1: Remove power line interference
    processed_signal = notch_filter.process(processed_signal);
    
    // Step 2: Motion artifact pre-filtering
    processed_signal = motion_filter.process(processed_signal);
    
    // Step 3: Bandpass filtering
    processed_signal = low_pass_filter.process(processed_signal);
    processed_signal = high_pass_filter.process(processed_signal);
    
    // Step 4: Kalman smoothing
    processed_signal = kalman_filter.process(processed_signal);
    
    // Step 5: Adaptive noise reduction
    processed_signal = adaptive_filter.process(processed_signal, signal_baseline);
    
    // Step 6: Get derivative for peak detection
    float current_diff = differentiator.process(processed_signal);
    
    // === ADVANCED SIGNAL QUALITY ASSESSMENT ===
    updateAdvancedSignalQuality(current_value_ir, current_value_red, processed_signal);
    
    // === ADVANCED MOTION DETECTION ===
    bool has_motion = detectAdvancedMotion(current_diff, current_value_ir);
    
    // === ADVANCED HEARTBEAT DETECTION ===
    if(!isnan(current_diff) && !isnan(last_diff) && 
       isAdvancedSignalQualityGood() && !has_motion) {
      
      // Zero-crossing detection
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Enhanced peak detection with adaptive threshold
      if(crossed && current_diff < adaptive_threshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 250) { // Min 240 BPM
          
          int interval = crossed_time - last_heartbeat;
          int bpm = 60000 / interval;
          
          // Advanced BPM validation
          if(bpm >= 40 && bpm <= 220 && validateAdvancedBPM(bpm)) {
            valid_detections++;
            
            int average_bpm = averager.process(bpm);
            
            if(averager.count() > kSampleThreshold) {
              Serial.print("❤️ WRIST HR: ");
              Serial.print(average_bpm);
              Serial.print(" bpm (Raw: ");
              Serial.print(bpm);
              Serial.print(") | Q: ");
              Serial.print(current_snr, 1);
              Serial.print("dB | S: ");
              Serial.print(signal_stability, 2);
              Serial.print(" | A: ");
              Serial.println((int)signal_amplitude);
            }
          }
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }
    
    last_diff = current_diff;
    
    // === ADVANCED DIAGNOSTICS ===
    printAdvancedDiagnostics();
    
    // Status feedback
    if(has_motion) {
      Serial.println("⚠️ Motion detected - measurement paused");
    } else if(!isAdvancedSignalQualityGood()) {
      Serial.print("📊 Improving signal quality... SNR: ");
      Serial.print(current_snr, 1);
      Serial.print("dB, Stability: ");
      Serial.println(signal_stability, 2);
    }
  }

  // === ADVANCED OUTPUT FOR ANALYSIS ===
  // Format: ProcessedSignal, SignalAmplitude, AdaptiveThreshold, SNR
  Serial.print(processed_signal);
  Serial.print(",");
  Serial.print(signal_amplitude);
  Serial.print(",");
  Serial.print(adaptive_threshold);
  Serial.print(",");
  Serial.println(current_snr);

  delay(15); // ~67Hz for better resolution
} 