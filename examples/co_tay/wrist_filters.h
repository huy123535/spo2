#ifndef WRIST_FILTERS_H
#define WRIST_FILTERS_H

#include "filters1.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265359
#endif

/**
 * @brief Notch Filter for power line interference (50Hz/60Hz)
 */
class NotchFilter {
private:
  float kFreq;
  float kBandwidth;
  float kSamplingFreq;
  float kR;
  float kCosTheta;
  
  // Filter coefficients
  float b0, b1, b2, a1, a2;
  
  // Delay elements
  float x1, x2, y1, y2;
  
public:
  NotchFilter(float notch_freq, float bandwidth, float sampling_freq) :
    kFreq(notch_freq),
    kBandwidth(bandwidth),
    kSamplingFreq(sampling_freq),
    x1(0), x2(0), y1(0), y2(0) {
    
    // Calculate filter parameters
    kR = 1.0 - (kBandwidth * PI / kSamplingFreq);
    kCosTheta = cos(2.0 * PI * kFreq / kSamplingFreq);
    
    // Calculate coefficients
    b0 = 1.0;
    b1 = -2.0 * kCosTheta;
    b2 = 1.0;
    a1 = -2.0 * kR * kCosTheta;
    a2 = kR * kR;
  }
  
  float process(float input) {
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    
    // Update delay elements
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    
    return output;
  }
  
  void reset() {
    x1 = x2 = y1 = y2 = 0;
  }
};

/**
 * @brief Adaptive Filter for dynamic noise reduction
 */
class AdaptiveFilter {
private:
  static const int kFilterLength = 8;
  float weights[kFilterLength];
  float buffer[kFilterLength];
  float mu; // Learning rate
  int index;
  
public:
  AdaptiveFilter(float learning_rate = 0.01) : 
    mu(learning_rate), index(0) {
    
    // Initialize weights and buffer
    for(int i = 0; i < kFilterLength; i++) {
      weights[i] = 0.0;
      buffer[i] = 0.0;
    }
    weights[0] = 1.0; // Initial weight
  }
  
  float process(float input, float reference = 0) {
    // Add input to circular buffer
    buffer[index] = input;
    index = (index + 1) % kFilterLength;
    
    // Calculate output
    float output = 0;
    for(int i = 0; i < kFilterLength; i++) {
      int buf_idx = (index - 1 - i + kFilterLength) % kFilterLength;
      output += weights[i] * buffer[buf_idx];
    }
    
    // Calculate error
    float error = reference - output;
    
    // Update weights (LMS algorithm)
    for(int i = 0; i < kFilterLength; i++) {
      int buf_idx = (index - 1 - i + kFilterLength) % kFilterLength;
      weights[i] += mu * error * buffer[buf_idx];
    }
    
    return output;
  }
  
  void reset() {
    for(int i = 0; i < kFilterLength; i++) {
      weights[i] = 0.0;
      buffer[i] = 0.0;
    }
    weights[0] = 1.0;
    index = 0;
  }
};

/**
 * @brief Kalman Filter for smooth signal tracking
 */
class KalmanFilter {
private:
  float q; // Process noise covariance
  float r; // Measurement noise covariance
  float x; // Value estimate
  float p; // Estimation error covariance
  float k; // Kalman gain
  
public:
  KalmanFilter(float process_noise = 0.01, float measurement_noise = 0.1) :
    q(process_noise), r(measurement_noise), x(0), p(1) {}
  
  float process(float measurement) {
    // Prediction update
    p = p + q;
    
    // Measurement update
    k = p / (p + r);
    x = x + k * (measurement - x);
    p = (1 - k) * p;
    
    return x;
  }
  
  void reset() {
    x = 0;
    p = 1;
  }
  
  void setEstimate(float estimate) {
    x = estimate;
  }
};

/**
 * @brief Signal Quality Estimator
 */
class SignalQualityEstimator {
private:
  static const int kWindowSize = 50;
  float signal_buffer[kWindowSize];
  int buffer_index;
  bool buffer_filled;
  
  float noise_buffer[kWindowSize];
  int noise_index;
  bool noise_filled;
  
public:
  SignalQualityEstimator() : 
    buffer_index(0), buffer_filled(false),
    noise_index(0), noise_filled(false) {
    
    for(int i = 0; i < kWindowSize; i++) {
      signal_buffer[i] = 0;
      noise_buffer[i] = 0;
    }
  }
  
  void updateSignal(float signal) {
    signal_buffer[buffer_index] = signal;
    buffer_index = (buffer_index + 1) % kWindowSize;
    if(buffer_index == 0) buffer_filled = true;
  }
  
  void updateNoise(float noise) {
    noise_buffer[noise_index] = abs(noise);
    noise_index = (noise_index + 1) % kWindowSize;
    if(noise_index == 0) noise_filled = true;
  }
  
  float getSNR() {
    if(!buffer_filled || !noise_filled) return 0;
    
    // Calculate signal power
    float signal_power = 0;
    for(int i = 0; i < kWindowSize; i++) {
      signal_power += signal_buffer[i] * signal_buffer[i];
    }
    signal_power /= kWindowSize;
    
    // Calculate noise power
    float noise_power = 0;
    for(int i = 0; i < kWindowSize; i++) {
      noise_power += noise_buffer[i] * noise_buffer[i];
    }
    noise_power /= kWindowSize;
    
    if(noise_power == 0) return 100; // Very high SNR
    return 10 * log10(signal_power / noise_power);
  }
  
  float getSignalStability() {
    if(!buffer_filled) return 0;
    
    // Calculate variance
    float mean = 0;
    for(int i = 0; i < kWindowSize; i++) {
      mean += signal_buffer[i];
    }
    mean /= kWindowSize;
    
    float variance = 0;
    for(int i = 0; i < kWindowSize; i++) {
      float diff = signal_buffer[i] - mean;
      variance += diff * diff;
    }
    variance /= kWindowSize;
    
    // Return stability metric (lower variance = higher stability)
    return 1.0 / (1.0 + variance / 1000.0);
  }
  
  void reset() {
    buffer_index = 0;
    buffer_filled = false;
    noise_index = 0;
    noise_filled = false;
  }
};

/**
 * @brief Motion Artifact Detector
 */
class MotionDetector {
private:
  static const int kBufferSize = 20;
  float accel_buffer[kBufferSize];
  float ppg_buffer[kBufferSize];
  int buffer_index;
  bool buffer_filled;
  
  float motion_threshold;
  float correlation_threshold;
  
public:
  MotionDetector(float motion_thresh = 2000.0, float corr_thresh = 0.7) :
    buffer_index(0), buffer_filled(false),
    motion_threshold(motion_thresh), correlation_threshold(corr_thresh) {
    
    for(int i = 0; i < kBufferSize; i++) {
      accel_buffer[i] = 0;
      ppg_buffer[i] = 0;
    }
  }
  
  void updateAcceleration(float accel) {
    accel_buffer[buffer_index] = accel;
  }
  
  void updatePPG(float ppg) {
    ppg_buffer[buffer_index] = ppg;
    buffer_index = (buffer_index + 1) % kBufferSize;
    if(buffer_index == 0) buffer_filled = true;
  }
  
  bool detectMotion() {
    if(!buffer_filled) return false;
    
    // Check acceleration magnitude
    float accel_magnitude = 0;
    for(int i = 0; i < kBufferSize; i++) {
      accel_magnitude += abs(accel_buffer[i]);
    }
    accel_magnitude /= kBufferSize;
    
    if(accel_magnitude > motion_threshold) return true;
    
    // Check correlation between acceleration and PPG
    float correlation = calculateCorrelation();
    return correlation > correlation_threshold;
  }
  
private:
  float calculateCorrelation() {
    // Calculate means
    float accel_mean = 0, ppg_mean = 0;
    for(int i = 0; i < kBufferSize; i++) {
      accel_mean += accel_buffer[i];
      ppg_mean += ppg_buffer[i];
    }
    accel_mean /= kBufferSize;
    ppg_mean /= kBufferSize;
    
    // Calculate correlation coefficient
    float numerator = 0, accel_sq = 0, ppg_sq = 0;
    for(int i = 0; i < kBufferSize; i++) {
      float accel_diff = accel_buffer[i] - accel_mean;
      float ppg_diff = ppg_buffer[i] - ppg_mean;
      
      numerator += accel_diff * ppg_diff;
      accel_sq += accel_diff * accel_diff;
      ppg_sq += ppg_diff * ppg_diff;
    }
    
    float denominator = sqrt(accel_sq * ppg_sq);
    if(denominator == 0) return 0;
    
    return abs(numerator / denominator);
  }
  
public:
  void reset() {
    buffer_index = 0;
    buffer_filled = false;
  }
};

#endif // WRIST_FILTERS_H 