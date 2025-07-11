#ifndef FILTERS1_H
#define FILTERS1_H

#include <Arduino.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265359
#endif

/**
 * @brief High Pass Filter 
 */
class HighPassFilter {
  const float kX;
  const float kA0;
  const float kA1;
  const float kB1;
  float last_filter_value_;
  float last_raw_value_;
public:
  /**
   * @brief Initialize the High Pass Filter
   * @param samples Number of samples until decay to 36.8 %
   * @remark Sample number is an RC time-constant equivalent
   */
  HighPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0((1+kX)/2),
    kA1(-kA0),
    kB1(kX),
    last_filter_value_(NAN),
    last_raw_value_(NAN){}

  /**
   * @brief Initialize the High Pass Filter
   * @param cutoff Cutoff frequency
   * @pram sampling_frequency Sampling frequency
   */
  HighPassFilter(float cutoff, float sampling_frequency) :
    HighPassFilter(sampling_frequency/(cutoff*2*PI)){}

  /**
   * @brief Applies the high pass filter
   */
  float process(float value) { 
    if(isnan(last_filter_value_) || isnan(last_raw_value_)) {
      last_filter_value_ = 0.0;
    }
    else {
      last_filter_value_ = 
        kA0 * value 
        + kA1 * last_raw_value_ 
        + kB1 * last_filter_value_;
    }
    
    last_raw_value_ = value;
    return last_filter_value_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_raw_value_ = NAN;
    last_filter_value_ = NAN;
  }
};

/**
 * @brief Low Pass Filter 
 */
class LowPassFilter {
  const float kX;
  const float kA0;
  const float kB1;
  float last_value_;
public:
  /**
   * @brief Initialize the Low Pass Filter
   * @param samples Number of samples until decay to 36.8 %
   * @remark Sample number is an RC time-constant equivalent
   */
  LowPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0(1-kX),
    kB1(kX),
    last_value_(NAN){}

  /**
   * @brief Initialize the Low Pass Filter
   * @param cutoff Cutoff frequency
   * @pram sampling_frequency Sampling frequency
   */
  LowPassFilter(float cutoff, float sampling_frequency) :
    LowPassFilter(sampling_frequency/(cutoff*2*PI)){}

  /**
   * @brief Applies the low pass filter
   */
  float process(float value) {  
    if(isnan(last_value_)) {
      last_value_ = value;
    }
    else {  
      last_value_ = kA0 * value + kB1 * last_value_;
    }
    return last_value_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_value_ = NAN;
  }
};

/**
 * @brief Differentiator
 */
class Differentiator {
  const float kSamplingFrequency;
  float last_value_;
public:
  /**
   * @brief Initializes the differentiator
   */
  Differentiator(float sampling_frequency) :
    kSamplingFrequency(sampling_frequency),
    last_value_(NAN){}

  /**
   * @brief Applies the differentiator
   */
  float process(float value) {  
      float diff = (value-last_value_)*kSamplingFrequency;
      last_value_ = value;
      return diff;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    last_value_ = NAN;
  }
};

/**
 * @brief MovingAverageFilter
 * @tparam buffer_size Number of samples to average over
 */
template<int kBufferSize> class MovingAverageFilter {
  int index_;
  int count_;
  float values_[kBufferSize];
public:
  /**
   * @brief Initalize moving average filter
   */
  MovingAverageFilter() :
    index_(0),
    count_(0){}

  /**
   * @brief Applies the moving average filter
   */
  float process(float value) {  
      // Add value
      values_[index_] = value;

      // Increase index and count
      index_ = (index_ + 1) % kBufferSize;
      if(count_ < kBufferSize) {
        count_++;  
      }

      // Calculate sum
      float sum = 0.0;
      for(int i = 0; i < count_; i++) {
          sum += values_[i];
      }

      // Calculate average
      return sum/count_;
  }

  /**
   * @brief Resets the stored values
   */
  void reset() {
    index_ = 0;
    count_ = 0;
  }

  /**
   * @brief Get number of samples
   * @return Number of stored samples
   */
  int count() const {
    return count_;
  }
};

#endif // FILTERS1_H