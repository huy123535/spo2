#include <MAX3010x.h>

/* 
 * Select your sensor by uncommenting
 * the corresponding line
 */

// MAX30100 sensor;
// MAX30101 sensor;
MAX30102 sensor;
// MAX30105 sensor;

void setup() {
  Serial.begin(115200);
  sensor.setLedCurrent(MAX30102::LED_RED, 60);  // 12mA
  sensor.setLedCurrent(MAX30102::LED_IR, 50);   // 10mA
  sensor.setSamplingRate(MAX30102::SAMPLING_RATE_400SPS);
  sensor.setSampleAveraging(MAX30102::SMP_AVE_32);  // Lấy trung bình 32 mẫu

  if(sensor.begin()) { 
    Serial.println("RED");  // Label for Serial Plotter
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }  
}

void loop() {
  auto sample = sensor.readSample(1000);
  Serial.println(sample.red);  // Only send RED values
}
