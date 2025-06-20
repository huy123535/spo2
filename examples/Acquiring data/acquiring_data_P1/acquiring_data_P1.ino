#include <MAX3010x.h>

/* 
 * Select your sensor by uncommenting
 * the corresponding line
 */

// MAX30100 sensor;
MAX30101 sensor;
// MAX30102 sensor;
// MAX30105 sensor;

uint32_t x = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);

  if(sensor.begin()) { 
    // Configure Multi-LED mode
    MAX30101::MultiLedConfiguration cfg;
    cfg.slot[0] = MAX30101::SLOT_RED;      // Slot 0: Red LED
    cfg.slot[1] = MAX30101::SLOT_IR;       // Slot 1: IR LED
    cfg.slot[2] = MAX30101::SLOT_GREEN;    // Slot 2: Green LED
    cfg.slot[3] = MAX30101::SLOT_OFF;      // Slot 3: Off
    
    sensor.setMultiLedConfiguration(cfg);
    
    // Set LED currents
    sensor.setLedCurrent(MAX30101::LED_RED, 60);
    sensor.setLedCurrent(MAX30101::LED_IR, 50);
    sensor.setLedCurrent(MAX30101::LED_GREEN_CH1, 70);
    
    // Set to MULTI_LED mode
    sensor.setMode(MAX30101::MODE_HR_ONLY);
    
    // Configure other settings
    sensor.setSamplingRate(MAX30101::SAMPLING_RATE_400SPS);
    sensor.setSampleAveraging(MAX30101::SMP_AVE_16);
    
    Serial.println("Red");  // Label for Serial Plotter
  }
  else {
    Serial.println("Sensor not found");  
    while(1);
  }  
}

void loop() {
  auto sample = sensor.readSample(1000);
  Serial.println(sample.red);
}
