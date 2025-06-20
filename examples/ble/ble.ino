#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <MAX3010x.h>
#include "filters2.h"

// Constants from SpO2_Measurements.ino
const unsigned long kFingerThreshold = 1000;
const unsigned int kFingerCooldownMs = 250;
const float kEdgeThreshold = -200.0;
const float kLowPassCutoff = 3.0;
const float kHighPassCutoff = 0.3;
const bool kEnableAveraging = false;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// Sensor configuration
MAX30101 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Variables for signal processing
float last_diff = 0;
bool crossed = false;
long crossed_time = 0;

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
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

// Timestamps
long last_heartbeat = 0;
long finger_timestamp = 0;
bool finger_detected = false;

// BLE variables
BLEServer *pServer = NULL;
BLECharacteristic *pHeartRateCharacteristic = NULL;
BLECharacteristic *pSpO2Characteristic = NULL;
BLECharacteristic *pControlPointCharacteristic = NULL;
bool deviceConnected = false;
bool sensorEnabled = false;

// Standard Bluetooth SIG UUIDs
#define HEART_RATE_SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // ESP32C3 BLE service
#define HEART_RATE_MEASUREMENT_UUID    "2A37"  // Standard Heart Rate Measurement Characteristic
#define BODY_SENSOR_LOCATION_UUID      "2A38"  // Standard Body Sensor Location Characteristic
#define HEART_RATE_CONTROL_POINT_UUID  "2A39"  // Standard Heart Rate Control Point Characteristic
#define SPO2_SERVICE_UUID             "1822"  // Standard SpO2 Service UUID
#define SPO2_MEASUREMENT_UUID         "2A5F"  // Standard SpO2 Measurement Characteristic

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      sensorEnabled = true;  // Tự động bật sensor khi kết nối
      Serial.println("Device connected and sensor enabled");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      sensorEnabled = false;  // Tự động tắt sensor khi ngắt kết nối
      Serial.println("Device disconnected and sensor disabled");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("Received Value: ");
        
        switch(rxValue[0]) {
          case '0': // Tắt sensor
            sensorEnabled = false;
            Serial.println("Sensor manually disabled");
            break;
          case '1': // Bật sensor
            sensorEnabled = true;
            Serial.println("Sensor manually enabled");
            break;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(1,0);
  
  // Initialize sensor
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) {
    // Configure Multi-LED mode
    MAX30101::MultiLedConfiguration cfg;
    cfg.slot[0] = MAX30101::SLOT_RED;
    cfg.slot[1] = MAX30101::SLOT_IR;
    cfg.slot[2] = MAX30101::SLOT_GREEN;
    cfg.slot[3] = MAX30101::SLOT_OFF;
    
    sensor.setMultiLedConfiguration(cfg);
    sensor.setLedCurrent(MAX30101::LED_RED, 150);
    sensor.setLedCurrent(MAX30101::LED_IR, 130);
    sensor.setLedCurrent(MAX30101::LED_GREEN_CH1, 150);
    sensor.setMode(MAX30101::MODE_MULTI_LED);
    sensor.setSamplingRate(MAX30101::SAMPLING_RATE_400SPS);
    sensor.setSampleAveraging(MAX30101::SMP_AVE_16);
    
    Serial.println("Sensor setup completed");
  } else {
    Serial.println("Sensor not found");
    while(1);
  }

  // Initialize BLE
  BLEDevice::init("ESP32-C3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create Heart Rate Service
  BLEService *pHeartRateService = pServer->createService(HEART_RATE_SERVICE_UUID);
  
  // Create Heart Rate Measurement Characteristic
  pHeartRateCharacteristic = pHeartRateService->createCharacteristic(
                      HEART_RATE_MEASUREMENT_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pHeartRateCharacteristic->addDescriptor(new BLE2902());

  // Create Heart Rate Control Point Characteristic
  pControlPointCharacteristic = pHeartRateService->createCharacteristic(
                      HEART_RATE_CONTROL_POINT_UUID,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pControlPointCharacteristic->setCallbacks(new MyCallbacks());

  // Create Body Sensor Location Characteristic
  BLECharacteristic *pBodySensorLocation = pHeartRateService->createCharacteristic(
                      BODY_SENSOR_LOCATION_UUID,
                      BLECharacteristic::PROPERTY_READ
                    );
  uint8_t location = 0x02; // 0x02 = Wrist
  pBodySensorLocation->setValue(&location, 1);

  // Create SpO2 Service
  BLEService *pSpO2Service = pServer->createService(SPO2_SERVICE_UUID);
  
  // Create SpO2 Measurement Characteristic
  pSpO2Characteristic = pSpO2Service->createCharacteristic(
                      SPO2_MEASUREMENT_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pSpO2Characteristic->addDescriptor(new BLE2902());

  // Start services
  pHeartRateService->start();
  pSpO2Service->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HEART_RATE_SERVICE_UUID);
  pAdvertising->addServiceUUID(SPO2_SERVICE_UUID);
  
  // Configure advertising parameters
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20);
  pAdvertising->setMaxInterval(0x40);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  
  // Start advertising
  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started. Waiting for a client connection...");
}

void loop() {
  if (deviceConnected && sensorEnabled) {
    auto sample = sensor.readSample(1000);
    float current_value_red = sample.red;
    float current_value_ir = sample.ir;
    float current_value_green = sample.green;
    
    // Finger detection
    if(sample.green > kFingerThreshold) {
      if(millis() - finger_timestamp > kFingerCooldownMs) {
        finger_detected = true;
      }
    } else {
      // Reset values if finger removed
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
      // Process sensor data
      current_value_red = low_pass_filter_red.process(current_value_red);
      current_value_ir = low_pass_filter_ir.process(current_value_ir);
      current_value_green = low_pass_filter_green.process(current_value_green);

      stat_red.process(current_value_red);
      stat_ir.process(current_value_ir);

      float current_value = high_pass_filter.process(current_value_green);
      float current_diff = differentiator.process(current_value);

      if(!isnan(current_diff) && !isnan(last_diff)) {
        // Heartbeat detection
        if(last_diff > 0 && current_diff < 0) {
          crossed = true;
          crossed_time = millis();
        }
        
        if(current_diff > 0) {
          crossed = false;
        }
    
        if(crossed && current_diff < kEdgeThreshold) {
          if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
            int bpm = 60000/(crossed_time - last_heartbeat);
            float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
            float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
            float r = rred/rir;
            float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;
            
            if(bpm > 50 && bpm < 250) {
              // Send Heart Rate data (2 bytes)
              uint8_t hrData[2];
              hrData[0] = 0x06;  // Flags: 16-bit value, no contact detected
              hrData[1] = bpm;   // Heart rate value
              pHeartRateCharacteristic->setValue(hrData, 2);
              pHeartRateCharacteristic->notify();

              // Send SpO2 data (3 bytes)
              uint8_t spo2Data[3];
              spo2Data[0] = 0x03;  // Flags: 16-bit value
              spo2Data[1] = (uint16_t)spo2 & 0xFF;  // SpO2 value (LSB)
              spo2Data[2] = ((uint16_t)spo2 >> 8) & 0xFF;  // SpO2 value (MSB)
              pSpO2Characteristic->setValue(spo2Data, 3);
              pSpO2Characteristic->notify();

              // Debug output
              Serial.print("Heart Rate: ");
              Serial.print(bpm);
              Serial.print(" bpm, SpO2: ");
              Serial.print(spo2);
              Serial.println("%");
            }
          }
          crossed = false;
          last_heartbeat = crossed_time;
        }
      }
      last_diff = current_diff;
    }
  } else {
    if (!deviceConnected) {
      Serial.println("Waiting for BLE connection...");
    }
    if (!sensorEnabled) {
      Serial.println("Sensor is disabled. Send '1' to enable.");
    }
    delay(100); // Add small delay to make output readable
  }
}
