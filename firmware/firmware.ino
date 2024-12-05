#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Servo.h>
#define PI 3.14159265

#include "isr_timer.h"
#include "network.h"
#include "packet.h"
#include "pwm.h"
#include "status_light.h"

uint16_t dist1, dist2, dist3, dist4;

class TFMiniS {
private:
    uint8_t addr;
    
    bool sendCommand(uint8_t* cmd, uint8_t len, const char* cmdName, uint16_t delayTime = 10) {
        Wire.beginTransmission(addr);
        Wire.write(cmd, len);
        uint8_t error = Wire.endTransmission();
        
        delay(delayTime);
        return true;
    }

public:
    TFMiniS(uint8_t address) : addr(address) {}

    void begin() {
        Serial.print("\nInitializing TFMini-S at address 0x");
        Serial.println(addr, HEX);
        
        uint8_t resetCmd[] = {0x5A, 0x04, 0x02, 0x60};
        if(!sendCommand(resetCmd, 4, "reset", 100)) {
            Serial.println("Reset failed");
            return;
        }

        uint8_t i2cModeCmd[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
        if(!sendCommand(i2cModeCmd, 5, "set I2C mode", 100)) {
            Serial.println("Set I2C mode failed");
            return;
        }

        uint8_t formatCmd[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
        if(!sendCommand(formatCmd, 5, "set format", 100)) {
            Serial.println("Set format failed");
            return;
        }

        uint8_t saveCmd[] = {0x5A, 0x04, 0x11, 0x6F};
        if(!sendCommand(saveCmd, 4, "save settings", 2000)) {
            Serial.println("Save settings failed");
            return;
        }

        Serial.print("Initialization complete for sensor at 0x");
        Serial.println(addr, HEX);
    }

    bool readDistance(uint16_t &distance) {
        uint8_t getDataCmd[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
        if(!sendCommand(getDataCmd, 5, "get data", 1)) {
            return false;
        }

        if(Wire.requestFrom(addr, (uint8_t)9) != 9) {
            Serial.print("Failed to read from sensor 0x");
            Serial.println(addr, HEX);
            return false;
        }

        uint8_t buffer[9];
        for(int i = 0; i < 9; i++) {
            buffer[i] = Wire.read();
        }

        if(buffer[0] != 0x59 || buffer[1] != 0x59) {
            Serial.print("Invalid frame header from sensor 0x");
            Serial.println(addr, HEX);
            return false;
        }

        distance = buffer[2] | (buffer[3] << 8);
        
        return true;
    }
};

TFMiniS sensor1(0x11);
TFMiniS sensor2(0x12);
TFMiniS sensor3(0x13);
TFMiniS sensor4(0x14);
///////////////////

unsigned int localPort = 2390;
unsigned long last_packet_ts = 0;
int encoder_s_pin = 14;

Servo steerServo;

const int DRIVE_AIN1 = 4;  
const int DRIVE_AIN2 = 5;
const int DRIVE_PWMA = 6;
const int DRIVE_STBY = 7;

volatile float throttle = 0.0;
// left positive, radians
volatile float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle = 110.0;    // 舵机左转最大值
float full_right_angle = 70.0;  // 舵机右转最大值
float center_angle = 90;       // mid point

float steering_deadzone_rad = 1.0 / 180.0 * PI;
volatile unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;
long servo_ts = 0;
bool flag_failsafe = false;
StatusLed led;

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef _SAMD21_ADC_COMPONENT_
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
  while (ADC->STATUS.bit.SYNCBUSY == 1);

#endif
  Serial.begin(115200);

  Serial.print("Connecting to Wi-Fi...");
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);  // Wait for 1 second before retrying
      Serial.print(".");
    }

    // Print network data once connected
    Serial.println("\nConnected to Wi-Fi!");
    printWifiData();
    printCurrentNet();

    // Start UDP communication
    Udp.begin(localPort);  // Begin UDP on the specified local port
    Serial.print("Local port: ");
    Serial.println(localPort);



    Wire.begin();
    Wire.setClock(400000);

    Serial.println("\nTFMini-S I2C Reader");

    sensor1.begin();
    sensor2.begin();
    sensor3.begin();
    sensor4.begin();

  ///////////////////



  // Enable motor driver
  pinMode(DRIVE_AIN1, OUTPUT);
  pinMode(DRIVE_AIN2, OUTPUT);
  pinMode(DRIVE_PWMA, OUTPUT);
  pinMode(DRIVE_STBY, OUTPUT);
  digitalWrite(DRIVE_STBY, HIGH);
  steerServo.attach(10);  

  pinMode(encoder_s_pin, INPUT);

  led.init();
  led.off();
  setupWifi();
  Udp.begin(localPort);

  PWM::setup();
}

void blinkTwice() {
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
}

void setupWifi() {
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true)
      ;
  }

  // while (status != WL_CONNECTED) {
  //   Serial.print("Attempting to connect to WPA SSID: ");
  //   Serial.println(ssid);
  //   status = WiFi.begin(ssid, pass);
  //   delay(10000);
  // }
  led.blink();

  printCurrentNet();
  printWifiData();
}

unsigned long periodic_print_1hz_ts = 0;

void loop() {
  static unsigned long lastRead = 0;
    
    if(millis() - lastRead >= 20) { ///////////////////////////////////////////////////////////////////////////////   send/1s, need to change from 1000 to 10 for 100hz
        
        // Serial.println("\n--- Reading Sensors ---");

        if(sensor1.readDistance(dist1)) {
            // Serial.print("Front Distance=");
            // Serial.print(dist1);
            // Serial.println(" cm");
        } else {
            Serial.println("Failed to read sensor 0x11");
        }

        if(sensor2.readDistance(dist2)) {
            // Serial.print("Left Distance=");
            // Serial.print(dist2);
            // Serial.println(" cm");
        } else {
            Serial.println("Failed to read sensor 0x12");
        }

        if(sensor3.readDistance(dist3)) {
            // Serial.print("Back Distance=");
            // Serial.print(dist3);
            // Serial.println(" cm");
        } else {
            Serial.println("Failed to read sensor 0x13");
        }

        if(sensor4.readDistance(dist4)) {
            // Serial.print("Right Distance=");
            // Serial.print(dist4);
            // Serial.println(" cm");
        } else {
            Serial.println("Failed to read sensor 0x14");
        }

        lastRead = millis();
    }

  led.update();
  //  Serial.println(millis() - loop_time);
  //  loop_time = millis();
  int packet_size = Udp.parsePacket();

  // process incoming packet
  if (packet_size) {
    if (packet_size != PACKET_SIZE) {
      Serial.println("err packet size");
    }

    // Serial.print("From ");
    //IPAddress remoteIp = Udp.remoteIP();
    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE) {
      Serial.print("err reading packet size ");
      Serial.println(len);
    }
    // Serial.println("parsing packet");
    parsePacket();
    last_packet_ts = millis();
    flag_failsafe = false;
    led.on();
    // response is handled by packet parser
  }

  if (millis() - last_packet_ts > 100 && !flag_failsafe) {
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    // Serial.print(millis());
    // Serial.println(" failsafe");
    led.blink();
  }
  PIDControl();
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// using global variable throttle and steering
// set pwm for servo and throttle
void actuateThrottle() {
  // throttle control
  if (abs(throttle) < throttle_deadzone) {
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
  } else {
    
    int pwmValue = abs(throttle) * 255;
    pwmValue = constrain(pwmValue, 0, 255);
    if (throttle > 0) {
      digitalWrite(DRIVE_AIN1, HIGH);
      digitalWrite(DRIVE_AIN2, LOW);
      analogWrite(DRIVE_PWMA, pwmValue);
    } else {
      digitalWrite(DRIVE_AIN1, LOW);
      digitalWrite(DRIVE_AIN2, HIGH);
      analogWrite(DRIVE_PWMA, pwmValue);
    }
  }
}

// for steering rack
void PIDControl() {
  float dt = (float)(micros() - last_pid_ts) / 1e6;
  last_pid_ts = micros();

  if (flag_failsafe) {
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
    steerServo.write(center_angle);

    return;
  }

  actuateThrottle();

  float steering_angle = (steering * 180.0 / PI);

  

  float target_angle = steeringMap(steering);
  target_angle = constrain(target_angle, full_right_angle, full_left_angle);
  
  if (millis()- servo_ts >20){
  steerServo.write(target_angle);
  servo_ts = millis();
  }
  Serial.println(target_angle);
  
}
float steeringMap(float steering) {
  return fmap(steering, -1.0, 1.0, full_right_angle, full_left_angle);
}
