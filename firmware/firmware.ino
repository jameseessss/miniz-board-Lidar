#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define PI 3.14159265

#include "isr_timer.h"
#include "network.h"
#include "packet.h"
#include "pwm.h"
#include "status_light.h"

unsigned int localPort = 2390;
unsigned long last_packet_ts = 0;
int encoder_s_pin = 14;

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 9; // originally 2 -> changed to 9
// right
int steer_fwd_pin = 10; // originally 3 -> changed to 10

int old_steer_rev_pin = 2;
int old_steer_fwd_pin = 3;


const int DRIVE_AIN1 = 4;  
const int DRIVE_AIN2 = 5;
const int DRIVE_PWMA = 6;
const int DRIVE_STBY = 7;
//int drive_rev_pin = 5;
//int drive_fwd_pin = 6;
// electronics calibration
// [-1,1]
volatile float throttle = 0.0;
// left positive, radians
volatile float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle_rad = 26.1 / 180.0 * PI;
float full_right_angle_rad = -26.1 / 180.0 * PI;
float full_left_encoder_value = 630.0;
float full_right_encoder_value = 410.0;
float steering_deadzone_rad = 1.0 / 180.0 * PI;
volatile unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;

bool flag_failsafe = false;
StatusLed led;

// the setup function runs once when you press reset or power the board
void setup() {
#ifdef _SAMD21_ADC_COMPONENT_
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  // Averaging (see datasheet table in AVGCTRL register description)
  /*
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_8 |    	// # of samples to accumulate. (1/2/4/8.../1024)
  ADC_AVGCTRL_ADJRES(3);   				// lookup datasheet table 33-3
  while (ADC->STATUS.bit.SYNCBUSY == 1);
  */

#endif
  Serial.begin(115200);

  pinMode(DRIVE_AIN1, OUTPUT);
  pinMode(DRIVE_AIN2, OUTPUT);
  pinMode(DRIVE_PWMA, OUTPUT);
  pinMode(DRIVE_STBY, OUTPUT);
  digitalWrite(DRIVE_STBY, HIGH);

  pinMode(encoder_s_pin, INPUT);
  // to be compatible with old board where pin 2,3 are connected to output
  pinMode(old_steer_rev_pin, INPUT);
  pinMode(old_steer_fwd_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);
  // pinMode(drive_rev_pin, OUTPUT);
  // pinMode(drive_fwd_pin, OUTPUT);
  led.init();
  led.off();
  setupWifi();
  Udp.begin(localPort);
  //timerSetup();
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

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  led.blink();

  printCurrentNet();
  printWifiData();
}

unsigned long periodic_print_1hz_ts = 0;

void loop() {
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
    // analogWrite(drive_fwd_pin, 0);
    // analogWrite(drive_rev_pin, 0);
    // PWM::set(drive_fwd_pin, 0.0);
    // PWM::set(drive_rev_pin, 0.0);
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
  } else {
    /*
    int throttle_value = (int)fmap(throttle, -1.0, 1.0, -255.0, 255.0);
    throttle_value = constrain(throttle_value, -255, 255);
    if (throttle_value > 0) {
      analogWrite(drive_fwd_pin, throttle_value);
      analogWrite(drive_rev_pin, 0);
    } else {
      analogWrite(drive_rev_pin, -throttle_value);
      analogWrite(drive_fwd_pin, 0);
    }
    */
    int pwmValue = abs(throttle) * 255;
    pwmValue = constrain(pwmValue, 0, 255);
    if (throttle > 0) {
      // PWM::set(drive_fwd_pin, throttle);
      // PWM::set(drive_rev_pin, 0);
      digitalWrite(DRIVE_AIN1, HIGH);
      digitalWrite(DRIVE_AIN2, LOW);
      analogWrite(DRIVE_PWMA, pwmValue);
    } else {
      // PWM::set(drive_rev_pin, -throttle);
      // PWM::set(drive_fwd_pin, 0);
      digitalWrite(DRIVE_AIN1, LOW);
      digitalWrite(DRIVE_AIN2, HIGH);
      analogWrite(DRIVE_PWMA, pwmValue);
    }
  }
}

// for steering rack
void PIDControl() {
  // float dt = (float)(millis() - last_packet_ts) / 1000.0;
  float dt = (float)(micros() - last_pid_ts) / 1e6;
  last_pid_ts = micros();

  if (flag_failsafe) {
    // brake mode for all
    // NOTE digitalWrite no longer works
    // analogWrite(drive_fwd_pin, 0);
    // analogWrite(drive_rev_pin, 0);
    // analogWrite(steer_fwd_pin, 255);
    // analogWrite(steer_rev_pin, 255);
    // PWM::set(drive_fwd_pin, 0);
    // PWM::set(drive_rev_pin, 0);
    analogWrite(DRIVE_PWMA, 0);
    digitalWrite(DRIVE_AIN1, LOW);
    digitalWrite(DRIVE_AIN2, LOW);
    PWM::set(steer_fwd_pin, 0);
    PWM::set(steer_rev_pin, 0);
    return;
  }

  actuateThrottle();
  // Steering PID control
  float raw_encoder = analogRead(encoder_s_pin);
  steering_measured =
      fmap(raw_encoder, full_left_encoder_value, full_right_encoder_value,
           full_left_angle_rad, full_right_angle_rad);
  steering_measured =
      constrain(steering_measured, full_right_angle_rad, full_left_angle_rad);

  // error in radians
  float err = steering - steering_measured;
  steering_requested = steering;

  // filter

  float freq = 10.0;
  float alfa = (2 * PI * dt * freq) / (2 * PI * dt * freq + 1.0);
  err = (1.0 - alfa) * last_err + alfa * err;
  steering_integral += err * dt;
  steering_integral = constrain(steering_integral, -steering_integral_limit,
                                steering_integral_limit);
  // PID
  float output = err * param_steering_P + steering_integral * param_steering_I +
                 (err - last_err) / dt * param_steering_D;

  /*
  Serial.print("err: ");
  Serial.print(err);
  Serial.print(" goal: ");
  Serial.print(steering / PI * 180.0, 5);
  Serial.print(" actual: ");
  Serial.print(steering_measured / PI * 180.0, 5);
  Serial.print("output: ");
  Serial.print(output);
  Serial.println();
  Serial.print("PID");
  Serial.println(micros()-last_pid_ts);
  */

  if (abs(err) < steering_deadzone_rad) {
    // analogWrite(steer_fwd_pin, 0);
    // analogWrite(steer_rev_pin, 0);
    PWM::set(steer_fwd_pin, 0);
    PWM::set(steer_rev_pin, 0);
  } else if (output > 0) {
    // analogWrite(steer_rev_pin, constrain(err * param_steering_P, 0, 255));
    // analogWrite(steer_fwd_pin, 0);
    PWM::set(steer_rev_pin, output);
    PWM::set(steer_fwd_pin, 0);
  } else if (output < 0) {
    // analogWrite(steer_fwd_pin, constrain(-err * param_steering_P, 0, 255));
    // analogWrite(steer_rev_pin, 0);
    PWM::set(steer_fwd_pin, -output);
    PWM::set(steer_rev_pin, 0);
  }

  last_err = err;
  //Serial.println(micros()-last_pid_ts);
}

