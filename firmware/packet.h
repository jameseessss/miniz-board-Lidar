#ifndef PACKET_H
#define PACKET_H
#include <Arduino.h>
#include "firmware.h"
#include "network.h"

#define PACKET_SIZE 64
extern char in_buffer[PACKET_SIZE];
extern char out_buffer[PACKET_SIZE];
// sensors
// steering_requested differs from steering in that it is synchronized to steering_measured
extern float steering_requested;
extern float steering_measured;

extern uint32_t current_seq_no;
// params
extern bool  param_sensor_update;
extern float param_steering_P;
extern float param_steering_I;
extern float param_steering_D;

extern uint16_t dist1, dist2, dist3, dist4;

typedef struct {
  uint32_t seq_no;
  uint32_t ts;
  uint8_t dest_addr;
  uint8_t src_addr;
  uint8_t type;
  uint8_t sub_type;
  union {
    // general
    char payload[52];
    // type 1.x, command packet
    struct {
      float throttle; // [-1,1]
      float steering; // in rad
    };
    // type 2.0, Sensor Update
    struct {
      uint16_t dist1; // front unit:cm
      uint16_t dist2; // left
      uint16_t dist3; // rear
      uint16_t dist4; // right
    };
    // type 3.0, Parameter Packet
    struct {
      bool sensor_update; // 4 bytes due to alignment
      float steering_P;
      float steering_I;
      float steering_D;
    };
  };
} Packet;


// parse packet in in_buffer
// set appropriate global variables
void parsePacket();
void parsePingPacket();
void parseSensorPacket();
void parseCommandPacket();
void parseParamPacket();
void buildPingResponsePacket();
void buildSensorResponsePacket();
void buildHeader(uint8_t type, uint8_t subtype);
int sendResponsePacket();

#endif
