#include "packet.h"

char in_buffer[PACKET_SIZE];
char out_buffer[PACKET_SIZE];
float steering_requested;
float steering_measured;

uint32_t current_seq_no = 0;
// params
bool param_sensor_update = true;
float param_steering_P = 1.5;
float param_steering_I = 0.0;
float param_steering_D = 0.05;

// parse packet in in_buffer
// set appropriate global variables
void parsePacket() {
  Packet *p = (Packet *)in_buffer;

  /*
  Serial.print("seq_no: ");
  Serial.println(p->seq_no);
  Serial.print("ts: ");
  Serial.println(p->ts);
  Serial.print("dest_addr: ");
  Serial.println(p->dest_addr);
  Serial.print("src_addr: ");
  Serial.println(p->src_addr);
  Serial.print("type: ");
  Serial.println(p->type);
  Serial.print("subtype: ");
  Serial.println(p->sub_type);
  */
  switch (p->type) {
  case 0:
    parsePingPacket();
    break;
  case 1:
    parseCommandPacket();
    break;
  case 2:
    parseSensorPacket();
    break;
  case 3:
    parseParamPacket();
    break;
  default:
    // should never reach here
    break;
  }
}

void parsePingPacket() {
  // Packet *p = (Packet *) in_buffer;
  buildPingResponsePacket();
  sendResponsePacket();
}
// send sensor response
void parseSensorPacket() {
  buildSensorResponsePacket();
  sendResponsePacket();
}
void parseCommandPacket() {
  /*
  Serial.print("throttle: ");
  Serial.println(p->throttle,5);
  Serial.print("steering: ");
  Serial.println(p->steering,5);
  */
  Packet *p = (Packet *)in_buffer;
  steering = p->steering;
  throttle = p->throttle;

  buildSensorResponsePacket();
  int sent_size = sendResponsePacket();
}

void parseParamPacket() {
  Packet *p = (Packet *)in_buffer;
  if (p->sub_type == 3) {
    param_sensor_update = p->sensor_update;
    param_steering_P = p->steering_P;
    param_steering_I = p->steering_I;
    param_steering_D = p->steering_D;
  }
  // all subtype get parameter printout 3.0
  p = (Packet *)out_buffer;
  p->sensor_update = param_sensor_update;
  p->steering_P = param_steering_P;
  p->steering_I = param_steering_I;
  p->steering_D = param_steering_D;
  buildHeader(3, 0);
  sendResponsePacket();
}

void buildPingResponsePacket() {
  buildHeader(0, 1);
  Packet *p = (Packet *)out_buffer;
}

void buildSensorResponsePacket() {
  buildHeader(2, 0);
  Packet *p = (Packet *)out_buffer;
  p->dist1 = dist1;
  p->dist2 = dist2;
  p->dist3 = dist3;
  p->dist4 = dist4;
}

void buildHeader(uint8_t type, uint8_t subtype) {
  Packet *p = (Packet *)out_buffer;
  p->seq_no = current_seq_no++;
  p->ts = micros();
  p->dest_addr = 0;
  p->src_addr = 1;
  p->type = type;
  p->sub_type = subtype;
}

int sendResponsePacket() {
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  int size = Udp.write(out_buffer, PACKET_SIZE);
  /*
  if (retval){
    Serial.println("sent success");
  } else {
    Serial.println("sent FAIL");
  }
  */
  Udp.endPacket();
  return size;
}
