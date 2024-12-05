#ifndef NETWORK_H
#define NETWORK_H
#include <WiFiNINA.h>
#include <WiFiUdp.h>

extern char ssid[];
extern char pass[];
extern int status;
extern WiFiUDP Udp;

// extern unsigned int localPort = 125;  // Local port to listen on
extern IPAddress remoteIp;  // Remote IP address (target device IP)                  CHANGE
extern unsigned int remotePort;  // Remote port (where the target device is listening)         CHANGE 

void printWifiData();
void printCurrentNet();
void printMacAddress(byte mac[]);

#endif NETWORK_H
