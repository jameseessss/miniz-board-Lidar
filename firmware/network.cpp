#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "network.h"
// network setting
char ssid[] = "TP-LINK_F4D4";
char pass[] = "15291356";
int status = WL_IDLE_STATUS;
WiFiUDP Udp;


////////////

//unsigned int localPort = 125;  // Local port to listen on
IPAddress remoteIp(192, 168, 10, 102);  // Remote IP address (target device IP)                  CHANGE
unsigned int remotePort=622;  // Remote port (where the target device is listening)         CHANGE 


////////////



void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // you're connected now, so print out the data:
  Serial.println("Module connected to the network:");
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println("----- entering loop -----");
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

