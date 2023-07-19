#include <Arduino.h>
#include <Capsule.h>  
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <../ERT_RF_Protocol_Interface/MacAdresses.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"

#include "config.h"
#include "sensor.h"

uint8_t* broadcastAddress = commandInputMac;
esp_now_peer_info_t peerInfo;

static bool sensorIsCalibrated = false;
static bool sensorIsInView = false;
static senClass sen;

void sendBinocGlobalStatus();

uint32_t colors[] = {
    0x32A8A0, // Cyan
    0x0000FF, // Blue
    0xFFEA00, // Yellow
    0x00FF00, // Green
    0xFF0000, // Red
    0xCF067C, // Purple
    0xFF0800  // Orange
}; 


void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
CapsuleStatic UartCapsule(handleUartCapsule);

void setup() {
  SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);
  SERIAL_TO_PC.setTxTimeoutMs(0);

  // ------------- PORT --------- DeviceRX, DeviceTX // 
  UART_PORT.begin(UART_BAUD, 134217756U, 9, 46); // This for cmdIn

  led.begin();
  uint32_t ledColor = colors[random(0,7)];
  led.fill(ledColor);
  led.setBrightness(20);
  led.show();
  sen.begin();

  pinMode(BUTTON_CALIBRATE_PIN, INPUT);
  pinMode(BUTTON_IS_IN_VIEW_PIN, INPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_now_init();

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {

  while (UART_PORT.available()) {
    UartCapsule.decode(UART_PORT.read());
  }
  if (sen.update()) {
    uint32_t ledColor = colors[random(0,7)];
    led.fill(ledColor);
    led.show();
    sendBinocGlobalStatus();
  }
  static long lastCalibrationTime = 0;
  if (millis()-lastCalibrationTime > 1000 and (!sensorIsInView or !sensorIsCalibrated)) {
    if (digitalRead(BUTTON_CALIBRATE_PIN) == BUTTON_CALIBRATE_PRESSED) {
      SERIAL_TO_PC.println("Calibrating");
      lastCalibrationTime = millis();
      sensorIsCalibrated = true;
      sen.calibrate();
    }
  }

  // if (digitalRead(BUTTON_IS_IN_VIEW_PIN) == LOW) {
  //   sensorIsInView = true;
  // }
  // else {
  //   sensorIsInView = false;
  // } 
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
}

void sendBinocGlobalStatus() {
  PacketBinocGlobalStatus packet;
  packet.attitude.azm = sen.get().attitude.yaw; 
  packet.attitude.elv = sen.get().attitude.pitch;
  packet.position.lat = sen.get().position.lat;
  packet.position.lon = sen.get().position.lng;
  packet.position.alt = sen.get().position.alt;
  packet.status.isCalibrated = sensorIsCalibrated;
  packet.status.isInView = sensorIsInView;

  // SERIAL_TO_PC.print("Azimuth: ");
  // SERIAL_TO_PC.print(packet.attitude.azm);
  // SERIAL_TO_PC.print(" Elevation: ");
  // SERIAL_TO_PC.println(packet.attitude.elv);

  uint8_t* buffer = new uint8_t[packetBinocGlobalStatusSize];
  memcpy(buffer, &packet, packetBinocGlobalStatusSize);
  uint8_t* packetToSend = UartCapsule.encode(CAPSULE_ID::BINOC_GLOBAL_STATUS,buffer,packetBinocGlobalStatusSize);
  UART_PORT.write(packetToSend,UartCapsule.getCodedLen(packetBinocGlobalStatusSize));
  esp_now_send(broadcastAddress, packetToSend, UartCapsule.getCodedLen(packetBinocGlobalStatusSize));
  delete[] packetToSend;
  delete[] buffer;
}