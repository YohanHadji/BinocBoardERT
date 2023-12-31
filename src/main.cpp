#include <Arduino.h>
#include <Capsule.h>  
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <../ERT_RF_Protocol_Interface/MacAdresses.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"
#include <movingAvg.h>

#include "config.h"
#include "sensor.h"

#include <Bounce2.h>

Bounce inViewButton = Bounce();
// movingAvg yawAvg(1);
// movingAvg pitchAvg(1);

uint8_t* broadcastAddress = commandInputMac;
esp_now_peer_info_t peerInfo;

static bool sensorIsCalibrated = false;
static bool sensorIsInView = false;
static senClass sen;

void sendBinocGlobalStatus();

double yawOffset = 0;
double pitchOffset = 0;

//double adjustedYaw = 0;
//double adjustedPitch = 0;

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
  led.fill(0xFF0000);
  led.setBrightness(20);
  led.show();

  pinMode(BUTTON_CALIBRATE_PIN, INPUT);
  inViewButton.attach(BUTTON_IS_IN_VIEW_PIN, INPUT);
  inViewButton.interval(10); // interval in ms

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  WiFi.setSleep(false);

  esp_now_init();
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  senSettings basicSettings;
  basicSettings.heatingTime = 10;
  basicSettings.noRotationTime = 20;
  basicSettings.fusionFilter = 11;
  basicSettings.ahs = true;
  basicSettings.inRunCompassCalibration = true;

  basicSettings = {15,30,	13,	1, 1}; 

  sen.begin(basicSettings);

  // yawAvg.begin();
  // pitchAvg.begin();
}

void loop() {
  while (UART_PORT.available()) {
    UartCapsule.decode(UART_PORT.read());
  }
  if (sen.update()) {
    if (sen.get().runningNoRotation) {
      led.fill(0x0000FF);
      led.show();
    }
    else {
      static bool ledOn = false;
      if (ledOn) {
        led.fill(0x000000);
        led.show();
        ledOn = false;
      }
      else {
        led.fill(0x00FF00);
        ledOn = true;
        led.show();
      }
    }
    sendBinocGlobalStatus();
  }
  static long lastCalibrationTime = 0;
  if (millis()-lastCalibrationTime > 1000 and (!sensorIsInView or !sensorIsCalibrated)) {
    if (digitalRead(BUTTON_CALIBRATE_PIN) == BUTTON_CALIBRATE_PRESSED) {
      SERIAL_TO_PC.println("Calibrating");
      lastCalibrationTime = millis();
      sensorIsCalibrated = true;
      // yawAvg.reset();
      // pitchAvg.reset();
      //sen.calibrate();
      // yawOffset = sen.get().attitude.yaw;
      // pitchOffset = sen.get().attitude.pitch;

      // Sending a calibrate telemetry packet
      uint8_t* buffer = new uint8_t[1];
      uint8_t* packetToSend = UartCapsule.encode(CAPSULE_ID::CALIBRATE_TELEMETRY,buffer,1);
      UART_PORT.write(packetToSend,UartCapsule.getCodedLen(1));
      delete[] packetToSend;
    }
  }
  
  inViewButton.update();

  if (inViewButton.changed()) {
    if (inViewButton.read() == BUTTON_IS_IN_VIEW_PRESSED) {
      sensorIsInView = true;
      //SERIAL_TO_PC.println("In view");
    }
    else {
      sensorIsInView = false;
      //SERIAL_TO_PC.println("Not in view");
    } 
  }
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
}

void sendBinocGlobalStatus() {
  PacketBinocGlobalStatus packet;

  packet.attitude.azm = sen.get().attitude.yaw;//yawAvg.reading(int(sen.get().attitude.yaw*10000.0))/10000.0;
  packet.attitude.elv = sen.get().attitude.pitch;//pitchAvg.reading(int(sen.get().attitude.pitch*10000.0))/10000.0;

  // SERIAL_TO_PC.print(packet.attitude.azm);
  // SERIAL_TO_PC.print(" ");
  // SERIAL_TO_PC.println(sen.get().attitude.yaw);

  packet.attitude.azm = packet.attitude.azm-yawOffset; 
  packet.attitude.elv = packet.attitude.elv-pitchOffset;

  if (packet.attitude.azm>180) {
    packet.attitude.azm -= 360;
  }
  else if (packet.attitude.azm<-180) {
    packet.attitude.azm += 360;
  }

  // if (packet.attitude.elv>180) {
  //   packet.attitude.elv -= 360;
  // }
  // else if (packet.attitude.elv<-180) {
  //   packet.attitude.elv += 360;
  // }

  //packet.attitude.elv = sen.get().attitude.pitch;
  packet.position.lat = sen.get().position.lat;
  packet.position.lon = sen.get().position.lng;
  packet.position.alt = sen.get().position.alt;
  packet.status.isCalibrated = sensorIsCalibrated;
  packet.status.isInView = sensorIsInView;

  SERIAL_TO_PC.print("Azimuth: ");
  SERIAL_TO_PC.print(packet.attitude.azm);
  SERIAL_TO_PC.print(" Elevation: ");
  SERIAL_TO_PC.print(packet.attitude.elv);
  SERIAL_TO_PC.print(" sensorIsCalibrated: ");
  SERIAL_TO_PC.println(packet.status.isCalibrated);

  uint8_t* buffer = new uint8_t[packetBinocGlobalStatusSize];
  memcpy(buffer, &packet, packetBinocGlobalStatusSize);
  uint8_t* packetToSend = UartCapsule.encode(CAPSULE_ID::BINOC_GLOBAL_STATUS,buffer,packetBinocGlobalStatusSize);
  UART_PORT.write(packetToSend,UartCapsule.getCodedLen(packetBinocGlobalStatusSize));

  esp_now_send(broadcastAddress, packetToSend, UartCapsule.getCodedLen(packetBinocGlobalStatusSize));
  delete[] packetToSend;
  delete[] buffer;
}