#include <Arduino.h>
#include <Capsule.h>  
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"
#include "config.h"
#include "sensor.h"

static bool sensorIsCalibrated = false;
static bool sensorIsInView = false;
static senClass sen;

uint32_t colors[] = {
    0x000000,
    0x32A8A0,
    0x0000FF,
    0xFFEA00,
    0x00FF00,
    0xFF0000,
    0xCF067C,
    0xFF0800
}; 

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len);

Adafruit_NeoPixel led(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800); // 1 led
CapsuleStatic UartCapsule(handleUartCapsule);

void setup() {
  SERIAL_TO_PC.begin(SERIAL_TO_PC_BAUD);
  SERIAL_TO_PC.setTxTimeoutMs(0);

  // put your setup code here, to run once:
  UART_PORT.begin(UART_BAUD, 134217756U, 6, 5); // This for radioboard
  //UART_PORT.begin(UART_BAUD, 134217756U, 9, 46); // This for cmdIn

  led.begin();
  uint32_t ledColor = colors[random(0,8)];
  led.fill(ledColor);
  led.show();
  sen.begin();

  pinMode(BUTTON_CALIBRATE_PIN, INPUT);
  pinMode(BUTTON_IS_IN_VIEW_PIN, INPUT);
}

void loop() {
  while (UART_PORT.available()) {
    UartCapsule.decode(UART_PORT.read());
  }
  if (sen.update()) {
    PacketBinocGlobalStatus packet;
    packet.attitude.azm = sen.get().attitude.yaw; 
    packet.attitude.elv = sen.get().attitude.pitch;
    packet.position.lat = sen.get().position.lat;
    packet.position.lon = sen.get().position.lng;
    packet.position.alt = sen.get().position.alt;
    packet.status.isCalibrated = sensorIsCalibrated;
    packet.status.isInView = sensorIsInView;

    uint8_t* buffer = new uint8_t[packetBinocGlobalStatusSize];
    memcpy(buffer, &packet, packetBinocGlobalStatusSize);
    uint8_t* packetToSend = UartCapsule.encode(CAPSULE_ID::BINOC_GLOBAL_STATUS,buffer,packetBinocGlobalStatusSize);
    UART_PORT.write(packetToSend,UartCapsule.getCodedLen(packetBinocGlobalStatusSize));
    delete[] packetToSend;
    delete[] buffer;
  }
  if (digitalRead(BUTTON_CALIBRATE_PIN) == LOW) {
    sensorIsCalibrated = true;
    sen.calibrate();
  }
  if (digitalRead(BUTTON_IS_IN_VIEW_PIN) == LOW) {
    sensorIsInView = true;
  }
  else {
    sensorIsInView = false;
  } 
}

void handleUartCapsule(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
}