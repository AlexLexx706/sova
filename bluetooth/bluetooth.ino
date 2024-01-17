//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("SOVA"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    int simbol = SerialBT.read();
    switch (simbol) {
      case '0': {
        Serial.println("Stop All");
        break;
      }
      case '1': {
        Serial.println("next music");
        break;
      }
      case '2': {
        Serial.println("pause");
        break;
      }
      case '3': {
        Serial.println("play");
        break;
      }
      case '4': {
        Serial.println("shake hands");
        break;
      }
      case '5': {
        Serial.println("stop shake hands");
        break;
      }

      case '6': {
        Serial.println("rotate body");
        break;
      }
      case '7': {
        Serial.println("stop rotate body");
        break;
      }
      case '8': {
        Serial.println("rotate head");
        break;
      }
      case '9': {
        Serial.println("stop rotate head");
        break;
      }

    };
    Serial.write(simbol);
  }
  delay(20);
}
