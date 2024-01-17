
#include <ESP32Servo.h>

Servo right_hand_servo;
Servo left_hand_servo;
Servo body_servo;
Servo head_servo;

int pos;

void setup() {
  right_hand_servo.attach(18);
  left_hand_servo.attach(13);
  body_servo.attach(19);
  head_servo.attach(12);
}

void loop() {
  for (pos = 60; pos <= 120; pos += 1) {
    right_hand_servo.write(pos);
    left_hand_servo.write(pos);
    body_servo.write(pos);
    head_servo.write(pos);
    delay(100);
  }

  for (pos = 120; pos >= 60; pos -= 1) {
    right_hand_servo.write(pos);
    left_hand_servo.write(pos);
    body_servo.write(pos);
    head_servo.write(pos);
    delay(100);
  }
}
