
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
  right_hand_servo.write(90);
  left_hand_servo.write(90);
  body_servo.write(90);
  head_servo.write(90);
}

void loop() {
}
