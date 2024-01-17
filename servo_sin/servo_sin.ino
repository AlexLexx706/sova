
#include <ESP32Servo.h>
#include <math.h>

Servo right_hand_servo;
Servo left_hand_servo;
Servo body_servo;
Servo head_servo;
Servo neck_servo;

Servo led_1;
Servo led_2;

int pos;

void setup() {
  Serial.begin(115200);
  right_hand_servo.attach(18);
  left_hand_servo.attach(13);
  body_servo.attach(19);
  head_servo.attach(12);
  neck_servo.attach(15);

  led_1.attach(27);
  led_2.attach(14);


  right_hand_servo.write(90);
  left_hand_servo.write(90);
  body_servo.write(90);
  head_servo.write(90);
  neck_servo.write(90);

  led_1.write(180);
  led_2.write(180);


}

const float period_ms = 3000;
const float max_angle = 45;

const float time_offset_period_ms = 8000;
const float max_time_offset = 10000;


void loop() {
  unsigned long cur_time = millis();

  //float period_offset = sin(cur_time / time_offset_period_ms * PI * 2) * max_time_offset;
  float period_offset = 0;
  float angle = sin((cur_time + period_offset) / period_ms * PI * 2) * max_angle;

  Serial.println(angle);
  float left_hand_angle = 90 + angle;
  float right_hand_angle = 90 - angle;
  

  right_hand_servo.write(right_hand_angle);
  left_hand_servo.write(left_hand_angle);
  body_servo.write(left_hand_angle);
  head_servo.write(left_hand_angle);
  neck_servo.write(left_hand_angle);

  delay(1000/50);
}
