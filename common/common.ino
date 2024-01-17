
#include "BluetoothSerial.h"
#include <DFRobotDFPlayerMini.h>
#include <ESP32Servo.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

static BluetoothSerial SerialBT;

static DFRobotDFPlayerMini myDFPlayer;

static Servo right_hand_servo;
static Servo left_hand_servo;
static Servo body_servo;
static Servo head_servo;

static Servo left_wheel_servo;
static Servo right_wheel_servo;


static TaskHandle_t servo_task;

static volatile bool shake_hands_flag = false;
static volatile bool shake_body_flag = false;
static volatile bool shake_head_flag = false;
static volatile bool shake_right_hand_flag = false;
static volatile bool shake_left_hand_flag = false;
static volatile bool rotate_left_wheel_flag = false;
static volatile bool rotate_right_wheel_flag = false;

#define LED_FREQ 1000
ESP32PWM left_led;
ESP32PWM right_led;

void setup() {
    right_hand_servo.attach(18);
    left_hand_servo.attach(13);
    body_servo.attach(19);
    head_servo.attach(12);

    left_led.attachPin(14, LED_FREQ, 10); // 1KHz 8 bit
    right_led.attachPin(27, LED_FREQ, 10); // 1KHz 8 bit

    left_wheel_servo.attach(22);
    right_wheel_servo.attach(21);

    left_led.writeScaled(0.);
    right_led.writeScaled(0.);

    Serial2.begin(9600);
    Serial.begin(115200);
    if (!myDFPlayer.begin(Serial2)) { // Serial2 to communicate with mp3.
        Serial.println(F("Unable to begin:"));
        Serial.println(F("1.Please recheck the connection!"));
        Serial.println(F("2.Please insert the SD card!"));
        while (true)
            ;
        delay(0); // Code to compatible with ESP8266 watch dog.
    }
    Serial.println(F("DFPlayer Mini online."));

    myDFPlayer.setTimeOut(500); // Set serial communictaion time out 500ms

    SerialBT.begin("SOVA"); // Bluetooth device name
    Serial.println("The device started, now you can pair it with bluetooth!");

    myDFPlayer.volume(2);
    myDFPlayer.stop();
    myDFPlayer.enableLoopAll();
   

    xTaskCreatePinnedToCore(
        servo_loop,   /* Task function. */
        "Task1",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        &servo_task,      /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */ 
}

class MoveHandler {
protected:
  bool active_flag = false;
  unsigned long start_time;
  unsigned long max_time = 1000;
  long max_angle = 50;

  virtual void handle(unsigned long cur_time) {
      float value = (cur_time - start_time) / float(max_time);

      //apply animation 
      if (value < 1.) {
          float angle_offset = sin(value * PI) * max_angle;

          float right_angle = 90 + angle_offset;
          right_hand_servo.write(right_angle);

          float left_angle = 90 - angle_offset;
          left_hand_servo.write(left_angle);
      //stop all
      } else {
          active_flag = false;
          right_hand_servo.release();
          left_hand_servo.release();
      }
  }

public:
    MoveHandler(long _max_angle):max_angle(_max_angle){
    }

    void start() {
        if (!active_flag) {
          start_time = millis();
        }
        active_flag = true;
    }

    void stop() {
        active_flag = false;
    }

    void process(unsigned long cur_time) {
        if (active_flag) {
            handle(cur_time);
        }
    }
};


static MoveHandler lift_hands_up(50);
static MoveHandler lift_hands_down(-90);

void shake_hands(unsigned long cur_time) {
    if (!shake_hands_flag) {
        return;
    }

    const float period_ms = 3000;
    const float max_angle = 45;

    const float time_offset_period_ms = 8000;
    const float max_time_offset = 10000;

    float period_offset = sin(cur_time / time_offset_period_ms * PI * 2) * max_time_offset;
    float angle = sin((cur_time + period_offset) / period_ms * PI * 2) * max_angle;

    float left_hand_angle = 90 + angle;
    float right_hand_angle = 90 - angle;

    right_hand_servo.write(right_hand_angle);
    left_hand_servo.write(left_hand_angle);
}


void shake_body(unsigned long cur_time) {
    if (!shake_body_flag) {
        return;
    }

    const float period_ms = 6000;
    const float max_angle = 45;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 90.;
    body_servo.write(angle);
}

void shake_head(unsigned long cur_time) {
    if (!shake_head_flag) {
        return;
    }

    const float period_ms = 2000;
    const float max_angle = 30;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 90;
    head_servo.write(angle);
}


void shake_right_hand(unsigned long cur_time) {
    if (!shake_right_hand_flag) {
        return;
    }

    const float period_ms = 500;
    const float max_angle = 30;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 110;
    right_hand_servo.write(angle);
}



void shake_left_hand(unsigned long cur_time) {
    if (!shake_left_hand_flag) {
        return;
    }

    const float period_ms = 500;
    const float max_angle = 30;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 70;
    left_hand_servo.write(angle);
}


void rotate_left_wheel(unsigned long cur_time) {
    if (!rotate_left_wheel_flag) {
        return;
    }

    const float period_ms = 2000;
    const float max_angle = 30;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 90;
    left_wheel_servo.write(angle);
}

void rotate_right_wheel(unsigned long cur_time) {
    if (!rotate_right_wheel_flag) {
        return;
    }

    const float period_ms = 2000;
    const float max_angle = 30;
    float angle = sin(cur_time / period_ms * PI * 2) * max_angle + 90;
    right_wheel_servo.write(angle);
}

void process_led(unsigned long cur_time) {
    const float period_ms = 2000;
    float value = (sin(cur_time / period_ms * PI * 2) + 1.) / 2.;

    left_led.writeScaled(value);
    right_led.writeScaled(value);

}
void servo_loop(void * _) {
  for(;;) {
    unsigned long cur_time = millis();
    shake_hands(cur_time);
    shake_body(cur_time);
    shake_head(cur_time);
    shake_right_hand(cur_time);
    shake_left_hand(cur_time);
    lift_hands_up.process(cur_time);
    lift_hands_down.process(cur_time);
    rotate_left_wheel(cur_time);
    rotate_right_wheel(cur_time);
    process_led(cur_time);
    delay(20);
  }
}

void loop() {
    if (SerialBT.available()) {
        int symbol = SerialBT.read();
        switch (symbol) {
            case '0': {
                Serial.println("Stop All");
                right_hand_servo.release();
                left_hand_servo.release();
                body_servo.release();
                head_servo.release();

                left_wheel_servo.release();
                right_wheel_servo.release();

                lift_hands_up.stop();
                lift_hands_down.stop();
                
                shake_hands_flag = false;
                shake_body_flag = false;
                shake_head_flag = false;
                shake_right_hand_flag = false;
                shake_left_hand_flag = false;
                rotate_left_wheel_flag = false;
                rotate_right_wheel_flag = false;

                myDFPlayer.stop();
                break;
            }
            case '1': {
                Serial.println("Start shake hands");
                shake_hands_flag = true;
                break;
            }
            case '2': {
                Serial.println("Stop shake hands");
                shake_hands_flag = false;
                right_hand_servo.release();
                left_hand_servo.release();
                break;
            }
            case '3': {
                Serial.println("Start shake body");
                shake_body_flag = true;
                break;
            }
            case '4': {
                Serial.println("Stop shake body");
                shake_body_flag = false;
                body_servo.release();
                break;
            }
            case '5': {
                Serial.println("Start shake head");
                shake_head_flag = true;
                break;
            }
            case '6': {
                Serial.println("Stop shake head");
                shake_head_flag = false;
                head_servo.release();
                break;
            }
            case '7': {
                myDFPlayer.start();
                break;
            }
            case '8': {
                myDFPlayer.stop();
                break;
            }
            case '9': {
                myDFPlayer.next();
                break;
            }
            case 'a': {
                myDFPlayer.volumeDown();
                break;
            }
            case 'b': {
                myDFPlayer.volumeUp();
                break;
            }
            case 'c': {
                Serial.println("left hand shake");
                shake_hands_flag = false;
                shake_left_hand_flag = true;
                break;
            }
            case 'd': {
                Serial.println("stop left hand shake");
                shake_left_hand_flag = false;
                left_hand_servo.release();
                break;
            }
            case 'e': {
                Serial.println("right hand shake");
                shake_hands_flag = false;
                shake_right_hand_flag = true;
                break;
            }
            case 'f': {
                Serial.println("stop right hand shake");
                shake_right_hand_flag = false;
                right_hand_servo.release();
                break;
            }
            case 'g': {
                Serial.println("lift hands up");
                shake_right_hand_flag = false;
                shake_hands_flag = false;
                shake_right_hand_flag = false;
                right_hand_servo.release();
                lift_hands_up.start();
                break;
            }
            case 'h': {
                Serial.println("lift hands down");
                shake_right_hand_flag = false;
                shake_hands_flag = false;
                shake_right_hand_flag = false;
                right_hand_servo.release();
                lift_hands_down.start();
                break;
            }
            case 'j': {
                Serial.println("left servo on");
                rotate_left_wheel_flag = true;
                break;
            }
            case 'k': {
                Serial.println("left servo off");
                rotate_left_wheel_flag = false;
                left_wheel_servo.release();
                break;
            }
            case 'l': {
                Serial.println("right servo on");
                rotate_right_wheel_flag = true;
                break;
            }
            case 'm': {
                Serial.println("right servo off");
                rotate_right_wheel_flag = false;
                right_wheel_servo.release();
                break;
            }

            default: {
                Serial.write(symbol);
                break;
            }
        };
    }
}
