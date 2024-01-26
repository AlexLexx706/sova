#include <DFRobotDFPlayerMini.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include "animation.h"

// Joystick MAC Address

#define LED_FREQ 1000
//#define DEBUG_VOODOO_JOYSTICK
#define DEBUG_VOODOO_JOYSTICK_ANGLE

struct JoystickState {
    char protocol;
    char row[4];
    float j_left_right;
    float j_front_back;
    char j_b;
} joystick_state;

struct VoodooJoystickState {
    char protocol;
    float values[5];
} voodoo_joystick_state;


static DFRobotDFPlayerMini myDFPlayer;

Servo right_hand_servo;
Servo left_hand_servo;
Servo body_servo;
Servo head_servo;
Servo neck_servo;
Servo left_wheel_servo;
Servo right_wheel_servo;
ESP32PWM left_led;
ESP32PWM right_led;

static unsigned long last_msg_time = 0;

static LiftHandsAnimation lift_hands_up(50);
static LiftHandsAnimation lift_hands_down(-90);
static ShakeHandsAnimation shake_hands_animation;
static ShakeBodyAnimation shake_body_animation;
static ShakeHeadAnimation shake_head_animation;
static LedAnimation led_animation;
static MoveAnimation move_left_hand_animation(MoveAnimation::left_hand);
static MoveAnimation move_right_hand_animation(MoveAnimation::right_hand);
static MoveAnimation move_head_animation(MoveAnimation::head);
static MoveAnimation move_neck_animation(MoveAnimation::neck);
static MoveAnimation move_body_animation(MoveAnimation::body);

void setup() {
    last_msg_time = millis();
    right_hand_servo.attach(18);
    left_hand_servo.attach(13);

    body_servo.attach(19);
    head_servo.attach(12);
    neck_servo.attach(15);

    left_wheel_servo.attach(22);
    right_wheel_servo.attach(21);

    left_led.attachPin(14, LED_FREQ, 10);  // 1KHz 8 bit
    right_led.attachPin(27, LED_FREQ, 10); // 1KHz 8 bit

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

    myDFPlayer.volume(2);
    myDFPlayer.stop();
    myDFPlayer.enableLoopAll();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register for a callback function that will be called when data is
    // received
    esp_now_register_recv_cb(on_data_recv);
}

class ServoInfo {
    Servo & servo;
    int j_index;
    const char * name;
    float min;
    float max;
    float window = 0.5;
    float cur_angle = 0.;
    float smooth_angle = 0.;
    float smooth_factor = 0.2;
    float len;

public:
    ServoInfo(Servo & _servo, int _j_index, const char * _name, float _min, float _max):
        servo(_servo),
        j_index(_j_index),
        name(_name),
        min(_min),
        max(_max) {
            len = max - min;
    }

    void set_angle(const VoodooJoystickState & j_state) {
        float angle = j_state.values[j_index] * len + min;
        if (angle > (cur_angle + window) || angle < (cur_angle - window)) {
            cur_angle = angle;
        }
        smooth_angle = cur_angle * smooth_factor + smooth_angle * (1. - smooth_factor);

        servo.write(smooth_angle);
        #ifdef DEBUG_VOODOO_JOYSTICK_ANGLE
            Serial.print(name);
            Serial.print(" c_a: ");
            Serial.print(cur_angle);
            Serial.print(" s_a: ");
            Serial.print(smooth_angle);
            Serial.print(" ");
        #endif
    }
};

ServoInfo servos_limits[] = {
    {head_servo, 0, "head", -40 + 90, 70 + 90},
    {neck_servo, 1, "neck", -90 + 90, 90 + 90},
    {left_hand_servo, 2, "left_wing", -50 + 90, 90 + 90},
    {right_hand_servo, 3, "right_wing", -85 + 90, 45 + 90},
    {body_servo, 4, "body", -50 + 90, 70 + 90}
};

void process_voodoo_joystick(const uint8_t *incoming_data, int len) {
    if (len == sizeof(voodoo_joystick_state)) {
        voodoo_joystick_state = *reinterpret_cast<const VoodooJoystickState *>(incoming_data);

        #ifdef DEBUG_VOODOO_JOYSTICK
            for (int i = 0;
                i < sizeof(voodoo_joystick_state.values) /
                    sizeof(voodoo_joystick_state.values[1]);
                i++) {
                Serial.print(voodoo_joystick_state.values[i]);
                Serial.print(" ");
            }
            Serial.println();
        #endif

        for (auto & s_info: servos_limits) {
            s_info.set_angle(voodoo_joystick_state);
        }
        #ifdef DEBUG_VOODOO_JOYSTICK_ANGLE
            Serial.println();
        #endif
    }
}


// Callback when data is received
void on_data_recv(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    if (len == 0) {
        return;
    }
    //processing of voodoo joystick
    if (incoming_data[0] == 1) {
        process_voodoo_joystick(incoming_data, len);
    //processing of normal joystick
    } else if (incoming_data[0] == 0) {
        if (len == sizeof(JoystickState)) {
            joystick_state = *(JoystickState *)(incoming_data);
            last_msg_time = millis();

            // button 1 pressed
            if (joystick_state.row[0] & 0x1) {
                shake_hands_animation.start();
            }

            // button 2 pressed
            if (joystick_state.row[0] & 0x2) {
                shake_body_animation.start();
            }

            // button 3 pressed
            if (joystick_state.row[0] & 0x4) {
                shake_head_animation.start();
            }

            // button A pressed
            if (joystick_state.row[0] & 0x8) {
                led_animation.start();
            }

            static char rows[4] = {0, 0, 0, 0};
            char row_1_diff = rows[1] ^ joystick_state.row[1];

            // button 4 - start paly
            if (row_1_diff & 0x1) {
                myDFPlayer.start();
            }

            // button 5
            if (row_1_diff & 0x2) {
                myDFPlayer.next();
            }

            // button 6
            if (row_1_diff & 0x4) {
                myDFPlayer.stop();
            }

            // button B
            if (row_1_diff & 0x8) {
                myDFPlayer.volumeUp();
            }

            char row_2_diff = rows[2] ^ joystick_state.row[2];

            // button 4
            if (row_2_diff & 0x8) {
                myDFPlayer.volumeDown();
            }

            // move left hand
            bool wheel_active = true;
            if (!shake_hands_animation.is_active()) {
                static float angle = 0.;
                if (joystick_state.row[2] & 0x1) {
                    move_left_hand_animation.start();
                    move_left_hand_animation.set_speed(joystick_state.j_front_back);
                    move_right_hand_animation.start();
                    move_right_hand_animation.set_speed(
                        joystick_state.j_left_right);
                    wheel_active = false;
                } else {
                    move_left_hand_animation.stop();
                    move_right_hand_animation.stop();
                }
            }

            if (!shake_head_animation.is_active()) {
                if (joystick_state.row[2] & 0x2) {
                    move_head_animation.start();
                    move_head_animation.set_speed(joystick_state.j_front_back);

                    move_neck_animation.start();
                    move_neck_animation.set_speed(joystick_state.j_left_right);
                    wheel_active = false;
                } else {
                    move_head_animation.stop();
                    move_neck_animation.stop();
                }
            }

            if (!shake_body_animation.is_active()) {
                if (joystick_state.row[2] & 0x4) {
                    move_body_animation.start();
                    move_body_animation.set_speed(joystick_state.j_front_back);
                    wheel_active = false;
                } else {
                    move_body_animation.stop();
                }
            }

            if (wheel_active && (joystick_state.j_front_back > 0.05 ||
                                joystick_state.j_front_back < -0.05 ||
                                joystick_state.j_left_right > 0.05 ||
                                joystick_state.j_left_right < -0.05)) {

                const float max_speed = 30.;
                const float max_rotation = 15.;

                float left_wheel_angle =
                    joystick_state.j_front_back * max_speed +
                    joystick_state.j_left_right * max_rotation + 90.;
                left_wheel_servo.write(left_wheel_angle);

                float right_wheel_angle =
                    -joystick_state.j_front_back * max_speed +
                    joystick_state.j_left_right * max_rotation + 90.;
                right_wheel_servo.write(right_wheel_angle);
            } else {
                left_wheel_servo.release();
                right_wheel_servo.release();
            }

            // save previous state of buttons
            memcpy(&rows, joystick_state.row, sizeof(joystick_state.row));
        }
    }
}

void loop() {
    for (;;) {
        unsigned long cur_time = millis();
        shake_body_animation.process(cur_time);
        shake_head_animation.process(cur_time);
        led_animation.process(cur_time);
        shake_hands_animation.process(cur_time);
        move_left_hand_animation.process(cur_time);
        move_right_hand_animation.process(cur_time);
        move_head_animation.process(cur_time);
        move_neck_animation.process(cur_time);
        move_body_animation.process(cur_time);
        delay(10);
    }
}
