#include "animation.h"
#include <DFRobotDFPlayerMini.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <EEPROM.h>

// Joystick MAC Address

#define LED_FREQ 1000
//#define DEBUG_VOODOO_JOYSTICK_CALIBRATION

SemaphoreHandle_t xMutex = NULL; // Create a mutex object

struct JoystickState {
    char protocol;
    char row[4];
    float j_left_right;
    float j_front_back;
    char j_b;
} joystick_state;

struct ModeState {
    char mode = 0;
    char clb_axis = 0;
    char ref_axis = 4;
    float min = 0;
    float max = 10;
};

struct VoodooJoystickState {
    char protocol;
    float values[5];
    ModeState mode;
} voodoo_joystick_state;

bool new_voodoo_joystick_packet = false;

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

TaskHandle_t voodoo_task;

struct ServoClbInfo {
    struct Point {
        float row = 0;
        float angle = 0;
    };
    Point min;
    Point max;
    bool calibrated = false;
};

ServoClbInfo servo_clb_data[5];
Servo * voodoo_servos[] = {&head_servo, &neck_servo, &left_hand_servo, &right_hand_servo, &body_servo};

void process_voodoo(void *_) {
    VoodooJoystickState vj_state;
    int clb_index = 0;
    float row = 0;
    float angle = 0;
    bool point_calibrated = false;
    unsigned long last_packet_timeout = 2000;
    unsigned long last_time = millis();
    ;

    for (;;) {
        //calculate joystick packet delay
        unsigned long cur_time = millis();
        unsigned long dt = cur_time - last_time;

        last_packet_timeout += dt;
        last_time = cur_time;  

        //reading joystick data
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            vj_state = voodoo_joystick_state;

            // new joystick packet, reset timeout 
            if (new_voodoo_joystick_packet) {
                new_voodoo_joystick_packet = false;
                last_packet_timeout = 0;
            }
            xSemaphoreGive(xMutex);
        }
        // no connection to joystick
        if (last_packet_timeout >= 1000) {
            for (int i = 0; i < sizeof(servo_clb_data) / sizeof(servo_clb_data[1]); i++) {
                Servo & servo(*voodoo_servos[i]);
                servo.release();
            }
        // joystick alive
        } else {
            switch (vj_state.mode.mode) {
                // normal mode
                case 0: {
                    for (int i = 0; i < sizeof(servo_clb_data) / sizeof(servo_clb_data[1]); i++) {
                        ServoClbInfo &info(servo_clb_data[i]);
                        Servo & servo(*voodoo_servos[i]);

                        if (info.calibrated) {
                            row = vj_state.values[i];
                            angle = ((row - info.min.row) / (info.max.row - info.min.row)) *
                                    (info.max.angle - info.min.angle) + info.min.angle;

                            //apply constrain
                            if (info.min.angle < info.max.angle) {
                                angle = constrain(
                                    angle, info.min.angle, info.max.angle);
                            } else {
                                angle = constrain(
                                    angle, info.max.angle, info.min.angle);
                            }
                            servo.write(angle);

                            #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                                Serial.print("axis: ");
                                Serial.print(i);

                                Serial.print(" row: ");
                                Serial.print(row);

                                Serial.print(" angle: ");
                                Serial.print(angle);

                                Serial.print(" min_row:");
                                Serial.print(info.min.row);

                                Serial.print(" min_angle:");
                                Serial.print(info.min.angle);

                                Serial.print(" max_row:");
                                Serial.print(info.max.row);

                                Serial.print(" max_angle:");
                                Serial.println(info.max.angle);
                            #endif
                        } else {
                            servo.release();
                        }
                    }
                    break;
                }
                // calibration
                case 1: {
                    ModeState &mode(vj_state.mode);
                    clb_index = mode.clb_axis;
                    servo_clb_data[clb_index].calibrated = false;
                    point_calibrated = false;

                    row = vj_state.values[clb_index];
                    angle = map(vj_state.values[mode.ref_axis], mode.min, mode.max, 0.,
                                180.);
                    angle = constrain(angle, 0., 180.);

                    #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                        // send cur angle
                        Serial.print("clb: clb_axis: ");
                        Serial.print(clb_index);

                        Serial.print(" ref_axis: ");
                        Serial.print(int(mode.ref_axis));

                        Serial.print(" min: ");
                        Serial.print(mode.min);

                        Serial.print(" max: ");
                        Serial.print(mode.max);

                        Serial.print(" row: ");
                        Serial.print(row);

                        Serial.print(" angle: ");
                        Serial.println(angle);
                    #endif
                    // servo_clb_data[clb_index].servo.release();
                    voodoo_servos[clb_index]->write(angle);
                    break;
                }
                // stop calib
                case 2: {
                    ModeState &mode(vj_state.mode);
                    switch (mode.clb_axis) {
                        // not apply calibration and stop calibration process
                        case 0: {
                            if (!servo_clb_data[clb_index].calibrated) {
                                servo_clb_data[clb_index].calibrated = false;
                                #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                                    Serial.print("reset clb:");
                                    Serial.println(clb_index);
                                #endif
                                voodoo_servos[clb_index]->release();
                            }
                            break;
                        }
                        // save point 1
                        case 1: {
                            if (!point_calibrated) {
                                point_calibrated = true;
                                servo_clb_data[clb_index].min.angle = angle;
                                servo_clb_data[clb_index].min.row = row;
                                voodoo_servos[clb_index]->release();

                                #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                                    Serial.print("set min clb point axis:");
                                    Serial.print(clb_index);

                                    Serial.print(" row:");
                                    Serial.print(servo_clb_data[clb_index].min.row);
                                    Serial.print(" angle:");
                                    Serial.println(servo_clb_data[clb_index].min.angle);
                                #endif
                            }
                            break;
                        }
                        // save point 2
                        case 2: {
                            if (!point_calibrated) {
                                point_calibrated = true;
                                servo_clb_data[clb_index].max.angle = angle;
                                servo_clb_data[clb_index].max.row = row;
                                voodoo_servos[clb_index]->release();

                                #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                                    Serial.print("set max clb point axis:");
                                    Serial.print(clb_index);

                                    Serial.print(" row:");
                                    Serial.print(servo_clb_data[clb_index].max.row);
                                    Serial.print(" angle:");
                                    Serial.println(servo_clb_data[clb_index].max.angle);
                                #endif
                            }
                            break;
                        }
                        //save calibration
                        case 3: {
                            if (!servo_clb_data[clb_index].calibrated) {
                                servo_clb_data[clb_index].calibrated = true;

                                //save calibration for axis
                                EEPROM.writeBytes(
                                    0,
                                    servo_clb_data,
                                    sizeof(servo_clb_data));

                                EEPROM.commit();

                                voodoo_servos[clb_index]->release();

                                #ifdef DEBUG_VOODOO_JOYSTICK_CALIBRATION
                                    Serial.print("apply clb axis:");
                                    Serial.println(clb_index);

                                    Serial.print(" min_row:");
                                    Serial.print(servo_clb_data[clb_index].min.row);

                                    Serial.print(" min_angle:");
                                    Serial.print(servo_clb_data[clb_index].min.angle);

                                    Serial.print(" max_row:");
                                    Serial.print(servo_clb_data[clb_index].max.row);

                                    Serial.print(" max_angle:");
                                    Serial.print(servo_clb_data[clb_index].max.angle);
                                #endif
                            }
                            break;
                        }
                    }
                    break;
                }
            }
        }
        delay(20);
    }
}

void setup() {
    xMutex = xSemaphoreCreateMutex(); // crete a mutex object
    last_msg_time = millis();

    right_hand_servo.attach(18);
    left_hand_servo.attach(13);

    body_servo.attach(19);
    head_servo.attach(12);
    neck_servo.attach(15);

    left_wheel_servo.attach(22);
    right_wheel_servo.attach(21);

    //release all servos
    right_hand_servo.release();
    left_hand_servo.release();
    body_servo.release();
    head_servo.release();
    neck_servo.release();
    left_wheel_servo.release();
    right_wheel_servo.release();

    left_led.attachPin(14, LED_FREQ, 10);  // 1KHz 8 bit
    right_led.attachPin(27, LED_FREQ, 10); // 1KHz 8 bit

    left_led.writeScaled(0.);
    right_led.writeScaled(0.);

    Serial2.begin(9600);
    Serial.begin(115200);

    //reading calibration 
    EEPROM.begin(sizeof(servo_clb_data));
    EEPROM.readBytes(0, servo_clb_data, sizeof(servo_clb_data));

    Serial.println(F("Calibration info:"));
    for (int i=0; i < sizeof(servo_clb_data)/sizeof(servo_clb_data[0]); i++) {
        ServoClbInfo & info(servo_clb_data[i]);

        Serial.print(F("axis:")); Serial.println(i);
        Serial.print(F("calibration:")); Serial.println(info.calibrated);
        Serial.print(F("min_row:")); Serial.println(info.min.row);
        Serial.print(F("min_angle:")); Serial.println(info.min.angle);
        Serial.print(F("max_row:")); Serial.println(info.max.row);
        Serial.print(F("max_angle:")); Serial.println(info.max.angle); Serial.println();
    }

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

    xTaskCreatePinnedToCore(
        process_voodoo, /* Task function. */
        "voodoo",       /* name of task. */
        10000,          /* Stack size of task */
        NULL,           /* parameter of the task */
        1,              /* priority of the task */
        &voodoo_task,   /* Task handle to keep track of created task */
        0);             /* pin task to core 0 */
    Serial.println("Setup finished");
}

void process_voodoo_joystick(const uint8_t *incoming_data, int len) {
    if (len == sizeof(voodoo_joystick_state)) {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            voodoo_joystick_state =
                *reinterpret_cast<const VoodooJoystickState *>(incoming_data);
            new_voodoo_joystick_packet = true;
            xSemaphoreGive(xMutex);
        }
    }
}

// Callback when data is received
void on_data_recv(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    if (len == 0) {
        Serial.print("wrong packet");
        return;
    }
    // processing of voodoo joystick
    if (incoming_data[0] == 1) {
        process_voodoo_joystick(incoming_data, len);
        // processing of normal joystick
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
                    move_left_hand_animation.set_speed(
                        joystick_state.j_front_back);
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
    } else {
        Serial.print("receive wrong packet: ");
        Serial.println(len);
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
