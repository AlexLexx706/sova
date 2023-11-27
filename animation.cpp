#include "animation.h"
#include <ESP32Servo.h>

extern Servo right_hand_servo;
extern Servo left_hand_servo;
extern Servo body_servo;
extern Servo head_servo;
extern Servo left_wheel_servo;
extern Servo right_wheel_servo;
extern ESP32PWM left_led;
extern ESP32PWM right_led;


void Animation::start() {
    if (!active_flag) {
        pause_flag = false;
        start_time = millis();
        last_time = 0;
        elapsed_time = 0;
        start_handler();
        active_flag = 1;
    }
}

void Animation::stop() {
    if (active_flag) {
        active_flag = 0;
        stop_handler();
    }
}

void Animation::pause(unsigned long cur_time) {
    if (active_flag) {
        if (!pause_flag) {
            pause_flag = 1;
            elapsed_time += cur_time - start_time;
        }
    }
}
void Animation::un_pause(unsigned long cur_time) {
    if (active_flag) {
        if (pause_flag) {
            start_time = cur_time;
            pause_flag = 0;
        }
    }
}

void Animation::process(unsigned long cur_time) {
    if (active_flag && !pause_flag) {
        cur_time = elapsed_time + cur_time - start_time;
        handle(cur_time, cur_time - last_time);
        last_time = cur_time;
    }
}

void LiftHandsAnimation::handle(unsigned long cur_time, unsigned long dt) {
    float value = cur_time / float(max_time);

    // apply animation
    if (value < 1.) {
        float angle_offset = sin(value * PI) * max_angle;

        float right_angle = 90 + angle_offset;
        right_hand_servo.write(right_angle);

        float left_angle = 90 - angle_offset;
        left_hand_servo.write(left_angle);
        // stop all
    } else {
        active_flag = false;
        right_hand_servo.release();
        left_hand_servo.release();
    }
}

void LiftHandsAnimation::paused_handler() {
    right_hand_servo.release();
    left_hand_servo.release();
}

void ShakeHandsAnimation::handle(unsigned long cur_time, unsigned long dt) {
    unsigned long angle_time = cur_time >= max_time ? max_time : cur_time;
    float angle = sin(angle_time / period_ms * PI * 2) * max_angle;
    cur_angle += (angle - cur_angle) * dt / period;

    float left_hand_angle = 90 + cur_angle;
    float right_hand_angle = 90 - cur_angle;

    right_hand_servo.write(right_hand_angle);
    left_hand_servo.write(left_hand_angle);

    if (cur_time >= max_time + period) {
        stop();
    }
}

void ShakeHandsAnimation::paused_handler() {
    right_hand_servo.release();
    left_hand_servo.release();
}

void ShakeBodyAnimation::handle(unsigned long cur_time, unsigned long dt) {
    unsigned long angle_time = cur_time >= max_time ? max_time : cur_time;
    float angle = sin(angle_time / period_ms * PI * 2) * max_angle;
    cur_angle += (angle - cur_angle) * dt / period;

    float left_hand_angle = 90 + cur_angle;
    float right_hand_angle = 90 - cur_angle;

    body_servo.write(right_hand_angle);

    if (cur_time >= max_time + period) {
        stop();
    }
}
void ShakeBodyAnimation::paused_handler() {
    body_servo.release();
}

void ShakeHeadAnimation::handle(unsigned long cur_time, unsigned long dt) {
    unsigned long angle_time = cur_time >= max_time ? max_time : cur_time;

    float angle = sin(angle_time / period_ms * PI * 2) * max_angle;
    cur_angle += (angle - cur_angle) * dt / period;

    float left_hand_angle = 90 + cur_angle;
    float right_hand_angle = 90 - cur_angle;

    head_servo.write(right_hand_angle);

    if (cur_time >= max_time + period) {
        stop();
    }
}

void ShakeHeadAnimation::paused_handler() {
    head_servo.release();
}

void LedAnimation::handle(unsigned long cur_time, unsigned long dt) {
    float value = (cos(cur_time / period_ms * PI * 2 + PI) + 1.) / 2.;
    left_led.writeScaled(value);
    right_led.writeScaled(value);

    if (cur_time >= max_time) {
        stop();
    }
}

void LedAnimation::paused_handler() {
    left_led.writeScaled(0.);
    right_led.writeScaled(0.);
}
void MoveAnimation::handle(unsigned long cur_time, unsigned long dt) {
    float _dt = dt / 1000.;
    angle += speed * _dt;
    // Serial.print("type:");
    // Serial.print(type);
    // Serial.print(" angle:");
    // Serial.println(angle);

    switch (type) {
    case left_hand: {
        angle = angle > 50 ? 50 : angle;
        angle = angle < -90 ? -90 : angle;
        left_hand_servo.write(-angle + 90);
        break;
    }
    case right_hand: {
        angle = angle > 85 ? 85 : angle;
        angle = angle < -45 ? -45 : angle;
        right_hand_servo.write(-angle + 90);
        break;
    }

    case head: {
        angle = angle > 70 ? 70 : angle;
        angle = angle < -40 ? -40 : angle;
        head_servo.write(angle + 90);
        break;
    }
    case body: {
        angle = angle > 70 ? 70 : angle;
        angle = angle < -50 ? -50 : angle;
        body_servo.write(angle + 90);
        break;
    }
    }
}
void MoveAnimation::stop_handler() {
    switch (type) {
    case left_hand:
        left_hand_servo.release();
        break;
    case right_hand:
        right_hand_servo.release();
        break;
    case head:
        head_servo.release();
        break;
    case body:
        body_servo.release();
        break;
    }
}
