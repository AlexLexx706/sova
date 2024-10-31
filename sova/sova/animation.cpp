#include "animation.h"
#include <ESP32Servo.h>

extern Servo right_hand_servo;
extern Servo left_hand_servo;
extern Servo body_servo;
extern Servo head_servo;
extern Servo neck_servo;
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
    body_servo.write(90 - cur_angle);

    if (cur_time >= max_time + period) {
        stop();
    }
}
void ShakeBodyAnimation::paused_handler() { body_servo.release(); }

void ShakeHeadAnimation::handle(unsigned long cur_time, unsigned long dt) {
    unsigned long angle_time = cur_time >= max_time ? max_time : cur_time;

    float angle = sin(angle_time / period_ms * PI * 2) * max_angle;
    cur_angle += (angle - cur_angle) * dt / period;
    head_servo.write(90 - cur_angle);

    if (cur_time >= max_time + period) {
        stop();
    }
}

void ShakeHeadAnimation::paused_handler() { head_servo.release(); }

void LedAnimation::handle(unsigned long cur_time, unsigned long dt) {
    if (cur_time < delay_ms) {
        left_led.writeScaled(0);
        right_led.writeScaled(0);
    } else {
        float value =
            (cos((cur_time - delay_ms) / (float)period_ms * PI * 2 + PI) + 1.) /
            2.;
        left_led.writeScaled(value);
        right_led.writeScaled(value);
    }

    if (cur_time >= (max_time_ms + delay_ms)) {
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

    case neck: {
        angle = angle > 90 ? 90 : angle;
        angle = angle < -90 ? -90 : angle;
        neck_servo.write(angle + 90);
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
    case neck:
        // neck_servo.release();
        break;
    case body:
        body_servo.release();
        break;
    }
}

void SequenceAnimation::set_commands(const Command *commands, int count) {
    commands_ = commands;
    commands_count_ = count;
    cur_command_index_ = 0;
}

/**
 * @brief Starts the animation by resetting the command index and wait time.
 */
void SequenceAnimation::start_handler() {
    cur_command_index_ = 0;
    start_waiting_time_ = 0;
    update_controller_settings();
}

/**
 * @brief Pauses the animation by releasing the servo and resetting the
 * controller.
 */
void SequenceAnimation::paused_handler() { servo_.release(); }

/**
 * @brief Main handling function for the animation, executing move and wait
 * commands.
 *
 * This function iterates through the command array, either moving the servo to
 * a target position or waiting for a specified duration, before proceeding to
 * the next command.
 *
 * @param cur_time Current timestamp in milliseconds.
 * @param dt Delta time in milliseconds since the last handle call.
 */
void SequenceAnimation::handle(unsigned long cur_time, unsigned long dt) {
    if (cur_command_index_ >= commands_count_) {
        stop();
        return;
    }
    const Command &command = commands_[cur_command_index_];
    float dt_sec = dt / 1000.f;

    if (command.is_move) {
        // Move command: update position and move the servo if not reached
        // target
        if (!pos_controller_.update(command.pos_or_ms, dt_sec)) {
            servo_.write(pos_controller_.pos() + 90);
        } else {
            cur_command_index_++; // Move to the next command
            start_waiting_time_ = cur_time;
            update_controller_settings();
        }
    } else {
        // Wait command: wait for the specified duration
        if (cur_time - start_waiting_time_ >= command.pos_or_ms) {
            cur_command_index_++; // Move to the next command
            start_waiting_time_ = cur_time;
            update_controller_settings();
        }
    }
}

void SequenceAnimation::update_controller_settings() {
    if (cur_command_index_ < commands_count_ &&
        commands_[cur_command_index_].is_move) {
        pos_controller_.reset();
        const Command &command = commands_[cur_command_index_];
        pos_controller_.set_max_speed(command.max_speed);
        pos_controller_.set_max_acceleration(command.max_accel);
    }
}

void PositionControlAnimation::handle(unsigned long cur_time,
                                      unsigned long dt) {
    float dt_sec = dt / 1000.f;
    float speed = generator.evaluate(dt_sec, cur_pos);
    cur_pos += speed * dt_sec;
    servo_.write(cur_pos + zero_pos_);
}

void PositionControlAnimation::stop_handler() { servo_.release(); }

void TimeAnimation::set_commands(const Command *commands, int count) {
    commands_ = commands;
    commands_count_ = count;
    cur_command_index_ = 1;
}

void TimeAnimation::start_handler() {
    cur_command_index_ = 1;
}

void TimeAnimation::paused_handler() { servo_.release(); }

void TimeAnimation::handle(unsigned long cur_time_, unsigned long dt) {
    float cur_time = cur_time_ / 1000.f;
    //process commands
    while (cur_command_index_ < commands_count_) {
        const Command &cur_command = commands_[cur_command_index_];
        const Command &prev_command = commands_[cur_command_index_ - 1];
        float scale = (cur_time - prev_command.time) / (cur_command.time - prev_command.time);

        // process pos
        if (scale < 1.f) {
            float pos = (cur_command.pos - prev_command.pos) * scale + prev_command.pos;
            servo_.write(pos + 90);
            return;
        // next command
        } else {
            cur_command_index_++; // Move to the next command
        }
    }

    stop();
}
