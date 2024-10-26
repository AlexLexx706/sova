#ifndef _H_ANIMATION_H_
#define _H_ANIMATION_H_
#include "pos_controller.h"
#include "speed_profile_generator.h"

class Servo;

class Animation {
 protected:
    int active_flag = 0;
    int pause_flag = 0;
    unsigned long start_time = 0;
    unsigned long elapsed_time = 0;
    unsigned long last_time = 0;

    virtual void handle(unsigned long cur_time, unsigned long dt) = 0;
    virtual void paused_handler() {};
    virtual void start_handler() {};
    virtual void stop_handler() {};

 public:
    void start();
    void stop();
    void pause(unsigned long cur_time);
    void un_pause(unsigned long cur_time);
    void process(unsigned long cur_time);

    int is_active() const { return active_flag; }
    int is_paused() const { return pause_flag; }
};

class LiftHandsAnimation : public Animation {
    unsigned long max_time = 1000;
    long max_angle = 50;

 public:
    LiftHandsAnimation(long _max_angle) : max_angle(_max_angle) {}

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void paused_handler() override;
};

class ShakeHandsAnimation : public Animation {
    static constexpr float period_ms = 1000;
    static constexpr float max_angle = 80;
    static constexpr float period = 200.;
    static constexpr unsigned long max_time = period_ms * 2;
    float cur_angle = 0.;

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void paused_handler() override;
    void stop_handler() override { paused_handler(); }
    void start_handler() override { cur_angle = 0.; }
};

class ShakeBodyAnimation : public Animation {
    static constexpr float period_ms = 3000;
    static constexpr float max_angle = 60;
    static constexpr float period = 200.;
    static constexpr unsigned long max_time = period_ms;
    float cur_angle = 0.;

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override { paused_handler(); }
    void start_handler() override { cur_angle = 0.; }
    void paused_handler() override;
};

class ShakeHeadAnimation : public Animation {
    static constexpr float period_ms = 1000;
    static constexpr float max_angle = 60;
    static constexpr float period = 200.;
    static constexpr unsigned long max_time = period_ms * 2;
    float cur_angle = 0.;

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override { paused_handler(); }
    void start_handler() override { cur_angle = 0.; }
    void paused_handler() override;
};

class LedAnimation : public Animation {
    unsigned long period_ms = 500;
    unsigned long max_time_ms = 2000;
    unsigned long delay_ms = 0;
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override { paused_handler(); }
    void paused_handler() override;
 public:
    void set_params(unsigned long _period_ms, unsigned long _max_time_ms, unsigned long _delay_ms=0) {
        period_ms = _period_ms;
        max_time_ms = _max_time_ms;
        delay_ms = _delay_ms;
    }
};

class MoveAnimation : public Animation {
 public:
    enum Type { left_hand = 0, right_hand = 1, body = 2, head = 3, neck = 4 };

    explicit MoveAnimation(Type _type) : type(_type) {}
    void set_speed(float _speed) { speed = _speed * 260.; }

 private:
    static constexpr float period = 500;
    static constexpr float max_angle = 90.;
    float angle = 0.;
    float speed = 0.; // deg/sec
    Type type;

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override;
};

class SequenceAnimation : public Animation {
 public:
    /**
     * @brief Command structure to hold each animation instruction.
     *
     * Commands are either move commands (is_move = true), which specify a
     * target position, or wait commands (is_move = false), which specify a
     * duration in milliseconds.
     */
    struct Command {
        bool is_move; ///< Command type: true for movement, false for waiting.
        float pos_or_ms; ///< Value of the command (angle for movement, ms for
                         ///< wait).
        float max_speed;
        float max_accel;
    };

    /**
     * @brief Construct a new Sequence Animation object.
     *
     * @param servo Reference to the servo motor controlled by this animation.
     */
    SequenceAnimation(Servo &servo, const Command *commands = nullptr,
                      int count = 0)
        : servo_(servo) {
        set_commands(commands, count);
    }

    /**
     * @brief Sets the sequence of commands for the animation.
     *
     * @param commands Array of Command objects defining the animation sequence.
     * @param count Number of commands in the array.
     */
    void set_commands(const Command *commands, int count);

 protected:
    /**
     * @brief Executes the current command based on time and state.
     *
     * This function is called periodically to progress through the command
     * sequence, moving the servo or waiting as instructed by the current
     * command.
     *
     * @param cur_time Current timestamp in milliseconds.
     * @param dt Delta time since the last call in milliseconds.
     */
    void handle(unsigned long cur_time, unsigned long dt) override;

    /**
     * @brief Handler called when the animation is stopped.
     *
     * Calls the paused handler to reset any necessary states.
     */
    void stop_handler() override { paused_handler(); }

    /**
     * @brief Handler called when the animation starts.
     *
     * Resets the command index and initializes the waiting time.
     */
    void start_handler() override;

    /**
     * @brief Handler called when the animation is paused.
     *
     * Resets the position controller and stops any servo movement.
     */
    void paused_handler() override;

 private:
    const Command *commands_ = nullptr;
    int commands_count_ = 0;
    int cur_command_index_ = 0;
    PosController pos_controller_;
    unsigned long start_waiting_time_ = 0;
    Servo &servo_;
    void update_controller_settings();
};

class PositionControlAnimation : public Animation {
 private:
    Servo &servo_;
    float zero_pos_;
    float cur_pos = 0.;
    SpeedProfileGenerator generator;

 public:
    PositionControlAnimation(Servo &servo, float zero_pos)
        : servo_(servo), zero_pos_(zero_pos) {}
    SpeedProfileGenerator &get_generator() { return generator; };

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override;
};
#endif