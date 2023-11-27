#ifndef _H_ANIMATION_H_
#define _H_ANIMATION_H_

class Animation {
 protected:
    int active_flag = 0;
    int pause_flag = 0;
    unsigned long start_time = 0;
    unsigned long elapsed_time = 0;
    unsigned long last_time = 0;

    virtual void handle(unsigned long cur_time, unsigned long dt) = 0;
    virtual void paused_handler(){};
    virtual void start_handler(){};
    virtual void stop_handler(){};

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
    void stop_handler() override  { paused_handler(); }
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
    static constexpr float period_ms = 500;
    static constexpr unsigned long max_time = period_ms * 4;

 protected:
    void handle(unsigned long cur_time, unsigned long dt) override;
    void stop_handler() override { paused_handler(); }
    void paused_handler() override;
};

class MoveAnimation : public Animation {
 public:
    enum Type { left_hand = 0, right_hand = 1, body = 2, head = 3 };

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
#endif