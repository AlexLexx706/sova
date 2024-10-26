#ifndef __SPEED_PROFILE_GENERATOR_H_
#define __SPEED_PROFILE_GENERATOR_H_

#include <math.h>
#define sign(a) (a >= 0 ? 1 : -1)
#define _min(a, b) (((a) < (b)) ? (a) : (b))


class SpeedProfileGenerator {
    float max_speed = 1.;
    float accel = 1.;
    float target_pos = 0.;
    float speed = 0;

 public:
    void set_max_speed(float _speed) { max_speed = _speed; }

    float get_max_speed() const { return max_speed; }

    void set_accel(float _accel) { accel = _accel; }

    float get_accel() const { return accel; }

    float get_speed() const { return speed; }

    void set_target_pos(float pos) { target_pos = pos; }

    float get_target_pos() const { return target_pos; }

    bool is_ready() { return false; }

    float evaluate(float dt, float cur_pos) {
        float position_error = target_pos - cur_pos;
        float direction = sign(position_error);

        // decrease speed
        if (sign(speed) != direction) {
            speed += direction * (accel * dt);
        } else {
            float safe_speed = sqrt(2 * fabs(position_error) * accel);
            float target_speed = _min(safe_speed, max_speed);
            float fabs_speed = fabs(speed);

            // increase speed to target
            if (fabs_speed < target_speed) {
                speed += direction * _min(accel * dt, target_speed - fabs_speed);
                // decrease speed to min
            } else {
                speed -= direction * _min(accel * dt, fabs_speed - target_speed);
            }
        }
        return speed;
    }
};
#endif