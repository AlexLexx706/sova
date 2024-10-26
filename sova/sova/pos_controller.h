#ifndef _POS_CONTROLLER_H_
#define _POS_CONTROLLER_H_
#include <cmath>
#include <algorithm>

class PosController {
public:
    /**
     * Updates the controller's position and speed based on the target position and time step.
     * Adjusts speed to smoothly approach the target without exceeding maximum speed or acceleration.
     * 
     * @param target_position Desired position to move towards.
     * @param dt Time step for the update (in seconds).
     * @return True if the target position is reached, false otherwise.
     */
    bool update(float target_position, float dt) {
        // If the target is already reached, do not update the state further
        if (target_reached_) return target_reached_;

        // Calculate the difference between the current and target position
        float position_error = target_position - current_position_;
        float direction = (position_error > 0) ? 1 : -1;

        // Check if the target has been reached
        if (std::abs(position_error) < position_tolerance_) {// && std::abs(current_speed_) < speed_tolerance_) {
            // If within allowable tolerance, mark target as reached
            target_reached_ = true;
            current_speed_ = 0.0;
            current_position_ = target_position; // Set position exactly to the target
            return target_reached_;
        }

        // Calculate safe speed for braking based on remaining distance
        float safe_speed = std::sqrt(2 * std::abs(position_error) * max_acceleration_);

        // Limit the target speed to avoid exceeding maximum allowable speed
        float target_speed = std::min(safe_speed, max_speed_);

        // Adjust current speed to approach target speed
        if (std::abs(current_speed_) < target_speed) {
            // Accelerate towards the target
            current_speed_ += direction * std::min(max_acceleration_ * dt, target_speed - std::abs(current_speed_));
        } else {
            // Smooth deceleration
            current_speed_ -= direction * std::min(max_acceleration_ * dt, std::abs(current_speed_) - target_speed);
        }

        // Update the current position
        current_position_ += current_speed_ * dt;
        return target_reached_;
    }

    /**
     * Resets the target reached status and initializes position and speed.
     * 
     * @param initial_position Initial position of the controller.
     * @param initial_speed Initial speed of the controller.
     */
    void reset(float initial_position, float initial_speed) {
        target_reached_ = false;
        current_position_ = initial_position;
        current_speed_ = initial_speed;
    }

    void reset() {
        target_reached_ = false;
    }

    /**
     * Sets the maximum speed allowed for the controller.
     * 
     * @param max_speed Maximum speed (units per second).
     */
    void set_max_speed(float max_speed) {
        max_speed_ = max_speed;
    }

    /**
     * Sets the maximum acceleration allowed for the controller.
     * 
     * @param max_acceleration Maximum acceleration (units per second squared).
     */
    void set_max_acceleration(float max_acceleration) {
        max_acceleration_ = max_acceleration;
    }

    /**
     * Retrieves the current position of the controller.
     * 
     * @return The current position.
     */
    float pos() const { return current_position_; }

    /**
     * Retrieves the current speed of the controller.
     * 
     * @return The current speed.
     */
    float speed() const { return current_speed_; }

    /**
     * Checks if the target position has been reached.
     * 
     * @return True if the target position is within tolerance, false otherwise.
     */
    bool target_reached() const { return target_reached_; }

private:
    float max_speed_ = 30.0;            ///< Maximum speed allowed (units per second).
    float max_acceleration_ = 30.0;     ///< Maximum acceleration allowed (units per second squared).
    float position_tolerance_ = 1.5;    ///< Acceptable tolerance for reaching the target position.
    float speed_tolerance_ = 1.5;       ///< Acceptable tolerance for the speed when reaching the target.
    float current_position_ = 0.0;      ///< Current position of the controller.
    float current_speed_ = 0.0;         ///< Current speed of the controller.
    bool target_reached_ = false;        ///< Indicates if the target position has been reached.
};

#endif //_POS_CONTROLLER_H_