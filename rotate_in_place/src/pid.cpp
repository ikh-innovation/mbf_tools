#include <rotate_in_place/pid.h>
#include <algorithm>
#include <cmath>

template <typename T>
T clamp(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(value, max_value));
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double angularError(double reference, double feedback) {
    return normalizeAngle(reference - feedback);
}

PIDController::PIDController(double kp, double ki, double kd,
                             double min_output, double max_output)
    : kp_(kp), ki_(ki), kd_(kd),
      min_output_(min_output), max_output_(max_output),
      previous_error_(0.0), integral_(0.0), first_run_(true) {}

void PIDController::compute(double reference, double feedback, const ros::Time& timestamp,
                            double& output, double& error) {
    // error = reference - feedback;
    error = angularError(feedback, reference);

    if (first_run_) {
        previous_time_ = timestamp;
        previous_error_ = error;
        first_run_ = false;
        output = clamp(kp_ * error, min_output_, max_output_);
        return;
    }

    ros::Duration duration = timestamp - previous_time_;
    double dt = duration.toSec();

    if (dt <= 0.0) {
        output = 0.0;
        return;
    }

    double derivative = (error - previous_error_) / dt;
    double temp_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    double clamped_output = clamp(temp_output, min_output_, max_output_);

    if (temp_output == clamped_output) {
        integral_ += error * dt;
    }

    temp_output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = clamp(temp_output, min_output_, max_output_);

    previous_error_ = error;
    previous_time_ = timestamp;
}

void PIDController::reset() {
    previous_error_ = 0.0;
    integral_ = 0.0;
    previous_time_ = ros::Time(0);
    first_run_ = true;
}
