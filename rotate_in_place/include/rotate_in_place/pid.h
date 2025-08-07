#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <ros/ros.h>

class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double min_output, double max_output);

    void compute(double reference, double feedback, const ros::Time& timestamp,
                            double& output, double& error); 

    void reset();

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;

    double previous_error_;
    double integral_;
    ros::Time previous_time_;
    bool first_run_;
};

#endif // PIDCONTROLLER_H
