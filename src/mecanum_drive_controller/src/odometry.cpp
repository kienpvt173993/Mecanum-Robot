#include "mecanum_drive_controller/odometry.hpp"

namespace mecanum_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
:   timestamp_(0.0),
    x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_x_(0.0),
    linear_y_(0.0),
    angular_(0.0),
    wheel_separation_width_(0.0),
    wheel_separation_height_(0.0),
    wheel_radius_(0.0),
    fl_wheel_old_pos_(0.0),
    fr_wheel_old_pos_(0.0),
    rl_wheel_old_pos_(0.0),
    rr_wheel_old_pos_(0.0),
    velocity_rolling_window_size_(velocity_rolling_window_size),
    linear_x_accumulator_(velocity_rolling_window_size),
    linear_y_accumulator_(velocity_rolling_window_size),
    angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
}

bool Odometry::update(double fl_pos, double fr_pos, double rl_pos, double rr_pos, const rclcpp::Time & time)
{
    // We cannot estimate the speed with very small time intervals:
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001)
    {
        return false;  // Interval too small to integrate with
    }

    // Get current wheel joint positions:
    const double fl_wheel_cur_pos = fl_pos * wheel_radius_;
    const double fr_wheel_cur_pos = fr_pos * wheel_radius_;
    const double rl_wheel_cur_pos = rl_pos * wheel_radius_;
    const double rr_wheel_cur_pos = rr_pos * wheel_radius_;

    // Estimate velocity of wheels using old and current position:
    const double fl_wheel_est_vel = fl_wheel_cur_pos - fl_wheel_old_pos_;
    const double fr_wheel_est_vel = fr_wheel_cur_pos - fr_wheel_old_pos_;
    const double rl_wheel_est_vel = rl_wheel_cur_pos - rl_wheel_old_pos_;
    const double rr_wheel_est_vel = rr_wheel_cur_pos - rr_wheel_old_pos_;

    // Update old position with current:
    fl_wheel_old_pos_ = fl_wheel_cur_pos;
    fr_wheel_old_pos_ = fr_wheel_cur_pos;
    rl_wheel_old_pos_ = rl_wheel_cur_pos;
    rr_wheel_old_pos_ = rr_wheel_cur_pos;

    updateFromVelocity(fl_wheel_est_vel, fr_wheel_est_vel, rl_wheel_est_vel, rr_wheel_est_vel, time);

    return true;
}

bool Odometry::updateFromVelocity(double fl_vel, double fr_vel, double rl_vel, double rr_vel,const rclcpp::Time & time)
{
    const double dt = time.seconds() - timestamp_.seconds();

    // Compute linear and angular diff:
    const double r_4 = wheel_radius_/4.0;
    const double linear_x = (fl_vel + fr_vel + rl_vel + rr_vel)*r_4;
    const double linear_y = (-fl_vel + fr_vel + rl_vel - rr_vel)*r_4;
    // Now there is a bug about scout angular velocity
    const double angular = (-fl_vel + fr_vel - rl_vel + rr_vel)*r_4/(wheel_separation_height_+wheel_separation_width_)*2;

    // Integrate odometry:
    integrateExact(linear_x, linear_y, angular);

    timestamp_ = time;

    // Estimate speeds using a rolling mean to filter them out:
    linear_x_accumulator_.accumulate(linear_x / dt);
    linear_y_accumulator_.accumulate(linear_y / dt);
    angular_accumulator_.accumulate(angular / dt);

    linear_x_ = linear_x_accumulator_.getRollingMean();
    linear_y_ = linear_y_accumulator_.getRollingMean();
    angular_ = angular_accumulator_.getRollingMean();

    return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
    /// Save last linear and angular velocity:
    linear_x_ = linear_x;
    linear_y_ = linear_y;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = time.seconds() - timestamp_.seconds();
    timestamp_ = time;
    integrateExact(linear_x * dt, linear_y * dt, angular * dt);
}

void Odometry::resetOdometry()
{
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_separation_width,
    double wheel_separation_height, double wheel_radius)
{
    wheel_separation_width_ = wheel_separation_width;
    wheel_separation_height_ = wheel_separation_height;
    wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double linear_y,double angular)
{
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    x_ += linear_x * cos(direction) - linear_y * sin(direction);
    y_ += linear_x * sin(direction) + linear_y * cos(direction);
    heading_ += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y,double angular)
{
    if (fabs(angular) < 1e-6)
    {
        integrateRungeKutta2(linear_x, linear_y, angular);
    }
    else
    {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = heading_;
        const double r_x = linear_x / angular;
        const double r_y = linear_y / angular;
        heading_ += angular;

        x_ += r_x * (sin(heading_) - sin(heading_old)) - r_y * (cos(heading_) - cos(heading_old));
        y_ += r_x * (cos(heading_) - cos(heading_old)) + r_y * (sin(heading_) - sin(heading_old));
    }
}

void Odometry::resetAccumulators()
{
    linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace diff_drive_controller