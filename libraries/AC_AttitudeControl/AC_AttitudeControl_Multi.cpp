#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: MRAC
    // @DisplayName: To say if we use the adaptive control or not
    // @Description: To say if we use the adaptive control or not
    // @Values: 1 if we use it, 0 otherwise
    AP_GROUPINFO("MRAC", 60, AC_AttitudeControl_Multi, _mrac, 0),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::InitializeMatrix()
{
    MRAC_coef.setZero();
    
    B.setZero();
    B(0, 0) = dh/Jxx;
    B(1, 1) = dh/Jyy;
    B(2, 2) = 1/Jzz;

    // P is calculated offline using python scipy to solve lyapunov equation
    P.setZero();
    P(0, 0) = 0.0063; P(0, 3) = 0.0063;
    P(1, 1) = 0.0063; P(1, 4) = 0.0063;
    P(2, 2) = 0.00315; P(2, 5) = 0.00315;
    P(3, 0) = 0.0063; P(3, 3) = 1.0063;
    P(4, 1) = 0.0063; P(4, 4) = 1.0063;
    P(5, 2) = 0.00315; P(5, 5) = 1.00315;
}


void AC_AttitudeControl_Multi::rate_controller_run()
{
    /* 
    Here we are trying to implement a Model Reference Adaptive Controller on a sub system (only angular velocities) based on the paper from the MIT attached in the folder.
    It does work not as good as we expected, and I didn't manage to fix the problem.
    Maybe it is because we cannot implement the Vz component in the controller here as in the python script we made 
    Or because we state that the reference model is equal to the target, whereas it is not excatly true, instead of propagating the reference model. 
    But we can't propagate properly the reference model as we don't have access to the desired position/velocity here. 
    */

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    // We initialize the Matrix just one time at the beginning, We created a function to initialize because we cannot set values to the matrix in the header file
    if (not(ini)){
        InitializeMatrix();
        ini = false;
    }

    // --------------------
    // Nominal controller
    // --------------------
    /* To implement the MRAC we usually create the nominal controller, which is stored in a matrix Kx,
    and then we deduced the P matrix from the Kx matrix. Here we calculated the P matrix offline using
    a PI controller. So we should use PI controller as our nominal controller as well. 
    Ardupilot are PIDs controller but the D component is very low so we can use it. */ 
    

    // Here we state thate Nominal controller = Results of the PIDs
    un_roll = get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, 1, _motors.limit.roll) + _actuator_sysid.x;
    un_pitch = get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, 2, _motors.limit.pitch) + _actuator_sysid.y;
    un_yaw = get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, 3, _motors.limit.yaw) + _actuator_sysid.z;
    // We can try other nominal controller but they are not as efficient as Ardupilot' PIDs
    // un_roll = (- 5*gyro_latest.x - x_error_integral[0])/300;
    // un_pitch = (- 5*gyro_latest.y - x_error_integral[1])/300;
    // un_yaw = (- 5*gyro_latest.z - x_error_integral[2])/300;


    // --------------------
    // Adaptive controller
    // --------------------
    // We apply the MRAC equation according to the paper from the MIT attached in the folder.
    float err[6] = {gyro_latest.x - xref[0] ,  gyro_latest.y - xref[1] ,  gyro_latest.z - xref[2] ,  x_error_integral[0] - xref[3] ,  x_error_integral[1] - xref[4] ,  x_error_integral[2] - xref[5]};  // Error between the state and the reference model
    float w_array[10] = {gyro_latest.x, gyro_latest.y, gyro_latest.z, x_error_integral[0], x_error_integral[1], x_error_integral[2], _ang_vel_body.x, _ang_vel_body.y, _ang_vel_body.z, 1};
    matrix::Matrix<float, 1, 6> e(err);
    matrix::Matrix<float, 10, 1> w(w_array);
    MRAC_coef += _dt*w*e*P*B;
    float Gamma = 2;  // Learning rate (if we increase it the MRAC will be more responsive)
    ua_roll = (-Gamma*MRAC_coef.transpose()*w)(0, 0);
    ua_pitch = (-Gamma*MRAC_coef.transpose()*w)(1, 0);
    ua_yaw = (-Gamma*MRAC_coef.transpose()*w)(2, 0);


    if (_mrac == 0)  // Parameter we can control from a python code to use only Ardupilot controller instead of our MRAC
    {    
        ua_roll = 0;
        ua_pitch = 0;
        ua_yaw = 0;
    }


    // Commande we send (each value should be between -1 and 1), it is not exactly torque
    u_roll = un_roll + ua_roll;  // Nominal controller + Adaptive controller 
    u_pitch = un_pitch + ua_pitch;
    u_yaw = un_yaw + ua_yaw;

    _motors.set_roll(u_roll);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(u_pitch);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(u_yaw);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);


    // Propagation

    // Error_integral 
    x_error_integral[0] += _dt*(gyro_latest.x - _ang_vel_body.x);
    x_error_integral[1] += _dt*(gyro_latest.y - _ang_vel_body.y);
    x_error_integral[2] += _dt*(gyro_latest.z - _ang_vel_body.z);

    // Ref model
    xref[3] += _dt*(xref[0] - _ang_vel_body.x);
    xref[4] += _dt*(xref[1] - _ang_vel_body.y);
    xref[5] += _dt*(xref[2] - _ang_vel_body.z);
    xref[0] = _ang_vel_body.x; // We state the that reference model is equal to the target.
    xref[1] = _ang_vel_body.y;
    xref[2] = _ang_vel_body.z;

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();


    // Logging the values
    roll = gyro_latest.x;
    pitch = gyro_latest.y;
    yaw = gyro_latest.z;
    target_roll = _ang_vel_body.x;
    target_pitch = _ang_vel_body.y;
    target_yaw = _ang_vel_body.z;
    e_roll = roll - target_roll;
    e_pitch = pitch - target_pitch;
    e_yaw = yaw - target_yaw;


    AP::logger().Write("ADA1", "TimeUS,un_roll,un_pitch,un_yaw,ua_roll,ua_pitch,ua_yaw",
                   "s------", // units: None
                   "F------", // mult: None
                   "Qffffff", // format: float
                   AP_HAL::micros64(),
                   un_roll,
                   un_pitch,
                   un_yaw,
                   ua_roll,
                   ua_pitch,
                   ua_yaw);

    AP::logger().Write("ADA2", "TimeUS,roll,pitch,yaw,targ_r,targ_p,targ_y",
                   "s------", // units: None
                   "F------", // mult: None
                   "Qffffff", // format: float
                   AP_HAL::micros64(),
                   roll,
                   pitch,
                   yaw,
                   target_roll,
                   target_pitch,
                   target_yaw);

    AP::logger().Write("ADA3", "TimeUS,e_roll,e_pitch,e_yaw,u_roll,u_pitch,u_yaw",
                   "s------", // units: None
                   "F------", // mult: None
                   "Qffffff", // format: float
                   AP_HAL::micros64(),
                   e_roll,
                   e_pitch,
                   e_yaw,
                   u_roll,
                   u_pitch,
                   u_yaw);

    control_monitor_update();
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}
