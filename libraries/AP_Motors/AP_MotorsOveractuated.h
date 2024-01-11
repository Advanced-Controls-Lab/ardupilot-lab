#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/math.hpp> 
#include <RC_Channel/RC_Channel.h>
#include "AP_MotorsMatrix.h"

#define MIN_TILT_SERVO_ANGLE 0  
#define MAX_TILT_SERVO_ANGLE  180
//#define AP_MOTORS_1PITCH  CH_5
//#define AP_MOTORS_1ROLL   CH_9
//#define AP_MOTORS_2PITCH  CH_6
//#define AP_MOTORS_2ROLL   CH_10
//#define AP_MOTORS_3PITCH  CH_7
//#define AP_MOTORS_3ROLL   CH_11 
//#define AP_MOTORS_4PITCH  CH_8
//#define AP_MOTORS_4ROLL   CH_12
// Trims for each of the servos
/*
#define AP_1PITCH_TRIM       100 
#define AP_1ROLL_TRIM        0
#define AP_2PITCH_TRIM       200 
#define AP_2ROLL_TRIM        -75 
#define AP_3PITCH_TRIM        -60
#define AP_3ROLL_TRIM        0 
#define AP_4PITCH_TRIM       -100 
#define AP_4ROLL_TRIM        20 
*/
#define MU                  0.000016
#define Km                  0.000160
#define TorqueLength                  0.16
#define angle_const                  0.707106781
#define filter_scale        1.0f
class AP_MotorsOveractuated : public AP_MotorsMatrix {
public:

    /// Constructor
    AP_MotorsOveractuated(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix 6DoF must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsOveractuated *get_singleton() {
        return _singleton;
    }

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // sets the roll and pitch offset, this rotates the thrust vector in body frame
    // these are typically set such that the throttle thrust vector is earth frame up
    void set_roll_pitch(float roll_deg, float pitch_deg) override;

    // add_motor using raw roll, pitch, throttle and yaw factors, to be called from scripting
    void add_motor(int8_t motor_num, float roll_factor, float pitch_factor, float yaw_factor, float throttle_factor, float forward_factor, float right_factor, bool reversible, uint8_t testing_order);

    // if the expected number of motors have been setup then set as initalized
    void init(motor_frame_class frame_class, motor_frame_type frame_type) override;
    bool init(uint8_t expected_num_motors) override; 

    // Current offset angles, radians

    

    
    
protected:
    // output - sends commands to the motors
    void output_armed_stabilizing() override;


    // nothing to do for setup, scripting will mark as initalized when done
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override {};

    const char* _get_frame_string() const override { return "6DoF scripting"; }
    virtual void _output_test_seq(uint8_t motor_seq, int16_t pwm) override; 
    float _forward_factor[AP_MOTORS_MAX_NUM_MOTORS];      // each motors contribution to forward thrust
    float _right_factor[AP_MOTORS_MAX_NUM_MOTORS];        // each motors contribution to right thrust

    // true if motor is reversible, it can go from -Spin max to +Spin max, if false motor is can go from Spin min to Spin max
    bool _reversible[AP_MOTORS_MAX_NUM_MOTORS];
    void Log_Write_Overactuated();
    // store last values to allow deadzone
    float _last_thrust_out[AP_MOTORS_MAX_NUM_MOTORS];
    

    float _servo_pitch1_angle;
    float _servo_pitch2_angle;
    float _servo_pitch3_angle;
    float _servo_pitch4_angle;
     
    float _servo_roll1_angle;
    float _servo_roll2_angle;
    float _servo_roll3_angle;
    float _servo_roll4_angle; 

    float _roll_offset;
    float _pitch_offset; 
    float _previous_pitch_angles[4] = {0.0f, 0.0f, 0.0f, 0.0f}; 
    float _previous_roll_angles[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float _previous_thrust[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

private:
    static AP_MotorsOveractuated *_singleton;

};