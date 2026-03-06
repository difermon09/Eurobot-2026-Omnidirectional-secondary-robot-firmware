#pragma once

#include <Arduino.h>
#include <teensystep4.h>

using namespace TS4; 

struct OmniBaseConfig {
    uint8_t enable_pin;
    float wheel_diameter;
    float robot_radius;
    float microsteps = 1.0f;
    float v_max_sps = 1000.0f; // Steps per second (Before lose steps)
    float motor_steps = 200.0f;
};

struct OmniBaseHardware {
    Stepper &baseMotorA, &baseMotorB, &baseMotorC;
};

struct WheelSpeeds {
    float v1, v2, v3; 
};
struct WheelSteps  {
    long s1, s2, s3; 
};

struct Twist {
    float x, y, w;
};

struct Waypoint {
    Twist target;
    float t;
};

class OmniController {
    public:
        OmniController(OmniBaseHardware hw, OmniBaseConfig cfg);
        void begin();
        void move(Waypoint wp);
        bool is_limit_active();
        bool is_moving();

    private:
        Stepper &_m1, &_m2, &_m3;
        
        const uint8_t _ENABLE_PIN;
        const float _MOTOR_STEPS;
        const float _V_MAX_SPS;
        const float _ROBOT_RADIUS;

        float _steps_per_mm;
        bool _is_limiting = false;
        
        WheelSteps calculate_inverse_kinematics(float x, float y, float w);
        
        void apply_speed_limits(WheelSpeeds &speeds);
};