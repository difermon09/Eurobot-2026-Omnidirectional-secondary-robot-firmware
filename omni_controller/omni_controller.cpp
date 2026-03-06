#include "omni_controller.h"

// Public
OmniController::OmniController(OmniBaseHardware hw, OmniBaseConfig cfg)
    : _m1(hw.baseMotorA), _m2(hw.baseMotorB), _m3(hw.baseMotorC),
    _ENABLE_PIN(cfg.enable_pin),
    _MOTOR_STEPS(cfg.motor_steps), 
    _V_MAX_SPS(cfg.v_max_sps),
    _ROBOT_RADIUS(cfg.robot_radius)
{
    float steps_per_rev = _MOTOR_STEPS * cfg.microsteps;
    float mm_per_rev = PI * cfg.wheel_diameter;
    _steps_per_mm = steps_per_rev / mm_per_rev;
}

void OmniController::begin() {
    TS4::begin();
    pinMode(_ENABLE_PIN, OUTPUT);
    digitalWrite(_ENABLE_PIN, LOW); 
}

void OmniController::move(Waypoint wp) {

    // Obtener pasos
    WheelSteps steps = calculate_inverse_kinematics(wp.target.x, wp.target.y, wp.target.w);

    // Calcular velocidades base
    WheelSpeeds speeds;
    speeds.v1 = abs((float)steps.s1) / wp.t;
    speeds.v2 = abs((float)steps.s2) / wp.t;
    speeds.v3 = abs((float)steps.s3) / wp.t;

    // Aplicar límites velocidades de forma proporcional
    apply_speed_limits(speeds);

    // Ejecutar movimento. tiempo en llegar a vel_max = vel / accel --> 0,5s en este caso
    _m1.setMaxSpeed(speeds.v1).setAcceleration(speeds.v1 * 2).moveRelAsync(steps.s1); 
    _m2.setMaxSpeed(speeds.v2).setAcceleration(speeds.v2 * 2).moveRelAsync(steps.s2);
    _m3.setMaxSpeed(speeds.v3).setAcceleration(speeds.v3 * 2).moveRelAsync(steps.s3);
}

bool OmniController::is_limit_active() {
    return _is_limiting;
}

bool OmniController::is_moving() {
    return _m1.isMoving || _m2.isMoving || _m3.isMoving;
}
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Private
WheelSteps OmniController::calculate_inverse_kinematics(float x, float y, float w) {
    float angular_displacement = (w * (PI / 180.0f)) * _ROBOT_RADIUS;

    float d1 = (-0.5f * x + 0.866f * y) + angular_displacement;;
    float d2 = (-0.5f * x - 0.866f * y) + angular_displacement;;
    float d3 = x + angular_displacement;;

    return {
        static_cast<long>(round(d1 * _steps_per_mm)),
        static_cast<long>(round(d2 * _steps_per_mm)),
        static_cast<long>(round(d3 * _steps_per_mm))
    };
}

void OmniController::apply_speed_limits(WheelSpeeds &speeds) {
    float max_requested_speed = std::max({speeds.v1, speeds.v2, speeds.v3});

    if (max_requested_speed > _V_MAX_SPS) {
        Serial.print("WARNING: The speed exceeds the limit. Reduce to the limit.");
        _is_limiting = true;

        float scaling_factor = _V_MAX_SPS / max_requested_speed;
        speeds.v1 *= scaling_factor;
        speeds.v2 *= scaling_factor;
        speeds.v3 *= scaling_factor;
    } else {
        _is_limiting = false;
    }
}