#include "simple_point_pid.h"
#include <math.h>

void SimplePointPid::reset()
{
    xState = PIDState();
    yState = PIDState();
}

PointPidOutput SimplePointPid::update(float targetX,
                                      float targetY,
                                      float currentX,
                                      float currentY,
                                      float dtSeconds)
{
    PointPidOutput out;

    float errorX = targetX - currentX;
    float errorY = targetY - currentY;

    out.yawSpeed = -updateOneAxis(xCfg, xState, errorX, dtSeconds);
    out.pitchSpeed = -updateOneAxis(yCfg, yState, errorY, dtSeconds);

    return out;
}

float SimplePointPid::clamp(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

float SimplePointPid::updateOneAxis(const PIDConfig& cfg,
                                    PIDState& state,
                                    float error,
                                    float dtSeconds)
{
    if (fabsf(error) <= cfg.errorDeadband) {
        error = 0.0f;
    }

    float derivative = 0.0f;

    if (dtSeconds > 0.0f) {
        state.integral += error * dtSeconds;
        state.integral = clamp(state.integral, -cfg.integralLimit, cfg.integralLimit);

        if (state.hasLastError) {
            derivative = (error - state.lastError) / dtSeconds;
        }
    }

    float output = cfg.kp * error +
                   cfg.ki * state.integral +
                   cfg.kd * derivative;

    output = clamp(output, -cfg.outputLimit, cfg.outputLimit);

    if (fabsf(output) <= cfg.outputDeadband) {
        output = 0.0f;
    }

    if (error == 0.0f && output == 0.0f) {
        state.integral = 0.0f;
    }

    state.lastError = error;
    state.hasLastError = true;

    return output;
}