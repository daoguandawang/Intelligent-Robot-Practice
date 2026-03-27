#ifndef SIMPLE_POINT_PID_H
#define SIMPLE_POINT_PID_H

struct PIDConfig {
    float kp = 4.0f;
    float ki = 0.2f;
    float kd = 0.1f;

    float integralLimit = 200.0f;
    float outputLimit = 1200.0f;

    float errorDeadband = 2.0f;
    float outputDeadband = 5.0f;
};

struct PIDState {
    float integral = 0.0f;
    float lastError = 0.0f;
    bool hasLastError = false;
};

struct PointPidOutput {
    float yawSpeed = 0.0f;
    float pitchSpeed = 0.0f;
};

class SimplePointPid {
public:
    PIDConfig xCfg;
    PIDConfig yCfg;

    PIDState xState;
    PIDState yState;

    void reset();

    PointPidOutput update(float targetX,
                          float targetY,
                          float currentX,
                          float currentY,
                          float dtSeconds);

private:
    static float clamp(float value,
                       float minValue,
                       float maxValue);

    static float updateOneAxis(const PIDConfig &cfg,
                               PIDState &state,
                               float error,
                               float dtSeconds);
};

#endif
