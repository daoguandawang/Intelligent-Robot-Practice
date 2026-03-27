#include <Arduino.h>
#include <math.h>
#include <simple_point_pid.h>
#include <stepmotor_ard.h>
#include <stdio.h>
#include <string.h>

stepmotor_ard motor_yaw(4, 5, 11, HIGH);
stepmotor_ard motor_pitch(2, 3, 10, LOW);
SimplePointPid pidController;

volatile bool printFlag = false;

enum ControlMode {
    MODE_IDLE = 0,
    MODE_PID = 1,
    MODE_CALIB = 2,
    MODE_SPEED = 3
};

ControlMode g_mode = MODE_IDLE;

struct CalibrationSample {
    long yawPos;
    long pitchPos;
    float laserX;
    float laserY;
};

constexpr size_t MAX_CAL_SAMPLES = 320;
CalibrationSample g_calTable[MAX_CAL_SAMPLES];
size_t g_calCount = 0;

struct CalibrationTask {
    bool active = false;
    bool waitingPoint = false;
    bool rowForward = true;

    long yawStart = 0;
    long yawEnd = 0;
    long yawStep = 1;

    long pitchStart = 0;
    long pitchEnd = 0;
    long pitchStep = 1;


    long currentYaw = 0;
    long currentPitch = 0;

    // ... existing code ...
    /**
     * @brief 运动控制参数配置
     *
     * 定义机器人运动控制的关键时间参数：
     * - moveFreqHz: 运动频率，设置为300Hz，表示每秒执行300次运动控制循环
     * - settleMs: 稳定时间，设置为500毫秒，表示运动到位后需要等待的稳定时间
     * - pointTimeoutMs: 点位超时时间，设置为1000毫秒，表示到达目标点位的最大允许时间
     */
    uint16_t moveFreqHz = 300;
    uint16_t settleMs = 500;
    uint16_t pointTimeoutMs = 1000;
    // ... existing code ...

    unsigned long waitStartMs = 0;
    uint32_t pointSeqAtWait = 0;
};

CalibrationTask g_calTask;

constexpr long YAW_MIN_POS = 0;
constexpr long YAW_MAX_POS = 500;
constexpr long PITCH_MIN_POS = 0;
constexpr long PITCH_MAX_POS = 750;

float targetX = 160.0f;
float targetY = 120.0f;
float currentX = 0.0f;
float currentY = 0.0f;
bool hasPidData = false;
unsigned long lastPidRxMs = 0;

float latestLaserX = 0.0f;
float latestLaserY = 0.0f;
uint32_t latestPointSeq = 0;

float lastYawSpeed = 0.0f;
float lastPitchSpeed = 0.0f;
float lastErrorX = 0.0f;
float lastErrorY = 0.0f;

long angleDegToSteps(float angleDeg)
{
    return lroundf((angleDeg / 1.8f) * 32.0f);
}

float stepsToAngleDeg(long steps)
{
    return ((float)steps * 1.8f) / 32.0f;
}

long absLong(long v)
{
    return (v >= 0) ? v : -v;
}

long clampLong(long value, long minValue, long maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

void enforceSpeedPositionLimit(stepmotor_ard& motor, long minPos, long maxPos)
{
    long pos = motor.getPosition();
    float speed = motor.getTargetSpeed();

    if (speed > 0.0f && pos <= minPos) {
        motor.stopSpeed();
        return;
    }

    if (speed < 0.0f && pos >= maxPos) {
        motor.stopSpeed();
    }
}

bool moveMotorTo(stepmotor_ard& motor, long targetPos, uint16_t stepFreqHz)
{
    if (&motor == &motor_yaw) {
        targetPos = clampLong(targetPos, YAW_MIN_POS, YAW_MAX_POS);
    } else if (&motor == &motor_pitch) {
        targetPos = clampLong(targetPos, PITCH_MIN_POS, PITCH_MAX_POS);
    }

    long cur = motor.getPosition();
    long delta = targetPos - cur;
    if (delta == 0) {
        return true;
    }

    bool directionPositive = (delta < 0);
    motor.step((uint32_t)absLong(delta), directionPositive, stepFreqHz);
    return true;
}

bool withinRange(long value, long end, bool increasing)
{
    if (increasing) {
        return value <= end;
    }
    return value >= end;
} //防止过冲，使用简单的等于判断，可能会导致传感器跑太快，单片机根本找不到相等的那个时刻

void requestPointForCurrentPose()
{
    g_calTask.waitingPoint = true;
    g_calTask.waitStartMs = millis();
    g_calTask.pointSeqAtWait = latestPointSeq;
    Serial.print("REQ_POINT ");
    Serial.print(g_calTask.currentYaw);
    Serial.print(' ');
    Serial.println(g_calTask.currentPitch);
}

void appendCalibrationSample(float x, float y)
{
    if (g_calCount >= MAX_CAL_SAMPLES) {
        Serial.println("CAL_ERR TABLE_FULL");
        g_calTask.active = false;
        g_calTask.waitingPoint = false;
        g_mode = MODE_IDLE;
        return;
    }

    CalibrationSample& sample = g_calTable[g_calCount++];//引用，但是实际上操作的是g_calTable数组的元素,相当于在函数内操作全局数组
    sample.yawPos = g_calTask.currentYaw;
    sample.pitchPos = g_calTask.currentPitch;
    sample.laserX = x;
    sample.laserY = y;
}

bool stepCalibrationGrid()
{
    bool yawIncreasing = (g_calTask.yawEnd >= g_calTask.yawStart);
    bool pitchIncreasing = (g_calTask.pitchEnd >= g_calTask.pitchStart);

    long nextYaw = g_calTask.currentYaw;
    if (g_calTask.rowForward) {
        nextYaw += yawIncreasing ? g_calTask.yawStep : -g_calTask.yawStep;
        if (withinRange(nextYaw, g_calTask.yawEnd, yawIncreasing)) {
            g_calTask.currentYaw = nextYaw;
            moveMotorTo(motor_yaw, g_calTask.currentYaw, g_calTask.moveFreqHz);
            return true;
        }
    } else {
        nextYaw += yawIncreasing ? -g_calTask.yawStep : g_calTask.yawStep;
        if (withinRange(nextYaw, g_calTask.yawStart, !yawIncreasing)) {
            g_calTask.currentYaw = nextYaw;
            moveMotorTo(motor_yaw, g_calTask.currentYaw, g_calTask.moveFreqHz);
            return true;
        }
    }

    long nextPitch = g_calTask.currentPitch + (pitchIncreasing ? g_calTask.pitchStep : -g_calTask.pitchStep);
    if (!withinRange(nextPitch, g_calTask.pitchEnd, pitchIncreasing)) {
        return false;
    }

    g_calTask.currentPitch = nextPitch;
    g_calTask.rowForward = !g_calTask.rowForward;
    moveMotorTo(motor_pitch, g_calTask.currentPitch, g_calTask.moveFreqHz);
    return true;
}

void finishCalibration()
{
    g_calTask.active = false;
    g_calTask.waitingPoint = false;
    g_mode = MODE_IDLE;
    Serial.print("CAL_DONE count=");
    Serial.println(g_calCount);
}

void updateCalibrationTask()
{
    if (!g_calTask.active) {
        return;
    }

    if (g_calTask.waitingPoint) {
        if (latestPointSeq != g_calTask.pointSeqAtWait) {
            appendCalibrationSample(latestLaserX, latestLaserY);
            g_calTask.waitingPoint = false;

            if (!g_calTask.active) {
                return;
            }//上位机命令层面，上位机没有发送ABORT中止命令

            if (!stepCalibrationGrid()) {//这里走一个step
                finishCalibration();
                return;
            }//下位机物理层面，没有超出边界限制或者扫描没有完成

            delay(g_calTask.settleMs);//等待步进电机稳定时间
            requestPointForCurrentPose();
            return;
        }

        if ((millis() - g_calTask.waitStartMs) >= g_calTask.pointTimeoutMs) {//超出数据接收超时时间
            appendCalibrationSample(-1.0f, -1.0f);
            g_calTask.waitingPoint = false;

            if (!g_calTask.active) {
                return;
            }

            if (!stepCalibrationGrid()) {
                finishCalibration();
                return;
            }

            delay(g_calTask.settleMs);
            requestPointForCurrentPose();
        }
    }
}

void startCalibration(float pitchStartDeg,
                      float pitchEndDeg,
                      float pitchStepDeg,
                      float yawStartDeg,
                      float yawEndDeg,
                      float yawStepDeg)
{
    long pitchStep = absLong(angleDegToSteps(pitchStepDeg));
    long yawStep = absLong(angleDegToSteps(yawStepDeg));
    if (pitchStep < 1 || yawStep < 1) {
        Serial.println("CAL_ERR STEP_TOO_SMALL");
        return;
    }

    motor_yaw.stopSpeed();
    motor_pitch.stopSpeed();
    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;

    g_calCount = 0;
    g_calTask.active = true;
    g_calTask.waitingPoint = false;
    g_calTask.rowForward = true;

    g_calTask.pitchStart = angleDegToSteps(pitchStartDeg);
    g_calTask.pitchEnd = angleDegToSteps(pitchEndDeg);
    g_calTask.pitchStep = pitchStep;

    g_calTask.yawStart = angleDegToSteps(yawStartDeg);
    g_calTask.yawEnd = angleDegToSteps(yawEndDeg);
    g_calTask.yawStep = yawStep;

    g_calTask.pitchStart = clampLong(g_calTask.pitchStart, PITCH_MIN_POS, PITCH_MAX_POS);
    g_calTask.pitchEnd = clampLong(g_calTask.pitchEnd, PITCH_MIN_POS, PITCH_MAX_POS);
    g_calTask.yawStart = clampLong(g_calTask.yawStart, YAW_MIN_POS, YAW_MAX_POS);
    g_calTask.yawEnd = clampLong(g_calTask.yawEnd, YAW_MIN_POS, YAW_MAX_POS);

    g_calTask.currentPitch = g_calTask.pitchStart;
    g_calTask.currentYaw = g_calTask.yawStart;

    g_mode = MODE_CALIB;

    moveMotorTo(motor_pitch, g_calTask.currentPitch, g_calTask.moveFreqHz);
    moveMotorTo(motor_yaw, g_calTask.currentYaw, g_calTask.moveFreqHz);
    delay(g_calTask.settleMs);

    Serial.print("CAL_BEGIN pitch[");
    Serial.print(stepsToAngleDeg(g_calTask.pitchStart));
    Serial.print(",");
    Serial.print(stepsToAngleDeg(g_calTask.pitchEnd));
    Serial.print("] yaw[");
    Serial.print(stepsToAngleDeg(g_calTask.yawStart));
    Serial.print(",");
    Serial.print(stepsToAngleDeg(g_calTask.yawEnd));
    Serial.println("]");

    requestPointForCurrentPose();
}

void dumpCalibrationTable()
{
    Serial.print("CAL_TABLE count=");
    Serial.println(g_calCount);
    for (size_t i = 0; i < g_calCount; ++i) {
        Serial.print(i);
        Serial.print(' ');
        Serial.print(g_calTable[i].yawPos);
        Serial.print(' ');
        Serial.print(g_calTable[i].pitchPos);
        Serial.print(' ');
        Serial.print(g_calTable[i].laserX, 3);
        Serial.print(' ');
        Serial.println(g_calTable[i].laserY, 3);
    }
}

void openLoopToLaserPoint(float x, float y)
{
    if (g_calCount == 0) {
        Serial.println("OPEN_ERR NO_TABLE");
        return;
    }

    size_t bestIdx = 0;
    float bestDist2 = 1e30f;
    bool foundValid = false;
    for (size_t i = 0; i < g_calCount; ++i) {
        if (g_calTable[i].laserX < 0.0f || g_calTable[i].laserY < 0.0f) {
            continue;
        }
        foundValid = true;
        float dx = g_calTable[i].laserX - x;
        float dy = g_calTable[i].laserY - y;
        float dist2 = dx * dx + dy * dy;
        if (dist2 < bestDist2) {
            bestDist2 = dist2;
            bestIdx = i;
        }
    }

    if (!foundValid) {
        Serial.println("OPEN_ERR NO_VALID_SAMPLE");
        return;
    }

    motor_yaw.stopSpeed();
    motor_pitch.stopSpeed();
    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;
    g_mode = MODE_IDLE;

    moveMotorTo(motor_pitch, g_calTable[bestIdx].pitchPos, 1000);
    moveMotorTo(motor_yaw, g_calTable[bestIdx].yawPos, 1000);

    Serial.print("OPEN_OK idx=");
    Serial.print(bestIdx);
    Serial.print(" yaw=");
    Serial.print(g_calTable[bestIdx].yawPos);
    Serial.print(" pitch=");
    Serial.println(g_calTable[bestIdx].pitchPos);
}

void moveToAnglePose(float pitchDeg, float yawDeg, uint16_t stepFreqHz)
{
    if (stepFreqHz < 1) {
        stepFreqHz = 1;
    }

    g_calTask.active = false;
    g_calTask.waitingPoint = false;

    motor_yaw.stopSpeed();
    motor_pitch.stopSpeed();
    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;
    g_mode = MODE_IDLE;

    long targetPitchSteps = angleDegToSteps(pitchDeg);
    long targetYawSteps = angleDegToSteps(yawDeg);
    targetPitchSteps = clampLong(targetPitchSteps, PITCH_MIN_POS, PITCH_MAX_POS);
    targetYawSteps = clampLong(targetYawSteps, YAW_MIN_POS, YAW_MAX_POS);

    moveMotorTo(motor_pitch, targetPitchSteps, stepFreqHz);
    moveMotorTo(motor_yaw, targetYawSteps, stepFreqHz);

    Serial.print("ANGLE_OK pitchDeg=");
    Serial.print(pitchDeg, 3);
    Serial.print(" yawDeg=");
    Serial.print(yawDeg, 3);
    Serial.print(" pitchStep=");
    Serial.print(targetPitchSteps);
    Serial.print(" yawStep=");
    Serial.println(targetYawSteps);
}

void moveToStepPose(long yawStep, long pitchStep, uint16_t stepFreqHz)
{
    if (stepFreqHz < 1) {
        stepFreqHz = 1;
    }

    yawStep = clampLong(yawStep, YAW_MIN_POS, YAW_MAX_POS);
    pitchStep = clampLong(pitchStep, PITCH_MIN_POS, PITCH_MAX_POS);

    g_calTask.active = false;
    g_calTask.waitingPoint = false;

    motor_yaw.stopSpeed();
    motor_pitch.stopSpeed();
    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;
    g_mode = MODE_IDLE;

    moveMotorTo(motor_pitch, pitchStep, stepFreqHz);
    moveMotorTo(motor_yaw, yawStep, stepFreqHz);

    Serial.print("POS_OK yawStep=");
    Serial.print(yawStep);
    Serial.print(" pitchStep=");
    Serial.println(pitchStep);
}

void setDirectSpeed(float yawSpeed, float pitchSpeed)
{
    g_calTask.active = false;
    g_calTask.waitingPoint = false;

    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;
    g_mode = MODE_SPEED;

    if (yawSpeed == 0.0f) {
        motor_yaw.stopSpeed();
    } else {
        long yawPos = motor_yaw.getPosition();
        if ((yawSpeed > 0.0f && yawPos <= YAW_MIN_POS) ||
            (yawSpeed < 0.0f && yawPos >= YAW_MAX_POS)) {
            motor_yaw.stopSpeed();
            yawSpeed = 0.0f;
        } else {
            motor_yaw.setTargetSpeed(fabs(yawSpeed), yawSpeed >= 0.0f);
        }
    }

    if (pitchSpeed == 0.0f) {
        motor_pitch.stopSpeed();
    } else {
        long pitchPos = motor_pitch.getPosition();
        if ((pitchSpeed > 0.0f && pitchPos <= PITCH_MIN_POS) ||
            (pitchSpeed < 0.0f && pitchPos >= PITCH_MAX_POS)) {
            motor_pitch.stopSpeed();
            pitchSpeed = 0.0f;
        } else {
            motor_pitch.setTargetSpeed(fabs(pitchSpeed), pitchSpeed >= 0.0f);
        }
    }

    lastYawSpeed = yawSpeed;
    lastPitchSpeed = pitchSpeed;
    lastErrorX = 0.0f;
    lastErrorY = 0.0f;

    Serial.print("SPEED_OK yawSpeed=");
    Serial.print(yawSpeed, 3);
    Serial.print(" pitchSpeed=");
    Serial.println(pitchSpeed, 3);
}

void moveByStepDelta(long yawDeltaStep, long pitchDeltaStep, uint16_t stepFreqHz)
{
    if (stepFreqHz < 1) {
        stepFreqHz = 1;
    }

    g_calTask.active = false;
    g_calTask.waitingPoint = false;

    motor_yaw.stopSpeed();
    motor_pitch.stopSpeed();
    // PID_DISABLED:
    // pidController.reset();
    // hasPidData = false;
    g_mode = MODE_IDLE;

    long targetYawStep = motor_yaw.getPosition() + yawDeltaStep;
    long targetPitchStep = motor_pitch.getPosition() + pitchDeltaStep;

    moveMotorTo(motor_pitch, targetPitchStep, stepFreqHz);
    moveMotorTo(motor_yaw, targetYawStep, stepFreqHz);

    Serial.print("DPOS_OK yawStep=");
    Serial.print(targetYawStep);
    Serial.print(" pitchStep=");
    Serial.print(targetPitchStep);
    Serial.print(" dYaw=");
    Serial.print(yawDeltaStep);
    Serial.print(" dPitch=");
    Serial.println(pitchDeltaStep);
}

void applyPidUpdate()
{
    // PID_DISABLED: keep entry point, disable closed-loop update logic.
    /*
    static unsigned long lastPidMs = 0;
    unsigned long nowMs = millis();

    if (g_mode != MODE_PID) {
        return;
    }

    if (!hasPidData) {
        motor_pitch.stopSpeed();
        motor_yaw.stopSpeed();
        return;
    }

    if (nowMs - lastPidRxMs > 200) {
        hasPidData = false;
        motor_pitch.stopSpeed();
        motor_yaw.stopSpeed();
        return;
    }

    if (nowMs - lastPidMs < 20) {
        return;
    }
    lastPidMs = nowMs;

    lastErrorX = targetX - currentX;
    lastErrorY = targetY - currentY;

    PointPidOutput out = pidController.update(
        targetX, targetY,
        currentX, currentY,
        0.02f
    );

    lastYawSpeed = out.yawSpeed;
    lastPitchSpeed = out.pitchSpeed;

    if (out.yawSpeed == 0.0f) {
        motor_yaw.stopSpeed();
    } else {
        motor_yaw.setTargetSpeed(fabs(out.yawSpeed), out.yawSpeed >= 0.0f);
    }

    if (out.pitchSpeed == 0.0f) {
        motor_pitch.stopSpeed();
    } else {
        motor_pitch.setTargetSpeed(fabs(out.pitchSpeed), out.pitchSpeed >= 0.0f);
    }
    */
}

void processLine(char* line)
{
    while (*line == ' ' || *line == '\t') {
        ++line;
    }
    if (*line == '\0') {
        return;
    }

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
    float e = 0.0f;
    float f = 0.0f;

    if (sscanf(line, "POINT %f %f", &a, &b) == 2) {
        latestLaserX = a;
        latestLaserY = b;
        latestPointSeq++;
        return;
    }

    // PID_DISABLED:
    // if (sscanf(line, "PID %f %f %f %f", &a, &b, &c, &d) == 4 ||
    //     sscanf(line, "%f %f %f %f", &a, &b, &c, &d) == 4) {
    //     targetX = a;
    //     targetY = b;
    //     currentX = c;
    //     currentY = d;
    //     hasPidData = true;
    //     lastPidRxMs = millis();
    //     if (g_mode != MODE_CALIB) {
    //         g_mode = MODE_PID;
    //     }
    //     return;
    // }

    if (sscanf(line, "CAL_START %f %f %f %f %f %f", &a, &b, &c, &d, &e, &f) == 6) {
        startCalibration(a, b, c, d, e, f);
        return;
    }

    if (strcmp(line, "CAL_ABORT") == 0) {
        g_calTask.active = false;
        g_calTask.waitingPoint = false;
        g_mode = MODE_IDLE;
        Serial.println("CAL_ABORTED");
        return;
    }

    if (strcmp(line, "CAL_DUMP") == 0) {
        dumpCalibrationTable();
        return;
    }

    if (sscanf(line, "OPEN %f %f", &a, &b) == 2) {
        openLoopToLaserPoint(a, b);
        return;
    }

    if (sscanf(line, "SPEED %f %f", &a, &b) == 2) {
        setDirectSpeed(a, b);
        return;
    }


    unsigned int freqHz = 0;
    if (sscanf(line, "ANGLE %f %f %u", &a, &b, &freqHz) == 3) {
        if (freqHz > 65535U) {
            freqHz = 65535U;
        }
        moveToAnglePose(a, b, (uint16_t)freqHz);
        return;
    }

    if (sscanf(line, "ANGLE %f %f", &a, &b) == 2) {
        moveToAnglePose(a, b, 1000);
        return;
    }

    if (sscanf(line, "DANGLE %f %f %u", &a, &b, &freqHz) == 3) {
        if (freqHz > 65535U) {
            freqHz = 65535U;
        }
        long dPitchStep = angleDegToSteps(a);
        long dYawStep = angleDegToSteps(b);
        moveByStepDelta(dYawStep, dPitchStep, (uint16_t)freqHz);
        return;
    }

    if (sscanf(line, "DANGLE %f %f", &a, &b) == 2) {
        long dPitchStep = angleDegToSteps(a);
        long dYawStep = angleDegToSteps(b);
        moveByStepDelta(dYawStep, dPitchStep, 1000);
        return;
    }

    long yawStep = 0;
    long pitchStep = 0;
    unsigned int freqHzU = 0;
    if (sscanf(line, "POS %ld %ld %u", &yawStep, &pitchStep, &freqHzU) == 3) {
        if (freqHzU > 65535U) {
            freqHzU = 65535U;
        }
        moveToStepPose(yawStep, pitchStep, (uint16_t)freqHzU);
        return;
    }

    if (sscanf(line, "POS %ld %ld", &yawStep, &pitchStep) == 2) {
        moveToStepPose(yawStep, pitchStep, 1000);
        return;
    }

    if (sscanf(line, "DPOS %ld %ld %u", &yawStep, &pitchStep, &freqHzU) == 3) {
        if (freqHzU > 65535U) {
            freqHzU = 65535U;
        }
        moveByStepDelta(yawStep, pitchStep, (uint16_t)freqHzU);
        return;
    }

    if (sscanf(line, "DPOS %ld %ld", &yawStep, &pitchStep) == 2) {
        moveByStepDelta(yawStep, pitchStep, 1000);
        return;
    }

    if (strcmp(line, "STOP") == 0) {
        motor_pitch.stopSpeed();
        motor_yaw.stopSpeed();
        g_mode = MODE_IDLE;
        // PID_DISABLED:
        // hasPidData = false;
        Serial.println("STOPPED");
        return;
    }

    Serial.print("CMD_UNKNOWN ");
    Serial.println(line);
}

void pollSerialLine()
{
    static char rxBuf[96];
    static uint8_t idx = 0;

    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            rxBuf[idx] = '\0';
            processLine(rxBuf);
            idx = 0;
            continue;
        }

        if (idx < sizeof(rxBuf) - 1U) {
            rxBuf[idx++] = ch;
        } else {
            idx = 0;//缓存区满了，丢弃数据
        }
    }
}

void setupTimer1_500us()
{
    cli();

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 999;

    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);

    sei();
}

ISR(TIMER1_COMPA_vect)
{
    enforceSpeedPositionLimit(motor_yaw, YAW_MIN_POS, YAW_MAX_POS);
    enforceSpeedPositionLimit(motor_pitch, PITCH_MIN_POS, PITCH_MAX_POS);
    motor_yaw.runSpeed(500);
    motor_pitch.runSpeed(500);
    enforceSpeedPositionLimit(motor_yaw, YAW_MIN_POS, YAW_MAX_POS);
    enforceSpeedPositionLimit(motor_pitch, PITCH_MIN_POS, PITCH_MAX_POS);

    static uint16_t tickCount = 0;
    tickCount++;
    if (tickCount >= 100) {
        tickCount = 0;
        printFlag = true;
    }
}

void setup()
{
    Serial.begin(115200);

    motor_yaw.begin();
    motor_pitch.begin();

    // PID_DISABLED:
    // pidController.xCfg.kp = 3.0f;
    // pidController.xCfg.ki = 0.15f;
    // pidController.xCfg.kd = 0.08f;
    // pidController.xCfg.integralLimit = 100.0f;
    // pidController.xCfg.outputLimit = 800.0f;
    // pidController.xCfg.errorDeadband = 2.0f;
    // pidController.xCfg.outputDeadband = 5.0f;

    // pidController.yCfg.kp = 3.0f;
    // pidController.yCfg.ki = 0.15f;
    // pidController.yCfg.kd = 0.08f;
    // pidController.yCfg.integralLimit = 100.0f;
    // pidController.yCfg.outputLimit = 800.0f;
    // pidController.yCfg.errorDeadband = 2.0f;
    // pidController.yCfg.outputDeadband = 5.0f;

    setupTimer1_500us();
    Serial.println("READY");
}

void loop()
{
    pollSerialLine();
    updateCalibrationTask();
    // PID_DISABLED:
    // applyPidUpdate();

    if (printFlag) {
        printFlag = false;
        Serial.print("POS ");
        Serial.print(motor_yaw.getPosition());
        Serial.print(' ');
        Serial.println(motor_pitch.getPosition());
    }
}


/*
* [开机] -> setup() -> 开启硬件定时器(后台掌控电机驱动)
            |
            v
[主线程] -> loop() =====(死循环)==============================>
            |                                               |
            |-- 1. pollSerialLine() --------------------|   |
            |       └─ processLine("CAL_START")         |   |
            |           └─ startCalibration()           |   |
            |               ├─ moveMotorTo(起点)         |   |
            |               └─ requestPointForCurrentPose() |
            |                                               |
            |-- 2. updateCalibrationTask() -------------|   |
            |       ├─ [如果正在等待视觉数据]               |   |
            |       │   ├─ 收到 POINT? -> 记录点数据      |   |
            |       │   │                └─ stepCalibrationGrid() --(走完了?)--> finishCalibration()
            |       │   │                └─ delay(稳定)
            |       │   │                └─ requestPointForCurrentPose() [循环索要坐标]
            |       │   │
            |       │   └─ 超时没收到? -> 记录无效点(-1,-1)并强行跳到下一个点
            |                                               |
            |-- 3. applyPidUpdate() [标定模式下直接return] -|   |
            =================================================
 */
