#ifndef CTRL_STEP_MOTOR_H
#define CTRL_STEP_MOTOR_H

#include <stdint.h>

class stepmotor_base {
public:
    explicit stepmotor_base(bool dir_level);
    virtual ~stepmotor_base() = default;

    void begin();

    void setDir(bool positive);
    bool getDir() const;

    void stepOnce();
    void step(uint32_t steps, bool direction, uint32_t stepFreqHZ);
    void step(uint32_t steps, bool direction);

    void forward(uint32_t steps);
    void backward(uint32_t steps);

    void setTargetSpeed(float stepsPerSecond,bool direction);
    float getTargetSpeed() const;
    void stopSpeed();
    void runSpeed(uint32_t deltaUs);
    bool isSpeedRunning() const;

    void setPulseWidthUs(uint16_t pulseWidthUs);
    uint16_t getPulseWidthUs() const;

    void setDefaultStepFreq(uint32_t freqHz);
    uint32_t getDefaultStepFreq() const;

    long getPosition() const;
    void setPosition(long pos);
    void resetPosition();

    void setTargetAngle(float angle);
    float getAngel();

protected:
    bool _dir_level;
    bool _dirpositive;
    bool _zeroflag=false;
    uint16_t _pulseWidthUs;
    uint32_t _defaultStepFreq;
    float _targetSpeedStepsPerSecond;
    float _stepAccumulator;
    long _position;
    float _angel;

    virtual void initInterface() = 0;
    virtual void applyDirectionLevel(bool level) = 0;
    virtual void writeStepHigh() = 0;
    virtual void writeStepLow() = 0;
    virtual void delayUs(uint32_t microseconds) = 0;
    virtual void Set2zero() = 0;
    void pulseStep();
    void updatePositionAfterStep();
};

#endif // CTRL_STEP_MOTOR_H
