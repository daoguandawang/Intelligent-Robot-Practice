#include "stepmotor_base.h"

#include <stdlib.h>

stepmotor_base::stepmotor_base(bool dir_level)
    : _dir_level(dir_level),
      _dirpositive(true),
      _pulseWidthUs(5),
      _defaultStepFreq(1000),
      _targetSpeedStepsPerSecond(0.0f),
      _stepAccumulator(0.0f),
      _position(0),
      _angel(0.0f) {}

void stepmotor_base::begin()
{
    initInterface();
    writeStepLow();
    setDir(true);
}

void stepmotor_base::setDir(bool positive)
{
    _dirpositive = positive;
    applyDirectionLevel(positive ? _dir_level : !_dir_level);
}

bool stepmotor_base::getDir() const
{
    return _dirpositive;
}

void stepmotor_base::updatePositionAfterStep()
{
    if (_dirpositive) {
        _position--;
    } else {
        _position++;
    }

    _angel=float((_position / 32.0f) * 1.8f);
}

void stepmotor_base::stepOnce()
{
    pulseStep();
    delayUs(1000);//太快了会跟不上啊我日
}

void stepmotor_base::pulseStep()
{
    writeStepHigh();
    delayUs(_pulseWidthUs);
    writeStepLow();
    updatePositionAfterStep();
}

void stepmotor_base::step(uint32_t steps, bool direction, uint32_t stepFreqHZ)
{
    if (stepFreqHZ == 0 || steps == 0) {
        return;
    }

    setDir(direction);

    uint32_t stepTimeUs = 1000000UL / stepFreqHZ;
    if (stepTimeUs < (_pulseWidthUs * 2UL)) {
        stepTimeUs = _pulseWidthUs * 2UL;

    }

    uint32_t lowTimeUs = stepTimeUs - _pulseWidthUs;
    for (uint32_t i = 0; i < steps; i++) {
        writeStepHigh();
        delayUs(_pulseWidthUs);
        writeStepLow();
        delayUs(lowTimeUs);
        updatePositionAfterStep();
    }
}

void stepmotor_base::step(uint32_t steps, bool direction)
{
    step(steps, direction, _defaultStepFreq);
}

void stepmotor_base::forward(uint32_t steps)
{
    step(steps, true, _defaultStepFreq);
}

void stepmotor_base::backward(uint32_t steps)
{
    step(steps, false, _defaultStepFreq);
}

void stepmotor_base::setTargetSpeed(float stepsPerSecond, bool direction)
{
    _targetSpeedStepsPerSecond = direction ? stepsPerSecond : -stepsPerSecond;
}

float stepmotor_base::getTargetSpeed() const
{
    return _targetSpeedStepsPerSecond;
}

void stepmotor_base::stopSpeed()
{
    _targetSpeedStepsPerSecond = 0.0f;

    _stepAccumulator = 0.0f;
}

void stepmotor_base::runSpeed(uint32_t deltaUs)
{
    if (deltaUs == 0 || _targetSpeedStepsPerSecond == 0.0f) {
        return;
    }

    float deltaSteps = (_targetSpeedStepsPerSecond * static_cast<float>(deltaUs)) / 1000000.0f;
    _stepAccumulator += deltaSteps;

    if (_stepAccumulator >= 1.0f) {
        setDir(true);
        while (_stepAccumulator >= 1.0f) {
            pulseStep();
            _stepAccumulator -= 1.0f;
        }
        return;
    }

    if (_stepAccumulator <= -1.0f) {
        setDir(false);
        while (_stepAccumulator <= -1.0f) {
            pulseStep();
            _stepAccumulator += 1.0f;
        }
    }
}

bool stepmotor_base::isSpeedRunning() const
{
    return _targetSpeedStepsPerSecond != 0.0f;
}

void stepmotor_base::setPulseWidthUs(uint16_t pulseWidthUs)
{
    if (pulseWidthUs < 1) {
        pulseWidthUs = 1;
    }

    _pulseWidthUs = pulseWidthUs;
}

uint16_t stepmotor_base::getPulseWidthUs() const
{
    return _pulseWidthUs;
}

void stepmotor_base::setDefaultStepFreq(uint32_t freqHz)
{
    if (freqHz < 1) {
        freqHz = 1;
    }

    _defaultStepFreq = freqHz;
}

uint32_t stepmotor_base::getDefaultStepFreq() const
{
    return _defaultStepFreq;
}

void stepmotor_base::setPosition(long pos)
{
    _position = pos;
    _angel = float((_position / 32.0f) * 1.8f);
}

void stepmotor_base::resetPosition()
{
    _position = 0;
    _angel = 0.0f;
}

long stepmotor_base::getPosition() const
{
    return _position;
}

float stepmotor_base::getAngel()
{
    return _angel;
}

void stepmotor_base::setTargetAngle(float angle)
{
    if ((_angel < angle)&&(abs(_angel-angle)>=1.0f)) {
        setDir(false);
        stepOnce();//极性不要反
    }
    else if ((_angel > angle)&&(abs(_angel-angle)>=1.0f)) {
        setDir(true);
        stepOnce();
    }
    else {
        return;
    }
}
