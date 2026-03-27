#include "stepmotor_ard.h"

stepmotor_ard::stepmotor_ard(uint8_t stepPin,
                             uint8_t dirPin,
                             uint8_t zeroPin,
                             bool dir_level)
    : stepmotor_base(dir_level),
      _stepPin(stepPin),
      _dirPin(dirPin),
      _zeroPin(zeroPin){}

void stepmotor_ard::initInterface()
{
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_zeroPin, INPUT_PULLUP);//上拉
}

void stepmotor_ard::applyDirectionLevel(bool level)
{
    digitalWrite(_dirPin, level ? HIGH : LOW);
}

void stepmotor_ard::writeStepHigh()
{
    digitalWrite(_stepPin, HIGH);
}

void stepmotor_ard::writeStepLow()
{
    digitalWrite(_stepPin, LOW);
}

void stepmotor_ard::delayUs(uint32_t microseconds)
{
    if (microseconds == 0) {
        return;
    }

    delayMicroseconds(microseconds);
}

void stepmotor_ard::Set2zero()
{
    while (digitalRead(_zeroPin) == HIGH) {
        stepOnce();
    }
    _zeroflag = true;
    resetPosition();
}
