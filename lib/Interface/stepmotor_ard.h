#ifndef CTRL_STEPMOTOR_ARD_H
#define CTRL_STEPMOTOR_ARD_H

#include <Arduino.h>
#include <stepmotor_base.h>

class stepmotor_ard : public stepmotor_base {
public:
    stepmotor_ard(uint8_t stepPin,
                  uint8_t dirPin,
                  uint8_t zeroPin,
                  bool dir_level);
    void begin()
    {
        stepmotor_base::begin();
        Set2zero();
    }

protected:
    void initInterface() override;
    void applyDirectionLevel(bool level) override;
    void writeStepHigh() override;
    void writeStepLow() override;
    void Set2zero() override;
    void delayUs(uint32_t microseconds) override;

private:
    uint8_t _stepPin;
    uint8_t _dirPin;
    uint8_t _zeroPin;
};

#endif // CTRL_STEPMOTOR_ARD_H
