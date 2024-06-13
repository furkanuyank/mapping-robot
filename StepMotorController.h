#ifndef STEPMOTORCONTROLLER_H
#define STEPMOTORCONTROLLER_H

#include <Arduino.h>

class StepMotorController {
public:
    StepMotorController(int dirPin1, int stepPin1, int enablePin1,
                        int dirPin2, int stepPin2, int enablePin2,
                        int stepDelay);

    void begin();
    void ileri();
    void geri();
    void sag();
    void sol();
    void tara();
    void dur();
    void update();

private:
    void move(int dirPin1State, int dirPin2State, unsigned long duration);
    void stepMotor(int stepPin);

    int _dirPin1;
    int _stepPin1;
    int _enablePin1;

    int _dirPin2;
    int _stepPin2;
    int _enablePin2;

    int _stepDelay;
    unsigned long _moveEndTime;
    bool _moving;
    bool _stopped;
};

#endif
